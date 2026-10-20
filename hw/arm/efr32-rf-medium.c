/*
 * Internal RF medium for EFR32 radio peripherals.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/arm/efr32-rf-medium.h"

struct Efr32RfMedium {
    Object parent_obj;

    GList *radios;
    uint64_t next_tx_sequence;
    Efr32RfTxSnapshot *latest_tx_snapshot;
    GQueue rx_offers;
};

struct Efr32RfTxSnapshot {
    Efr32RfRadio *radio;
    Efr32RfTxMetadata metadata;
    uint64_t sequence;
    GBytes *data;
};

struct Efr32RfRxDeliveryRequest {
    Efr32RfRadio *receiver;
    Efr32RfRxMetadata metadata;
    GBytes *data;
};

bool efr32_rf_radio_is_rx_ready(Efr32RfRadio *radio)
{
    Efr32RfRadioClass *klass = EFR32_RF_RADIO_GET_CLASS(radio);

    return klass->is_rx_ready && klass->is_rx_ready(radio);
}

void efr32_rf_medium_attach(Efr32RfMedium *medium, Efr32RfRadio *radio,
                            Error **errp)
{
    if (!object_dynamic_cast(OBJECT(radio), TYPE_EFR32_RF_RADIO)) {
        error_setg(errp, "%s does not implement %s",
                   object_get_typename(OBJECT(radio)), TYPE_EFR32_RF_RADIO);
        return;
    }

    if (g_list_find(medium->radios, radio)) {
        return;
    }

    medium->radios = g_list_append(medium->radios, radio);
}

void efr32_rf_medium_detach(Efr32RfMedium *medium, Efr32RfRadio *radio)
{
    GList *member = g_list_find(medium->radios, radio);

    if (!member) {
        return;
    }

    efr32_rf_medium_reset_radio(medium, radio);
    medium->radios = g_list_delete_link(medium->radios, member);
}

bool efr32_rf_medium_has_radio(const Efr32RfMedium *medium,
                                const Efr32RfRadio *radio)
{
    return g_list_find(medium->radios, radio) != NULL;
}

Efr32RfTxSnapshot *efr32_rf_medium_snapshot_tx(
    Efr32RfMedium *medium, Efr32RfRadio *radio,
    const Efr32RfTxMetadata *metadata, const uint8_t *data, size_t length,
    Error **errp)
{
    Efr32RfTxSnapshot *snapshot;

    if (!efr32_rf_medium_has_radio(medium, radio)) {
        error_setg(errp, "%s is not attached to this RF medium",
                   object_get_typename(OBJECT(radio)));
        return NULL;
    }
    if (!metadata) {
        error_setg(errp, "RF TX snapshot requires metadata");
        return NULL;
    }
    if (metadata->trigger != EFR32_RF_TX_TRIGGER_RAC_CMD_TXEN) {
        error_setg(errp, "RF TX snapshot has an unsupported trigger");
        return NULL;
    }
    if (metadata->buffer_channel != EFR32_RF_TX_BUFC_CHANNEL) {
        error_setg(errp, "RF TX snapshot must use BUFC channel %u",
                   EFR32_RF_TX_BUFC_CHANNEL);
        return NULL;
    }
    if (!data && length) {
        error_setg(errp, "RF TX snapshot has data length but no data");
        return NULL;
    }

    snapshot = g_new0(Efr32RfTxSnapshot, 1);
    snapshot->radio = EFR32_RF_RADIO(object_ref(OBJECT(radio)));
    snapshot->metadata = *metadata;
    snapshot->sequence = medium->next_tx_sequence++;
    snapshot->data = g_bytes_new(data, length);
    return snapshot;
}

void efr32_rf_tx_snapshot_free(Efr32RfTxSnapshot *snapshot)
{
    if (!snapshot) {
        return;
    }

    object_unref(OBJECT(snapshot->radio));
    g_bytes_unref(snapshot->data);
    g_free(snapshot);
}

Efr32RfRadio *efr32_rf_tx_snapshot_get_radio(
    const Efr32RfTxSnapshot *snapshot)
{
    return snapshot->radio;
}

const Efr32RfTxMetadata *efr32_rf_tx_snapshot_get_metadata(
    const Efr32RfTxSnapshot *snapshot)
{
    return &snapshot->metadata;
}

uint64_t efr32_rf_tx_snapshot_get_sequence(
    const Efr32RfTxSnapshot *snapshot)
{
    return snapshot->sequence;
}

GBytes *efr32_rf_tx_snapshot_get_data(const Efr32RfTxSnapshot *snapshot)
{
    return snapshot->data;
}

void efr32_rf_medium_set_latest_tx_snapshot(Efr32RfMedium *medium,
                                             Efr32RfTxSnapshot *snapshot)
{
    efr32_rf_tx_snapshot_free(medium->latest_tx_snapshot);
    medium->latest_tx_snapshot = snapshot;
}

const Efr32RfTxSnapshot *efr32_rf_medium_get_latest_tx_snapshot(
    const Efr32RfMedium *medium)
{
    return medium->latest_tx_snapshot;
}

void efr32_rf_medium_clear_latest_tx_snapshot(Efr32RfMedium *medium)
{
    efr32_rf_medium_set_latest_tx_snapshot(medium, NULL);
}

Efr32RfRxDeliveryRequest *efr32_rf_medium_prepare_rx_delivery(
    Efr32RfMedium *medium, Efr32RfRadio *receiver,
    const Efr32RfRxMetadata *metadata, const uint8_t *data, size_t length,
    Error **errp)
{
    Efr32RfRxDeliveryRequest *request;

    if (!efr32_rf_medium_has_radio(medium, receiver)) {
        error_setg(errp, "%s is not attached to this RF medium",
                   object_get_typename(OBJECT(receiver)));
        return NULL;
    }
    if (!metadata) {
        error_setg(errp, "RF RX delivery request requires metadata");
        return NULL;
    }
    if (!efr32_rf_radio_is_rx_ready(receiver)) {
        error_setg(errp, "RF RX delivery request requires RAC RXSEARCH");
        return NULL;
    }
    if (metadata->buffer_channel != EFR32_RF_RX_BUFC_CHANNEL) {
        error_setg(errp, "RF RX delivery request must use BUFC channel %u",
                   EFR32_RF_RX_BUFC_CHANNEL);
        return NULL;
    }
    if (!data && length) {
        error_setg(errp, "RF RX delivery request has data length but no data");
        return NULL;
    }

    request = g_new0(Efr32RfRxDeliveryRequest, 1);
    request->receiver = EFR32_RF_RADIO(object_ref(OBJECT(receiver)));
    request->metadata = *metadata;
    request->data = g_bytes_new(data, length);
    return request;
}

void efr32_rf_rx_delivery_request_free(Efr32RfRxDeliveryRequest *request)
{
    if (!request) {
        return;
    }

    object_unref(OBJECT(request->receiver));
    g_bytes_unref(request->data);
    g_free(request);
}

Efr32RfRadio *efr32_rf_rx_delivery_request_get_receiver(
    const Efr32RfRxDeliveryRequest *request)
{
    return request->receiver;
}

const Efr32RfRxMetadata *efr32_rf_rx_delivery_request_get_metadata(
    const Efr32RfRxDeliveryRequest *request)
{
    return &request->metadata;
}

GBytes *efr32_rf_rx_delivery_request_get_data(
    const Efr32RfRxDeliveryRequest *request)
{
    return request->data;
}

unsigned efr32_rf_medium_offer_latest_tx(Efr32RfMedium *medium,
                                         Error **errp)
{
    const Efr32RfTxSnapshot *snapshot = medium->latest_tx_snapshot;
    Efr32RfRxMetadata metadata = {
        .buffer_channel = EFR32_RF_RX_BUFC_CHANNEL,
    };
    unsigned offers = 0;

    if (!snapshot) {
        return 0;
    }

    /*
     * This is deliberately a mailbox, not an RF channel model: it has no
     * propagation delay, collision, PHY, or receiver-side side effects.
     * GList preserves the board's attachment order for deterministic tests.
     */
    for (GList *member = medium->radios; member; member = member->next) {
        Efr32RfRadio *receiver = member->data;
        Efr32RfRxDeliveryRequest *request;
        gsize length;
        const uint8_t *data;

        if (receiver == efr32_rf_tx_snapshot_get_radio(snapshot) ||
            !efr32_rf_radio_is_rx_ready(receiver)) {
            continue;
        }

        data = g_bytes_get_data(efr32_rf_tx_snapshot_get_data(snapshot),
                                &length);
        request = efr32_rf_medium_prepare_rx_delivery(
            medium, receiver, &metadata, data, length, errp);
        if (!request) {
            return offers;
        }
        g_queue_push_tail(&medium->rx_offers, request);
        offers++;
    }

    return offers;
}

unsigned efr32_rf_medium_get_rx_offer_count(const Efr32RfMedium *medium)
{
    return medium->rx_offers.length;
}

Efr32RfRxDeliveryRequest *efr32_rf_medium_take_rx_offer(
    Efr32RfMedium *medium)
{
    return g_queue_pop_head(&medium->rx_offers);
}

void efr32_rf_medium_clear_rx_offers(Efr32RfMedium *medium)
{
    Efr32RfRxDeliveryRequest *request;

    while ((request = efr32_rf_medium_take_rx_offer(medium))) {
        efr32_rf_rx_delivery_request_free(request);
    }
}

void efr32_rf_medium_reset_radio(Efr32RfMedium *medium,
                                 Efr32RfRadio *radio)
{
    bool reset_tx_source = medium->latest_tx_snapshot &&
        efr32_rf_tx_snapshot_get_radio(medium->latest_tx_snapshot) == radio;
    GList *link;

    if (reset_tx_source) {
        efr32_rf_medium_clear_latest_tx_snapshot(medium);
        efr32_rf_medium_clear_rx_offers(medium);
        return;
    }

    for (link = medium->rx_offers.head; link;) {
        Efr32RfRxDeliveryRequest *request = link->data;
        GList *next = link->next;

        if (efr32_rf_rx_delivery_request_get_receiver(request) == radio) {
            g_queue_delete_link(&medium->rx_offers, link);
            efr32_rf_rx_delivery_request_free(request);
        }
        link = next;
    }
}

static void efr32_rf_medium_init(Object *obj)
{
    Efr32RfMedium *medium = EFR32_RF_MEDIUM(obj);

    g_queue_init(&medium->rx_offers);
}

static void efr32_rf_medium_finalize(Object *obj)
{
    Efr32RfMedium *medium = EFR32_RF_MEDIUM(obj);

    efr32_rf_medium_clear_rx_offers(medium);
    efr32_rf_medium_clear_latest_tx_snapshot(medium);
    g_list_free(medium->radios);
}

static const TypeInfo efr32_rf_medium_info = {
    .name = TYPE_EFR32_RF_MEDIUM,
    .parent = TYPE_OBJECT,
    .instance_size = sizeof(Efr32RfMedium),
    .instance_init = efr32_rf_medium_init,
    .instance_finalize = efr32_rf_medium_finalize,
};

static const TypeInfo efr32_rf_radio_info = {
    .name = TYPE_EFR32_RF_RADIO,
    .parent = TYPE_INTERFACE,
};

void efr32_rf_medium_register_types(void)
{
    type_register_static(&efr32_rf_medium_info);
    type_register_static(&efr32_rf_radio_info);
}
