/*
 * Internal RF medium contract for EFR32 radio peripherals.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_ARM_EFR32_RF_MEDIUM_H
#define HW_ARM_EFR32_RF_MEDIUM_H

#include "qapi/error.h"
#include "qom/object.h"

#define TYPE_EFR32_RF_MEDIUM "efr32-rf-medium"
OBJECT_DECLARE_SIMPLE_TYPE(Efr32RfMedium, EFR32_RF_MEDIUM)

#define TYPE_EFR32_RF_RADIO "efr32-rf-radio"

typedef struct Efr32RfRadio Efr32RfRadio;
typedef struct Efr32RfRadioClass Efr32RfRadioClass;
DECLARE_CLASS_CHECKERS(Efr32RfRadioClass, EFR32_RF_RADIO,
                       TYPE_EFR32_RF_RADIO)
#define EFR32_RF_RADIO(obj) \
    INTERFACE_CHECK(Efr32RfRadio, (obj), TYPE_EFR32_RF_RADIO)

struct Efr32RfRadioClass {
    InterfaceClass parent_class;

    bool (*is_rx_ready)(Efr32RfRadio *radio);
    void (*rx_offer_available)(Efr32RfRadio *radio);
};

bool efr32_rf_radio_is_rx_ready(Efr32RfRadio *radio);
void efr32_rf_radio_rx_offer_available(Efr32RfRadio *radio);

typedef enum Efr32RfTxTrigger {
    /* EFR32 RAC.CMD.TXEN caused the peripheral to enter RAC->TX. */
    EFR32_RF_TX_TRIGGER_RAC_CMD_TXEN,
} Efr32RfTxTrigger;

#define EFR32_RF_TX_BUFC_CHANNEL 0

/*
 * Information known at the hardware-to-medium boundary.  In particular,
 * buffer_channel identifies a BUFC staging channel; it is not an over-the-air
 * Bluetooth channel or a PHY configuration.
 */
typedef struct Efr32RfTxMetadata {
    Efr32RfTxTrigger trigger;
    uint8_t buffer_channel;
} Efr32RfTxMetadata;

typedef struct Efr32RfTxSnapshot Efr32RfTxSnapshot;

#define EFR32_RF_RX_BUFC_CHANNEL 1

/*
 * Handoff data that does not belong to receiver state. The radio interface
 * itself supplies the RXSEARCH readiness check.
 */
typedef struct Efr32RfRxMetadata {
    uint8_t buffer_channel;
} Efr32RfRxMetadata;

typedef struct Efr32RfRxDeliveryRequest Efr32RfRxDeliveryRequest;

/*
 * This interface deliberately models only membership in an in-process RF
 * medium.  It does not define an air-frame format or a host HCI transport.
 * The intended EFR32 abstraction boundary is a snapshot of staged BUFC
 * channel 0 at RAC.CMD.TXEN/RAC->TX. Receive delivery remains deferred until a
 * receiver is in RXSEARCH and the RXRAWDATA-to-BUFC channel 1 path can produce
 * a source-backed RXDONE.  The controller firmware remains responsible for
 * exposing standard HCI through its own transport when that exists.
 */

void efr32_rf_medium_attach(Efr32RfMedium *medium, Efr32RfRadio *radio,
                            Error **errp);
void efr32_rf_medium_detach(Efr32RfMedium *medium, Efr32RfRadio *radio);
bool efr32_rf_medium_has_radio(const Efr32RfMedium *medium,
                                const Efr32RfRadio *radio);

/*
 * Create an immutable, caller-owned copy of bytes supplied by a future TX
 * handoff. This does not read BUFC itself and does not transmit, schedule, or
 * queue an RF packet. @data may be NULL only when @length is zero.
 */
Efr32RfTxSnapshot *efr32_rf_medium_snapshot_tx(
    Efr32RfMedium *medium, Efr32RfRadio *radio,
    const Efr32RfTxMetadata *metadata, const uint8_t *data, size_t length,
    Error **errp);
void efr32_rf_tx_snapshot_free(Efr32RfTxSnapshot *snapshot);

/* These accessors return borrowed data valid until snapshot_free(). */
Efr32RfRadio *efr32_rf_tx_snapshot_get_radio(
    const Efr32RfTxSnapshot *snapshot);
const Efr32RfTxMetadata *efr32_rf_tx_snapshot_get_metadata(
    const Efr32RfTxSnapshot *snapshot);
uint64_t efr32_rf_tx_snapshot_get_sequence(
    const Efr32RfTxSnapshot *snapshot);
GBytes *efr32_rf_tx_snapshot_get_data(const Efr32RfTxSnapshot *snapshot);

/* The medium takes ownership and retains only its newest snapshot. */
void efr32_rf_medium_set_latest_tx_snapshot(Efr32RfMedium *medium,
                                             Efr32RfTxSnapshot *snapshot);
const Efr32RfTxSnapshot *efr32_rf_medium_get_latest_tx_snapshot(
    const Efr32RfMedium *medium);
void efr32_rf_medium_clear_latest_tx_snapshot(Efr32RfMedium *medium);

/*
 * Discard medium state owned by, or addressed to, one radio. A reset of a
 * receiver must not discard offers for other attached receivers. A reset of
 * the source of the retained TX snapshot discards that snapshot and its
 * undelivered offers.
 */
void efr32_rf_medium_reset_radio(Efr32RfMedium *medium,
                                 Efr32RfRadio *radio);

/*
 * Create an immutable, caller-owned request for a future RX handoff.  This
 * does not inject a packet, alter any peripheral register, or raise an IRQ.
 */
Efr32RfRxDeliveryRequest *efr32_rf_medium_prepare_rx_delivery(
    Efr32RfMedium *medium, Efr32RfRadio *receiver,
    const Efr32RfRxMetadata *metadata, const uint8_t *data, size_t length,
    Error **errp);
void efr32_rf_rx_delivery_request_free(Efr32RfRxDeliveryRequest *request);

/* These accessors return borrowed data valid until request_free(). */
Efr32RfRadio *efr32_rf_rx_delivery_request_get_receiver(
    const Efr32RfRxDeliveryRequest *request);
const Efr32RfRxMetadata *efr32_rf_rx_delivery_request_get_metadata(
    const Efr32RfRxDeliveryRequest *request);
GBytes *efr32_rf_rx_delivery_request_get_data(
    const Efr32RfRxDeliveryRequest *request);

/*
 * Create mailbox offers from the newest TX snapshot for RX-ready attached
 * peers, in attachment order. This is not RF delivery or a timing model.
 */
unsigned efr32_rf_medium_offer_latest_tx(Efr32RfMedium *medium,
                                         Error **errp);
unsigned efr32_rf_medium_get_rx_offer_count(const Efr32RfMedium *medium);
Efr32RfRxDeliveryRequest *efr32_rf_medium_take_rx_offer(
    Efr32RfMedium *medium);
/* Transfer the oldest offer addressed to @receiver, preserving other offers. */
Efr32RfRxDeliveryRequest *efr32_rf_medium_take_rx_offer_for_radio(
    Efr32RfMedium *medium, Efr32RfRadio *receiver);
void efr32_rf_medium_clear_rx_offers(Efr32RfMedium *medium);

void efr32_rf_medium_register_types(void);

#endif /* HW_ARM_EFR32_RF_MEDIUM_H */
