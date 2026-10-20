/*
 * Unit tests for the EFR32 in-process RF medium contract.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/arm/efr32-rf-medium.h"

#define TYPE_TEST_EFR32_RF_RADIO "test-efr32-rf-radio"

typedef struct TestEfr32RfRadio {
    Object parent_obj;
    bool rx_ready;
    unsigned rx_offer_notifications;
} TestEfr32RfRadio;

static bool test_efr32_rf_radio_is_rx_ready(Efr32RfRadio *radio)
{
    return ((TestEfr32RfRadio *)radio)->rx_ready;
}

static void test_efr32_rf_radio_rx_offer_available(Efr32RfRadio *radio)
{
    ((TestEfr32RfRadio *)radio)->rx_offer_notifications++;
}

static void test_efr32_rf_radio_class_init(ObjectClass *oc, void *data)
{
    Efr32RfRadioClass *radio_class = EFR32_RF_RADIO_CLASS(oc);

    radio_class->is_rx_ready = test_efr32_rf_radio_is_rx_ready;
    radio_class->rx_offer_available = test_efr32_rf_radio_rx_offer_available;
}

static const TypeInfo test_efr32_rf_radio_info = {
    .name = TYPE_TEST_EFR32_RF_RADIO,
    .parent = TYPE_OBJECT,
    .instance_size = sizeof(TestEfr32RfRadio),
    .class_init = test_efr32_rf_radio_class_init,
    .interfaces = (InterfaceInfo[]){{TYPE_EFR32_RF_RADIO}, {}},
};

typedef struct RfMediumFixture {
    Efr32RfMedium *medium;
    Efr32RfRadio *radio;
} RfMediumFixture;

static void rf_medium_fixture_setup(RfMediumFixture *fixture, gconstpointer data)
{
    fixture->medium = EFR32_RF_MEDIUM(object_new(TYPE_EFR32_RF_MEDIUM));
    fixture->radio = EFR32_RF_RADIO(object_new(TYPE_TEST_EFR32_RF_RADIO));
    ((TestEfr32RfRadio *)fixture->radio)->rx_ready = true;
    efr32_rf_medium_attach(fixture->medium, fixture->radio, &error_abort);
}

static void rf_medium_fixture_teardown(RfMediumFixture *fixture,
                                       gconstpointer data)
{
    efr32_rf_medium_detach(fixture->medium, fixture->radio);
    object_unref(OBJECT(fixture->radio));
    object_unref(OBJECT(fixture->medium));
}

static Efr32RfTxMetadata valid_tx_metadata(void)
{
    return (Efr32RfTxMetadata){
        .trigger = EFR32_RF_TX_TRIGGER_RAC_CMD_TXEN,
        .buffer_channel = EFR32_RF_TX_BUFC_CHANNEL,
    };
}

static Efr32RfRxMetadata valid_rx_metadata(void)
{
    return (Efr32RfRxMetadata){
        .buffer_channel = EFR32_RF_RX_BUFC_CHANNEL,
    };
}

static void test_snapshot_copies_data(RfMediumFixture *fixture,
                                      gconstpointer data)
{
    uint8_t staged[] = {0x11, 0x22, 0x33};
    Efr32RfTxMetadata metadata = valid_tx_metadata();
    Efr32RfTxSnapshot *snapshot;
    gsize length;
    const uint8_t *captured;

    snapshot = efr32_rf_medium_snapshot_tx(
        fixture->medium, fixture->radio, &metadata, staged, sizeof(staged),
        &error_abort);
    staged[0] = 0xff;

    captured = g_bytes_get_data(efr32_rf_tx_snapshot_get_data(snapshot),
                                &length);
    g_assert_cmpuint(length, ==, sizeof(staged));
    g_assert_cmpuint(captured[0], ==, 0x11);
    g_assert_true(efr32_rf_tx_snapshot_get_radio(snapshot) == fixture->radio);
    g_assert_cmpint(efr32_rf_tx_snapshot_get_metadata(snapshot)->trigger, ==,
                    EFR32_RF_TX_TRIGGER_RAC_CMD_TXEN);
    g_assert_cmpuint(efr32_rf_tx_snapshot_get_metadata(snapshot)->buffer_channel,
                     ==, EFR32_RF_TX_BUFC_CHANNEL);
    efr32_rf_tx_snapshot_free(snapshot);
}

static void test_snapshot_validation(RfMediumFixture *fixture,
                                     gconstpointer data)
{
    Efr32RfTxMetadata metadata = valid_tx_metadata();
    Efr32RfTxSnapshot *snapshot;
    Error *err = NULL;

    metadata.trigger = 99;
    snapshot = efr32_rf_medium_snapshot_tx(fixture->medium, fixture->radio,
                                            &metadata, NULL, 0, &err);
    g_assert_null(snapshot);
    g_assert_nonnull(err);
    error_free(err);

    metadata = valid_tx_metadata();
    metadata.buffer_channel = 1;
    err = NULL;
    snapshot = efr32_rf_medium_snapshot_tx(fixture->medium, fixture->radio,
                                            &metadata, NULL, 0, &err);
    g_assert_null(snapshot);
    g_assert_nonnull(err);
    error_free(err);

    metadata = valid_tx_metadata();
    err = NULL;
    snapshot = efr32_rf_medium_snapshot_tx(fixture->medium, fixture->radio,
                                            &metadata, NULL, 1, &err);
    g_assert_null(snapshot);
    g_assert_nonnull(err);
    error_free(err);
}

static void test_snapshot_requires_membership(RfMediumFixture *fixture,
                                              gconstpointer data)
{
    Efr32RfRadio *other =
        EFR32_RF_RADIO(object_new(TYPE_TEST_EFR32_RF_RADIO));
    Efr32RfTxMetadata metadata = valid_tx_metadata();
    Efr32RfTxSnapshot *snapshot;
    Error *err = NULL;

    snapshot = efr32_rf_medium_snapshot_tx(fixture->medium, other, &metadata,
                                            NULL, 0, &err);
    g_assert_null(snapshot);
    g_assert_nonnull(err);
    error_free(err);
    object_unref(OBJECT(other));
}

static void test_snapshot_sequences_are_ordered(RfMediumFixture *fixture,
                                                gconstpointer data)
{
    Efr32RfTxMetadata metadata = valid_tx_metadata();
    Efr32RfTxSnapshot *first;
    Efr32RfTxSnapshot *second;

    first = efr32_rf_medium_snapshot_tx(fixture->medium, fixture->radio,
                                         &metadata, NULL, 0, &error_abort);
    second = efr32_rf_medium_snapshot_tx(fixture->medium, fixture->radio,
                                          &metadata, NULL, 0, &error_abort);
    g_assert_cmpuint(efr32_rf_tx_snapshot_get_sequence(first), ==, 0);
    g_assert_cmpuint(efr32_rf_tx_snapshot_get_sequence(second), ==, 1);
    efr32_rf_tx_snapshot_free(second);
    efr32_rf_tx_snapshot_free(first);
}

static void test_rx_delivery_request_copies_data(RfMediumFixture *fixture,
                                                  gconstpointer data)
{
    uint8_t received[] = {0xc3, 0x5a};
    Efr32RfRxMetadata metadata = valid_rx_metadata();
    Efr32RfRxDeliveryRequest *request;
    const uint8_t *captured;
    gsize length;

    request = efr32_rf_medium_prepare_rx_delivery(
        fixture->medium, fixture->radio, &metadata, received, sizeof(received),
        &error_abort);
    received[0] = 0;

    captured = g_bytes_get_data(
        efr32_rf_rx_delivery_request_get_data(request), &length);
    g_assert_cmpuint(length, ==, sizeof(received));
    g_assert_cmpuint(captured[0], ==, 0xc3);
    g_assert_true(efr32_rf_rx_delivery_request_get_receiver(request) ==
                  fixture->radio);
    g_assert_cmpuint(
        efr32_rf_rx_delivery_request_get_metadata(request)->buffer_channel, ==,
        EFR32_RF_RX_BUFC_CHANNEL);
    efr32_rf_rx_delivery_request_free(request);
}

static void test_rx_delivery_request_validation(RfMediumFixture *fixture,
                                                gconstpointer data)
{
    Efr32RfRxMetadata metadata = valid_rx_metadata();
    Efr32RfRxDeliveryRequest *request;
    Efr32RfRadio *other;
    Error *err = NULL;

    ((TestEfr32RfRadio *)fixture->radio)->rx_ready = false;
    request = efr32_rf_medium_prepare_rx_delivery(
        fixture->medium, fixture->radio, &metadata, NULL, 0, &err);
    g_assert_null(request);
    g_assert_nonnull(err);
    error_free(err);

    ((TestEfr32RfRadio *)fixture->radio)->rx_ready = true;
    metadata = valid_rx_metadata();
    metadata.buffer_channel = 0;
    err = NULL;
    request = efr32_rf_medium_prepare_rx_delivery(
        fixture->medium, fixture->radio, &metadata, NULL, 0, &err);
    g_assert_null(request);
    g_assert_nonnull(err);
    error_free(err);

    metadata = valid_rx_metadata();
    err = NULL;
    request = efr32_rf_medium_prepare_rx_delivery(
        fixture->medium, fixture->radio, &metadata, NULL, 1, &err);
    g_assert_null(request);
    g_assert_nonnull(err);
    error_free(err);

    other = EFR32_RF_RADIO(object_new(TYPE_TEST_EFR32_RF_RADIO));
    err = NULL;
    request = efr32_rf_medium_prepare_rx_delivery(
        fixture->medium, other, &metadata, NULL, 0, &err);
    g_assert_null(request);
    g_assert_nonnull(err);
    error_free(err);
    object_unref(OBJECT(other));
}

static Efr32RfRadio *add_rx_ready_radio(RfMediumFixture *fixture)
{
    Efr32RfRadio *radio =
        EFR32_RF_RADIO(object_new(TYPE_TEST_EFR32_RF_RADIO));

    ((TestEfr32RfRadio *)radio)->rx_ready = true;
    efr32_rf_medium_attach(fixture->medium, radio, &error_abort);
    return radio;
}

static void test_tx_offers_follow_rx_ready_attachment_order(
    RfMediumFixture *fixture, gconstpointer data)
{
    uint8_t staged[] = {0x31, 0x62};
    Efr32RfTxMetadata tx_metadata = valid_tx_metadata();
    Efr32RfTxSnapshot *snapshot;
    Efr32RfRadio *first_peer;
    Efr32RfRadio *second_peer;
    Efr32RfRadio *not_ready_peer;
    Efr32RfRxDeliveryRequest *first_offer;
    Efr32RfRxDeliveryRequest *second_offer;
    const uint8_t *payload;
    gsize length;

    first_peer = add_rx_ready_radio(fixture);
    not_ready_peer = EFR32_RF_RADIO(object_new(TYPE_TEST_EFR32_RF_RADIO));
    efr32_rf_medium_attach(fixture->medium, not_ready_peer, &error_abort);
    second_peer = add_rx_ready_radio(fixture);
    snapshot = efr32_rf_medium_snapshot_tx(
        fixture->medium, fixture->radio, &tx_metadata, staged, sizeof(staged),
        &error_abort);
    efr32_rf_medium_set_latest_tx_snapshot(fixture->medium, snapshot);
    staged[0] = 0;

    g_assert_cmpuint(efr32_rf_medium_offer_latest_tx(fixture->medium,
                                                       &error_abort), ==, 2);
    g_assert_cmpuint(efr32_rf_medium_get_rx_offer_count(fixture->medium), ==,
                     2);
    g_assert_cmpuint(((TestEfr32RfRadio *)fixture->radio)->rx_offer_notifications,
                     ==, 0);
    g_assert_cmpuint(((TestEfr32RfRadio *)first_peer)->rx_offer_notifications,
                     ==, 1);
    g_assert_cmpuint(((TestEfr32RfRadio *)second_peer)->rx_offer_notifications,
                     ==, 1);
    g_assert_cmpuint(
        ((TestEfr32RfRadio *)not_ready_peer)->rx_offer_notifications, ==, 0);
    first_offer = efr32_rf_medium_take_rx_offer(fixture->medium);
    second_offer = efr32_rf_medium_take_rx_offer(fixture->medium);
    g_assert_true(efr32_rf_rx_delivery_request_get_receiver(first_offer) ==
                  first_peer);
    g_assert_true(efr32_rf_rx_delivery_request_get_receiver(second_offer) ==
                  second_peer);
    payload = g_bytes_get_data(
        efr32_rf_rx_delivery_request_get_data(first_offer), &length);
    g_assert_cmpuint(length, ==, sizeof(staged));
    g_assert_cmpuint(payload[0], ==, 0x31);
    g_assert_null(efr32_rf_medium_take_rx_offer(fixture->medium));
    efr32_rf_rx_delivery_request_free(first_offer);
    efr32_rf_rx_delivery_request_free(second_offer);
    efr32_rf_medium_detach(fixture->medium, second_peer);
    efr32_rf_medium_detach(fixture->medium, not_ready_peer);
    efr32_rf_medium_detach(fixture->medium, first_peer);
    object_unref(OBJECT(second_peer));
    object_unref(OBJECT(not_ready_peer));
    object_unref(OBJECT(first_peer));
}

static void test_rx_offers_are_receiver_scoped(RfMediumFixture *fixture,
                                                gconstpointer data)
{
    uint8_t staged[] = {0x31};
    Efr32RfTxMetadata tx_metadata = valid_tx_metadata();
    Efr32RfTxSnapshot *snapshot;
    Efr32RfRadio *first_peer;
    Efr32RfRadio *second_peer;
    Efr32RfRxDeliveryRequest *request;

    first_peer = add_rx_ready_radio(fixture);
    second_peer = add_rx_ready_radio(fixture);
    snapshot = efr32_rf_medium_snapshot_tx(
        fixture->medium, fixture->radio, &tx_metadata, staged, sizeof(staged),
        &error_abort);
    efr32_rf_medium_set_latest_tx_snapshot(fixture->medium, snapshot);
    g_assert_cmpuint(efr32_rf_medium_offer_latest_tx(fixture->medium,
                                                       &error_abort), ==, 2);
    g_assert_true(efr32_rf_medium_has_rx_offer_for_radio(fixture->medium,
                                                          first_peer));
    g_assert_true(efr32_rf_medium_has_rx_offer_for_radio(fixture->medium,
                                                          second_peer));

    request = efr32_rf_medium_take_rx_offer_for_radio(fixture->medium,
                                                       second_peer);
    g_assert_nonnull(request);
    g_assert_true(efr32_rf_rx_delivery_request_get_receiver(request) ==
                  second_peer);
    efr32_rf_rx_delivery_request_free(request);
    g_assert_false(efr32_rf_medium_has_rx_offer_for_radio(fixture->medium,
                                                           second_peer));
    request = efr32_rf_medium_take_rx_offer_for_radio(fixture->medium,
                                                       first_peer);
    g_assert_nonnull(request);
    g_assert_true(efr32_rf_rx_delivery_request_get_receiver(request) ==
                  first_peer);
    efr32_rf_rx_delivery_request_free(request);
    g_assert_false(efr32_rf_medium_has_rx_offer_for_radio(fixture->medium,
                                                           first_peer));
    g_assert_null(efr32_rf_medium_take_rx_offer_for_radio(fixture->medium,
                                                           first_peer));

    efr32_rf_medium_detach(fixture->medium, second_peer);
    efr32_rf_medium_detach(fixture->medium, first_peer);
    object_unref(OBJECT(second_peer));
    object_unref(OBJECT(first_peer));
}

static void test_reset_radio_keeps_other_receiver_offer(
    RfMediumFixture *fixture, gconstpointer data)
{
    uint8_t staged[] = {0x31, 0x62};
    Efr32RfTxMetadata tx_metadata = valid_tx_metadata();
    Efr32RfTxSnapshot *snapshot;
    Efr32RfRadio *first_peer;
    Efr32RfRadio *second_peer;
    Efr32RfRxDeliveryRequest *offer;

    first_peer = add_rx_ready_radio(fixture);
    second_peer = add_rx_ready_radio(fixture);
    snapshot = efr32_rf_medium_snapshot_tx(
        fixture->medium, fixture->radio, &tx_metadata, staged, sizeof(staged),
        &error_abort);
    efr32_rf_medium_set_latest_tx_snapshot(fixture->medium, snapshot);
    g_assert_cmpuint(efr32_rf_medium_offer_latest_tx(fixture->medium,
                                                       &error_abort), ==, 2);

    efr32_rf_medium_reset_radio(fixture->medium, first_peer);
    g_assert_nonnull(efr32_rf_medium_get_latest_tx_snapshot(fixture->medium));
    g_assert_cmpuint(efr32_rf_medium_get_rx_offer_count(fixture->medium), ==,
                     1);
    offer = efr32_rf_medium_take_rx_offer(fixture->medium);
    g_assert_true(efr32_rf_rx_delivery_request_get_receiver(offer) ==
                  second_peer);
    efr32_rf_rx_delivery_request_free(offer);

    efr32_rf_medium_reset_radio(fixture->medium, fixture->radio);
    g_assert_null(efr32_rf_medium_get_latest_tx_snapshot(fixture->medium));
    g_assert_cmpuint(efr32_rf_medium_get_rx_offer_count(fixture->medium), ==,
                     0);
    efr32_rf_medium_detach(fixture->medium, second_peer);
    efr32_rf_medium_detach(fixture->medium, first_peer);
    object_unref(OBJECT(second_peer));
    object_unref(OBJECT(first_peer));
}

/*
 * A later source can replace the retained latest snapshot while an earlier
 * source still has a deferred offer.  Resetting the later source must remove
 * only its own offers: the earlier receiver-visible mailbox record belongs to
 * a different completed TXEN boundary.
 */
static void test_reset_latest_source_keeps_other_source_offer(
    RfMediumFixture *fixture, gconstpointer data)
{
    const uint8_t first_payload[] = {0x31};
    const uint8_t second_payload[] = {0x62};
    Efr32RfTxMetadata tx_metadata = valid_tx_metadata();
    Efr32RfRadio *first_receiver;
    Efr32RfRadio *later_source;
    Efr32RfTxSnapshot *snapshot;
    Efr32RfRxDeliveryRequest *offer;
    const uint8_t *payload;
    gsize length;

    first_receiver = add_rx_ready_radio(fixture);
    later_source = add_rx_ready_radio(fixture);

    snapshot = efr32_rf_medium_snapshot_tx(
        fixture->medium, fixture->radio, &tx_metadata, first_payload,
        sizeof(first_payload), &error_abort);
    efr32_rf_medium_set_latest_tx_snapshot(fixture->medium, snapshot);
    g_assert_cmpuint(efr32_rf_medium_offer_latest_tx(fixture->medium,
                                                       &error_abort), ==, 2);

    snapshot = efr32_rf_medium_snapshot_tx(
        fixture->medium, later_source, &tx_metadata, second_payload,
        sizeof(second_payload), &error_abort);
    efr32_rf_medium_set_latest_tx_snapshot(fixture->medium, snapshot);
    g_assert_cmpuint(efr32_rf_medium_offer_latest_tx(fixture->medium,
                                                       &error_abort), ==, 2);

    efr32_rf_medium_reset_radio(fixture->medium, later_source);
    g_assert_null(efr32_rf_medium_get_latest_tx_snapshot(fixture->medium));
    g_assert_cmpuint(efr32_rf_medium_get_rx_offer_count(fixture->medium), ==,
                     1);
    offer = efr32_rf_medium_take_rx_offer(fixture->medium);
    g_assert_true(efr32_rf_rx_delivery_request_get_receiver(offer) ==
                  first_receiver);
    payload = g_bytes_get_data(efr32_rf_rx_delivery_request_get_data(offer),
                               &length);
    g_assert_cmpuint(length, ==, sizeof(first_payload));
    g_assert_cmpuint(payload[0], ==, first_payload[0]);
    efr32_rf_rx_delivery_request_free(offer);

    efr32_rf_medium_detach(fixture->medium, later_source);
    efr32_rf_medium_detach(fixture->medium, first_receiver);
    object_unref(OBJECT(later_source));
    object_unref(OBJECT(first_receiver));
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    module_call_init(MODULE_INIT_QOM);
    efr32_rf_medium_register_types();
    type_register_static(&test_efr32_rf_radio_info);

    g_test_add("/efr32-rf-medium/snapshot-copies-data", RfMediumFixture,
               NULL, rf_medium_fixture_setup, test_snapshot_copies_data,
               rf_medium_fixture_teardown);
    g_test_add("/efr32-rf-medium/snapshot-validation", RfMediumFixture, NULL,
               rf_medium_fixture_setup, test_snapshot_validation,
               rf_medium_fixture_teardown);
    g_test_add("/efr32-rf-medium/snapshot-requires-membership",
               RfMediumFixture, NULL, rf_medium_fixture_setup,
               test_snapshot_requires_membership, rf_medium_fixture_teardown);
    g_test_add("/efr32-rf-medium/snapshot-sequences-are-ordered",
               RfMediumFixture, NULL, rf_medium_fixture_setup,
               test_snapshot_sequences_are_ordered, rf_medium_fixture_teardown);
    g_test_add("/efr32-rf-medium/rx-delivery-request-copies-data",
               RfMediumFixture, NULL, rf_medium_fixture_setup,
               test_rx_delivery_request_copies_data, rf_medium_fixture_teardown);
    g_test_add("/efr32-rf-medium/rx-delivery-request-validation",
               RfMediumFixture, NULL, rf_medium_fixture_setup,
               test_rx_delivery_request_validation, rf_medium_fixture_teardown);
    g_test_add("/efr32-rf-medium/tx-offers-follow-rx-ready-attachment-order",
               RfMediumFixture, NULL, rf_medium_fixture_setup,
               test_tx_offers_follow_rx_ready_attachment_order,
               rf_medium_fixture_teardown);
    g_test_add("/efr32-rf-medium/reset-radio-keeps-other-receiver-offer",
               RfMediumFixture, NULL, rf_medium_fixture_setup,
               test_reset_radio_keeps_other_receiver_offer,
               rf_medium_fixture_teardown);
    g_test_add("/efr32-rf-medium/reset-latest-source-keeps-other-source-offer",
               RfMediumFixture, NULL, rf_medium_fixture_setup,
               test_reset_latest_source_keeps_other_source_offer,
               rf_medium_fixture_teardown);
    g_test_add("/efr32-rf-medium/rx-offers-are-receiver-scoped",
               RfMediumFixture, NULL, rf_medium_fixture_setup,
               test_rx_offers_are_receiver_scoped,
               rf_medium_fixture_teardown);

    return g_test_run();
}
