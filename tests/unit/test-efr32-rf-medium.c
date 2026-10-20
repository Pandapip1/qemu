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
} TestEfr32RfRadio;

static const TypeInfo test_efr32_rf_radio_info = {
    .name = TYPE_TEST_EFR32_RF_RADIO,
    .parent = TYPE_OBJECT,
    .instance_size = sizeof(TestEfr32RfRadio),
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
        .receiver_state = EFR32_RF_RX_RECEIVER_STATE_RAC_RXSEARCH,
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
    g_assert_cmpint(
        efr32_rf_rx_delivery_request_get_metadata(request)->receiver_state, ==,
        EFR32_RF_RX_RECEIVER_STATE_RAC_RXSEARCH);
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

    metadata.receiver_state = 99;
    request = efr32_rf_medium_prepare_rx_delivery(
        fixture->medium, fixture->radio, &metadata, NULL, 0, &err);
    g_assert_null(request);
    g_assert_nonnull(err);
    error_free(err);

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

    return g_test_run();
}
