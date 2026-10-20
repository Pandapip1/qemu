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
#define EFR32_RF_RADIO(obj) \
    INTERFACE_CHECK(Efr32RfRadio, (obj), TYPE_EFR32_RF_RADIO)

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

/*
 * This interface deliberately models only membership in an in-process RF
 * medium.  It does not define an air-frame format or a host HCI transport.
 * The intended EFR32 abstraction boundary is a snapshot of staged BUFC
 * channel 0 at RAC.CMD.TXEN/RAC->TX; no peripheral call site exists yet.
 * Receive delivery remains deferred until a
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

void efr32_rf_medium_register_types(void);

#endif /* HW_ARM_EFR32_RF_MEDIUM_H */
