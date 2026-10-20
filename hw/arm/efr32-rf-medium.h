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

/*
 * This interface deliberately models only membership in an in-process RF
 * medium.  It does not define an air-frame format or a host HCI transport.
 * The current EFR32 abstraction boundary is a snapshot of staged BUFC channel
 * 0 at RAC.CMD.TXEN/RAC->TX.  Receive delivery remains deferred until a
 * receiver is in RXSEARCH and the RXRAWDATA-to-BUFC channel 1 path can produce
 * a source-backed RXDONE.  The controller firmware remains responsible for
 * exposing standard HCI through its own transport when that exists.
 */

void efr32_rf_medium_attach(Efr32RfMedium *medium, Efr32RfRadio *radio,
                            Error **errp);
void efr32_rf_medium_detach(Efr32RfMedium *medium, Efr32RfRadio *radio);
bool efr32_rf_medium_has_radio(const Efr32RfMedium *medium,
                                const Efr32RfRadio *radio);

void efr32_rf_medium_register_types(void);

#endif /* HW_ARM_EFR32_RF_MEDIUM_H */
