/*
 * EFR32xG22 LETIMER0, low energy timer (RM rev. 1.2 chapter 18).
 *
 * Lines: "out" (outputs 0/1, to the GPIO LETIMER route), "prs" (the PRS
 * producers LETIMER0 CH0/CH1), "prs-in" (PRS consumers START, STOP, CLEAR).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_ARM_EFR32_LETIMER_H
#define HW_ARM_EFR32_LETIMER_H

#include "hw/irq.h"
#include "hw/arm/efr32-soc-link.h"

#define TYPE_EFR32_LETIMER "efr32-letimer"
#define EFR32_LETIMER0_BASE 0x4a000000u
#define EFR32_LETIMER0_IRQ 19
#define EFR32_LETIMER_OUT "out"
#define EFR32_LETIMER_PRS "prs"
#define EFR32_LETIMER_PRS_IN "prs-in"

DeviceState *efr32_letimer_create(Object *soc, MemoryRegion *memory,
                                  const Efr32SocLink *link, hwaddr base,
                                  hwaddr ns_offset, qemu_irq irq);

#endif
