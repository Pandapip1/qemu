/*
 * EFR32xG22 BURAM, backup RAM (RM rev. 1.2 chapter 17).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_ARM_EFR32_BURAM_H
#define HW_ARM_EFR32_BURAM_H

#include "hw/arm/efr32-soc-link.h"

#define TYPE_EFR32_BURAM "efr32-buram"
#define EFR32_BURAM_BASE 0x40080000u

DeviceState *efr32_buram_create(Object *soc, MemoryRegion *memory,
                                const Efr32SocLink *link, hwaddr base,
                                hwaddr ns_offset);

#endif
