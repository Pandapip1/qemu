/*
 * EFR32 Series 2 I2C in leader (master) mode.
 *
 * The line side is a QEMU I2C bus ("i2c0"): board models, or a host I2C
 * adapter through a passthrough device, attach to it as followers.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_ARM_EFR32_I2C_H
#define HW_ARM_EFR32_I2C_H

#include "exec/memory.h"
#include "hw/clock.h"
#include "sysemu/cpu-timers.h"

#define TYPE_EFR32_I2C "efr32-i2c"
#define EFR32_I2C_BLOCK_SIZE 0x4000u

/*
 * Create I2C @index of @soc at @base (and its non-secure alias), clocked by
 * @pclk (I2C0 runs from LSPCLK = PCLK / 2, I2C1 from PCLK).
 */
DeviceState *efr32_i2c_create(Object *soc, MemoryRegion *memory,
                              DeviceState *nvic, Clock *pclk,
                              IcountBusModel *bus_model, unsigned index,
                              hwaddr base, hwaddr ns_offset, unsigned irq);

#endif
