/*
 * EFR32BG22 debug blocks (see the header).
 *
 * Configuration that is specific to this part, with the evidence for it in
 * owo-open-fw docs/debug-port-model.md:
 *  - 8 FPB code comparators and 4 DWT comparators: OpenOCD reports
 *    "target has 8 breakpoints, 4 watchpoints" for the BG22's M33.
 *  - Reset vector catch is not honoured: `reset halt` (DEMCR.VC_CORERESET
 *    and SYSRESETREQ) leaves the core running on silicon.
 *  - Secure invasive debug is enabled: DHCSR.S_SDE reads 1 on the suit.
 *
 * Copyright (c) 2026 Gavin John
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/arm/efr32bg22-debug.h"
#include "hw/irq.h"
#include "hw/misc/armv8m-debug.h"
#include "hw/misc/efr32-dap.h"
#include "hw/qdev-clock.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"

#define EFR32BG22_FPB_CODE_COMPARATORS 8
#define EFR32BG22_DWT_COMPARATORS 4
/* Above the NVIC's SCS (priority 0) and the board memory (-1) */
#define EFR32BG22_DEBUG_PRIORITY 1

static const hwaddr debug_region_base[ARMV8M_DEBUG_MMIO_COUNT] = {
    [ARMV8M_DEBUG_MMIO_DCB] = ARMV8M_DEBUG_DCB_BASE,
    [ARMV8M_DEBUG_MMIO_DSCSR] = ARMV8M_DEBUG_DSCSR_BASE,
    [ARMV8M_DEBUG_MMIO_SCS_ID] = ARMV8M_DEBUG_SCS_ID_BASE,
    [ARMV8M_DEBUG_MMIO_DWT] = ARMV8M_DEBUG_DWT_BASE,
    [ARMV8M_DEBUG_MMIO_FPB] = ARMV8M_DEBUG_FPB_BASE,
    [ARMV8M_DEBUG_MMIO_ROM] = ARMV8M_DEBUG_ROM_BASE,
};

void efr32bg22_debug_realize(Object *soc, ARMv7MState *armv7m,
                             MemoryRegion *flash, Clock *cpuclk,
                             Error **errp)
{
    DeviceState *dbg = qdev_new(TYPE_ARMV8M_DEBUG);
    DeviceState *dap = qdev_new(TYPE_EFR32_DAP);

    object_property_add_child(soc, "m33-debug", OBJECT(dbg));
    object_property_set_link(OBJECT(dbg), "cpu", OBJECT(armv7m->cpu),
                             &error_abort);
    qdev_prop_set_uint32(dbg, "num-fpb-code", EFR32BG22_FPB_CODE_COMPARATORS);
    qdev_prop_set_uint32(dbg, "num-dwt", EFR32BG22_DWT_COMPARATORS);
    qdev_prop_set_bit(dbg, "vc-corereset-honoured", false);
    qdev_prop_set_bit(dbg, "secure-debug-enabled", true);
    qdev_connect_clock_in(dbg, "cpuclk", cpuclk);
    if (!sysbus_realize_and_unref(SYS_BUS_DEVICE(dbg), errp)) {
        return;
    }
    for (unsigned i = 0; i < ARMV8M_DEBUG_MMIO_COUNT; i++) {
        memory_region_add_subregion_overlap(
            &armv7m->container, debug_region_base[i],
            sysbus_mmio_get_region(SYS_BUS_DEVICE(dbg), i),
            EFR32BG22_DEBUG_PRIORITY);
    }

    object_property_add_child(soc, "dap", OBJECT(dap));
    object_property_set_link(OBJECT(dap), "ap0-memory",
                             OBJECT(&armv7m->container), &error_abort);
    object_property_set_link(OBJECT(dap), "flash", OBJECT(flash),
                             &error_abort);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dap), errp);
}

void efr32bg22_debug_system_reset(Object *soc)
{
    Object *dap = object_resolve_path_component(soc, "dap");

    if (dap) {
        qemu_irq_raise(qdev_get_gpio_in_named(DEVICE(dap), "sysresetreq", 0));
    }
}
