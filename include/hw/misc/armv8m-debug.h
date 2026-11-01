/*
 * Armv8-M halting debug unit: DCB (DHCSR, DCRSR, DCRDR, DEMCR, DSCSR,
 * DAUTHSTATUS), FPB, DWT and the PPB ROM table of one Cortex-M33.
 *
 * Copyright (c) 2026 Gavin John
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ARMV8M_DEBUG_H
#define HW_MISC_ARMV8M_DEBUG_H

#include "hw/sysbus.h"
#include "hw/clock.h"
#include "qom/object.h"

#define TYPE_ARMV8M_DEBUG "armv8m-debug"
OBJECT_DECLARE_SIMPLE_TYPE(ARMv8MDebugState, ARMV8M_DEBUG)

/* Architected addresses of the regions the board maps (Armv8-M ARM D1.1). */
#define ARMV8M_DEBUG_ITM_BASE 0xe0000000u
#define ARMV8M_DEBUG_DWT_BASE 0xe0001000u
#define ARMV8M_DEBUG_FPB_BASE 0xe0002000u
#define ARMV8M_DEBUG_DCB_BASE 0xe000edf0u
#define ARMV8M_DEBUG_DSCSR_BASE 0xe000ee08u
#define ARMV8M_DEBUG_SCS_ID_BASE 0xe000efb8u
#define ARMV8M_DEBUG_ROM_BASE 0xe00ff000u

/* sysbus MMIO region indices, in the order the board maps them. */
enum {
    ARMV8M_DEBUG_MMIO_DCB,
    ARMV8M_DEBUG_MMIO_DSCSR,
    ARMV8M_DEBUG_MMIO_SCS_ID,
    ARMV8M_DEBUG_MMIO_DWT,
    ARMV8M_DEBUG_MMIO_FPB,
    ARMV8M_DEBUG_MMIO_ROM,
    ARMV8M_DEBUG_MMIO_COUNT,
};

#define ARMV8M_DEBUG_MAX_FPB_COMPARATORS 8
#define ARMV8M_DEBUG_MAX_DWT_COMPARATORS 4

struct ARMv8MDebugState {
    SysBusDevice parent_obj;

    MemoryRegion mmio[ARMV8M_DEBUG_MMIO_COUNT];
    Clock *cpuclk;

    /* Properties */
    Object *cpu_obj;
    uint32_t num_fpb_code;
    uint32_t num_dwt;
    bool vc_corereset_honoured;
    bool secure_debug_enabled;

    /* DCB */
    uint32_t dhcsr_ctrl;   /* C_* control bits as last written */
    bool s_reset_st;       /* sticky, cleared by a DHCSR read */
    bool s_retire_st;      /* sticky, cleared by a DHCSR read */
    bool s_regrdy;
    uint32_t dcrdr;
    uint32_t demcr;
    uint32_t dscsr;
    bool stepping;
    bool halt_queued;
    /*
     * "halted" GPIO output: DHCSR.S_HALT, like the Cortex-M33 HALTED signal;
     * the SoC stops the peripherals that do not run in debug (DEBUGRUN = 0).
     */
    qemu_irq halted_out;

    /* FPB */
    bool fpb_enable;
    uint32_t fp_comp[ARMV8M_DEBUG_MAX_FPB_COMPARATORS];

    /* DWT */
    uint32_t dwt_ctrl;
    uint64_t cyccnt_base;      /* CYCCNT at cyccnt_since_ns */
    int64_t cyccnt_since_ns;   /* virtual time CYCCNT last re-based, or -1 */
    uint32_t dwt_comp[ARMV8M_DEBUG_MAX_DWT_COMPARATORS];
    uint32_t dwt_function[ARMV8M_DEBUG_MAX_DWT_COMPARATORS];
};

/* True while the core is in Debug state (DHCSR.S_HALT). */
bool armv8m_debug_halted(ARMv8MDebugState *s);

#endif
