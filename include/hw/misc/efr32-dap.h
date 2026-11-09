/*
 * EFR32 Series 2 debug access port: SWJ-DP (SW-DP), the Cortex-M33 AHB-AP
 * and the Secure Engine's Debug Challenge Interface (DCI) AP.
 *
 * Copyright (c) 2026 Gavin John
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_EFR32_DAP_H
#define HW_MISC_EFR32_DAP_H

#include "hw/sysbus.h"
#include "exec/memory.h"
#include "qom/object.h"

#define TYPE_EFR32_DAP "efr32-dap"
OBJECT_DECLARE_SIMPLE_TYPE(Efr32DapState, EFR32_DAP)

#define EFR32_DAP_NUM_APS 2

typedef struct Efr32MemAp {
    uint32_t idr;
    uint32_t base;
    uint32_t cfg;
    uint32_t csw;
    uint32_t tar;
    AddressSpace *as;
} Efr32MemAp;

typedef enum {
    EFR32_SWD_JTAG,         /* power-on mode: SWJ-DP selects JTAG */
    EFR32_SWD_RESET,        /* line reset seen, waiting for idle */
    EFR32_SWD_IDLE,
    EFR32_SWD_REQUEST,
    EFR32_SWD_TURN_TO_ACK,
    EFR32_SWD_ACK,
    EFR32_SWD_READ_DATA,
    EFR32_SWD_TURN_TO_HOST,
    EFR32_SWD_WRITE_DATA,
    EFR32_SWD_LOCKOUT,      /* protocol error: silent until line reset */
} Efr32SwdPhase;

struct Efr32DapState {
    SysBusDevice parent_obj;

    /* Properties */
    MemoryRegion *ap0_memory;   /* the Cortex-M33 bus as the AHB-AP sees it */
    MemoryRegion *flash;        /* main flash, for the DCI device erase */
    uint32_t vtref_mv;
    uint64_t sysreset_blackout_ns;
    bool dci_device_erase_enabled;

    AddressSpace ap0_as;
    AddressSpace dci_as;
    MemoryRegion dci_root;
    MemoryRegion dci_regs;
    Efr32MemAp ap[EFR32_DAP_NUM_APS];

    /* SWD wire state */
    Efr32SwdPhase phase;
    unsigned bit_index;
    unsigned ones;          /* consecutive host-driven 1s (line reset) */
    uint16_t select_shift;  /* last 16 host bits (JTAG-to-SWD select) */
    uint8_t request;
    uint8_t ack;
    uint64_t shift;         /* data phase: 32 data bits + parity */
    bool awaiting_dpidr;    /* after line reset only a DPIDR read answers */

    /* DP registers */
    uint32_t ctrl_stat;
    uint32_t select;
    uint32_t rdbuff;

    /* System reset window: AP accesses fail (docs/debug-port-model.md) */
    bool in_sysreset;
    int64_t blackout_until_ns;

    /* nRESET pin held low by the probe: the SW-DP does not answer */
    bool nreset_asserted;
    qemu_irq nreset_out;    /* to the SoC: pin reset */
    qemu_irq erase_reset;   /* to the SoC: the reset after a DCI erase */
    qemu_irq debug_powered; /* to the SoC: CTRL/STAT.CDBGPWRUPREQ */
    /* Debug port wedged by the radio's clock retiming (efr32-dap.c) */
    bool wedged;

    /* DCI mailbox */
    uint32_t dci_cmd[4];
    unsigned dci_cmd_words;
    uint32_t dci_resp[8];
    unsigned dci_resp_len;
    unsigned dci_resp_pos;
};

#endif
