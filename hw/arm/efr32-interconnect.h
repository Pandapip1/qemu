/*
 * EFR32xG22 bus interconnect model for the per-CPU clocks (see the .c file).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_ARM_EFR32_INTERCONNECT_H
#define HW_ARM_EFR32_INTERCONNECT_H

#include "hw/core/cpu.h"

/* Managers. */
enum {
    EFR32_IC_M33,
    EFR32_IC_SEQ,
    EFR32_IC_MANAGERS,
};

/* Subordinate ports: where accesses of different managers meet. */
enum {
    EFR32_IC_PORT_SEQRAM,
    EFR32_IC_PORT_RADIO,
    EFR32_IC_PORT_BUFC,
    EFR32_IC_PORT_SRAM,
    EFR32_IC_PORT_HFAPB,
    EFR32_IC_PORT_LFAPB,
    EFR32_IC_PORTS,
};

/* Register targets (what an access costs depends on the target). */
typedef enum Efr32IcTarget {
    EFR32_IC_SEQRAM,
    EFR32_IC_RADIO,
    EFR32_IC_BUFC,
    EFR32_IC_SRAM,
    EFR32_IC_TIMER,
    EFR32_IC_HFPERIPH,
    EFR32_IC_LFPERIPH,
    EFR32_IC_TARGETS,
} Efr32IcTarget;

#define EFR32_IC_RING 128

typedef struct Efr32IcTenure {
    int64_t start_ps, end_ps;
} Efr32IcTenure;

typedef struct Efr32IcTimeline {
    Efr32IcTenure t[EFR32_IC_RING];
    unsigned head, count;
} Efr32IcTimeline;

typedef struct Efr32Interconnect {
    CPUState *cpu[EFR32_IC_MANAGERS];
    /* Clocks, Hz: HCLK (matrix, APB bridges) and the radio subsystem's. */
    uint64_t hclk_hz, rhclk_hz;
    /* The M0+ executes (released, clocked, not sleeping). */
    bool seq_executing;
    Efr32IcTimeline tl[EFR32_IC_MANAGERS][EFR32_IC_PORTS];
    /* End of each manager's last access to the matrix / radio bus. */
    int64_t last_end_ps[EFR32_IC_MANAGERS];
    /* Start of the M0+'s current run of SEQRAM-local execution. */
    int64_t seq_local_from_ps;
} Efr32Interconnect;

/* Extra millicycles of @manager for one access to @target at the
 * manager's current time; *port gets the occupancy class. */
uint64_t efr32_ic_access(Efr32Interconnect *ic, unsigned manager,
                         Efr32IcTarget target, bool m33_code_in_sram,
                         unsigned *port);
void efr32_ic_round_end(Efr32Interconnect *ic, int64_t start_ns,
                        int64_t end_ns);
/* Exception entry of the M0+ now, millicycles, under the M33's SEQRAM use. */
uint64_t efr32_ic_seq_exception_millicycles(Efr32Interconnect *ic);

#endif
