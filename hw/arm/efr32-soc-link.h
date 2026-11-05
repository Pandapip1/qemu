/*
 * What an EFR32BG22 block outside the core peripheral model needs from the
 * SoC: its bus clock enable (CMU_CLKENn), the frequency of the clock trees
 * it runs from, and the bus-cost model its CPU accesses go through.
 *
 * RM SS4.2.1.2: an access to a peripheral whose bus clock is disabled is a
 * bus fault; on silicon reads and writes of gated TIMER, PRS, BURTC and RTCC
 * registers both fault (owo-open-fw docs/radio-tables.md SS7.71, SS7.73).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_ARM_EFR32_SOC_LINK_H
#define HW_ARM_EFR32_SOC_LINK_H

#include "exec/memory.h"
#include "sysemu/cpu-timers.h"

/*
 * MemTxAttrs.requester_id of accesses from a debug access port, for the
 * registers whose debugger reads have no side effect on silicon (TIMER
 * CCn_ICF, owo-open-fw docs/radio-tables.md SS7.71).
 */
#define EFR32_REQUESTER_DEBUG_AP 0xdb9u

/* CMU_CLKEN0 and CMU_CLKEN1 offsets (efr32bg22_cmu.h). */
#define EFR32_LINK_CLKEN0 0x064u
#define EFR32_LINK_CLKEN1 0x068u

typedef enum Efr32ClockTree {
    EFR32_TREE_HCLK,
    EFR32_TREE_PCLK,
    EFR32_TREE_EM01GRPACLK,
    EFR32_TREE_EM23GRPACLK,
    EFR32_TREE_EUSART0CLK,
} Efr32ClockTree;

typedef struct Efr32SocLink {
    void *opaque;
    /* CMU_CLKENn (@offset) bit @bit is set. */
    bool (*bus_clock)(void *opaque, uint32_t offset, uint32_t bit);
    /* The frequency of a clock tree now, 0 while it does not run. */
    uint64_t (*tree_hz)(void *opaque, Efr32ClockTree tree);
    IcountBusModel *bus_model;
    /* System address space, for bus managers (LDMA). */
    AddressSpace *dma_as;
} Efr32SocLink;

static inline bool efr32_link_bus_clock(const Efr32SocLink *link,
                                        uint32_t offset, uint32_t bit)
{
    return !link || !link->bus_clock ||
           link->bus_clock(link->opaque, offset, bit);
}

static inline uint64_t efr32_link_tree_hz(const Efr32SocLink *link,
                                          Efr32ClockTree tree)
{
    return link && link->tree_hz ? link->tree_hz(link->opaque, tree) : 0;
}

static inline void efr32_link_bus_access(const Efr32SocLink *link,
                                         hwaddr address, unsigned size,
                                         bool is_write)
{
    if (link && link->bus_model) {
        icount_bus_access(link->bus_model, current_cpu, address, size,
                          is_write);
    }
}

/* Series 2 register aliases: bits 13:12 of the offset (RM SS4.2.4.3). */
#define EFR32_ALIAS_MASK 0x3000u
#define EFR32_ALIAS_SHIFT 12

static inline uint32_t efr32_alias_apply(hwaddr addr, uint32_t old,
                                         uint32_t value)
{
    switch ((addr & EFR32_ALIAS_MASK) >> EFR32_ALIAS_SHIFT) {
    case 1:
        return old | value;
    case 2:
        return old & ~value;
    case 3:
        return old ^ value;
    default:
        return value;
    }
}

#endif
