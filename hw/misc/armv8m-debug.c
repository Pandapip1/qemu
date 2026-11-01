/*
 * Armv8-M halting debug unit for one Cortex-M33: the Debug Control Block
 * (DHCSR, DCRSR, DCRDR, DEMCR, DSCSR, DAUTHSTATUS), the FPB, the DWT cycle
 * counter and PC sampler, and the PPB ROM table.
 *
 * Register layouts are from the Armv8-M Architecture Reference Manual
 * (DDI0553, chapter D1 "Register specification": DHCSR, DCRSR, DCRDR,
 * DEMCR, DSCSR, DAUTHSTATUS, FP_CTRL, FP_COMPn, DWT_*) and the CoreSight
 * Architecture Specification v3.0 (component and peripheral ID registers,
 * ROM table format).  Where a value is specific to the EFR32BG22's
 * Cortex-M33 configuration it is cited to silicon evidence in owo-open-fw
 * docs/debug-port-model.md.
 *
 * The core enters and leaves Debug state through the ARMMDebugOps hooks in
 * target/arm: BKPT, FPB comparator matches (QEMU BP_CPU breakpoints) and
 * the end of a halting step (QEMU single-step) are reported here.
 *
 * Copyright (c) 2026 Gavin John
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "qemu/log.h"
#include "qemu/main-loop.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/misc/armv8m-debug.h"
#include "hw/qdev-clock.h"
#include "hw/qdev-properties.h"
#include "exec/exec-all.h"
#include "exec/tb-flush.h"
#include "hw/core/cpu.h"
#include "target/arm/cpu.h"

/* DHCSR (D1.2.38) */
#define DHCSR_DBGKEY 0xa05fu
#define DHCSR_DBGKEY_SHIFT 16
#define DHCSR_C_DEBUGEN (1u << 0)
#define DHCSR_C_HALT (1u << 1)
#define DHCSR_C_STEP (1u << 2)
#define DHCSR_C_MASKINTS (1u << 3)
#define DHCSR_C_SNAPSTALL (1u << 5)
#define DHCSR_C_PMOV (1u << 6)
#define DHCSR_C_MASK (DHCSR_C_DEBUGEN | DHCSR_C_HALT | DHCSR_C_STEP | \
                      DHCSR_C_MASKINTS | DHCSR_C_SNAPSTALL | DHCSR_C_PMOV)
#define DHCSR_S_REGRDY (1u << 16)
#define DHCSR_S_HALT (1u << 17)
#define DHCSR_S_SLEEP (1u << 18)
#define DHCSR_S_SDE (1u << 20)
#define DHCSR_S_RETIRE_ST (1u << 24)
#define DHCSR_S_RESET_ST (1u << 25)

/* DCRSR (D1.2.31) */
#define DCRSR_REGSEL_MASK 0x7fu
#define DCRSR_REGWNR (1u << 16)

/* DEMCR (D1.2.36): VC_* bits, monitor control, UMON_EN, TRCENA */
#define DEMCR_VC_CORERESET (1u << 0)
#define DEMCR_SDME (1u << 20)
#define DEMCR_TRCENA (1u << 24)
#define DEMCR_WRITABLE 0x012f0ff1u

/* DFSR (D1.2.37); the register itself is the NVIC's env->v7m.dfsr */
#define DFSR_HALTED (1u << 0)
#define DFSR_BKPT (1u << 1)
#define DFSR_VCATCH (1u << 3)

/* DSCSR (D1.2.42) */
#define DSCSR_SBRSELEN (1u << 0)
#define DSCSR_SBRSEL (1u << 1)
#define DSCSR_CDS (1u << 16)
#define DSCSR_CDSKEY (1u << 17)

/* DAUTHSTATUS (D1.2.33): each field 0b11 = implemented and enabled */
#define DAUTH_ENABLED 0x3u
#define DAUTH_DISABLED 0x2u
#define DAUTH_NSID_SHIFT 0
#define DAUTH_NSNID_SHIFT 2
#define DAUTH_SID_SHIFT 4
#define DAUTH_SNID_SHIFT 6

/* FPB version 2 (D1.2.89 FP_CTRL, D1.2.87 FP_COMPn) */
#define FP_CTRL_ENABLE (1u << 0)
#define FP_CTRL_KEY (1u << 1)
#define FP_CTRL_NUM_CODE_LO_SHIFT 4
#define FP_CTRL_NUM_CODE_HI_SHIFT 12
#define FP_CTRL_NUM_CODE_LO_BITS 4
#define FP_CTRL_REV_SHIFT 28
#define FP_CTRL_REV_V2 1u
#define FP_COMP_BE (1u << 0)
#define FP_COMP_BPADDR_MASK 0xfffffffeu
#define FP_CTRL 0x000
#define FP_REMAP 0x004
#define FP_COMP0 0x008

/* DWT (D1.2.59 onwards) */
#define DWT_CTRL 0x000
#define DWT_CYCCNT 0x004
#define DWT_PCSR 0x01c
#define DWT_COMP0 0x020
#define DWT_FUNCTION0 0x028
#define DWT_COMP_STRIDE 0x10
#define DWT_CTRL_CYCCNTENA (1u << 0)
#define DWT_CTRL_NUMCOMP_SHIFT 28
#define DWT_CTRL_RO_MASK 0xff000000u
/* PCSR reads as all ones while the core is halted (D1.2.75). */
#define DWT_PCSR_HALTED 0xffffffffu

/* CoreSight identification block, offsets within a 4 KiB component. */
#define CS_DEVARCH 0xfbc
#define CS_DEVTYPE 0xfcc
#define CS_PIDR4 0xfd0
#define CS_PIDR0 0xfe0
#define CS_PIDR1 0xfe4
#define CS_PIDR2 0xfe8
#define CS_CIDR0 0xff0
#define CS_CIDR1 0xff4
#define CS_CIDR2 0xff8
#define CS_CIDR3 0xffc
#define CS_COMPONENT_SIZE 0x1000
/* Preamble bytes and component classes (CoreSight v3.0, B2.2.1) */
#define CS_CIDR0_PREAMBLE 0x0d
#define CS_CIDR2_PREAMBLE 0x05
#define CS_CIDR3_PREAMBLE 0xb1
#define CS_CLASS_ROM_TABLE 0x1
#define CS_CLASS_CORESIGHT 0x9
#define CS_CIDR1_CLASS_SHIFT 4
/* Arm's JEP106 code: continuation 4, identity 0x3b */
#define ARM_JEP106_CONTINUATION 0x4
#define ARM_JEP106_IDENTITY 0x3b
#define CS_PIDR2_JEDEC (1u << 3)
/* DEVARCH: ARCHITECT = Arm (0x23b) at [31:21], PRESENT at bit 20 */
#define CS_DEVARCH_ARM_PRESENT 0x47700000u
#define CS_ARCHID_SCS_V8M 0x2a04u
#define CS_ARCHID_DWT 0x1a02u
#define CS_ARCHID_FPB 0x1a03u
/*
 * Part numbers as OpenOCD's CoreSight table names them (src/target/
 * arm_adi_v5.c): 0x4c9 "Cortex-M33 ROM", 0xd21 "Cortex-M33 Debug".
 * The EFR32BG22's exact PIDR/revision values were not captured (TODO.md).
 */
#define CS_PART_M33_ROM 0x4c9u
#define CS_PART_M33_DEBUG 0xd21u
/* ROM table entry: offset to the component, FORMAT (32-bit) and PRESENT */
#define ROM_ENTRY_PRESENT (1u << 0)
#define ROM_ENTRY_FORMAT_32 (1u << 1)
#define ROM_MEMTYPE 0xfcc
#define ROM_MEMTYPE_SYSMEM (1u << 0)
#define SCS_BASE 0xe000e000u

/* SCS identification window: DAUTHSTATUS at its base, then CS_* above */
#define SCS_ID_WINDOW_SIZE (CS_COMPONENT_SIZE - (ARMV8M_DEBUG_SCS_ID_BASE - \
                                                 SCS_BASE))

static ARMCPU *dbg_cpu(ARMv8MDebugState *s)
{
    return ARM_CPU(s->cpu_obj);
}

bool armv8m_debug_halted(ARMv8MDebugState *s)
{
    return dbg_cpu(s)->m_debug_halted;
}

/* --- DWT cycle counter ---------------------------------------------- */

static bool dwt_cyccnt_counting(ARMv8MDebugState *s)
{
    return (s->demcr & DEMCR_TRCENA) && (s->dwt_ctrl & DWT_CTRL_CYCCNTENA) &&
           !dbg_cpu(s)->m_debug_halted;
}

static uint32_t dwt_cyccnt(ARMv8MDebugState *s)
{
    uint64_t count = s->cyccnt_base;

    if (s->cyccnt_since_ns >= 0) {
        count += clock_ns_to_ticks(s->cpuclk,
                                   qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) -
                                   s->cyccnt_since_ns);
    }
    return count;
}

/* Re-base CYCCNT whenever anything that gates it changes. */
static void dwt_cyccnt_rebase(ARMv8MDebugState *s, uint32_t value)
{
    s->cyccnt_base = value;
    s->cyccnt_since_ns = dwt_cyccnt_counting(s) ?
        qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) : -1;
}

/* --- Debug state ------------------------------------------------------ */

/* FPB comparators as BP_CPU breakpoints; runs on the vCPU or while paused */
static void fpb_sync(ARMv8MDebugState *s)
{
    CPUState *cs = CPU(dbg_cpu(s));

    cpu_breakpoint_remove_all(cs, BP_CPU);
    if (s->fpb_enable) {
        for (unsigned n = 0; n < s->num_fpb_code; n++) {
            if (s->fp_comp[n] & FP_COMP_BE) {
                cpu_breakpoint_insert(cs, s->fp_comp[n] & FP_COMP_BPADDR_MASK,
                                      BP_CPU, NULL);
            }
        }
    }
    tb_flush(cs);
}

static void fpb_sync_work(CPUState *cs, run_on_cpu_data data)
{
    fpb_sync(data.host_ptr);
}

static void fpb_changed(ARMv8MDebugState *s)
{
    async_run_on_cpu(CPU(dbg_cpu(s)), fpb_sync_work, RUN_ON_CPU_HOST_PTR(s));
}

static bool fpb_matches(ARMv8MDebugState *s, uint32_t pc)
{
    if (!s->fpb_enable) {
        return false;
    }
    for (unsigned n = 0; n < s->num_fpb_code; n++) {
        if ((s->fp_comp[n] & FP_COMP_BE) &&
            (s->fp_comp[n] & FP_COMP_BPADDR_MASK) == (pc & ~1u)) {
            return true;
        }
    }
    return false;
}

/* Enter Debug state (B13.2.1): the core stops and reports why in DFSR. */
static void dbg_enter_debug_state(ARMv8MDebugState *s, uint32_t dfsr)
{
    ARMCPU *cpu = dbg_cpu(s);
    CPUState *cs = CPU(cpu);
    uint32_t cyccnt = dwt_cyccnt(s);

    if (s->stepping) {
        s->stepping = false;
        cpu_single_step(cs, 0);
    }
    cpu->m_debug_halted = true;
    cs->halted = 1;
    qemu_set_irq(s->halted_out, 1);
    s->dhcsr_ctrl |= DHCSR_C_HALT;
    s->halt_queued = false;
    s->s_regrdy = true;
    cpu->env.v7m.dfsr |= dfsr;
    dwt_cyccnt_rebase(s, cyccnt);
}

static void dbg_leave_debug_state(ARMv8MDebugState *s, bool step)
{
    ARMCPU *cpu = dbg_cpu(s);
    CPUState *cs = CPU(cpu);
    uint32_t cyccnt = dwt_cyccnt(s);

    if (step) {
        s->stepping = true;
        /* C_MASKINTS: no interrupt is taken during the step (C1.5.2) */
        cpu_single_step(cs, SSTEP_ENABLE |
                        ((s->dhcsr_ctrl & DHCSR_C_MASKINTS) ? SSTEP_NOIRQ :
                         0));
    }
    s->s_regrdy = false;
    s->s_retire_st = true;
    cpu->m_debug_halted = false;
    cs->halted = 0;
    qemu_set_irq(s->halted_out, 0);
    dwt_cyccnt_rebase(s, cyccnt);
    qemu_cpu_kick(cs);
}

static void dbg_halt_work(CPUState *cs, run_on_cpu_data data)
{
    ARMv8MDebugState *s = data.host_ptr;

    if (s->halt_queued && (s->dhcsr_ctrl & DHCSR_C_DEBUGEN) &&
        (s->dhcsr_ctrl & DHCSR_C_HALT) && !dbg_cpu(s)->m_debug_halted) {
        dbg_enter_debug_state(s, DFSR_HALTED);
    }
    s->halt_queued = false;
}

/* ARMMDebugOps, called by target/arm */

static bool dbg_op_bkpt(void *opaque)
{
    ARMv8MDebugState *s = opaque;

    if (!(s->dhcsr_ctrl & DHCSR_C_DEBUGEN)) {
        return false;
    }
    dbg_enter_debug_state(s, DFSR_BKPT);
    cpu_exit(CPU(dbg_cpu(s)));
    return true;
}

static bool dbg_op_check_breakpoint(void *opaque)
{
    ARMv8MDebugState *s = opaque;

    return (s->dhcsr_ctrl & DHCSR_C_DEBUGEN) &&
           fpb_matches(s, dbg_cpu(s)->env.regs[15]);
}

static void dbg_op_debug_event(void *opaque)
{
    ARMv8MDebugState *s = opaque;
    CPUState *cs = CPU(dbg_cpu(s));
    bool locked = bql_locked();
    uint32_t dfsr;

    if (!locked) {
        bql_lock();
    }
    if (s->stepping) {
        dfsr = DFSR_HALTED;
    } else if ((s->dhcsr_ctrl & DHCSR_C_DEBUGEN) &&
               fpb_matches(s, dbg_cpu(s)->env.regs[15])) {
        dfsr = DFSR_BKPT;
    } else {
        if (!locked) {
            bql_unlock();
        }
        return;
    }
    dbg_enter_debug_state(s, dfsr);
    if (!locked) {
        bql_unlock();
    }
    cs->exception_index = EXCP_HLT;
    cpu_loop_exit(cs);
}

static void dbg_op_reset(void *opaque)
{
    ARMv8MDebugState *s = opaque;
    CPUState *cs = CPU(dbg_cpu(s));

    /*
     * A Warm reset leaves the debug unit alone except for the run control
     * request bits.  Silicon evidence (docs/debug-port-model.md): FPB
     * comparators survive SYSRESETREQ and catch the next match; a core that
     * was halted with C_HALT before SYSRESETREQ is not halted by it.
     */
    s->s_reset_st = true;
    s->halt_queued = false;
    s->dhcsr_ctrl &= ~(DHCSR_C_HALT | DHCSR_C_STEP);
    if (s->stepping) {
        s->stepping = false;
        cpu_single_step(cs, 0);
    }
    fpb_sync(s);
    dwt_cyccnt_rebase(s, dwt_cyccnt(s));
    /*
     * Reset vector catch.  On the EFR32BG22 `reset halt` (DEMCR.VC_CORERESET
     * plus SYSRESETREQ) never leaves the core halted; vc-corereset-honoured
     * is false for this part (docs/debug-port-model.md).
     */
    if (s->vc_corereset_honoured && (s->demcr & DEMCR_VC_CORERESET) &&
        (s->dhcsr_ctrl & DHCSR_C_DEBUGEN)) {
        dbg_enter_debug_state(s, DFSR_VCATCH);
    }
}

static const ARMMDebugOps dbg_ops = {
    .bkpt = dbg_op_bkpt,
    .check_breakpoint = dbg_op_check_breakpoint,
    .debug_event = dbg_op_debug_event,
    .reset = dbg_op_reset,
};

/* --- DCB ------------------------------------------------------------ */

static uint32_t dbg_dhcsr_read(ARMv8MDebugState *s)
{
    ARMCPU *cpu = dbg_cpu(s);
    CPUState *cs = CPU(cpu);
    bool halted = cpu->m_debug_halted;
    uint32_t value = s->dhcsr_ctrl;

    if (halted) {
        value |= DHCSR_S_HALT;
    } else if (cs->halted) {
        value |= DHCSR_S_SLEEP;
    }
    if (halted && s->s_regrdy) {
        value |= DHCSR_S_REGRDY;
    }
    if (s->secure_debug_enabled) {
        value |= DHCSR_S_SDE;
    }
    /* Running and not sleeping: instructions retire continuously */
    if (s->s_retire_st || (!halted && !cs->halted)) {
        value |= DHCSR_S_RETIRE_ST;
    }
    if (s->s_reset_st) {
        value |= DHCSR_S_RESET_ST;
    }
    s->s_retire_st = false;
    s->s_reset_st = false;
    return value;
}

static void dbg_dhcsr_write(ARMv8MDebugState *s, uint32_t value)
{
    ARMCPU *cpu = dbg_cpu(s);
    uint32_t ctrl;
    bool halted = cpu->m_debug_halted;

    if ((value >> DHCSR_DBGKEY_SHIFT) != DHCSR_DBGKEY) {
        return;
    }
    ctrl = value & DHCSR_C_MASK;
    if (!(ctrl & DHCSR_C_DEBUGEN)) {
        ctrl &= ~(DHCSR_C_HALT | DHCSR_C_STEP | DHCSR_C_MASKINTS);
    }
    s->dhcsr_ctrl = ctrl;
    if (halted && !(ctrl & DHCSR_C_HALT)) {
        dbg_leave_debug_state(s, ctrl & DHCSR_C_STEP);
    } else if (!halted && (ctrl & DHCSR_C_HALT) && !s->halt_queued) {
        s->halt_queued = true;
        async_run_on_cpu(CPU(cpu), dbg_halt_work, RUN_ON_CPU_HOST_PTR(s));
    }
}

static void dbg_dcrsr_write(ARMv8MDebugState *s, uint32_t value)
{
    ARMCPU *cpu = dbg_cpu(s);
    unsigned regsel = value & DCRSR_REGSEL_MASK;
    bool secure = (s->dscsr & DSCSR_SBRSELEN) ? !!(s->dscsr & DSCSR_SBRSEL) :
                  cpu->env.v7m.secure;
    bool ok;

    /* Transfers only happen in Debug state; S_REGRDY stays clear otherwise */
    s->s_regrdy = false;
    if (!cpu->m_debug_halted) {
        return;
    }
    if (value & DCRSR_REGWNR) {
        ok = arm_m_debug_write_reg(cpu, regsel, secure, s->dcrdr);
    } else {
        ok = arm_m_debug_read_reg(cpu, regsel, secure, &s->dcrdr);
    }
    if (!ok) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "armv8m-debug: reserved DCRSR.REGSEL 0x%x\n", regsel);
    }
    s->s_regrdy = true;
}

static uint32_t dbg_dscsr_read(ARMv8MDebugState *s)
{
    return (s->dscsr & (DSCSR_SBRSELEN | DSCSR_SBRSEL)) |
           (dbg_cpu(s)->env.v7m.secure ? DSCSR_CDS : 0);
}

static void dbg_dscsr_write(ARMv8MDebugState *s, uint32_t value)
{
    ARMCPU *cpu = dbg_cpu(s);

    s->dscsr = value & (DSCSR_SBRSELEN | DSCSR_SBRSEL);
    /* CDS is written only with CDSKEY clear, and only in Debug state */
    if (!(value & DSCSR_CDSKEY) && cpu->m_debug_halted) {
        arm_m_debug_set_secure(cpu, value & DSCSR_CDS);
    }
}

/*
 * Accesses from the PE itself (current_cpu set) never change halting debug
 * state: DHCSR, DCRSR and DSCSR writes are debugger-only (D1.2.38 notes
 * that C_DEBUGEN is not writable by software).
 */
static bool dbg_access_from_pe(void)
{
    return current_cpu != NULL;
}

static uint64_t dbg_dcb_read(void *opaque, hwaddr offset, unsigned size)
{
    ARMv8MDebugState *s = opaque;

    switch (offset) {
    case 0x0:
        return dbg_dhcsr_read(s);
    case 0x8:
        return s->dcrdr;
    case 0xc:
        return s->demcr | (s->secure_debug_enabled ? DEMCR_SDME : 0);
    default:
        return 0;
    }
}

static void dbg_dcb_write(void *opaque, hwaddr offset, uint64_t value,
                          unsigned size)
{
    ARMv8MDebugState *s = opaque;
    uint32_t cyccnt;

    switch (offset) {
    case 0x0:
        if (!dbg_access_from_pe()) {
            dbg_dhcsr_write(s, value);
        }
        break;
    case 0x4:
        if (!dbg_access_from_pe()) {
            dbg_dcrsr_write(s, value);
        }
        break;
    case 0x8:
        s->dcrdr = value;
        break;
    case 0xc:
        cyccnt = dwt_cyccnt(s);
        s->demcr = value & DEMCR_WRITABLE;
        dwt_cyccnt_rebase(s, cyccnt);
        break;
    }
}

static uint64_t dbg_dscsr_mmio_read(void *opaque, hwaddr offset,
                                    unsigned size)
{
    return dbg_dscsr_read(opaque);
}

static void dbg_dscsr_mmio_write(void *opaque, hwaddr offset, uint64_t value,
                                 unsigned size)
{
    if (!dbg_access_from_pe()) {
        dbg_dscsr_write(opaque, value);
    }
}

/* --- CoreSight identification -------------------------------------- */

static uint32_t cs_id_read(hwaddr offset, unsigned part, unsigned class,
                           uint32_t devarch)
{
    switch (offset) {
    case CS_DEVARCH:
        return devarch;
    case CS_PIDR4:
        return ARM_JEP106_CONTINUATION;
    case CS_PIDR0:
        return part & 0xff;
    case CS_PIDR1:
        return ((part >> 8) & 0xf) | ((ARM_JEP106_IDENTITY & 0xf) << 4);
    case CS_PIDR2:
        return ((ARM_JEP106_IDENTITY >> 4) & 0x7) | CS_PIDR2_JEDEC;
    case CS_CIDR0:
        return CS_CIDR0_PREAMBLE;
    case CS_CIDR1:
        return class << CS_CIDR1_CLASS_SHIFT;
    case CS_CIDR2:
        return CS_CIDR2_PREAMBLE;
    case CS_CIDR3:
        return CS_CIDR3_PREAMBLE;
    default:
        return 0;
    }
}

static uint64_t dbg_scs_id_read(void *opaque, hwaddr offset, unsigned size)
{
    ARMv8MDebugState *s = opaque;
    hwaddr reg = offset + (ARMV8M_DEBUG_SCS_ID_BASE - SCS_BASE);
    uint32_t field = s->secure_debug_enabled ? DAUTH_ENABLED : DAUTH_DISABLED;

    if (offset == 0) {
        /* DAUTHSTATUS: Secure fields follow S_SDE; Non-secure always on */
        return (DAUTH_ENABLED << DAUTH_NSID_SHIFT) |
               (DAUTH_ENABLED << DAUTH_NSNID_SHIFT) |
               (field << DAUTH_SID_SHIFT) | (field << DAUTH_SNID_SHIFT);
    }
    return cs_id_read(reg, CS_PART_M33_DEBUG, CS_CLASS_CORESIGHT,
                      CS_DEVARCH_ARM_PRESENT | CS_ARCHID_SCS_V8M);
}

static void dbg_ro_write(void *opaque, hwaddr offset, uint64_t value,
                         unsigned size)
{
}

static uint64_t dbg_rom_read(void *opaque, hwaddr offset, unsigned size)
{
    static const uint32_t components[] = {
        SCS_BASE, ARMV8M_DEBUG_DWT_BASE, ARMV8M_DEBUG_FPB_BASE,
    };
    unsigned index = offset / 4;

    if (index < ARRAY_SIZE(components)) {
        return ((components[index] - ARMV8M_DEBUG_ROM_BASE) &
                ~(CS_COMPONENT_SIZE - 1)) |
               ROM_ENTRY_FORMAT_32 | ROM_ENTRY_PRESENT;
    }
    if (offset == ROM_MEMTYPE) {
        return ROM_MEMTYPE_SYSMEM;
    }
    return cs_id_read(offset, CS_PART_M33_ROM, CS_CLASS_ROM_TABLE, 0);
}

/* --- FPB ------------------------------------------------------------ */

static uint64_t dbg_fpb_read(void *opaque, hwaddr offset, unsigned size)
{
    ARMv8MDebugState *s = opaque;
    unsigned n;

    if (offset == FP_CTRL) {
        return (s->fpb_enable ? FP_CTRL_ENABLE : 0) |
               ((s->num_fpb_code & 0xf) << FP_CTRL_NUM_CODE_LO_SHIFT) |
               ((s->num_fpb_code >> FP_CTRL_NUM_CODE_LO_BITS) <<
                FP_CTRL_NUM_CODE_HI_SHIFT) |
               (FP_CTRL_REV_V2 << FP_CTRL_REV_SHIFT);
    }
    if (offset == FP_REMAP) {
        return 0;
    }
    n = (offset - FP_COMP0) / 4;
    if (offset >= FP_COMP0 && n < s->num_fpb_code) {
        return s->fp_comp[n];
    }
    if (offset >= CS_DEVARCH) {
        return cs_id_read(offset, CS_PART_M33_DEBUG, CS_CLASS_CORESIGHT,
                          CS_DEVARCH_ARM_PRESENT | CS_ARCHID_FPB);
    }
    return 0;
}

static void dbg_fpb_write(void *opaque, hwaddr offset, uint64_t value,
                          unsigned size)
{
    ARMv8MDebugState *s = opaque;
    unsigned n;

    if (offset == FP_CTRL) {
        if (value & FP_CTRL_KEY) {
            s->fpb_enable = value & FP_CTRL_ENABLE;
            fpb_changed(s);
        }
        return;
    }
    n = (offset - FP_COMP0) / 4;
    if (offset >= FP_COMP0 && n < s->num_fpb_code) {
        s->fp_comp[n] = value & (FP_COMP_BPADDR_MASK | FP_COMP_BE);
        fpb_changed(s);
    }
}

/* --- DWT ------------------------------------------------------------ */

static uint64_t dbg_dwt_read(void *opaque, hwaddr offset, unsigned size)
{
    ARMv8MDebugState *s = opaque;
    ARMCPU *cpu = dbg_cpu(s);
    unsigned n = (offset - DWT_COMP0) / DWT_COMP_STRIDE;

    switch (offset) {
    case DWT_CTRL:
        return s->dwt_ctrl | (s->num_dwt << DWT_CTRL_NUMCOMP_SHIFT);
    case DWT_CYCCNT:
        return dwt_cyccnt(s);
    case DWT_PCSR:
        /*
         * Sampled PC (silicon: sampling works while the core runs).  The
         * model returns the PC at the last translation-block boundary.
         */
        return cpu->m_debug_halted ? DWT_PCSR_HALTED : cpu->env.regs[15];
    }
    if (offset >= DWT_COMP0 && n < s->num_dwt) {
        switch ((offset - DWT_COMP0) % DWT_COMP_STRIDE) {
        case DWT_COMP0 - DWT_COMP0:
            return s->dwt_comp[n];
        case DWT_FUNCTION0 - DWT_COMP0:
            return s->dwt_function[n];
        }
    }
    if (offset >= CS_DEVARCH) {
        return cs_id_read(offset, CS_PART_M33_DEBUG, CS_CLASS_CORESIGHT,
                          CS_DEVARCH_ARM_PRESENT | CS_ARCHID_DWT);
    }
    return 0;
}

static void dbg_dwt_write(void *opaque, hwaddr offset, uint64_t value,
                          unsigned size)
{
    ARMv8MDebugState *s = opaque;
    unsigned n = (offset - DWT_COMP0) / DWT_COMP_STRIDE;
    uint32_t cyccnt = dwt_cyccnt(s);

    switch (offset) {
    case DWT_CTRL:
        s->dwt_ctrl = value & ~DWT_CTRL_RO_MASK;
        dwt_cyccnt_rebase(s, cyccnt);
        return;
    case DWT_CYCCNT:
        dwt_cyccnt_rebase(s, value);
        return;
    }
    if (offset >= DWT_COMP0 && n < s->num_dwt) {
        switch ((offset - DWT_COMP0) % DWT_COMP_STRIDE) {
        case DWT_COMP0 - DWT_COMP0:
            s->dwt_comp[n] = value;
            break;
        case DWT_FUNCTION0 - DWT_COMP0:
            /* Stored only: watchpoint matching is not modelled (TODO.md) */
            s->dwt_function[n] = value;
            break;
        }
    }
}

#define DBG_OPS(name, rd, wr) \
    static const MemoryRegionOps name = { \
        .read = rd, \
        .write = wr, \
        .endianness = DEVICE_LITTLE_ENDIAN, \
        .valid = { .min_access_size = 4, .max_access_size = 4 }, \
        .impl = { .min_access_size = 4, .max_access_size = 4 }, \
    }

DBG_OPS(dbg_dcb_ops, dbg_dcb_read, dbg_dcb_write);
DBG_OPS(dbg_dscsr_ops, dbg_dscsr_mmio_read, dbg_dscsr_mmio_write);
DBG_OPS(dbg_scs_id_ops, dbg_scs_id_read, dbg_ro_write);
DBG_OPS(dbg_dwt_ops, dbg_dwt_read, dbg_dwt_write);
DBG_OPS(dbg_fpb_ops, dbg_fpb_read, dbg_fpb_write);
DBG_OPS(dbg_rom_ops, dbg_rom_read, dbg_ro_write);

/* DCB: DHCSR, DCRSR, DCRDR, DEMCR */
#define DCB_SIZE 0x10
#define DSCSR_SIZE 0x4

static void armv8m_debug_init(Object *obj)
{
    ARMv8MDebugState *s = ARMV8M_DEBUG(obj);
    static const struct {
        const MemoryRegionOps *ops;
        const char *name;
        uint64_t size;
    } regions[ARMV8M_DEBUG_MMIO_COUNT] = {
        [ARMV8M_DEBUG_MMIO_DCB] = { &dbg_dcb_ops, "dcb", DCB_SIZE },
        [ARMV8M_DEBUG_MMIO_DSCSR] = { &dbg_dscsr_ops, "dscsr", DSCSR_SIZE },
        [ARMV8M_DEBUG_MMIO_SCS_ID] = { &dbg_scs_id_ops, "scs-id",
                                       SCS_ID_WINDOW_SIZE },
        [ARMV8M_DEBUG_MMIO_DWT] = { &dbg_dwt_ops, "dwt", CS_COMPONENT_SIZE },
        [ARMV8M_DEBUG_MMIO_FPB] = { &dbg_fpb_ops, "fpb", CS_COMPONENT_SIZE },
        [ARMV8M_DEBUG_MMIO_ROM] = { &dbg_rom_ops, "rom-table",
                                    CS_COMPONENT_SIZE },
    };

    for (unsigned i = 0; i < ARMV8M_DEBUG_MMIO_COUNT; i++) {
        memory_region_init_io(&s->mmio[i], obj, regions[i].ops, s,
                              regions[i].name, regions[i].size);
        sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio[i]);
    }
    s->cpuclk = qdev_init_clock_in(DEVICE(obj), "cpuclk", NULL, NULL, 0);
    /* DHCSR.S_HALT as a line, for peripherals that freeze in Debug state. */
    qdev_init_gpio_out_named(DEVICE(obj), &s->halted_out, "halted", 1);
    s->cyccnt_since_ns = -1;
}

static void armv8m_debug_realize(DeviceState *dev, Error **errp)
{
    ARMv8MDebugState *s = ARMV8M_DEBUG(dev);
    ARMCPU *cpu;

    if (!s->cpu_obj) {
        error_setg(errp, "armv8m-debug: 'cpu' link not set");
        return;
    }
    if (s->num_fpb_code > ARMV8M_DEBUG_MAX_FPB_COMPARATORS ||
        s->num_dwt > ARMV8M_DEBUG_MAX_DWT_COMPARATORS) {
        error_setg(errp, "armv8m-debug: too many comparators");
        return;
    }
    cpu = dbg_cpu(s);
    cpu->m_debug_ops = &dbg_ops;
    cpu->m_debug_opaque = s;
}

static Property armv8m_debug_properties[] = {
    DEFINE_PROP_LINK("cpu", ARMv8MDebugState, cpu_obj, TYPE_ARM_CPU,
                     Object *),
    DEFINE_PROP_UINT32("num-fpb-code", ARMv8MDebugState, num_fpb_code,
                       ARMV8M_DEBUG_MAX_FPB_COMPARATORS),
    DEFINE_PROP_UINT32("num-dwt", ARMv8MDebugState, num_dwt,
                       ARMV8M_DEBUG_MAX_DWT_COMPARATORS),
    DEFINE_PROP_BOOL("vc-corereset-honoured", ARMv8MDebugState,
                     vc_corereset_honoured, true),
    DEFINE_PROP_BOOL("secure-debug-enabled", ARMv8MDebugState,
                     secure_debug_enabled, true),
    DEFINE_PROP_END_OF_LIST(),
};

static void armv8m_debug_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = armv8m_debug_realize;
    /* The debug unit is not reset by a system reset (Warm reset, B13.1) */
    dc->user_creatable = false;
    device_class_set_props(dc, armv8m_debug_properties);
}

static const TypeInfo armv8m_debug_info = {
    .name = TYPE_ARMV8M_DEBUG,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ARMv8MDebugState),
    .instance_init = armv8m_debug_init,
    .class_init = armv8m_debug_class_init,
};

static void armv8m_debug_register_types(void)
{
    type_register_static(&armv8m_debug_info);
}

type_init(armv8m_debug_register_types)
