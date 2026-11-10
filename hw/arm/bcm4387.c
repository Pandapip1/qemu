/*
 * Broadcom BCM4387 (rev C2) Wi-Fi dongle ARM core, as used in Apple T6000
 * Macs. Reverse-engineering scaffold, not a faithful silicon model; see
 * docs/bcm4387-model-fidelity.md for what each piece is based on.
 *
 * It provides:
 *   - TCM laid out as the firmware and brcmfmac expect (RAM 0x740000-0x939000,
 *     ATCM also visible at 0),
 *   - the on-chip ROM window 0x400000-0x57ffff, from a dump file if one is
 *     given, else filled with "bx lr" (use contrib/plugins/romtrap to log and
 *     stub calls into it),
 *   - just enough of the AI backplane for early boot: ChipCommon chip ID and
 *     EROM pointer, a synthesised EROM, the ARM CR4 core's bank registers,
 *   - the host's pre-boot writes (firmware at rambase, NVRAM + random seed at
 *     the top of RAM, reset vector) and a watch on the shared-RAM word the
 *     firmware posts when it has booted, printing its console when it does,
 *   - a catch-all tracer: every other access is logged with the PC and
 *     reads back the last value written (0 if never written).
 *
 * Machine properties: firmware=, hostblob= (scripts/mkhostblob.py output),
 * rom= (optional ROM dump, up to 0x19b000 bytes), atcm-size=.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/bitops.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "target/arm/cpu.h"
#include "hw/core/boards.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/loader.h"
#include "hw/arm/machines-qom.h"
#include "system/address-spaces.h"
#include "system/reset.h"
#include "system/system.h"
#include "system/runstate.h"
#include "hw/arm/bcm4387.h"

#define TYPE_BCM4387_MACHINE MACHINE_TYPE_NAME("bcm4387")
OBJECT_DECLARE_SIMPLE_TYPE(BCM4387MachineState, BCM4387_MACHINE)

#define RAM_BASE        0x740000
#define RAM_SIZE        0x1f9000
#define RAM_TOP         (RAM_BASE + RAM_SIZE)

/*
 * RAM above the TCM. The firmware's ROM-patch veneers (0x750000) branch to
 * patch code in 0x939000-0xb36000, above the TCM window; the image is not
 * compressed, so the firmware is expected to populate this region itself at
 * runtime. Map it as plain RAM (separate from the TCM banks so the ramsize
 * calc at RAM 0x82c354 is unaffected) so those writes land instead of being
 * dropped by the catch-all tracer.
 */
#define SOCRAM_BASE     RAM_TOP
#define SOCRAM_SIZE     0x207000
#define ROM_BASE        0x400000
/*
 * Mask ROM 0x400000-0x59b000, as copied out in full by the open firmware at
 * reset (reads fault at 0x59b000, where the ARM default slave begins). [F]
 */
#define ROM_SIZE        0x19b000


/*
 * Firmware ROM-patch table (static image data): entries of {rom_addr,
 * veneer_addr}, 8 bytes each. The firmware programs a ROM-patch redirect unit
 * so reads of each rom_addr return the 8-byte RAM veneer; it verifies this at
 * RAM 0x82c3f0. See the pre-apply in bcm4387_init(). [F]
 */
#define PATCH_TBL       0x8a0338
#define PATCH_TBL_END   0x8a1d70

#define CC_BASE         0x18000000
#define CR4_BASE        0x18030000
#define D11_BASE(n)     (0x18031000 + (n) * 0x1000)    /* 3 slices */
#define OOBR_BASE       0x18006000
#define GCI_BASE        0x18010000
#define PMU_BASE        0x18012000
#define EROM_BASE       0x18105000
#define EROM_SIZE       0x1000

/*
 * Address 0 is its own ECC-protected TCM (not an alias of 0x740000): the
 * host writes only image word 0 there, and reading a never-written word
 * there (e.g. 0x1e0) data-aborts with DFSR 0x409 on silicon. Its size is not
 * known; model up to the ROM. The hole between the end of ROM and RAM
 * (0x59b000-0x740000) aborts the same way (seen at 0x59b000 and 0x5a0000). [F]
 */
#define LOWTCM_SIZE     ROM_BASE
#define HOLE_BASE       (ROM_BASE + ROM_SIZE)
#define HOLE_SIZE       (RAM_BASE - HOLE_BASE)

#define TCAM_SLOTS      1024
#define SRPWR           (CC_BASE + 0x1e8)

struct BCM4387MachineState {
    MachineState parent;

    char *hostblob;
    char *rom;
    char *patch;
    uint32_t atcm_size;
    bool identity;
    bool relocate;
    bool relocraw;

    ARMCPU *cpu;
    MemoryRegion atcm, btcm, socram, rom_mr, erom, trace;
    GHashTable *store;
    uint32_t cr4_bankidx;
    uint32_t bank_info[8];
    int nab, nbb;
    QEMUTimer *watch;
    uint32_t shared;
    uint16_t phy_addr[3], radio_addr[3];
    uint16_t phy_latch[3], radio_latch[3];  /* port data latches */
    int32_t phy_pending[3];     /* phyversion value for the next PHY read */
    bool phy_fresh[3];          /* other port activity since the last PHY read */
    GHashTable *phyregs;        /* values written through the ports */
    GHashTable *phyknown;       /* values known (silicon / exploration seeds) */
    Bcm4387PmuState *pmu;
    Bcm4387GciState *gci;
    Bcm4387CcState *cc;
    Bcm4387Sc *sc[3];
    Bcm4387Psm *psm[3];
    char *iq_source, *iq_format;

    bool strict;                /* unmodelled MMIO -> AXI slave error */
    MemoryRegion lowtcm, hole;
    uint8_t *lowtcm_mem;
    unsigned long *lowtcm_valid;    /* ECC: one bit per written word */
    uint32_t rstvec;                /* image word 0, written to 0 at reset */
    uint8_t *rom_pristine;
    uint32_t tcam_rom[TCAM_SLOTS];  /* programmed ROM address per slot, 0=off */
    uint32_t tcam_base, tcam_op;
    bool d11_phyflags[3];   /* left reset with PCLKE|PRST (0xc) held */
    bool ucode_loaded[3];   /* ucode memory written through objaddr/objdata */
    char *phy_live;         /* port liveness: auto, on or off */
    char *psm_dead_port;    /* PSM access to a dead port: abort or busy */
};

/* ---- catch-all tracer and modelled backplane registers ------------------ */

bool bcm4387_strict = true;
static BCM4387MachineState *the_machine;

uint32_t bcm4387_pc(void)
{
    if (!current_cpu) {
        return 0;
    }
    return cpu_env(current_cpu)->regs[15];
}

uint32_t bcm4387_lr(void)
{
    return current_cpu ? cpu_env(current_cpu)->regs[14] : 0;
}

/*
 * Memory attributes of backplane accesses. Silicon distinguishes them, the
 * model cannot: with the MPU off, the ARMv7-R default memory map makes
 * 0x00000000-0x7fffffff Normal memory, so the Cortex-R4 may merge stores in
 * its store buffer, let a load overtake an earlier buffered store, or
 * forward a load from a buffered store (ARMv7 memory model) [B: ARM
 * architecture]. Stock enables its MPU before its first backplane access
 * (RAM 0x829c94 + 0x75357a; region 0x18000000/128M and 0xe8000000/32M with
 * DRACR 0x1305 = shareable Device) [F]. Our firmware ran runs 7-37 with the
 * MPU off (SCTLR 0x40e50878) and saw a dead/latching PHY port and a PHYPLL
 * grant that depended on the binary; with stock's MPU both went away (runs
 * 38/39: port live from attach, PHYPLL granted for every image) [F]. TCG
 * performs every access in program order with no store buffer, i.e. as a
 * Device access, so the model's register behaviour is the Device one and a
 * Normal-memory MMIO run is flagged, once, as not modelled.
 */
static bool mmio_attr_flagged;

static bool pmsa_device(CPUARMState *env, ARMCPU *cpu, uint32_t a,
                        uint32_t *dracr)
{
    *dracr = 0;
    if (!(env->cp15.sctlr_el[1] & SCTLR_M)) {
        return a >= 0x80000000;         /* default map: Device / SO */
    }
    for (int n = cpu->pmsav7_dregion - 1; n >= 0; n--) {
        uint32_t drsr = env->pmsav7.drsr[n], base = env->pmsav7.drbar[n];
        uint64_t size = 2ull << extract32(drsr, 1, 5);

        if (!(drsr & 1) || a < base || a - base >= size) {
            continue;
        }
        if (size >= 256 && (drsr & BIT(8 + (a - base) / (size / 8)))) {
            continue;                   /* subregion disabled */
        }
        *dracr = env->pmsav7.dracr[n];
        /* TEX[5:3] C[1] B[0]: 000 0 x = Strongly-ordered/Device, 010 0 0 Device */
        return !(*dracr & 2) && (extract32(*dracr, 3, 3) == 0 ||
               (extract32(*dracr, 3, 3) == 2 && !(*dracr & 1)));
    }
    return a >= 0x80000000;             /* background = default map */
}

void bcm4387_mmio_attr(hwaddr addr, bool write)
{
    ARMCPU *cpu;
    uint32_t dracr;

    if (!current_cpu || mmio_attr_flagged || !bcm4387_strict) {
        return;
    }
    cpu = ARM_CPU(current_cpu);
    if (pmsa_device(&cpu->env, cpu, addr, &dracr)) {
        return;
    }
    mmio_attr_flagged = true;
    qemu_log_mask(LOG_GUEST_ERROR, "bcm4387: FIDELITY: Normal-memory MMIO "
                  "%c 0x%08" HWADDR_PRIx " (SCTLR 0x%08x, %s 0x%x): store "
                  "merging/reordering/forwarding not modelled; register "
                  "outcomes follow Device semantics pc~0x%08x\n",
                  write ? 'W' : 'R', addr,
                  (uint32_t)cpu->env.cp15.sctlr_el[1],
                  cpu->env.cp15.sctlr_el[1] & SCTLR_M ? "region DRACR" :
                  "MPU off, default map; DRACR", dracr, bcm4387_pc());
}

static uint32_t store_get(BCM4387MachineState *s, hwaddr a)
{
    return GPOINTER_TO_UINT(g_hash_table_lookup(s->store, GUINT_TO_POINTER(a)));
}

static void store_set(BCM4387MachineState *s, hwaddr a, uint32_t v)
{
    g_hash_table_insert(s->store, GUINT_TO_POINTER(a), GUINT_TO_POINTER(v));
}

#define D11_WRAP(n)     (D11_BASE(n) + 0x100000)
#define WRAP_IOCTL      0x408
#define WRAP_IOSTATUS   0x500
#define WRAP_RESETCTRL  0x800

/* save/restore power domain of each d11 slice (srpwr bit), RAM 0x8491ae [F] */
#define SLICE_DOMAINS   0x1cu
static const uint32_t d11_domain[3] = { 0x8, 0x4, 0x10 };

static uint32_t srpwr_status(BCM4387MachineState *s)
{
    return bcm4387_cc_srpwr_status(s->cc);
}

/* MACPHY_CLK resource of each slice: main, aux, scan (bcmdhd rsc_4387) [B] */
static const uint8_t d11_macphy_res[3] = {
    BCM4387_RES_MACPHY_CLK_MAIN, BCM4387_RES_MACPHY_CLK_AUX,
    BCM4387_RES_MACPHY_CLK_SCAN,
};

/*
 * A slice's PHY clock (clk_ctl_st request 0x200) asks the PMU for its
 * MACPHY_CLK resource; the clock is available while that resource is up.
 * [B] names, [F] check: the main slice's request takes res_state from
 * 0x1e4efbdf to 0x5ecefbff on silicon (run 11), the closure of MACPHY_CLK_MAIN.
 */
static bool d11_phypll(BCM4387MachineState *s, int n)
{
    return bcm4387_pmu_res_up(s->pmu, d11_macphy_res[n]);
}

/*
 * The extra wrappers of d11 slices 0/1 (master 0x18134000/0x18135000 and
 * 0x18137000/0x18138000, slave 0x18136000/0x18139000): reset_ctl is a plain
 * register (1 at handoff), everything else reads 0 and ignores writes, in
 * every power state seen. Stock leaves them at ioctl/reset_ctl/iostatus 0
 * (phylog v6 snapshot at its first PHY write); our IOCTL writes read back 0
 * (queue runs 9, 11). Slice 1's are assumed to behave like slice 0's. [F]/[I]
 */
static const hwaddr d11_extra_wraps[] = {
    0x18134000, 0x18135000, 0x18136000, 0x18137000, 0x18138000, 0x18139000,
};

/*
 * Their error-log block (+0x900..+0x93f, errlogctrl at +0x900) is plain
 * storage: stock's errlogctrl 0x330 on 0x18136900/0x18139900 reads back
 * (run 32: 14/14 wrappers). [F]
 */
static bool d11_extra_errlog(hwaddr a)
{
    return (a & 0xfff) >= 0x900 && (a & 0xfff) < 0x940;
}

static bool d11_extra_wrap(hwaddr a)
{
    for (int i = 0; i < ARRAY_SIZE(d11_extra_wraps); i++) {
        if (a >= d11_extra_wraps[i] && a < d11_extra_wraps[i] + 0x1000) {
            return true;
        }
    }
    return false;
}

static bool port_live(BCM4387MachineState *s, int n);

/*
 * The PHY window (radio port, phyversion, PHY port) answers with the PHYPLL
 * locked (queue runs 7-9); without it, AXI slave error (runs 6, 22, 30-32).
 * [F] Stock's answers at its attach read without PHYPLL, with the same
 * resources as ours (run 31), for an unknown reason [U]: the window is
 * open while the ports are assumed live (phy-live=on, see port_live).
 * strict=off leaves the window open.
 */
static bool d11_phy_ready(BCM4387MachineState *s, int n)
{
    return !s->strict || d11_phypll(s, n) || port_live(s, n);
}

/* index of the d11 core whose register space contains a, or -1 */
static int d11_core(hwaddr a)
{
    for (int n = 0; n < 3; n++) {
        if (a >= D11_BASE(n) && a < D11_BASE(n) + 0x1000) {
            return n;
        }
    }
    return -1;
}

static bool d11_fclka(BCM4387MachineState *s, int n);

static bool model_read(BCM4387MachineState *s, hwaddr a, uint32_t *v)
{
    for (int n = 0; n < 3; n++) {
        if (a == D11_WRAP(n) + WRAP_IOSTATUS) {
            /* band straps (F) | SISF_FCLKA (bit 2) while d11_fclka() */
            *v = store_get(s, a) | (d11_fclka(s, n) ? 0x4 : 0);
            return true;
        }
    }
    if (d11_extra_wrap(a)) {
        *v = (a & 0xfff) == WRAP_RESETCTRL || d11_extra_errlog(a) ?
             store_get(s, a) : 0;
        return true;
    }
    if (d11_core(a) >= 0 && (a & 0xfff) == 0x1e0) {
        /*
         * d11 clk_ctl_st: request bits [15:0] as written; ALP/HT available
         * and the backplane on HT (0xb0000); bits 24/26/28 read set once the
         * core is out of reset (0x150b0040, meaning unknown); the PHY clock
         * request (0x200) adds bits 25/27 once MACPHY_CLK is up (stock and
         * run 11: 0x1f0b0342). [F] Bit 25 is the status of
         * request bit 9 in hndsoc.h's CCS_ERSRC_REQ/STS layout [B]; bit 27
         * is unknown. [U]
         */
        *v = store_get(s, a) & 0xffff;
        *v |= 0x150b0000 |
              ((*v & 0x200) && d11_phypll(s, d11_core(a)) ? 0x0a000000 : 0);
        return true;
    }
    if (a == CR4_BASE + 0x1e0 || a == 0x18001000 + 0x1e0) {
        /*
         * clk_ctl_st of the other cores: requests as written, ALP/HT
         * available and the backplane on HT while the PMU has those
         * resources up (hndsoc.h: "common to all cores to communicate w/PMU
         * regarding clock control"). [B] Stock reads 0x000b0000 [I].
         */
        bool ht = bcm4387_pmu_res_up(s->pmu, BCM4387_RES_HT_AVAIL);

        *v = (store_get(s, a) & 0xffff) |
             (bcm4387_pmu_res_up(s->pmu, BCM4387_RES_ALP_AVAIL) ? 0x10000 : 0) |
             (ht ? 0xa0000 : 0);
        return true;
    }
    switch (a) {
    /*
     * d11 PHY version (regs +0x3e0): analog rev [15:12], PHY type [11:8]
     * (0xb), PHY rev [7:0]. Silicon decodes 0xdb32 for the d11 cores at
     * 0x18031000/0x18032000 (2-core PHYs) and 0xdb34 for 0x18033000 (1-core);
     * per-slice struct at [wlc_hw+4], d11 base at [wlc_hw+0xc8], in
     * fw/dump/ram_identity.bin. [F]
     */
    case D11_BASE(0) + 0x3e0:
    case D11_BASE(1) + 0x3e0:
    case D11_BASE(2) + 0x3e0:
        *v = d11_core(a) == 2 ? 0xdb34 : 0xdb32;
        s->phy_pending[d11_core(a)] = *v;       /* see phy_init [I] */
        return true;
    case CR4_BASE + 0x04:
        /*
         * Capabilities: nab in bits[3:0], nbb in bits[7:4] (Linux brcmfmac
         * ARMCR4_CAP layout), and the total bank count in bits[11:8], which is
         * where the on-die firmware's TCM-ramsize loop (RAM 0x82c354) reads the
         * iteration count. Without bits[11:8] that loop runs zero times and
         * ramsize comes back 0, tripping the section-bound assert at 0x82c594.
         */
        *v = s->nab | (s->nbb << 4) | ((s->nab + s->nbb) << 8);
        return true;
    case CR4_BASE + 0x40:
        *v = s->cr4_bankidx;
        return true;
    case CR4_BASE + 0x44:
        *v = s->cr4_bankidx < s->nab + s->nbb ?
             s->bank_info[s->cr4_bankidx] : 0;
        return true;
    case CR4_BASE + 0x70:
        /*
         * Patch-unit command/status: the firmware writes a command here (and an
         * operand at +0x74) and polls bit31 for completion (RAM 0x82c440).
         * Report "done" so the per-patch commit doesn't burn its full retry
         * budget (~101 polls each) and fall through to an error path.
         */
        *v = 0x80000000;
        return true;
    }
    return false;
}

static void tcam_program(BCM4387MachineState *s, uint32_t slot);

/*
 * How a d11 slice asks the PMU for its PHY clock (MACPHY_CLK resource). [I]
 *  - MAC asleep (maccontrol MCTL_WAKE clear): the clk_ctl_st PHY PLL
 *    request (0x200) asks for MACPHY_CLK directly. [F] runs 7-11: our
 *    firmware, maccontrol never written, request 0x342 -> 0x1f0b0342 and
 *    res_state 0x5ecefbff.
 *  - MAC held awake (MCTL_WAKE) and asking for the PHY PLL (clk_ctl_st
 *    0x200), out of reset with its clock on: the MAC asserts its clock
 *    request to the PMU MAC resource request (mac_res_req_timer/mask for
 *    this slice; see bcm4387-pmu.c). Out of reset alone it does not [F]
 *    run 30 (request programmed, attach done: clk_ctl_st 0x150b0040,
 *    res_state 0x1e4efbdf), nor with ucode written [F] run 31.
 *    The PMU grants it (closure of the mask over res_dep_mask). [F] run
 *    38, with Device MMIO: every image got PHYPLL (0x1f0b0342) from the
 *    one-shot |= 0x302 request with mask 0x40000000 or 0x56cac1ff. The
 *    binary-dependent grant of runs 17-37 (bfa8c1f source 4/4, later
 *    sources 0/11) was measured with the MPU off (Normal-memory MMIO, see
 *    bcm4387_mmio_attr) and is not modelled; the mac-pll knob fitted to it
 *    is gone, as are the rules refuted on the way ("granted once ucode is
 *    written", run 31; "granted after FORCEHT settled", run 32).
 *  - MAC held awake (MCTL_WAKE): the clk_ctl_st request no longer reaches
 *    the PMU, only the MAC resource request does. [F] run 16: maccontrol
 *    0x04000400 written before the 0x302 request, mac_res_req never
 *    programmed (0 since the reload) -> clk_ctl_st stuck at 0x150b0342
 *    (PHY PLL never available). Stock programs mac_res_req (timer
 *    0x15000000, mask MACPHY_CLK_MAIN closed to 0x56cac1ff) long before it
 *    sets MCTL_WAKE (bcmdhd si_pmu_set_mac_rsrc_req). [B]
 *  Aux/scan slices are in reset at stock's first PHY write, which is why
 *  their programmed MAC requests do not show in res_state (run 11). [F] check
 */
static void d11_mac_request(BCM4387MachineState *s, int n)
{
    bool awake = store_get(s, D11_BASE(n) + 0x120) & (1u << 26);
    bool up = !(store_get(s, D11_WRAP(n) + WRAP_RESETCTRL) & 1) &&
              (store_get(s, D11_WRAP(n) + WRAP_IOCTL) & 1);

    bcm4387_pmu_mac_active(s->pmu, n, up && awake &&
                           (store_get(s, D11_BASE(n) + 0x1e0) & 0x200));
    bcm4387_pmu_request(s->pmu, BCM4387_REQ_MAC0 + n,
                        !awake && (store_get(s, D11_BASE(n) + 0x1e0) & 0x200) ?
                        BIT(d11_macphy_res[n]) : 0);
}

static bool model_write(BCM4387MachineState *s, hwaddr a, uint32_t v)
{
    int n = d11_core(a);

    if (n >= 0 && (a == D11_BASE(n) + 0x1e0 || a == D11_BASE(n) + 0x120)) {
        store_set(s, a, v);
        d11_mac_request(s, n);
        if (a == D11_BASE(n) + 0x120) {
            bcm4387_psm_control(s->psm[n], v);
        }
        return true;
    }
    if (d11_extra_wrap(a)) {
        if ((a & 0xfff) == WRAP_RESETCTRL || d11_extra_errlog(a)) {
            store_set(s, a, v);
        }
        return true;
    }
    for (n = 0; n < 3; n++) {
        if (a == D11_WRAP(n) + WRAP_RESETCTRL) {
            /* note whether the core left reset with PHY clock+reset held */
            if ((store_get(s, a) & 1) && !(v & 1)) {
                s->d11_phyflags[n] =
                    (store_get(s, D11_WRAP(n) + WRAP_IOCTL) & 0xc) == 0xc;
            }
            if (!(store_get(s, a) & 1) && (v & 1)) {
                bcm4387_psm_core_reset(s->psm[n]);
                /*
                 * core reset drops the clk_ctl_st requests: 0x150b0040
                 * right after the post-PLL PHY reset (runs 38/39, MPU on)
                 * [F]
                 */
                store_set(s, D11_BASE(n) + 0x1e0, 0x40);
            }
            store_set(s, a, v);
            d11_mac_request(s, n);
            return true;
        }
        if (a == D11_WRAP(n) + WRAP_IOCTL) {
            store_set(s, a, v);
            d11_mac_request(s, n);
            return true;
        }
    }
    switch (a) {
    case CR4_BASE + 0x6c:           /* TCAM replacement base */
        s->tcam_base = v;
        store_set(s, a, v);
        return true;
    case CR4_BASE + 0x74:           /* TCAM operand: rom address | enable */
        s->tcam_op = v;
        store_set(s, a, v);
        return true;
    case CR4_BASE + 0x70:           /* TCAM command: 0x20000 | slot = write */
        store_set(s, a, v);
        if (v & 0x20000) {
            tcam_program(s, v & (TCAM_SLOTS - 1));
        }
        return true;
    case CR4_BASE + 0x40:
        s->cr4_bankidx = v & 0xff;
        return true;
    }
    return false;
}

/* ---- d11 PHY and radio register files (indirect ports) --------------------- */

/*
 * Each d11 slice has a PHY port (address +0x3fc, data +0x3fe; a 32-bit write
 * to +0x3fc is address | data << 16) and a radio port (address +0x3d8, data
 * +0x3da). Registers are kept per slice and port; writes are logged as
 * "PHYn"/"RADn" lines for the trace tooling.
 */
enum { PORT_PHY, PORT_RADIO };
#define PORT_KEY(port, n, r) GUINT_TO_POINTER((port) << 24 | (n) << 16 | (r))

static void port_seed(BCM4387MachineState *s, int port, int n, uint16_t r,
                      uint16_t v)
{
    g_hash_table_insert(s->phyknown, PORT_KEY(port, n, r), GUINT_TO_POINTER(v));
}

/*
 * Port data (queue runs 8, 9, 11): in every state our firmware has reached,
 * including stock's power, clocks and IOCTL, the PHY and radio data
 * registers behave as latches (run 11: write A, write B, read A returns B). The PHY latch resets to 0xce00, the
 * radio latch to 0x000c (what "radio ID" regs 0/1 read before any radio
 * write); any write to a data half (16-bit store to +0x3fe/+0x3da, or the
 * high half of a 32-bit store to +0x3fc/+0x3d8) loads the latch, and reads
 * return it (after the replay image's writes every PHY read gave its last
 * written 0x004b, every radio read its last written 0x0000). [F]
 *
 * In runs 7, 8 and 9 the first PHY read after a phyversion (+0x3e0) read
 * returned the phyversion value (0xdb32), while all later reads returned
 * 0xce00 with no write in between. Modelled as a one-shot: a phyversion read
 * primes the next PHY data read. [I]
 *
 * With all slice power domains on (srpwr status 0x1c, runs 10 and 11) a PHY
 * read returns the latch only as the first PHY data read after other port
 * activity (a PHY data write, or any radio data access); back-to-back PHY
 * reads return the phyversion value instead. Fits runs 10/11: liveness read
 * right after writes -> 0x0084 (latch); after the radio liveness accesses the
 * dump's first read -> 0x0084, the next 63 -> 0xdb32. With partial power
 * (runs 8/9) every read returns the latch (64 x 0x004b after the replay). [I]
 *
 * Live ports (port_live) and strict=off read back what was written,
 * seeded with the two PHY fields the stock firmware decodes on
 * silicon (PHY 0x02 bits[3:0] = 1, PHY 0x0b bits[2:0] = PHY cores 2/2/1,
 * from the per-slice struct at [wlc_hw+4] in fw/dump/ram_identity.bin). [I]
 */
/*
 * d11 maccontrol (+0x120), bits per bcmdhd 101.10.361 d11.h: MCTL_WAKE
 * (bit 26) keeps the MAC awake, MCTL_IHR_EN (bit 10) enables its internal
 * hardware registers, MCTL_PSM_RUN (bit 1) runs the microcode. [B]
 *
 * SISF_FCLKA (wrapper iostatus bit 2, "FastClkAvailable", d11.h) [B]: the
 * MAC has its fast clock. Modelled as: the slice is out of reset with its
 * clock on, HT is up, and the MAC is held awake (MCTL_WAKE) or its
 * microcode runs. [I] Fits silicon: stock (maccontrol 0x04000400) reads
 * iostatus 0x11006 at its first PHY write, our firmware (maccontrol never
 * written, same power/clocks/IOCTL) 0x11002 (queue run 11).
 */
#define MCTL_WAKE       (1u << 26)
#define MCTL_IHR_EN     (1u << 10)
#define MCTL_PSM_RUN    (1u << 1)

static bool d11_fclka(BCM4387MachineState *s, int n)
{
    uint32_t mctl = store_get(s, D11_BASE(n) + 0x120);

    return !(store_get(s, D11_WRAP(n) + WRAP_RESETCTRL) & 1) &&
           (store_get(s, D11_WRAP(n) + WRAP_IOCTL) & 1) &&
           bcm4387_pmu_res_up(s->pmu, BCM4387_RES_HT_AVAIL) &&
           (mctl & (MCTL_WAKE | MCTL_PSM_RUN));
}

/*
 * The PHY and radio data ports reach the PHY/radio register files whenever
 * the core answers at all (powered, out of reset, clocked: d11_gate), from
 * the attach on, PHY PLL or not. [F] stock at its attach read (runs 22/31:
 * IOCTL 0x145, res_state 0x1e4efbdf, maccontrol 0, PHY 0x000/0x001/0x00b
 * = 0xdb32/0x1510/0x9bd2) and ours with the MPU on (run 38 attach probe:
 * 0x00b 0x9bd2, 0x031 0x03ff, 0x000 0xdb32, 0x001 0x1510; run 39: the same
 * after every enable_phy step; radio 0/1 = 0x000c/0x50dd). Values at that
 * point are seeded (phy_init). A never-written register reads its seed or
 * 0, logged as unknown. [U]
 *
 * Runs 7-37 of ours instead saw latches and echoes (the first read after
 * a phyversion read returns 0xdb32, later 0xce00 or the last data written)
 * and slave errors at attach. They ran with the MPU off, so every MMIO
 * access was Normal memory (bcm4387_mmio_attr): store merging, load/store
 * reordering and store-to-load forwarding explain that phenomenology, and
 * the model does not reproduce it. phy-live=off still emulates it (the
 * latch rules in port_read) for reference only; it is a description of
 * Normal-memory observations, not of the hardware. phy-live=on/auto
 * (default) is the Device behaviour above. Refuted earlier rules for the
 * "trigger": ucode memory written (runs 21, 31), PHYPLL + FCLKA + IHR_EN
 * (run 31), a first-pass attach (run 32), all measured with Normal MMIO.
 *
 * Some PHY registers answer an AXI slave error when read before the PHY is
 * initialised. [F] runs 38/39 (MPU on, after enable_phy): PHY 0x12e read
 * back 0x1012 after 0x12e/0x12f were written 0x1012/0x0084, radio 0x07 read
 * 0x0010 (liveness slot 0x11a = 0x10120010), PHY 0x480 read 0xce00, and the
 * raw dump read 0x000-0x005 = 0xdb32, 0x1510, 0x0041, 0, 0x0080, 0 and
 * then took a synchronous external abort (DFSR 0x1008, DFAR 0x180313fe) on
 * PHY 0x006, in all four images. Only 0x006 is known to fault; which other
 * registers do is unknown [U] (the rest of the dump, 0x007-0x00f, 0x400-,
 * 0x480-, 0x720-, was never reached). When the fault clears (after which
 * part of stock's init) is UNKNOWN [U]: the model keeps it for the whole
 * boot (phy_preinit_fault), so firmware that reads 0x006 after its own
 * init is flagged rather than given a guessed value. Queue run 40 (our
 * capture image with the MPU, raw dump included) crashed the host and gave
 * no data; its cause is not known. [F]
 */
static bool port_live(BCM4387MachineState *s, int n)
{
    return !s->phy_live || strcmp(s->phy_live, "off");
}

/* PHY registers known to fault on read before PHY init [F]; see above */
static const uint16_t phy_preinit_faulting[] = { 0x006 };

static bool phy_preinit_fault(BCM4387MachineState *s, hwaddr a,
                              unsigned size)
{
    int n = d11_core(a);
    hwaddr off;

    if (n < 0 || !s->strict || !port_live(s, n)) {
        return false;
    }
    off = a - D11_BASE(n);
    if (off != 0x3fe && !(off == 0x3fc && size == 4)) {
        return false;
    }
    for (int i = 0; i < ARRAY_SIZE(phy_preinit_faulting); i++) {
        if (s->phy_addr[n] == phy_preinit_faulting[i]) {
            qemu_log_mask(LOG_GUEST_ERROR, "bcm4387: AXI slave error: d11 %d "
                          "PHY 0x%04x read before PHY init (runs 38/39) "
                          "pc~0x%08x\n", n, s->phy_addr[n], bcm4387_pc());
            return true;
        }
    }
    return false;
}

static void phy_init(BCM4387MachineState *s)
{
    static const uint8_t ncores[3] = { 2, 2, 1 };

    s->phyregs = g_hash_table_new(g_direct_hash, g_direct_equal);
    s->phyknown = g_hash_table_new(g_direct_hash, g_direct_equal);
    for (int n = 0; n < 3; n++) {
        s->phy_latch[n] = 0xce00;
        s->radio_latch[n] = 0x000c;
        s->phy_pending[n] = -1;
        s->phy_fresh[n] = true;
        port_seed(s, PORT_PHY, n, 0x02, 1);
        port_seed(s, PORT_PHY, n, 0x0b, ncores[n]);
    }
    /* slice 0 at attach: stock (runs 18, 31) and ours with the MPU (runs
     * 38, 39) [F] */
    port_seed(s, PORT_PHY, 0, 0x000, 0xdb32);
    port_seed(s, PORT_PHY, 0, 0x001, 0x1510);
    port_seed(s, PORT_PHY, 0, 0x002, 0x0041);
    port_seed(s, PORT_PHY, 0, 0x003, 0x0000);
    port_seed(s, PORT_PHY, 0, 0x004, 0x0080);
    port_seed(s, PORT_PHY, 0, 0x005, 0x0000);
    port_seed(s, PORT_PHY, 0, 0x009, 0xffef);   /* run 41 [F] */
    port_seed(s, PORT_PHY, 0, 0x00b, 0x9bd2);
    port_seed(s, PORT_PHY, 0, 0x00c, 0xd7f5);   /* run 41 [F] */
    port_seed(s, PORT_PHY, 0, 0x031, 0x03ff);
    port_seed(s, PORT_PHY, 0, 0x033, 0x11ff);   /* run 41 [F] */
    port_seed(s, PORT_PHY, 0, 0x480, 0xce00);
    port_seed(s, PORT_RADIO, 0, 0x000, 0x000c);
    port_seed(s, PORT_RADIO, 0, 0x001, 0x50dd);
}

static void port_put(BCM4387MachineState *s, int port, int n, uint16_t v)
{
    uint16_t a = port == PORT_PHY ? s->phy_addr[n] : s->radio_addr[n];

    /*
     * Radio 0x070 does not keep what is written: stock writes 0x0010 twice
     * (phylog 209, 258; ROM radio-list writer 0x47b310) and nothing later,
     * yet after our replay of stock's writes it reads 0 (run 42) [F]. Why
     * (strobe, self-clearing, write-only) is [U]; reads return 0.
     */
    if (port == PORT_RADIO && a == 0x070) {
        v = 0;
    }
    g_hash_table_insert(s->phyregs, PORT_KEY(port, n, a), GUINT_TO_POINTER(v));
    if (port == PORT_PHY) {
        s->phy_latch[n] = v;
    } else {
        s->radio_latch[n] = v;
    }
    s->phy_fresh[n] = true;
    qemu_log_mask(LOG_UNIMP, "bcm4387: %s%d W 0x%04x <- 0x%04x pc~0x%08x "
                  "lr~0x%08x\n", port == PORT_PHY ? "PHY" : "RAD", n, a, v,
                  bcm4387_pc(), bcm4387_lr());
}

/* A data read through a port (see phy_init for the latch behaviour). */
static uint16_t port_read(BCM4387MachineState *s, int port, int n)
{
    uint16_t a = port == PORT_PHY ? s->phy_addr[n] : s->radio_addr[n];
    gpointer key = PORT_KEY(port, n, a);
    uint16_t v;

    if (s->strict && !port_live(s, n)) {
        if (port == PORT_RADIO) {
            v = s->radio_latch[n];
            s->phy_fresh[n] = true;
        } else {
            if (s->phy_pending[n] >= 0) {
                v = s->phy_pending[n];
                s->phy_pending[n] = -1;
            } else if (!s->phy_fresh[n] &&
                       (srpwr_status(s) & SLICE_DOMAINS) == SLICE_DOMAINS) {
                v = n == 2 ? 0xdb34 : 0xdb32;
            } else {
                v = s->phy_latch[n];
            }
            s->phy_fresh[n] = false;
        }
    } else if (g_hash_table_contains(s->phyregs, key)) {
        v = GPOINTER_TO_UINT(g_hash_table_lookup(s->phyregs, key));
    } else {
        if (s->strict && !g_hash_table_contains(s->phyknown, key)) {
            qemu_log_mask(LOG_GUEST_ERROR, "bcm4387: unknown %s%d reg 0x%04x "
                          "read (reset value not known; 0) pc~0x%08x\n",
                          port == PORT_PHY ? "PHY" : "RAD", n, a, bcm4387_pc());
        }
        v = GPOINTER_TO_UINT(g_hash_table_lookup(s->phyknown, key));
    }
    qemu_log_mask(LOG_UNIMP, "bcm4387: %s%d R 0x%04x -> 0x%04x pc~0x%08x\n",
                  port == PORT_PHY ? "PHY" : "RAD", n, a, v, bcm4387_pc());
    return v;
}

/* Returns true if the access hit a PHY or radio port; *v receives read data. */
static bool phy_port(BCM4387MachineState *s, hwaddr a, unsigned size,
                     bool wr, uint32_t val, uint32_t *v)
{
    int n = d11_core(a), port;
    hwaddr off;
    uint16_t *areg;

    if (n < 0) {
        return false;
    }
    off = a - D11_BASE(n);
    if (off == 0x3fc || off == 0x3fe) {
        port = PORT_PHY;
        areg = &s->phy_addr[n];
    } else if (off == 0x3d8 || off == 0x3da) {
        port = PORT_RADIO;
        areg = &s->radio_addr[n];
    } else {
        return false;
    }
    if (wr) {
        if (off == 0x3fc || off == 0x3d8) {
            *areg = val;
            qemu_log_mask(LOG_UNIMP, "bcm4387: %s%d A 0x%04x pc~0x%08x "
                          "lr~0x%08x\n", port == PORT_PHY ? "PHY" : "RAD", n,
                          (uint16_t)val, bcm4387_pc(), bcm4387_lr());
            if (size == 4) {
                port_put(s, port, n, val >> 16);
            }
        } else {
            port_put(s, port, n, val);
        }
        return true;
    }
    if (off == 0x3fc || off == 0x3d8) {
        *v = *areg;
        if (size == 4) {
            *v |= (uint32_t)port_read(s, port, n) << 16;
        }
    } else {
        *v = port_read(s, port, n);
    }
    return true;
}

/* ---- ROM-patch (TCAM) unit ----------------------------------------------- */

/*
 * The firmware programs slots through CR4+0x74 (operand: ROM address | 1 to
 * enable, 0 to clear) and CR4+0x70 (0x20000 | slot); CR4+0x6c holds the base
 * of the 8-byte replacement words (0x750000 in stock). Reads of a patched
 * 8-byte ROM slot then return base + 8 * slot. Model it by copying those RAM
 * bytes into the ROM when the slot is programmed (restoring the pristine
 * bytes when it is cleared), so a moved table or a bad slot shows up exactly
 * as it would on silicon. Later changes to the RAM copy are not tracked. [F]
 */
static void tcam_program(BCM4387MachineState *s, uint32_t slot)
{
    uint32_t base = s->tcam_base ? s->tcam_base : 0x750000;
    uint32_t old = s->tcam_rom[slot], rom = s->tcam_op & ~7u;
    uint8_t buf[8];

    if (old) {
        address_space_write_rom(&address_space_memory, old,
                                MEMTXATTRS_UNSPECIFIED,
                                s->rom_pristine + (old - ROM_BASE), 8);
        s->tcam_rom[slot] = 0;
    }
    if (!(s->tcam_op & 1)) {
        return;
    }
    if (rom < ROM_BASE || rom + 8 > ROM_BASE + ROM_SIZE) {
        qemu_log_mask(LOG_GUEST_ERROR, "bcm4387: TCAM slot %u: ROM address "
                      "0x%08x outside the ROM, ignored\n", slot, rom);
        return;
    }
    address_space_read(&address_space_memory, base + 8 * slot,
                       MEMTXATTRS_UNSPECIFIED, buf, 8);
    address_space_write_rom(&address_space_memory, rom, MEMTXATTRS_UNSPECIFIED,
                            buf, 8);
    s->tcam_rom[slot] = rom;
}

/* ---- address 0 (ECC TCM) and the unbacked hole above the ROM -------------- */

static MemTxResult lowtcm_read(void *opaque, hwaddr addr, uint64_t *data,
                               unsigned size, MemTxAttrs attrs)
{
    BCM4387MachineState *s = opaque;

    for (hwaddr w = addr >> 2; w <= (addr + size - 1) >> 2; w++) {
        if (!test_bit(w, s->lowtcm_valid)) {
            qemu_log_mask(LOG_GUEST_ERROR, "bcm4387: ECC abort: read of "
                          "never-written TCM word 0x%08" HWADDR_PRIx
                          " pc~0x%08x\n", w << 2, bcm4387_pc());
            return MEMTX_ACCESS_ERROR;
        }
    }
    *data = ldn_le_p(s->lowtcm_mem + addr, size);
    return MEMTX_OK;
}

static MemTxResult lowtcm_write(void *opaque, hwaddr addr, uint64_t data,
                                unsigned size, MemTxAttrs attrs)
{
    BCM4387MachineState *s = opaque;

    stn_le_p(s->lowtcm_mem + addr, size, data);
    for (hwaddr w = addr >> 2; w <= (addr + size - 1) >> 2; w++) {
        set_bit(w, s->lowtcm_valid);
    }
    return MEMTX_OK;
}

static const MemoryRegionOps lowtcm_ops = {
    .read_with_attrs = lowtcm_read,
    .write_with_attrs = lowtcm_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static MemTxResult hole_read(void *opaque, hwaddr addr, uint64_t *data,
                             unsigned size, MemTxAttrs attrs)
{
    qemu_log_mask(LOG_GUEST_ERROR, "bcm4387: ECC abort: read of unbacked "
                  "0x%08" HWADDR_PRIx " pc~0x%08x\n", HOLE_BASE + addr,
                  bcm4387_pc());
    return MEMTX_ACCESS_ERROR;
}

/* writes to the hole: not observed on silicon; dropped. [I] */
static MemTxResult hole_write(void *opaque, hwaddr addr, uint64_t data,
                              unsigned size, MemTxAttrs attrs)
{
    qemu_log_mask(LOG_GUEST_ERROR, "bcm4387: write to unbacked 0x%08"
                  HWADDR_PRIx " dropped pc~0x%08x\n", HOLE_BASE + addr,
                  bcm4387_pc());
    return MEMTX_OK;
}

static const MemoryRegionOps hole_ops = {
    .read_with_attrs = hole_read,
    .write_with_attrs = hole_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

/* ---- shared device helpers ------------------------------------------------ */

int bcm4387_reg_index(const Bcm4387Reg *tab, int n, hwaddr off)
{
    for (int i = 0; i < n; i++) {
        if (tab[i].off == off) {
            return i;
        }
    }
    return -1;
}

MemTxResult bcm4387_unknown(const char *dev, hwaddr base, hwaddr off,
                            bool write, uint64_t *data, unsigned size)
{
    BCM4387MachineState *s = the_machine;
    hwaddr a = base + off;

    if (bcm4387_strict) {
        qemu_log_mask(LOG_GUEST_ERROR, "bcm4387: AXI slave error: unmodelled "
                      "%s %c%u 0x%08" HWADDR_PRIx " (%s+0x%03" HWADDR_PRIx ")"
                      " <- 0x%08" PRIx64 " pc~0x%08x\n", dev, write ? 'W' : 'R',
                      size * 8, a, dev, off, write ? *data : 0, bcm4387_pc());
        return MEMTX_ERROR;
    }
    if (write) {
        store_set(s, a, *data);
    } else {
        *data = store_get(s, a);
    }
    qemu_log_mask(LOG_UNIMP, "bcm4387: MMIO? %c%u 0x%08" HWADDR_PRIx
                  " %s 0x%08" PRIx64 " pc~0x%08x lr~0x%08x\n",
                  write ? 'W' : 'R', size * 8, a, write ? "<-" : "->", *data,
                  bcm4387_pc(), bcm4387_lr());
    return MEMTX_OK;
}

/* ---- bus rules: d11 access, strict whitelist --------------------------------- */

/*
 * Silicon behaviour of the d11 cores (fw/open queue runs 3-6, notes/50-openfw.md):
 * - power domain off (srpwr) or core in reset / clock off: AXI slave error.
 * - core left reset without PHY clock+reset held (ioctl & 0xc) and later
 *   given bandwidth/0x100 cflags: register reads get no response (stall).
 * - PHY window (radio port 0x3d8/0x3da, phyversion 0x3e0, PHY port
 *   0x3fc/0x3fe): reads get a slave error until the PHY side is clocked
 *   (d11_phy_ready). [F]/[I] Writes there do not abort: in runs 22 and 30
 *   the attach probe's address write to +0x3fc went through and the data
 *   read of +0x3fe took the abort (DFSR 0x1008, DFAR 0x180313fe). [F]
 *   Whether such a write reaches the port is unknown; it is dropped. [U]
 */
static MemTxResult d11_gate(BCM4387MachineState *s, hwaddr a, bool wr,
                            bool *dropped)
{
    int n = d11_core(a);
    uint32_t ioctl;

    if (n < 0) {
        return MEMTX_OK;
    }
    ioctl = store_get(s, D11_WRAP(n) + WRAP_IOCTL);
    if (!(srpwr_status(s) & d11_domain[n])) {
        qemu_log_mask(LOG_GUEST_ERROR, "bcm4387: AXI slave error: d11 %d "
                      "power domain off, 0x%08" HWADDR_PRIx " pc~0x%08x\n",
                      n, a, bcm4387_pc());
        return MEMTX_ERROR;
    }
    if ((store_get(s, D11_WRAP(n) + WRAP_RESETCTRL) & 1) || !(ioctl & 1)) {
        qemu_log_mask(LOG_GUEST_ERROR, "bcm4387: AXI slave error: d11 %d in "
                      "reset or unclocked, 0x%08" HWADDR_PRIx " pc~0x%08x\n",
                      n, a, bcm4387_pc());
        return MEMTX_ERROR;
    }
    if (!s->d11_phyflags[n] && (ioctl & 0x1c0)) {
        qemu_log_mask(LOG_GUEST_ERROR, "bcm4387: STALL: d11 %d left reset "
                      "without PHY flags, 0x%08" HWADDR_PRIx " pc~0x%08x\n",
                      n, a, bcm4387_pc());
        qemu_system_vmstop_request_prepare();
        qemu_system_vmstop_request(RUN_STATE_PAUSED);
        cpu_exit(current_cpu);
        return MEMTX_OK;
    }
    if (a - D11_BASE(n) >= 0x3d8 && a - D11_BASE(n) < 0x400 &&
        !d11_phy_ready(s, n) && wr) {
        qemu_log_mask(LOG_GUEST_ERROR, "bcm4387: d11 %d PHY window write "
                      "without PHYPLL dropped, 0x%08" HWADDR_PRIx " pc~0x%08x\n",
                      n, a, bcm4387_pc());
        *dropped = true;
        return MEMTX_OK;
    }
    if (a - D11_BASE(n) >= 0x3d8 && a - D11_BASE(n) < 0x400 &&
        !d11_phy_ready(s, n)) {
        qemu_log_mask(LOG_GUEST_ERROR, "bcm4387: AXI slave error: d11 %d PHY "
                      "window without PHYPLL, 0x%08" HWADDR_PRIx " pc~0x%08x\n",
                      n, a, bcm4387_pc());
        return MEMTX_ERROR;
    }
    return MEMTX_OK;
}

/*
 * Registers the model knows (modelled or plain storage with known
 * semantics). In strict mode every other access is an AXI slave error, so
 * firmware that touches something we have not characterised fails visibly.
 */
/*
 * Plain registers of the cores without a device of their own: storage only,
 * effect unknown. Names: brcmsmac d11regs_t / bcmdhd sbchipc.h / brcmfmac
 * pcie.c where given. Each is a gap (docs/bcm4387-model-fidelity.md). [U]
 */
static const struct { hwaddr a; const char *name; } plain_regs[] = {
    { CR4_BASE + 0x000, "cr4 corecontrol" },
    { CR4_BASE + 0x014, "cr4 0x014 (read)" },
    { CR4_BASE + 0x018, "cr4 0x018 (interrupt mask?)" },
    { CR4_BASE + 0x01c, "cr4 0x01c" },
    { CR4_BASE + 0x020, "cr4 0x020 (timer intstatus? stock writes 1, ROM 0x414a64)" },
    { CR4_BASE + 0x024, "cr4 0x024 (read)" },
    { CR4_BASE + 0x028, "cr4 0x028" },
    { CR4_BASE + 0x02c, "cr4 0x02c (timer count? ROM 0x414a66)" },
    { CR4_BASE + 0x1e0, "cr4 clk_ctl_st (requests; avail bits modelled)" },
    { 0x180011e0, "pcie clk_ctl_st (requests; avail bits modelled)" },
    { CR4_BASE + 0x050, "cr4 0x050" },
    { CR4_BASE + 0x098, "cr4 0x098 (ASLR code, 0x400000)" },
    { CR4_BASE + 0x1e8, "cr4 powerctl" },
    { 0x18013004, "sr0 sr_control0" }, { 0x18013008, "sr0 sr_control1" },
    { 0x18014004, "sr1 sr_control0" }, { 0x18014008, "sr1 sr_control1" },
    { 0x1801f004, "sr2 sr_control0" }, { 0x1801f008, "sr2 sr_control1" },
    { 0x1801f030, "sr2 0x030" },
    { 0x18020004, "sr3 sr_control0" }, { 0x18020008, "sr3 sr_control1" },
    { 0x18001000, "pcie 0x000" }, { 0x180010f8, "pcie 0x0f8" },
    { 0x180010fc, "pcie 0x0fc" }, { 0x18001100, "pcie 0x100" },
    { 0x18001104, "pcie 0x104" }, { 0x1800110c, "pcie 0x10c" },
    { 0x18001110, "pcie 0x110" }, { 0x18001128, "pcie mdio control" },
    { 0x1800112c, "pcie mdio wrdata" }, { 0x180011c0, "pcie 0x1c0" },
    { 0x18001408, "pcie 0x408" }, { 0x18001410, "pcie 0x410" },
    { 0x18001820, "pcie 0x820" }, { 0x18001824, "pcie 0x824" },
    { 0x18001830, "pcie 0x830" }, { 0x18001c10, "pcie 0xc10" },
    { 0x18001148, "pcie 0x148 (D2H mailbox? stock writes 0x12345678)" },
    { 0x18001c30, "pcie 0xc30" },
    { 0x18001c14, "pcie 0xc14" },
    { 0x18001c18, "pcie 0xc18" },
    { 0x18001120, "pcie configaddr" }, { 0x18001124, "pcie configdata" },
    { 0x1803115c, "d11.0 machwcap" },
    { 0x1803118c, "d11.0 tsf_cfpstart" },
    { 0x180311a4, "d11.0 machwcap1" },
    { 0x1803215c, "d11.1 machwcap" },
    { 0x1803218c, "d11.1 tsf_cfpstart" },
    { 0x180321a4, "d11.1 machwcap1" },
    { 0x1803315c, "d11.2 machwcap" },
    { 0x1803318c, "d11.2 tsf_cfpstart" },
    { 0x180331a4, "d11.2 machwcap1" },
    { 0x18001010, "pcie 0x010" },
    { 0x18001018, "pcie 0x018" },
    { 0x1800103c, "pcie 0x03c" },
    { 0x18001040, "pcie 0x040" },
    { 0x18001044, "pcie 0x044" },
    { 0x180011ac, "pcie 0x1ac" },
    { 0x180011b0, "pcie 0x1b0" },
    { 0x180011b4, "pcie 0x1b4" },
    { 0x180011b8, "pcie 0x1b8" },
    { 0x180011bc, "pcie 0x1bc" },
    { 0x180011c4, "pcie 0x1c4" },
    { 0x180011c8, "pcie 0x1c8" },
    { 0x180011cc, "pcie 0x1cc" },
    { 0x180011e8, "pcie 0x1e8" },
    { 0x18001200, "pcie 0x200" },
    { 0x18001204, "pcie 0x204" },
    { 0x18001208, "pcie 0x208" },
    { 0x1800120c, "pcie 0x20c" },
    { 0x18001210, "pcie 0x210" },
    { 0x18001214, "pcie 0x214" },
    { 0x18001218, "pcie 0x218" },
    { 0x1800121c, "pcie 0x21c" },
    { 0x18001220, "pcie 0x220" },
    { 0x18001224, "pcie 0x224" },
    { 0x18001228, "pcie 0x228" },
    { 0x1800122c, "pcie 0x22c" },
    { 0x18001230, "pcie 0x230" },
    { 0x18001234, "pcie 0x234" },
    { 0x18001238, "pcie 0x238" },
    { 0x1800123c, "pcie 0x23c" },
    { 0x18001240, "pcie 0x240" },
    { 0x18001244, "pcie 0x244" },
    { 0x18001248, "pcie 0x248" },
    { 0x1800124c, "pcie 0x24c" },
    { 0x18001250, "pcie 0x250" },
    { 0x18001254, "pcie 0x254" },
    { 0x18001258, "pcie 0x258" },
    { 0x1800125c, "pcie 0x25c" },
    { 0x18001260, "pcie 0x260" },
    { 0x18001264, "pcie 0x264" },
    { 0x18001268, "pcie 0x268" },
    { 0x1800126c, "pcie 0x26c" },
    { 0x18001270, "pcie 0x270" },
    { 0x18001274, "pcie 0x274" },
    { 0x1800140c, "pcie 0x40c" },
    { 0x18001414, "pcie 0x414" },
    { 0x18001580, "pcie 0x580" },
    { 0x18001584, "pcie 0x584" },
    { 0x18001588, "pcie 0x588" },
    { 0x18001590, "pcie 0x590" },
    { 0x18001594, "pcie 0x594" },
    { 0x18001598, "pcie 0x598" },
    { 0x180015a0, "pcie 0x5a0" },
    { 0x180015a4, "pcie 0x5a4" },
    { 0x180015a8, "pcie 0x5a8" },
    { 0x180015b0, "pcie 0x5b0" },
    { 0x180015b4, "pcie 0x5b4" },
    { 0x180015b8, "pcie 0x5b8" },
    { 0x18001600, "pcie 0x600" },
    { 0x18001604, "pcie 0x604" },
    { 0x18001608, "pcie 0x608" },
    { 0x18001610, "pcie 0x610" },
    { 0x18001c00, "pcie 0xc00" },
    { 0x18001c1c, "pcie 0xc1c" },
    { 0x18001c34, "pcie 0xc34" },
    { 0x18001c38, "pcie 0xc38" },
    { 0x18001c60, "pcie 0xc60" },
    { 0x18001c64, "pcie 0xc64" },
    { 0x18031098, "d11.0 0x098" },
    { 0x1803109c, "d11.0 0x09c" },
    { 0x180310a0, "d11.0 0x0a0" },
    { 0x180310c0, "d11.0 0x0c0" },
    { 0x180310c4, "d11.0 0x0c4" },
    { 0x180310c8, "d11.0 0x0c8" },
    { 0x180310cc, "d11.0 0x0cc" },
    { 0x18031220, "d11.0 dma rcv0 control" },
    { 0x18031224, "d11.0 dma rcv0 ptr" },
    { 0x18031228, "d11.0 dma rcv0 addrlow" },
    { 0x18031260, "d11.0 dma rcv1 control" },
    { 0x18031264, "d11.0 dma rcv1 ptr" },
    { 0x18031268, "d11.0 dma rcv1 addrlow" },
    { 0x180312a0, "d11.0 dma rcv2 control" },
    { 0x180312a4, "d11.0 dma rcv2 ptr" },
    { 0x180312a8, "d11.0 dma rcv2 addrlow" },
    { 0x18032098, "d11.1 0x098" },
    { 0x1803209c, "d11.1 0x09c" },
    { 0x180320a0, "d11.1 0x0a0" },
    { 0x180320c0, "d11.1 0x0c0" },
    { 0x180320c4, "d11.1 0x0c4" },
    { 0x180320c8, "d11.1 0x0c8" },
    { 0x180320cc, "d11.1 0x0cc" },
    { 0x18032220, "d11.1 dma rcv0 control" },
    { 0x18032224, "d11.1 dma rcv0 ptr" },
    { 0x18032228, "d11.1 dma rcv0 addrlow" },
    { 0x18032260, "d11.1 dma rcv1 control" },
    { 0x18032264, "d11.1 dma rcv1 ptr" },
    { 0x18032268, "d11.1 dma rcv1 addrlow" },
    { 0x180322a0, "d11.1 dma rcv2 control" },
    { 0x180322a4, "d11.1 dma rcv2 ptr" },
    { 0x180322a8, "d11.1 dma rcv2 addrlow" },
    { 0x18033098, "d11.2 0x098" },
    { 0x1803309c, "d11.2 0x09c" },
    { 0x180330a0, "d11.2 0x0a0" },
    { 0x180330c0, "d11.2 0x0c0" },
    { 0x180330c4, "d11.2 0x0c4" },
    { 0x180330c8, "d11.2 0x0c8" },
    { 0x180330cc, "d11.2 0x0cc" },
    { 0x18033220, "d11.2 dma rcv0 control" },
    { 0x18033224, "d11.2 dma rcv0 ptr" },
    { 0x18033228, "d11.2 dma rcv0 addrlow" },
    { 0x18033260, "d11.2 dma rcv1 control" },
    { 0x18033264, "d11.2 dma rcv1 ptr" },
    { 0x18033268, "d11.2 dma rcv1 addrlow" },
    { 0x180332a0, "d11.2 dma rcv2 control" },
    { 0x180332a4, "d11.2 dma rcv2 ptr" },
    { 0x180332a8, "d11.2 dma rcv2 addrlow" },
    { 0x180313e4, "d11.0 0x3e6 (phyanacore, classic layout)" },
    { 0x180323e4, "d11.1 0x3e6 (phyanacore, classic layout)" },
    { 0x180333e4, "d11.2 0x3e6 (phyanacore, classic layout)" },
    { 0x18101000, "pcie wrapper 0x000" }, { 0x18101004, "pcie wrapper 0x004" },
    { 0x18101064, "pcie wrapper 0x064" },
};

static bool reg_known(hwaddr a)
{
    for (int i = 0; i < ARRAY_SIZE(plain_regs); i++) {
        if (plain_regs[i].a == (a & ~3)) {
            return true;
        }
    }
    static const struct { hwaddr base, len; } ranges[] = {
        { CR4_BASE + 0x04, 0x04 }, { CR4_BASE + 0x40, 0x08 },
        { CR4_BASE + 0x68, 0x10 },      /* TCAM */
        { OOBR_BASE, 0xa00 },
    };

    for (int i = 0; i < ARRAY_SIZE(ranges); i++) {
        if (a >= ranges[i].base && a < ranges[i].base + ranges[i].len) {
            return true;
        }
    }
    if (a >= 0x18100000 && a < 0x18200000) {    /* AI wrappers */
        hwaddr o = a & 0xfff;

        return o == WRAP_IOCTL || o == WRAP_IOSTATUS || o == WRAP_RESETCTRL ||
               o == WRAP_RESETCTRL + 4 || (o >= 0x900 && o < 0x940);
    }
    if (d11_core(a) >= 0) {
        hwaddr o = a - D11_BASE(d11_core(a));

        return o == 0x1e0 || o == 0x120 || o == 0x3e0 || o == 0x3e6;
    }
    return false;
}

static bool sc_hook_live(void *opaque, int n)
{
    return port_live(opaque, n);
}

static uint16_t sc_hook_phyreg(void *opaque, int n, uint16_t reg)
{
    BCM4387MachineState *s = opaque;
    gpointer key = PORT_KEY(PORT_PHY, n, reg);

    return GPOINTER_TO_UINT(g_hash_table_lookup(
        g_hash_table_contains(s->phyregs, key) ? s->phyregs : s->phyknown, key));
}

/* template RAM / SampleCollect registers of a d11 slice */
static bool sc_port(BCM4387MachineState *s, hwaddr a, unsigned size, bool wr,
                    uint64_t *v)
{
    int n = d11_core(a);
    bool hit;

    if (n < 0) {
        return false;
    }
    hit = bcm4387_sc_access(s->sc[n], a - D11_BASE(n), size, wr, v);
    if (hit) {
        qemu_log_mask(LOG_UNIMP, "bcm4387: SC%d  %c%u +0x%03" HWADDR_PRIx
                      " %s 0x%08" PRIx64 " pc~0x%08x\n", n, wr ? 'W' : 'R',
                      size * 8, a - D11_BASE(n), wr ? "<-" : "->", *v,
                      bcm4387_pc());
    }
    return hit;
}

/*
 * Object memory port (objaddr +0x160, objdata +0x164/+0x166): microcode
 * memory and SHM of the slice's PSM (bcm4387-psm.c). Selects the PSM does
 * not model stay plain storage, logged. A write while objaddr selects UCM
 * also marks the slice's ucode loaded (no effect modelled).
 */
static bool obj_port(BCM4387MachineState *s, hwaddr a, unsigned size, bool wr,
                     uint64_t *v)
{
    int n = d11_core(a);
    hwaddr off = a - D11_BASE(n);

    if (n < 0 || (off != 0x160 && off != 0x164 && off != 0x166)) {
        return false;
    }
    if (wr && off != 0x160 && !(store_get(s, D11_BASE(n) + 0x160) & 0x7f0000)) {
        s->ucode_loaded[n] = true;
    }
    if (wr && off == 0x160) {
        store_set(s, a, *v);
    }
    if (!bcm4387_psm_objmem(s->psm[n], off, size, wr, v)) {
        qemu_log_mask(LOG_GUEST_ERROR, "bcm4387: d11.%d object memory %c%u "
                      "+0x%03" HWADDR_PRIx " with objaddr 0x%08x not "
                      "modelled (plain storage) pc~0x%08x\n", n,
                      wr ? 'W' : 'R', size * 8, off,
                      store_get(s, D11_BASE(n) + 0x160), bcm4387_pc());
        if (wr) {
            store_set(s, a, *v);
        } else {
            *v = store_get(s, a);
        }
    }
    qemu_log_mask(LOG_UNIMP, "bcm4387: OBJ%d %c%u +0x%03" HWADDR_PRIx " %s "
                  "0x%08" PRIx64 " pc~0x%08x\n", n, wr ? 'W' : 'R', size * 8,
                  off, wr ? "<-" : "->", *v, bcm4387_pc());
    return true;
}

/* the PSM runs on the MAC clock: slice out of reset, clock on, FCLKA [I] */
static bool psm_hook_clocked(void *opaque, int n)
{
    return d11_fclka(opaque, n);
}

/* IHRs the machine models for the PSM: SampleCollect / template RAM */
static bool psm_hook_ihr(void *opaque, int n, uint16_t idx, bool wr,
                         uint16_t *v)
{
    BCM4387MachineState *s = opaque;
    uint64_t v64 = *v;

    if (!bcm4387_sc_access(s->sc[n], 0x400 + 2 * idx, 2, wr, &v64)) {
        return false;
    }
    *v = v64;
    return true;
}

/*
 * The PSM's own PHY and radio ports (IHR 0x91/0x92, 0x94/0x95, see
 * bcm4387-psm.c) reach the same register files as the host ports. Their
 * reachability is taken to be the host ports' (port_live); what a PSM access
 * returns while the PHY is not reachable is unknown, so it is refused. [I]
 */
static bool psm_hook_port(void *opaque, int n, int port, bool wr,
                          uint16_t addr, uint16_t *v)
{
    BCM4387MachineState *s = opaque;
    int p = port ? PORT_RADIO : PORT_PHY;
    gpointer key = PORT_KEY(p, n, addr);

    if (s->strict && !port_live(s, n)) {
        return false;
    }
    if (wr) {
        g_hash_table_insert(s->phyregs, key, GUINT_TO_POINTER(*v));
    } else if (g_hash_table_contains(s->phyregs, key)) {
        *v = GPOINTER_TO_UINT(g_hash_table_lookup(s->phyregs, key));
    } else {
        if (s->strict && !g_hash_table_contains(s->phyknown, key)) {
            qemu_log_mask(LOG_GUEST_ERROR, "bcm4387: unknown %s%d reg 0x%04x "
                          "read by the PSM (reset value not known; 0)\n",
                          p == PORT_PHY ? "PHY" : "RAD", n, addr);
        }
        *v = GPOINTER_TO_UINT(g_hash_table_lookup(s->phyknown, key));
    }
    qemu_log_mask(LOG_UNIMP, "bcm4387: PSM-%s%d %c 0x%04x %s 0x%04x\n",
                  p == PORT_PHY ? "PHY" : "RAD", n, wr ? 'W' : 'R', addr,
                  wr ? "<-" : "->", *v);
    return true;
}

static MemTxResult trace_read(void *opaque, hwaddr addr, uint64_t *data,
                              unsigned size, MemTxAttrs attrs)
{
    BCM4387MachineState *s = opaque;
    uint32_t v = 0;
    bool modelled, dropped = false;
    MemTxResult r;

    bcm4387_mmio_attr(addr, false);
    r = d11_gate(s, addr, false, &dropped);
    if (r != MEMTX_OK) {
        return r;
    }
    if (phy_preinit_fault(s, addr, size)) {
        return MEMTX_ERROR;
    }
    if (phy_port(s, addr, size, false, 0, &v)) {
        *data = v;
        return MEMTX_OK;
    }
    if (sc_port(s, addr, size, false, data) ||
        obj_port(s, addr, size, false, data)) {
        return MEMTX_OK;
    }
    modelled = model_read(s, addr, &v);
    if (!modelled) {
        if (s->strict && !reg_known(addr)) {
            qemu_log_mask(LOG_GUEST_ERROR, "bcm4387: AXI slave error: "
                          "unmodelled R%u 0x%08" HWADDR_PRIx " pc~0x%08x\n",
                          size * 8, addr, bcm4387_pc());
            return MEMTX_ERROR;
        }
        v = store_get(s, addr);
    }
    qemu_log_mask(LOG_UNIMP, "bcm4387: %s R%u 0x%08" HWADDR_PRIx
                  " -> 0x%08x pc~0x%08x lr~0x%08x\n",
                  modelled ? "MODEL" : reg_known(addr) ? "MMIO " : "MMIO?", size * 8, addr, v,
                  bcm4387_pc(), bcm4387_lr());
    *data = v;
    return MEMTX_OK;
}

static MemTxResult trace_write(void *opaque, hwaddr addr, uint64_t val,
                               unsigned size, MemTxAttrs attrs)
{
    BCM4387MachineState *s = opaque;
    bool modelled, dropped = false;
    MemTxResult r;

    bcm4387_mmio_attr(addr, true);
    r = d11_gate(s, addr, true, &dropped);

    if (r != MEMTX_OK || dropped) {
        return r;
    }
    if (phy_port(s, addr, size, true, val, NULL)) {
        return MEMTX_OK;
    }
    if (sc_port(s, addr, size, true, &val) ||
        obj_port(s, addr, size, true, &val)) {
        return MEMTX_OK;
    }
    if (s->strict && !reg_known(addr)) {
        qemu_log_mask(LOG_GUEST_ERROR, "bcm4387: AXI slave error: unmodelled "
                      "W%u 0x%08" HWADDR_PRIx " <- 0x%08" PRIx64
                      " pc~0x%08x\n", size * 8, addr, val, bcm4387_pc());
        return MEMTX_ERROR;
    }
    modelled = model_write(s, addr, val);
    if (!modelled) {
        store_set(s, addr, val);
    }
    qemu_log_mask(LOG_UNIMP, "bcm4387: %s W%u 0x%08" HWADDR_PRIx
                  " <- 0x%08" PRIx64 " pc~0x%08x lr~0x%08x\n",
                  modelled ? "MODEL" : reg_known(addr) ? "MMIO " : "MMIO?", size * 8, addr, val,
                  bcm4387_pc(), bcm4387_lr());
    return MEMTX_OK;
}

static const MemoryRegionOps trace_ops = {
    .read_with_attrs = trace_read,
    .write_with_attrs = trace_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

/* ---- OOB router (HND_OOBR, core 0x85c) ----------------------------------- */

/*
 * Interrupt routing. hnd_oobr_get_intr_config (RAM 0x82b424) takes the source
 * core's OOB output selectors (percore[i] +0x10, one byte each: bit7 valid,
 * bits[6:0] signal id) and finds the ARM CR4 input carrying the same signal
 * (percore[cr4] +0x00); the signal id is the interrupt line. percore[i] +0x38
 * holds the output (bits[15:8]) and input (bits[23:16]) counts, +0x00 of the
 * router the core count. The lines are as registered on silicon (ISR list in
 * fw/dump/ram_identity.bin): ChipCommon 0, PCIe 2, d11 cores 5/7/9; the
 * per-core output counts filling the gaps and the identity input map are
 * inferred. Seeded into the register store, so firmware writes still apply.
 */
static void oobr_init(BCM4387MachineState *s)
{
    /* si core index, first signal id, number of outputs */
    static const struct { int idx, sig, n; } out[] = {
        { 0, 0, 2 }, { 1, 2, 3 }, { 3, 5, 2 }, { 4, 7, 2 }, { 5, 9, 2 },
    };
    const int ncores = 18, cr4 = 2, nin = 16;

    store_set(s, OOBR_BASE, ncores);
    for (int i = 0; i < ARRAY_SIZE(out); i++) {
        hwaddr pc = OOBR_BASE + 0x100 + out[i].idx * 0x80;

        for (int j = 0; j < out[i].n; j++) {
            hwaddr w = pc + 0x10 + (j & ~3);

            store_set(s, w, store_get(s, w) |
                      ((0x80u | (out[i].sig + j)) << (8 * (j & 3))));
        }
        store_set(s, pc + 0x38, out[i].n << 8);
    }
    for (int k = 0; k < nin; k++) {
        hwaddr w = OOBR_BASE + 0x100 + cr4 * 0x80 + (k & ~3);

        store_set(s, w, store_get(s, w) | ((0x80u | k) << (8 * (k & 3))));
    }
    store_set(s, OOBR_BASE + 0x100 + cr4 * 0x80 + 0x38,
              store_get(s, OOBR_BASE + 0x100 + cr4 * 0x80 + 0x38) | nin << 16);
}

/* ---- EROM --------------------------------------------------------------- */

/*
 * The chip's enumeration ROM as read on silicon by the open firmware
 * (fw/dump/erom_silicon.bin, 190 words at 0x18105000 = ChipCommon+0xfc). It
 * includes 64-bit address descriptors (type 0xD) that a reconstruction from
 * the firmware's parsed core table lacked. [F]
 */
static const uint32_t erom_words[] = {
    0x4bf80001, 0x45084411, 0x00000003, 0x18000005, 0x1c000135, 0x04000000,
    0x181000c5, 0x18103185, 0x4bf83c01, 0x47084411, 0x00000103, 0x18001005,
    0x18002005, 0x18003005, 0x08000135, 0x08000000, 0x0000013d, 0x80000000,
    0x00000008, 0x80000000, 0x181010c5, 0x18102185, 0x4bf83e01, 0x12084411,
    0x00000203, 0x18030005, 0x18034025, 0x00000135, 0x0059b000, 0x00740135,
    0x001f9000, 0x181300c5, 0x1813c185, 0x4bf81201, 0x5508c431, 0x00000303,
    0x00000413, 0x00000523, 0x18031005, 0xe8800135, 0x00800000, 0x181310c5,
    0x181341c5, 0x181352c5, 0x18136185, 0x4bf81201, 0x5508c431, 0x00000603,
    0x00000713, 0x00000823, 0x18032005, 0xe8000135, 0x00800000, 0x181320c5,
    0x181371c5, 0x181382c5, 0x18139185, 0x4bf81201, 0x55084411, 0x00000903,
    0x18033005, 0xe9000135, 0x00800000, 0x181330c5, 0x1813a185, 0x4bf85701,
    0x02084211, 0x00000a03, 0x19000035, 0x01000000, 0x181080c5, 0x18109085,
    0x4bf85501, 0x03000201, 0x18004005, 0x4bf85501, 0x03000201, 0x18005005,
    0x4bf85c01, 0x00000201, 0x18006005, 0x4bf84101, 0x09000201, 0x18007005,
    0x4bf84001, 0x13000201, 0x18010015, 0x4bf82701, 0x27000201, 0x18012005,
    0x4bf84101, 0x09000201, 0x18013005, 0x4bf84101, 0x09000201, 0x18014005,
    0x4bf84a01, 0x0c000201, 0x18015005, 0x18016005, 0x18017005, 0x1801c005,
    0x4bf84a01, 0x0e000201, 0x18018005, 0x1801d005, 0x4bf84a01, 0x0d000201,
    0x18019005, 0x1801a005, 0x1801b005, 0x1801e005, 0x4bf84101, 0x09000201,
    0x1801f005, 0x4bf84101, 0x09000201, 0x18020005, 0x43b13501, 0x00080201,
    0x18000075, 0x00006000, 0x18106085, 0x43b13501, 0x00080201, 0x18010075,
    0x00010000, 0x18020055, 0x18107085, 0x43b13501, 0x00080201, 0x18030075,
    0x00008000, 0x1813d085, 0x43b36601, 0x00000201, 0x18105005, 0x43b36601,
    0x00000201, 0x18140005, 0x43b30101, 0x00000201, 0x18200035, 0x00100000,
    0x43b30101, 0x00000201, 0x18300035, 0x00100000, 0x43b03101, 0x00080011,
    0x00000b03, 0x1810a085, 0x43b03101, 0x00084011, 0x00000c03, 0x1810b0c5,
    0x1813e085, 0x43b03101, 0x00084011, 0x00000d03, 0x1810c0c5, 0x1813f085,
    0x43bfff01, 0x00080201, 0x0059b035, 0x001a5000, 0x00939035, 0x076c7000,
    0x10000035, 0x08000000, 0x18008035, 0x00008000, 0x18021035, 0x0000f000,
    0x18038035, 0x000c8000, 0x1810d035, 0x00023000, 0x18141035, 0x000bf000,
    0x18400035, 0x00c00000, 0x1a000035, 0x02000000, 0x20000035, 0xc8000000,
    0xe9800035, 0x16800000, 0x18104085, 0x0000000f,
};

static void erom_build(BCM4387MachineState *s)
{
    uint32_t *e = memory_region_get_ram_ptr(&s->erom);

    QEMU_BUILD_BUG_ON(sizeof(erom_words) > EROM_SIZE);
    memset(e, 0, EROM_SIZE);
    for (size_t i = 0; i < ARRAY_SIZE(erom_words); i++) {
        e[i] = cpu_to_le32(erom_words[i]);
    }
}

/* ---- CR4 TCM banks -------------------------------------------------------- */

static uint32_t bank_encode(uint32_t size)
{
    if (size % (8 * KiB) == 0 && size / (8 * KiB) <= 128) {
        return size / (8 * KiB) - 1;
    }
    assert(size % KiB == 0 && size / KiB <= 128);
    return (1 << 9) | (size / KiB - 1);
}

static int banks_fill(uint32_t *info, int n, uint32_t size)
{
    while (size) {
        uint32_t chunk;

        if (size >= MiB) {
            chunk = MiB;
        } else if (size >= 8 * KiB) {
            chunk = size & ~(8 * KiB - 1);
        } else {
            chunk = size;
        }
        info[n++] = bank_encode(chunk);
        size -= chunk;
    }
    return n;
}

/* ---- host side: shared-RAM watch and console ---------------------------- */

static uint32_t ram_ld(hwaddr a)
{
    return address_space_ldl_le(&address_space_memory, a,
                                MEMTXATTRS_UNSPECIFIED, NULL);
}

static void dump_console(uint32_t shared)
{
    uint32_t cons = ram_ld(shared + 20);
    uint32_t buf = ram_ld(cons + 8), size = ram_ld(cons + 12);
    uint32_t widx = ram_ld(cons + 16);
    g_autofree char *text = g_malloc0(size + 1);

    if (!size || size > 64 * KiB) {
        return;
    }
    address_space_read(&address_space_memory, buf, MEMTXATTRS_UNSPECIFIED,
                       text, size);
    fprintf(stderr, "bcm4387: console base 0x%x buf 0x%x size %u widx %u\n",
            cons, buf, size, widx);
    /* oldest data starts at the write index once the ring has wrapped */
    fwrite(text + widx, 1, strnlen(text + widx, size - widx), stderr);
    fwrite(text, 1, strnlen(text, widx), stderr);
    fputc('\n', stderr);
}

static void watch_shared(void *opaque)
{
    BCM4387MachineState *s = opaque;
    uint32_t v = ram_ld(RAM_TOP - 4);

    if (v != s->shared) {
        s->shared = v;
        fprintf(stderr, "bcm4387: shared-RAM word at 0x%x = 0x%08x\n",
                RAM_TOP - 4, v);
        if (v >= RAM_BASE && v < RAM_TOP) {
            dump_console(v);
        }
    }
    timer_mod(s->watch, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 100);
}

/* ---- machine ------------------------------------------------------------ */

static void bcm4387_reset(void *opaque)
{
    BCM4387MachineState *s = opaque;

    cpu_reset(CPU(s->cpu));
    for (int n = 0; n < 3; n++) {
        bcm4387_sc_reset(s->sc[n]);
        bcm4387_psm_reset(s->psm[n]);
    }
    /*
     * brcmf_chip_cr4_set_active(): the host writes only image word 0 (the
     * reset vector) to address 0; the rest of that TCM is never written.
     */
    memset(s->lowtcm_mem, 0, LOWTCM_SIZE);
    bitmap_zero(s->lowtcm_valid, LOWTCM_SIZE / 4);
    stl_le_p(s->lowtcm_mem, s->rstvec);
    set_bit(0, s->lowtcm_valid);
    s->shared = ram_ld(RAM_TOP - 4);
    timer_mod(s->watch, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 100);
}

static void bcm4387_init(MachineState *machine)
{
    BCM4387MachineState *s = BCM4387_MACHINE(machine);
    MemoryRegion *sysmem = get_system_memory();
    Object *cpuobj;
    int64_t sz;

    if (!machine->firmware) {
        error_report("bcm4387: firmware=<file> (or -bios) is required");
        exit(1);
    }
    if (s->atcm_size == 0 || s->atcm_size >= RAM_SIZE ||
        s->atcm_size % KiB) {
        error_report("bcm4387: bad atcm-size 0x%x", s->atcm_size);
        exit(1);
    }

    /*
     * CP15 as read on silicon at reset: ATCM region 0x39 (base 0, 8 MiB),
     * BTCM region 0x00800031 (base 0x800000, 2 MiB), SCTLR 0x40e50878 (TE
     * from cfgte). Never-written TCM words read as ECC (parity) aborts. [F]
     */
    cpuobj = object_new(machine->cpu_type);
    object_property_set_bool(cpuobj, "cfgte", true, &error_fatal);
    object_property_set_bool(cpuobj, "access-error-parity", true, &error_fatal);
    object_property_set_uint(cpuobj, "atcm-region", 0x39, &error_fatal);
    object_property_set_uint(cpuobj, "btcm-region", 0x00800031, &error_fatal);
    ARM_CPU(cpuobj)->reset_sctlr = 0x00e50878;
    /* MPUIR reads 0xc00 on silicon: 12 data regions (run 38) [F] */
    object_property_set_uint(cpuobj, "pmsav7-dregion", 12, &error_fatal);
    qdev_realize(DEVICE(cpuobj), NULL, &error_fatal);
    s->cpu = ARM_CPU(cpuobj);

    /* RAM: the ATCM part at rambase, BTCM from 0x800000 up to RAM_TOP */
    memory_region_init_ram(&s->atcm, NULL, "bcm4387.atcm", s->atcm_size,
                           &error_fatal);
    memory_region_add_subregion(sysmem, RAM_BASE, &s->atcm);
    s->lowtcm_mem = g_malloc0(LOWTCM_SIZE);
    s->lowtcm_valid = bitmap_new(LOWTCM_SIZE / 4);
    memory_region_init_io(&s->lowtcm, NULL, &lowtcm_ops, s, "bcm4387.tcm0",
                          LOWTCM_SIZE);
    memory_region_add_subregion(sysmem, 0, &s->lowtcm);
    memory_region_init_io(&s->hole, NULL, &hole_ops, s, "bcm4387.hole",
                          HOLE_SIZE);
    memory_region_add_subregion(sysmem, HOLE_BASE, &s->hole);
    memory_region_init_ram(&s->btcm, NULL, "bcm4387.btcm",
                           RAM_SIZE - s->atcm_size, &error_fatal);
    memory_region_add_subregion(sysmem, RAM_BASE + s->atcm_size, &s->btcm);

    /* RAM above the TCM for the firmware's ROM-patch code (see SOCRAM_BASE).
     * socram=<file> pre-loads it from a hardware dump of 0x939000 upward (the
     * firmware would otherwise populate it from the init path we can't yet
     * reach). */
    memory_region_init_ram(&s->socram, NULL, "bcm4387.socram", SOCRAM_SIZE,
                           &error_fatal);
    memory_region_add_subregion(sysmem, SOCRAM_BASE, &s->socram);
    if (s->patch) {
        int64_t psz = get_image_size(s->patch, &error_fatal);

        if (psz <= 0 || psz > SOCRAM_SIZE ||
            load_image_size(s->patch,
                            memory_region_get_ram_ptr(&s->socram), psz) != psz) {
            error_report("bcm4387: cannot load socram %s", s->patch);
            exit(1);
        }
        info_report("bcm4387: loaded socram %s at 0x%x (0x%" PRIx64 " bytes)",
                    s->patch, SOCRAM_BASE, psz);
    }

    s->nab = banks_fill(s->bank_info, 0, s->atcm_size);
    s->nbb = banks_fill(s->bank_info, s->nab, RAM_SIZE - s->atcm_size) -
             s->nab;

    memory_region_init_rom(&s->rom_mr, NULL, "bcm4387.rom", ROM_SIZE,
                           &error_fatal);
    memory_region_add_subregion(sysmem, ROM_BASE, &s->rom_mr);
    {
        /* Default the whole ROM window to "bx lr", then overlay any real ROM
         * given via rom= on top. rom= is a ';'-separated
         * list of chunks, each "path" (loaded at ROM_BASE) or "path@0xADDR"
         * (loaded at that absolute address in the ROM window). Partial chunks
         * are fine; whatever isn't covered stays a stub. */
        uint16_t *h = memory_region_get_ram_ptr(&s->rom_mr);

        for (int i = 0; i < ROM_SIZE / 2; i++) {
            h[i] = cpu_to_le16(0x4770);     /* bx lr */
        }

        if (s->rom) {
            g_auto(GStrv) chunks = g_strsplit(s->rom, ";", 0);

            for (int i = 0; chunks[i]; i++) {
                g_auto(GStrv) pa = g_strsplit(chunks[i], "@", 2);
                hwaddr base = pa[1] ? strtoull(pa[1], NULL, 0) : ROM_BASE;
                int64_t fsz = get_image_size(pa[0], NULL);

                if (!pa[0][0]) {
                    continue;
                }
                if (fsz < 0 || base < ROM_BASE ||
                    base + fsz > ROM_BASE + ROM_SIZE) {
                    error_report("bcm4387: rom chunk %s: bad path/addr/size",
                                 chunks[i]);
                    exit(1);
                }
                if (load_image_size(pa[0], (uint8_t *)h + (base - ROM_BASE),
                                    fsz) != fsz) {
                    error_report("bcm4387: rom chunk %s: load failed",
                                 chunks[i]);
                    exit(1);
                }
                info_report("bcm4387: mapped ROM chunk %s at 0x%" HWADDR_PRIx
                            " (0x%" PRIx64 " bytes)", pa[0], base, fsz);
            }
        }
    }

    s->rom_pristine = g_memdup2(memory_region_get_ram_ptr(&s->rom_mr),
                                ROM_SIZE);

    memory_region_init_rom(&s->erom, NULL, "bcm4387.erom", EROM_SIZE,
                           &error_fatal);
    memory_region_add_subregion(sysmem, EROM_BASE, &s->erom);
    erom_build(s);

    s->store = g_hash_table_new(g_direct_hash, g_direct_equal);
    oobr_init(s);
    phy_init(s);
    /* register state at the host->firmware handoff, read on silicon [F] */
    for (int i = 0; i < ARRAY_SIZE(d11_extra_wraps); i++) {
        store_set(s, d11_extra_wraps[i] + WRAP_RESETCTRL, 1);
    }
    {
        static const uint32_t iostatus[3] = { 0x11002, 0x11001, 0x11018 };

        for (int n = 0; n < 3; n++) {
            store_set(s, D11_WRAP(n) + WRAP_IOCTL, 0);
            store_set(s, D11_WRAP(n) + WRAP_RESETCTRL, 1);
            store_set(s, D11_WRAP(n) + WRAP_IOSTATUS, iostatus[n]);
            store_set(s, D11_BASE(n) + 0x1e0, 0x40);

        }
    }
    if (s->phy_live && strcmp(s->phy_live, "auto") &&
        strcmp(s->phy_live, "on") && strcmp(s->phy_live, "off")) {
        error_report("bcm4387: phy-live must be auto, on or off");
        exit(1);
    }
    if (s->psm_dead_port && strcmp(s->psm_dead_port, "abort") &&
        strcmp(s->psm_dead_port, "busy")) {
        error_report("bcm4387: psm-dead-port must be abort or busy");
        exit(1);
    }
    bcm4387_strict = s->strict;
    the_machine = s;
    s->pmu = bcm4387_pmu_create(PMU_BASE);
    s->gci = bcm4387_gci_create(GCI_BASE, s->pmu);
    s->cc = bcm4387_cc_create(CC_BASE, s->pmu);
    {
        Bcm4387ScHooks hooks = {
            .opaque = s, .phy_live = sc_hook_live, .phyreg = sc_hook_phyreg,
        };

        Bcm4387PsmHooks psm_hooks = {
            .opaque = s, .clocked = psm_hook_clocked, .ihr = psm_hook_ihr,
            .port = psm_hook_port,
            .dead_port_busy = s->psm_dead_port &&
                              !strcmp(s->psm_dead_port, "busy"),
        };

        for (int n = 0; n < 3; n++) {
            s->sc[n] = bcm4387_sc_new(n, &hooks);
            s->psm[n] = bcm4387_psm_new(n, &psm_hooks);
        }
        if (s->iq_source) {
            bcm4387_iq_load(s->iq_source, s->iq_format);
        }
    }
    memory_region_init_io(&s->trace, NULL, &trace_ops, s, "bcm4387.trace",
                          UINT64_MAX);
    memory_region_add_subregion_overlap(sysmem, 0, &s->trace, -1000);

    /*
     * Host download (brcmf_pcie_download_fw_nvram). Read the image into a
     * buffer so we can apply identity relocation before it is installed, then
     * register it as the firmware rom blob. (load_image_targphys defers the
     * load to reset, so patching memory afterwards gets clobbered.)
     *
     * Identity relocation: force the two ASLR placement helpers (0x8a3e48
     * intra-page, 0x8a3eb8 page) to "movs r0,#0; bx lr" so every section's ASLR
     * offset is 0 and code stays at its link address — matching the hw patch.
     * (Same two patches as scripts/rompatch.py applies on hardware.)
     */
    {
        g_autofree char *fw = NULL;
        gsize fwlen = 0;

        if (!g_file_get_contents(machine->firmware, &fw, &fwlen, NULL) ||
            fwlen == 0 || fwlen > RAM_SIZE) {
            error_report("bcm4387: cannot read firmware %s", machine->firmware);
            exit(1);
        }
        if (s->relocraw) {
            /*
             * relocraw=on: run the real ASLR relocator with its genuine
             * intra-page placement helper (0x8a3e48, which returns the real
             * +0x400), and the genuine PAGE placement helper (0x8a3eb8) — RNG,
             * bounds checks and all, so the RNG stream (stack canary, later
             * placements) stays faithful — but force the page offset it RETURNS
             * to 0 by patching its epilogue "mov r0,r4" (0x8a3f04) to
             * "movs r0,#0".
             *
             * Why: each section descriptor is filled as [+0]=place_page() (page
             * ASLR) and [+4]=place_intra() (=+0x400); the re-linker computes a
             * branch fixup delta of [+0]-[+4]. For a branch whose target is in a
             * FIXED section (ROM at 0x4xxxxx, which does not move in our model —
             * we map it once at 0x400000), the correct delta is -0x400 (only the
             * instruction moved +0x400, the ROM target stays put), which needs
             * [+0]=0. Genuine place_page instead returns a random page offset
             * (observed -0x270be8) that lands in ROM and corrupts every
             * ROM-target branch (e.g. 0x82c430 "bl 0x47eb84" -> bl 0x20df9c,
             * which sleds into the dead gap and aborts). On real hardware the
             * host applies that page placement by actually loading the image at
             * the random page; our model keeps the image at its link base and
             * only honours the +0x400 intra-page shift, so the faithful page
             * offset here is 0.
             *
             * TODO (fidelity): boot then stops at the ROM-patch verify (RAM
             * 0x82c3f0), because the re-linker rewrites the veneers at 0x750000
             * (e.g. 5c f3 1e bf -> 5d f3 1e b9) after the load-time veneer->ROM
             * copy below has already run. Faithful fix: make ROM reads of a
             * patched address return the live RAM veneer (a dynamic TCAM
             * redirect) instead of a static copy.
             */
            static const uint8_t movs_r0_0[] = { 0x00, 0x20 };
            gsize op = 0x8a3f04 - RAM_BASE;

            if (op + 2 > fwlen) {
                error_report("bcm4387: firmware too small for relocraw patch");
                exit(1);
            }
            memcpy(fw + op, movs_r0_0, sizeof(movs_r0_0));
        } else if (s->identity && !s->relocate) {
            /* stock image only: the patch sites must hold the stock bytes */
            static const uint8_t ret0[] = { 0x00, 0x20, 0x70, 0x47 };
            static const uint8_t orig1[] = { 0x08, 0xb5, 0x07, 0x4b };
            static const uint8_t orig2[] = { 0x13, 0x4b, 0x70, 0xb5 };
            gsize o1 = 0x8a3e48 - RAM_BASE, o2 = 0x8a3eb8 - RAM_BASE;

            if (o2 + 4 <= fwlen && !memcmp(fw + o1, orig1, 4) &&
                !memcmp(fw + o2, orig2, 4)) {
                memcpy(fw + o1, ret0, 4);
                memcpy(fw + o2, ret0, 4);
            } else {
                info_report("bcm4387: not the stock image, identity patch "
                            "not applied");
            }
        } else if (s->relocate) {
            /*
             * relocate=on: let the real ASLR relocator run, but force a
             * deterministic, model-mapped section offset. Keep each placement
             * helper's prologue (which sets up the [ctx+0x40] magic the copy
             * loop needs) and override only its RETURN: intra-page offset 0,
             * page offset +0x108ffc (into mapped BTCM/socram). The relocator
             * then re-links per its tables by itself.
             * - 0x8a3e4e (after 0x8a3e48's prologue): "movs r0,#0; pop {r3,pc}"
             * - 0x8a3ec0 (after 0x8a3eb8's prologue): "ldr r0,[pc,#0];
             *   pop {r4,r5,r6,pc}; .word 0x108ffc"
             * STATUS: the relocator runs and re-links (the stack-canary RNG
             * 0x47eb10 is re-linked away and the canary clears), but the
             * re-link currently also moves a ROM target (0x40a5b4 -> 0x5135b0),
             * faulting. The per-region (RAM +0x108ffc, ROM 0) descriptor setup
             * still needs work.
             */
            static const uint8_t intra0[] = { 0x00, 0x20, 0x08, 0xbd };
            static const uint8_t page_off[] = { 0x00, 0x48, 0x70, 0xbd,
                                                0xfc, 0x8f, 0x10, 0x00 };
            gsize o1 = (0x8a3e48 + 6) - RAM_BASE;
            gsize o2 = (0x8a3eb8 + 8) - RAM_BASE;

            if (o2 + 8 > fwlen) {
                error_report("bcm4387: firmware too small for relocate patch");
                exit(1);
            }
            memcpy(fw + o1, intra0, sizeof(intra0));
            memcpy(fw + o2, page_off, sizeof(page_off));
        }
        s->rstvec = ldl_le_p(fw);
        rom_add_blob_fixed("bcm4387.fw", fw, fwlen, RAM_BASE);
    }

    if (s->hostblob) {
        sz = get_image_size(s->hostblob, &error_fatal);
        if (sz <= 0 || sz > RAM_SIZE / 2 ||
            load_image_targphys(s->hostblob, RAM_TOP - sz, sz,
                                &error_fatal) < 0) {
            error_report("bcm4387: cannot load hostblob %s", s->hostblob);
            exit(1);
        }
    }

    s->watch = timer_new_ms(QEMU_CLOCK_VIRTUAL, watch_shared, s);
    qemu_register_reset(bcm4387_reset, s);
}

static char *get_str(char **field)
{
    return g_strdup(*field);
}

#define STR_PROP(name, field)                                               \
    static char *get_##field(Object *o, Error **e)                          \
    {                                                                       \
        return get_str(&BCM4387_MACHINE(o)->field);                         \
    }                                                                       \
    static void set_##field(Object *o, const char *v, Error **e)            \
    {                                                                       \
        g_free(BCM4387_MACHINE(o)->field);                                  \
        BCM4387_MACHINE(o)->field = g_strdup(v);                            \
    }

STR_PROP("hostblob", hostblob)
STR_PROP("rom", rom)
STR_PROP("socram", patch)
STR_PROP("iq-source", iq_source)
STR_PROP("iq-format", iq_format)
STR_PROP("phy-live", phy_live)
STR_PROP("psm-dead-port", psm_dead_port)

static void get_atcm(Object *o, Visitor *v, const char *name, void *opaque,
                     Error **errp)
{
    visit_type_uint32(v, name, &BCM4387_MACHINE(o)->atcm_size, errp);
}

static void set_atcm(Object *o, Visitor *v, const char *name, void *opaque,
                     Error **errp)
{
    visit_type_uint32(v, name, &BCM4387_MACHINE(o)->atcm_size, errp);
}

static bool get_identity(Object *o, Error **errp)
{
    return BCM4387_MACHINE(o)->identity;
}

static void set_identity(Object *o, bool v, Error **errp)
{
    BCM4387_MACHINE(o)->identity = v;
}

static bool get_relocate(Object *o, Error **errp)
{
    return BCM4387_MACHINE(o)->relocate;
}

static void set_relocate(Object *o, bool v, Error **errp)
{
    BCM4387_MACHINE(o)->relocate = v;
}

static bool get_relocraw(Object *o, Error **errp)
{
    return BCM4387_MACHINE(o)->relocraw;
}

static void set_relocraw(Object *o, bool v, Error **errp)
{
    BCM4387_MACHINE(o)->relocraw = v;
}

static bool get_strict(Object *o, Error **errp)
{
    return BCM4387_MACHINE(o)->strict;
}

static void set_strict(Object *o, bool v, Error **errp)
{
    BCM4387_MACHINE(o)->strict = v;
}

static void bcm4387_instance_init(Object *obj)
{
    /* CP15 at reset on silicon: ATCM region 0-8 MiB, BTCM base 0x800000 */
    BCM4387_MACHINE(obj)->atcm_size = 0x800000 - RAM_BASE;
    BCM4387_MACHINE(obj)->identity = true;
    BCM4387_MACHINE(obj)->strict = true;
}

static void bcm4387_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    static const char * const valid_cpu_types[] = {
        ARM_CPU_TYPE_NAME("cortex-r5"),
        NULL
    };

    mc->desc = "Broadcom BCM4387 Wi-Fi dongle ARM core (RE scaffold)";
    mc->init = bcm4387_init;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-r5");
    mc->valid_cpu_types = valid_cpu_types;

    object_class_property_add_str(oc, "hostblob", get_hostblob, set_hostblob);
    object_class_property_add_str(oc, "rom", get_rom, set_rom);
    object_class_property_add_str(oc, "socram", get_patch, set_patch);
    object_class_property_add_str(oc, "iq-source", get_iq_source, set_iq_source);
    object_class_property_add_str(oc, "iq-format", get_iq_format, set_iq_format);
    object_class_property_add_str(oc, "phy-live", get_phy_live, set_phy_live);
    object_class_property_add_str(oc, "psm-dead-port", get_psm_dead_port,
                                  set_psm_dead_port);
    object_class_property_add(oc, "atcm-size", "uint32", get_atcm, set_atcm,
                              NULL, NULL);
    object_class_property_add_bool(oc, "identity", get_identity, set_identity);
    object_class_property_add_bool(oc, "relocate", get_relocate, set_relocate);
    object_class_property_add_bool(oc, "relocraw", get_relocraw, set_relocraw);
    object_class_property_add_bool(oc, "strict", get_strict, set_strict);
}

static const TypeInfo bcm4387_machine_types[] = {
    {
        .name = TYPE_BCM4387_MACHINE,
        .parent = TYPE_MACHINE,
        .instance_size = sizeof(BCM4387MachineState),
        .instance_init = bcm4387_instance_init,
        .class_init = bcm4387_class_init,
        .interfaces = arm_machine_interfaces,
    },
};

DEFINE_TYPES(bcm4387_machine_types)
