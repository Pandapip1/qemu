/*
 * BCM4387 PMU core (0x827 rev 39) at 0x18012000: the resource manager that
 * powers and clocks the chip, and the always-on register files.
 *
 * Register layout and names: bcmdhd 101.10 sbchipc.h pmuregs_t. [B]
 * 4387 resource numbers: sbchipc.h RES4387_* (HT_AVAIL 28, ALP_AVAIL 19,
 * ARMCLK_AVAIL 27, MACPHY_CLK_MAIN/AUX/SCAN 30/29/24, CORE_RDY_MAIN/AUX/SCAN
 * 22/18/13, PWRSW_MAIN/AUX/SCAN 14/11/12, CORE_RDY_BTMAIN 9, CORE_RDY_DIG
 * 17). [B]
 *
 * Modelled:
 *  - Resources 0..30. A resource is requested by min_res_mask, by one of
 *    the requesters (bcm4387_pmu_request: srpwr power domains, d11 MAC
 *    clock requests, the external/BT side, ChipCommon), or by the mask of
 *    a resource request timer while its force ALP/HT/HQ flags are set:
 *    res_req (the ARM's, always asserted; its handoff
 *    mask/timer are stock's 0x160880df (= si_pmu_rsrc_ht_avail_clk_deps |
 *    si_pmu_rsrc_cb_ready_deps over the run-14 table) / 0x15000000, taken
 *    to persist like the rest of the PMU [I] (never read on silicon; the MAC
 *    ones do not persist, run 16); in stock at its attach read it reads
 *    0x17000000 with that mask and XTAL_HQ down (run 31) [F], so its
 *    FORCE_HQ flag does not bring up XTAL_HQ (notes/36 §6's inference,
 *    refuted by run 31)), mac_res_req 0/1/2 while that d11
 *    slice's MAC asserts its clock request (see d11_mac_request in
 *    bcm4387.c). Stock programs the main one in si_lhl_timer_config (RAM
 *    0x82b0be): mac_res_req_mask 0x56cac1ff = MACPHY_CLK_MAIN and its
 *    dependencies (si_pmu_rsrc_macphy_clk_deps), wait for SLOW_WR, timer
 *    0x15000000. [B] flags/names, [I] the request condition.
 *    The MAC timers are reset by a reload (run 16: WAKE and the PLL
 *    request with no programming got nothing). [F] For min_res_mask and the requesters the PMU brings up the
 *    dependency closure of everything requested (res_dep_mask table),
 *    limited to max_res_mask, and drops the rest; the same for a request
 *    timer's mask. [F] run 38 (Device MMIO): mask 0x40000000 (MACPHY_CLK_MAIN
 *    alone) was granted with its dependencies (0x5ecefbff), as was
 *    0x56cac1ff. The "mask honoured without closure" rule fitted to runs
 *    23, 26-29 is withdrawn: those ran with Normal-memory MMIO. A timer
 *    reads bit 25 (REQ_ACTIVE) while its request is asserted and every
 *    masked resource is up. [I] fits stock's 0x17000000 (run 31) and our
 *    0x15000000 read-backs before the MAC asks. [F] Transitions complete at
 *    once (res_pending reads 0); timing is not modelled. [B] semantics
 *    (bcmdhd si_pmu_res_deps, si_pmu_res_init), [F] check: the closure of
 *    the run-14 table reproduces every res_state seen on silicon:
 *      handoff 0x1e0a83df = closure(BTMAIN, DIG, ARMCLK, HT),
 *      + srpwr 0x1e: + CORE_RDY_MAIN/AUX/SCAN -> 0x1e4efbdf (run 11),
 *      + d11 main PHY clock: + MACPHY_CLK_MAIN -> 0x5ecefbff (run 11, stock).
 *  - The external requester at handoff is BTMAIN|DIG|ARMCLK|HT: BT core
 *    ready from the BT side, the ARM/backplane clocks of the WLAN side. [I]
 *  - Indirect files: res_table_sel selects the entry of res_dep_mask /
 *    res_updn_timer; chipcontrol, regcontrol and pllcontrol behind their
 *    address/data pairs. They are not reset by a firmware reload (queue run
 *    15 read stock's values at our handoff), so they hold stock's run-14
 *    values at handoff. [F] This is the reload-persistent state: a firmware
 *    reload (brcmfmac's ChipCommon watchdog with PCIe function 0 kept)
 *    resets the cores and slice domains but not the PMU or GCI/LHL
 *    (notes/36 §3.1: handoff pmucontrol 0x380, min 1, max 0x7fffffff,
 *    pmucontrol_ext 0x14291 are stock's values) [F], so every boot of our
 *    firmware starts on stock's tables, including the res_dep_mask table
 *    whose closure of bit 30 is 0x56cac1ff. A dep_mask write re-evaluates
 *    the resources. [B]
 *    pllcontrol is committed by pmucontrol.PLLCTL_UPD (self-clearing). [B]
 *  - Always-on write synchroniser: until the firmware itself writes
 *    pmucontrol_ext with FAST_TRANS_ENAB (bit 0) set, a write to an index register
 *    (PMU 0x620/0x650/0x658/0x660, GCI 0x40) is posted. It lands when the
 *    register is read back, or after the next read of the always-on block
 *    has returned (the old state). A second write to the same register before it has
 *    been read back is dropped and the register stops taking writes; after
 *    that srpwr requests are not acted on either. [F] queue run 15 (our
 *    write-index/read-data dump at handoff: first read old index, all later
 *    reads the first index written, srpwr dead afterwards); stock always
 *    reads back (ai_corereg) until it sets FAST_TRANS_ENAB (RAM 0x82bf93),
 *    then writes res_table_sel without read-back. [I] mechanism. The gate
 *    is not the register bit: pmucontrol_ext already reads 0x14291
 *    (FAST_TRANS set) at handoff (run 30) [F], yet run 15's writes were
 *    posted. What really separates run 15 from stock is unknown [U]; the
 *    model keys it on the firmware's own FAST_TRANS write. [I]
 *  - pmustatus PST_SLOW_WR_PENDING: set while a posted index write is
 *    pending, and for 3 ILP cycles after a write to an ILP-domain register
 *    (pmuwatchdog, res_req_timer, mac_res_req_timer 0/1/2) made while the
 *    register bit FAST_TRANS_ENAB is clear; a second such write in that
 *    window is dropped (pmu_ilp_reg). [B] register list and status bit,
 *    [I] FAST_TRANS gating, duration and the drop. With stock's 0x14291 at
 *    handoff it never pends for our firmware (run 30: 0 polls). [F]
 *  - pmucontrol_ext FASTLPO_ENAB (bit 7) starts the 1 MHz FLL LPO; its lock
 *    shows in GCI chipstatus[15] bit 1 (GCI_CS_4387_FLL1MHZ_LOCK_MASK). [B]
 *    Lock is immediate. [I]
 *  - pmutimer counts ILP (32.768 kHz) cycles. [B]
 *  - core_cap_ext: 3 MAC resource request timers. [B] field, [I] count.
 *  - every other register in pmu_regs[] is plain storage. [U]
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/arm/bcm4387.h"
#include "trace.h"

OBJECT_DECLARE_SIMPLE_TYPE(Bcm4387PmuState, BCM4387_PMU)

#define NRES            31
#define RES(n)          (1u << (n))
#define RES_BTMAIN      9
#define RES_CORE_RDY_SCAN 13
#define RES_DIG         17
#define RES_CORE_RDY_AUX 18
#define RES_ALP         19
#define RES_CORE_RDY_MAIN 22
#define RES_MACPHY_SCAN 24
#define RES_ARMCLK      27
#define RES_HT          28
#define RES_MACPHY_AUX  29
#define RES_MACPHY_MAIN 30

#define PMU_CONTROL     0x600
#define PMU_CAPS        0x604
#define PMU_STATUS      0x608
#define PMU_RES_STATE   0x60c
#define PMU_RES_PENDING 0x610
#define PMU_TIMER       0x614
#define PMU_MIN_RES     0x618
#define PMU_MAX_RES     0x61c
#define PMU_RES_SEL     0x620
#define PMU_RES_DEP     0x624
#define PMU_RES_UPDN    0x628
#define PMU_REQ_TIMER   0x644
#define PMU_REQ_MASK    0x648
#define PMU_CAP_EXT     0x64c
#define PMU_CC_ADDR     0x650
#define PMU_CC_DATA     0x654
#define PMU_REG_ADDR    0x658
#define PMU_REG_DATA    0x65c
#define PMU_PLL_ADDR    0x660
#define PMU_PLL_DATA    0x664
#define PMU_CTL_EXT     0x6d8

#define PCTL_PLL_PLLCTL_UPD     0x400
#define PCTL_EXT_FAST_TRANS     0x001
#define PCTL_EXT_FASTLPO_ENAB   0x080
#define PST_HTAVAIL     0x004
#define PST_ALPAVAIL    0x008
#define PST_SBCLKST     0x030
#define PST_SBCLKST_ALP 0x020
#define PST_SBCLKST_HT  0x030
#define PST_SLOW_WR_PENDING 0x400

#define NFILE           32
enum { F_CHIPCTL, F_REGCTL, F_PLLCTL, F_DEP, F_UPDN, NFILES };

/* Stock's values at its first PHY write (run 14) = state at handoff [F] */
static const struct { uint8_t f, idx; uint32_t v; } pmu_handoff[] = {
    { F_CHIPCTL, 1, 0x08800a00 }, { F_CHIPCTL, 2, 0xbc01b8fc },
    { F_CHIPCTL, 6, 0x40003011 }, { F_CHIPCTL, 7, 0x00200000 },
    { F_CHIPCTL, 8, 0x00000000 }, { F_CHIPCTL, 10, 0x42100102 },
    { F_CHIPCTL, 13, 0x00840888 }, { F_CHIPCTL, 17, 0x0020080b },
    { F_REGCTL, 1, 0x00400000 }, { F_REGCTL, 4, 0x00000e00 },
    { F_REGCTL, 5, 0x00000003 }, { F_REGCTL, 6, 0x0000010a },
    { F_REGCTL, 7, 0x20000000 }, { F_REGCTL, 8, 0x00000090 },
    { F_REGCTL, 13, 0x001f0e00 }, { F_REGCTL, 14, 0x00840000 },
    { F_REGCTL, 16, 0x00098003 },
    { F_PLLCTL, 2, 0x00240c06 },
};
static const uint32_t dep_handoff[NRES] = {
    0x00000000, 0x00000005, 0x00000001, 0x00000007, 0x0000000f, 0x000000c7,
    0x00000007, 0x00000047, 0x060000df, 0x000000cf, 0x000000c7, 0x000000cf,
    0x000000cf, 0x060010df, 0x000000cf, 0x000000c7, 0x00000000, 0x060001df,
    0x060209df, 0x000080c7, 0x060609df, 0x060030df, 0x060241df, 0x064241df,
    0x1628b0df, 0x0400001f, 0x0000001f, 0x000880cf, 0x000880cf, 0x161e89ff,
    0x16cac1ff,
};
static const uint32_t updn_handoff[NRES] = {
    0x00220022, 0x00960001, 0x00960022, 0x00320022, 0x00010001, 0x00210021,
    0x03e80033, 0x00220022, 0x01320172, 0x00220022, 0x00bd0022, 0x01c201e2,
    0x019201b2, 0x00220022, 0x02220242, 0x00220022, 0x00220022, 0x00220044,
    0x00220044, 0x00220044, 0x00650022, 0x00650022, 0x00220022, 0x00650022,
    0x00220022, 0x000a0033, 0x006400ca, 0x00a80022, 0x00a80022, 0x00220022,
    0x00220022,
};

static const Bcm4387Reg pmu_regs[] = {
    { 0x600, "pmucontrol", 0x00000380, ~0u, "F handoff; B PLLCTL_UPD" },
    { 0x604, "pmucapabilities", 0x9c565f27, 0, "F" },
    { 0x608, "pmustatus", 0x0001613e, 0, "F handoff; B HT/ALP/SBCLK bits" },
    { 0x618, "min_res_mask", 0x00000001, ~0u, "F handoff; B" },
    { 0x61c, "max_res_mask", 0x7fffffff, ~0u, "F handoff; B" },
    { 0x634, "pmuwatchdog", 0, ~0u, "B name; U (no reset modelled)" },
    { 0x644, "res_req_timer", 0x15000000, ~(1u << 25),
      "B; I handoff = stock's (persists); I asserted on LHL ARM timer expiry" },
    { 0x648, "res_req_mask", 0x160880df, ~0u,
      "B; I handoff = stock's value over the run-14 table (persists)" },
    { 0x64c, "core_cap_ext", 2u << 20, 0, "B field; I 3 MAC req timers" },
    { 0x630, "clkstretch", 0, ~0u, "B name; U" },
    { 0x688, "mac_res_req_timer", 0, ~(1u << 25), "B; I force flags request the mask" },
    { 0x68c, "mac_res_req_mask", 0, ~0u, "B" },
    { 0x6d8, "pmucontrol_ext", 0x00014291, ~0u, "F handoff (run 30); B bits" },
    { 0x6dc, "slowclkperiod", 0, ~0u, "B name; U" },
    { 0x6f0, "mac_res_req_timer1", 0, ~(1u << 25), "B; I as mac_res_req_timer" },
    { 0x6f4, "mac_res_req_mask1", 0, ~0u, "B" },
    { 0x700, "pmuintmask0", 0, ~0u, "B name; U (no interrupt)" },
    { 0x720, "rsrc_event0", 0, ~0u, "B name; U" },
    { 0x738, "mac_res_req_timer2", 0, ~(1u << 25), "B; I as mac_res_req_timer" },
    { 0x73c, "mac_res_req_mask2", 0, ~0u, "B" },
    { 0x780, "pmuintctrl0", 0, ~0u, "B name; U" },
    { 0x7e8, "core_cap_ext1", 0, 0, "B name; U value (0 assumed)" },
    { 0x804, "rsrc_substate_trans_tmr", 0, ~0u, "B name; U (per res_table_sel)" },
};
#define PMU_NREGS ARRAY_SIZE(pmu_regs)

/* always-on index registers whose writes go through the synchroniser */
static const hwaddr sync_regs[] = {
    BCM4387_PMU_BASE + PMU_RES_SEL, BCM4387_PMU_BASE + PMU_CC_ADDR,
    BCM4387_PMU_BASE + PMU_REG_ADDR, BCM4387_PMU_BASE + PMU_PLL_ADDR,
    BCM4387_GCI_BASE + 0x40,
};
#define NSYNC ARRAY_SIZE(sync_regs)

typedef struct SyncSlot {
    bool pending, unread, wedged;
    uint32_t val;
    Bcm4387SyncApply apply;
    void *opaque;
} SyncSlot;

struct Bcm4387PmuState {
    SysBusDevice parent_obj;
    MemoryRegion mmio;

    uint32_t reg[PMU_NREGS];
    uint32_t file[NFILES][NFILE];
    uint32_t known[NFILES];     /* entries with a known value */
    uint32_t pll_committed[NFILE];
    uint32_t sel[4];            /* res_table_sel, cc/reg/pll address */
    uint32_t req[BCM4387_REQ_N];
    bool mac_active[3];         /* d11 slice clock request asserted */
    uint32_t res_state;
    SyncSlot sync[NSYNC];
    bool sync_any_wedged;
    bool aon_fast;              /* firmware set FAST_TRANS_ENAB itself */
    int64_t slow_wr_until;      /* PST_SLOW_WR_PENDING until (ns, virtual) */
};

static uint32_t *pmu_reg(Bcm4387PmuState *s, hwaddr off)
{
    int i = bcm4387_reg_index(pmu_regs, PMU_NREGS, off);

    return i < 0 ? NULL : &s->reg[i];
}

uint32_t bcm4387_pmu_reg(Bcm4387PmuState *s, hwaddr off)
{
    uint32_t *r = pmu_reg(s, off);

    return r ? *r : 0;
}

/* ---- resource manager ----------------------------------------------------- */

static uint32_t res_closure(Bcm4387PmuState *s, uint32_t m)
{
    uint32_t prev;

    do {
        prev = m;
        for (int r = 0; r < NRES; r++) {
            if (m & RES(r)) {
                m |= s->file[F_DEP][r];
            }
        }
    } while (m != prev);
    return m;
}

/* {timer, mask}: res_req, mac_res_req (main), 1 (aux), 2 (scan) */
static const hwaddr req_timers[][2] = {
    { PMU_REQ_TIMER, PMU_REQ_MASK }, { 0x688, 0x68c }, { 0x6f0, 0x6f4 },
    { 0x738, 0x73c },
};

static void res_update(Bcm4387PmuState *s)
{
    uint32_t want = *pmu_reg(s, PMU_MIN_RES), old = s->res_state;

    for (int i = 0; i < BCM4387_REQ_N; i++) {
        want |= s->req[i];
    }
    /*
     * Resource request timers: the force ALP/HT/HQ flags (bits 26..28,
     * MAC_RSRC_REQ_TIMER_FORCE_*; PRRT_*_REQ << 14) request the timer's
     * mask while set; a MAC timer only while its slice's MAC asserts its
     * clock request (d11_mac_request in bcm4387.c). [B] flags, [I] request
     * held while a flag is set.
     */
    for (int i = 0; i < ARRAY_SIZE(req_timers); i++) {
        if ((i == 0 || s->mac_active[i - 1]) &&
            (*pmu_reg(s, req_timers[i][0]) & (7u << 26))) {
            want |= *pmu_reg(s, req_timers[i][1]);
        }
    }
    s->res_state = res_closure(s, want) & *pmu_reg(s, PMU_MAX_RES) &
                   (RES(NRES) - 1);
    if (s->res_state != old) {
        trace_bcm4387_pmu_res_state(old, s->res_state, want);
    }
}

static bool req_active(Bcm4387PmuState *s, hwaddr timer)
{
    for (int i = 0; i < ARRAY_SIZE(req_timers); i++) {
        if (req_timers[i][0] == timer) {
            uint32_t m = *pmu_reg(s, req_timers[i][1]) &
                         *pmu_reg(s, PMU_MAX_RES) & (RES(NRES) - 1);

            return (i == 0 || s->mac_active[i - 1]) &&
                   (*pmu_reg(s, timer) & (7u << 26)) && !(m & ~s->res_state);
        }
    }
    return false;
}

void bcm4387_pmu_request(Bcm4387PmuState *s, Bcm4387PmuReq who,
                         uint32_t mask)
{
    if (s->req[who] != mask) {
        trace_bcm4387_pmu_request(who, s->req[who], mask);
        s->req[who] = mask;
        res_update(s);
    }
}

void bcm4387_pmu_mac_active(Bcm4387PmuState *s, int n, bool on)
{
    if (s->mac_active[n] != on) {
        trace_bcm4387_pmu_mac_active(n, on);
        s->mac_active[n] = on;
        res_update(s);
    }
}

uint32_t bcm4387_pmu_res_state(Bcm4387PmuState *s)
{
    return s->res_state;
}

bool bcm4387_pmu_res_up(Bcm4387PmuState *s, unsigned res)
{
    return s->res_state & RES(res);
}

bool bcm4387_pmu_fll_locked(Bcm4387PmuState *s)
{
    return *pmu_reg(s, PMU_CTL_EXT) & PCTL_EXT_FASTLPO_ENAB;
}

static uint32_t pmu_status(Bcm4387PmuState *s)
{
    uint32_t v = *pmu_reg(s, PMU_STATUS) &
                 ~(PST_HTAVAIL | PST_ALPAVAIL | PST_SBCLKST | PST_SLOW_WR_PENDING);
    bool pend = false;

    for (int i = 0; i < NSYNC; i++) {
        pend |= s->sync[i].pending;
    }
    if (bcm4387_pmu_res_up(s, RES_HT)) {
        v |= PST_HTAVAIL | PST_SBCLKST_HT;
    } else if (bcm4387_pmu_res_up(s, RES_ALP)) {
        v |= PST_SBCLKST_ALP;
    }
    if (bcm4387_pmu_res_up(s, RES_ALP)) {
        v |= PST_ALPAVAIL;
    }
    pend |= qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) < s->slow_wr_until;
    return v | (pend ? PST_SLOW_WR_PENDING : 0);
}

/*
 * ILP-domain registers: "if such a register is written, it takes multiple
 * ILP clocks for the PMU block to absorb the write. During that time the
 * 'SlowWritePending' bit in the PMUStatus register is set" (bcmdhd
 * siutils.c PMUREGS_ILP_SENSITIVE: pmutimer, pmuwatchdog, res_req_timer)
 * [B]; the MAC timers are written the same way (HND_PMU_SYNC_WR, hndlhl.c
 * si_lhl_timer_config) [B]. Not the masks: stock writes mac_res_req_mask
 * with no wait. Only while FAST_TRANS_ENAB is clear, for 3 ILP cycles. [I]
 * A second such write while the first is pending is dropped (the register
 * keeps, and reads back, its old value). bcmdhd calls the hazard a
 * "backplane stall" [B]; silicon has no double write here, but the one
 * double always-on write it has (queue run 15, index registers) was dropped,
 * not stalled [F], so dropping is the model's rule. [I]
 */
static bool pmu_ilp_reg(hwaddr off)
{
    switch (off) {
    case 0x634: case PMU_REQ_TIMER: case 0x688: case 0x6f0: case 0x738:
        return true;
    }
    return false;
}

#define SLOW_WR_ILP_CYCLES 3


/* ---- always-on write synchroniser ---------------------------------------- */

static int sync_slot(hwaddr addr)
{
    for (int i = 0; i < NSYNC; i++) {
        if (sync_regs[i] == addr) {
            return i;
        }
    }
    return -1;
}

static void sync_land(Bcm4387PmuState *s, int i)
{
    SyncSlot *k = &s->sync[i];

    if (k->pending) {
        k->pending = false;
        trace_bcm4387_aon_sync_land(sync_regs[i], k->val);
        k->apply(k->opaque, sync_regs[i], k->val);
    }
}

static void sync_land_all(Bcm4387PmuState *s)
{
    for (int i = 0; i < NSYNC; i++) {
        sync_land(s, i);
    }
}

bool bcm4387_pmu_sync_wedged(Bcm4387PmuState *s)
{
    return s->sync_any_wedged;
}

void bcm4387_pmu_sync_post(Bcm4387PmuState *s, Bcm4387SyncApply apply,
                           void *opaque, hwaddr addr, uint32_t val)
{
    int i = sync_slot(addr);
    SyncSlot *k;

    assert(i >= 0);
    k = &s->sync[i];
    if (k->wedged) {
        trace_bcm4387_aon_sync_drop(addr, val);
        return;
    }
    if (s->aon_fast) {
        apply(opaque, addr, val);
        return;
    }
    if (k->unread) {
        qemu_log_mask(LOG_GUEST_ERROR, "bcm4387: always-on write 0x%08"
                      HWADDR_PRIx " <- 0x%x before the previous one was read "
                      "back: dropped, register wedged pc~0x%08x\n", addr, val,
                      bcm4387_pc());
        trace_bcm4387_aon_sync_wedge(addr, val);
        k->wedged = true;
        s->sync_any_wedged = true;
        return;
    }
    k->pending = k->unread = true;
    k->val = val;
    k->apply = apply;
    k->opaque = opaque;
}

/* A read of always-on register addr is about to be served. */
void bcm4387_pmu_sync_read(Bcm4387PmuState *s, hwaddr addr)
{
    int i = sync_slot(addr);

    if (i >= 0) {
        sync_land(s, i);
        s->sync[i].unread = false;
    }
}

/* A read of the always-on block has been served: posted writes land. */
void bcm4387_pmu_sync_read_done(Bcm4387PmuState *s)
{
    sync_land_all(s);
}

/* ---- registers -------------------------------------------------------------- */

static int file_of(hwaddr off, int *sel)
{
    switch (off) {
    case PMU_RES_DEP:   *sel = 0; return F_DEP;
    case PMU_RES_UPDN:  *sel = 0; return F_UPDN;
    case PMU_CC_DATA:   *sel = 1; return F_CHIPCTL;
    case PMU_REG_DATA:  *sel = 2; return F_REGCTL;
    case PMU_PLL_DATA:  *sel = 3; return F_PLLCTL;
    }
    return -1;
}

static int sel_of(hwaddr off)
{
    switch (off) {
    case PMU_RES_SEL:   return 0;
    case PMU_CC_ADDR:   return 1;
    case PMU_REG_ADDR:  return 2;
    case PMU_PLL_ADDR:  return 3;
    }
    return -1;
}

static const char *const file_name[NFILES] = {
    "chipcontrol", "regcontrol", "pllcontrol", "res_dep_mask", "res_updn_timer",
};

static void sel_apply(void *opaque, hwaddr addr, uint32_t val)
{
    Bcm4387PmuState *s = opaque;
    int k = sel_of(addr - BCM4387_PMU_BASE);

    trace_bcm4387_pmu_select(file_name[k == 0 ? F_DEP : k - 1], val);
    s->sel[k] = val;
}

static MemTxResult pmu_read(void *opaque, hwaddr off, uint64_t *data,
                            unsigned size, MemTxAttrs attrs)
{
    Bcm4387PmuState *s = opaque;
    uint32_t v, *r;
    int f, k;

    if (size != 4) {
        return bcm4387_unknown("pmu", BCM4387_PMU_BASE, off, false, data, size);
    }
    bcm4387_mmio_attr(BCM4387_PMU_BASE + off, false);
    bcm4387_pmu_sync_read(s, BCM4387_PMU_BASE + off);
    if ((k = sel_of(off)) >= 0) {
        v = s->sel[k];
    } else if ((f = file_of(off, &k)) >= 0) {
        uint32_t idx = s->sel[k] & 0xff;

        if (idx >= NFILE || !(s->known[f] & BIT(idx))) {
            qemu_log_mask(LOG_GUEST_ERROR, "bcm4387-pmu: unknown %s[%u] read "
                          "(0xdead) pc~0x%08x lr~0x%08x\n", file_name[f], idx,
                          bcm4387_pc(), bcm4387_lr());
            v = 0xdead;
        } else {
            v = s->file[f][idx];
        }
    } else {
        switch (off) {
        case PMU_STATUS:
            v = pmu_status(s);
            break;
        case PMU_RES_STATE:
            v = s->res_state;
            break;
        case PMU_RES_PENDING:
            v = 0;
            break;
        case PMU_REQ_TIMER: case 0x688: case 0x6f0: case 0x738:
            v = *pmu_reg(s, off) | (req_active(s, off) ? (1u << 25) : 0);
            break;
        case PMU_TIMER:
            v = muldiv64(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL),
                         BCM4387_ILP_HZ, NANOSECONDS_PER_SECOND);
            break;
        default:
            r = pmu_reg(s, off);
            if (!r) {
                return bcm4387_unknown("pmu", BCM4387_PMU_BASE, off, false, data, size);
            }
            v = *r;
        }
    }
    *data = v;
    trace_bcm4387_pmu_read(off, v);
    bcm4387_pmu_sync_read_done(s);
    return MEMTX_OK;
}

static MemTxResult pmu_write(void *opaque, hwaddr off, uint64_t val,
                             unsigned size, MemTxAttrs attrs)
{
    Bcm4387PmuState *s = opaque;
    uint32_t *r;
    int f, k;

    if (size != 4) {
        return bcm4387_unknown("pmu", BCM4387_PMU_BASE, off, true, &val, size);
    }
    bcm4387_mmio_attr(BCM4387_PMU_BASE + off, true);
    trace_bcm4387_pmu_write(off, val);
    if (sel_of(off) >= 0) {
        bcm4387_pmu_sync_post(s, sel_apply, s, BCM4387_PMU_BASE + off, val);
        return MEMTX_OK;
    }
    if ((f = file_of(off, &k)) >= 0) {
        uint32_t idx = s->sel[k] & 0xff;

        if (idx < NFILE) {
            trace_bcm4387_pmu_file(file_name[f], idx, s->file[f][idx], val);
            s->file[f][idx] = val;
            s->known[f] |= BIT(idx);
            if (f == F_DEP) {
                res_update(s);
            }
        }
        return MEMTX_OK;
    }
    r = pmu_reg(s, off);
    if (!r) {
        return bcm4387_unknown("pmu", BCM4387_PMU_BASE, off, true, &val, size);
    }
    if (pmu_ilp_reg(off) && !(*pmu_reg(s, PMU_CTL_EXT) & PCTL_EXT_FAST_TRANS)) {
        int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

        if (now < s->slow_wr_until) {
            qemu_log_mask(LOG_GUEST_ERROR, "bcm4387-pmu: +0x%03" HWADDR_PRIx
                          " <- 0x%08" PRIx64 " while PST_SLOW_WR_PENDING: "
                          "dropped pc~0x%08x\n", off, val, bcm4387_pc());
            trace_bcm4387_pmu_slow_wr_drop(off, val);
            return MEMTX_OK;
        }
        s->slow_wr_until = now + muldiv64(SLOW_WR_ILP_CYCLES,
                                          NANOSECONDS_PER_SECOND,
                                          BCM4387_ILP_HZ);
    }
    *r = (*r & ~pmu_regs[r - s->reg].wmask) | (val & pmu_regs[r - s->reg].wmask);
    switch (off) {
    case PMU_CONTROL:
        if (val & PCTL_PLL_PLLCTL_UPD) {
            memcpy(s->pll_committed, s->file[F_PLLCTL], sizeof(s->pll_committed));
            trace_bcm4387_pmu_pll_commit();
            *r &= ~PCTL_PLL_PLLCTL_UPD;
        }
        break;
    case PMU_CTL_EXT:
        s->aon_fast |= val & PCTL_EXT_FAST_TRANS;
        break;
    case PMU_MIN_RES:
    case PMU_MAX_RES:
    case PMU_REQ_TIMER:
    case PMU_REQ_MASK:
    case 0x688: case 0x68c: case 0x6f0: case 0x6f4: case 0x738: case 0x73c:
        res_update(s);
        break;
    }
    return MEMTX_OK;
}

static const MemoryRegionOps pmu_ops = {
    .read_with_attrs = pmu_read,
    .write_with_attrs = pmu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void pmu_reset(DeviceState *dev)
{
    Bcm4387PmuState *s = BCM4387_PMU(dev);

    for (int i = 0; i < PMU_NREGS; i++) {
        s->reg[i] = pmu_regs[i].reset;
    }
    memset(s->file, 0, sizeof(s->file));
    memset(s->known, 0, sizeof(s->known));
    for (int i = 0; i < ARRAY_SIZE(pmu_handoff); i++) {
        s->file[pmu_handoff[i].f][pmu_handoff[i].idx] = pmu_handoff[i].v;
        s->known[pmu_handoff[i].f] |= BIT(pmu_handoff[i].idx);
    }
    for (int i = 0; i < NRES; i++) {
        s->file[F_DEP][i] = dep_handoff[i];
        s->file[F_UPDN][i] = updn_handoff[i];
    }
    s->known[F_DEP] = s->known[F_UPDN] = RES(NRES) - 1;
    memcpy(s->pll_committed, s->file[F_PLLCTL], sizeof(s->pll_committed));
    /* index registers as stock left them: regcontrol at 0xd (run 15) [F] */
    s->sel[0] = 0;
    s->sel[1] = 0;
    s->sel[2] = 0xd;
    s->sel[3] = 0;
    memset(s->sync, 0, sizeof(s->sync));
    s->sync_any_wedged = false;
    s->aon_fast = false;
    s->slow_wr_until = 0;
    memset(s->req, 0, sizeof(s->req));
    memset(s->mac_active, 0, sizeof(s->mac_active));
    s->req[BCM4387_REQ_EXTERNAL] = RES(RES_BTMAIN) | RES(RES_DIG) |
                                   RES(RES_ARMCLK) | RES(RES_HT);
    res_update(s);
}

static void pmu_init(Object *obj)
{
    Bcm4387PmuState *s = BCM4387_PMU(obj);

    memory_region_init_io(&s->mmio, obj, &pmu_ops, s, "bcm4387.pmu",
                          BCM4387_CORE_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
}

static void pmu_class_init(ObjectClass *oc, const void *data)
{
    device_class_set_legacy_reset(DEVICE_CLASS(oc), pmu_reset);
}

static const TypeInfo pmu_types[] = {
    {
        .name = TYPE_BCM4387_PMU,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(Bcm4387PmuState),
        .instance_init = pmu_init,
        .class_init = pmu_class_init,
    },
};
DEFINE_TYPES(pmu_types)

Bcm4387PmuState *bcm4387_pmu_create(hwaddr base)
{
    DeviceState *dev = qdev_new(TYPE_BCM4387_PMU);

    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, base);
    return BCM4387_PMU(dev);
}
