/*
 * EFR32xG22 LETIMER0, low energy timer (RM rev. 1.2 chapter 18), at
 * 0x4A000000 (non-secure 0x5A000000), IRQ 19, bus clock CMU_CLKEN0.LETIMER0
 * (bit 12).  Register layout and encodings: RM SS18.13-18.14 and the public
 * efr32bg22_letimer.h.
 *
 * Modelled:
 *  - the 24-bit down-counter clocked by EM23GRPACLK / 2^CTRL.CNTPRESC
 *    (SS18.4); prescaled edges are aligned to the LF clock's edges from
 *    time 0, like the RTCC and BURTC here;
 *  - the per-edge state machines of Figures 18.2-18.5: a running counter
 *    at 0 reloads TOP (CTRL.CNTTOPEN) or 0xFFFFFF; the edge that applies
 *    START does not decrement REP0; REPMODE FREE, ONESHOT (stop when REP0
 *    would go from 1 to 0), BUFFERED (REP1 into REP0 once per write,
 *    TOPBUFF into TOP with BUFTOP) and DOUBLE; START with CNT and REP0 at 0
 *    in buffered mode loads REP1 (SS18.3.4);
 *  - IF.UF when the counter reaches 0 (Figure 18.8), IF.COMP0/1 when it
 *    equals COMPn, IF.REP0/REP1 when a repeat counter reaches 0 or REP1 is
 *    loaded into REP0; IRQ = IF & IEN; IF is an INTFLAG register (only the
 *    SET/CLR aliases change it, RM SS4.2.4.4.6);
 *  - the outputs (SS18.7, Table 18.2): UFOAn NONE, TOGGLE (at underflow),
 *    PULSE (active for the edge the counter rests at 0), PWM (idle at
 *    underflow, active at a COMPn match), OPOLn, CMD.CTOn; in DOUBLE mode
 *    output n acts only while REPn != 0.  They reach the GPIO
 *    (GPIO_LETIMERROUTE) and the PRS (producers LETIMER0 CH0/CH1);
 *  - CMD START/STOP/CLEAR (CLEAR has priority and sets CNT to 0 with no
 *    underflow); the PRS consumers LETIMER0 START/STOP/CLEAR with
 *    PRSMODE's edge selection, acting on the next prescaled edge;
 *  - LFSYNC registers (CMD, CNT, TOP, REP0, REP1; SS18.13, RM SS4.2.4.4.4):
 *    a write takes effect after 3 LF clock edges, SYNCBUSY shows it, a read
 *    returns the pending value, a second write while pending is a bus
 *    fault.  [I] 3 edges: the RM gives 3-4 PERCLK cycles; the BURTC on
 *    silicon takes CNT/COMP/CMD writes at the third LF edge
 *    (owo-open-fw docs/radio-tables.md SS7.73).  [I] With EN clear the
 *    module has no clock to synchronise to and the write lands at once;
 *  - CMU_CLKEN0.LETIMER0 clear: reads and writes are bus errors (silicon:
 *    a gated LETIMER0 read faulted, SS7.73; writes [I] as TIMER/PRS).
 * [U] Not modelled: DEBUGRUN (the counter keeps running in a debug halt),
 * EM2/EM3 clock switching, the 2-3 cycle enable delay, the PRS input
 * synchroniser's exact delay.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/arm/efr32-letimer.h"

OBJECT_DECLARE_SIMPLE_TYPE(Efr32LetimerState, EFR32_LETIMER)

enum {
    LT_IPVERSION = 0x000,
    LT_EN = 0x004,
    LT_CTRL = 0x008,
    LT_CMD = 0x00c,
    LT_STATUS = 0x010,
    LT_CNT = 0x018,
    LT_COMP0 = 0x01c,
    LT_COMP1 = 0x020,
    LT_TOP = 0x024,
    LT_TOPBUFF = 0x028,
    LT_REP0 = 0x02c,
    LT_REP1 = 0x030,
    LT_IF = 0x034,
    LT_IEN = 0x038,
    LT_SYNCBUSY = 0x040,
    LT_PRSMODE = 0x050,
};

#define LT_IPVERSION_VALUE 0x0u
#define LT_CTRL_MASK 0x000f13ffu
#define LT_CTRL_REPMODE_MASK 0x3u
#define LT_REPMODE_FREE 0u
#define LT_REPMODE_ONESHOT 1u
#define LT_REPMODE_BUFFERED 2u
#define LT_REPMODE_DOUBLE 3u
#define LT_CTRL_UFOA_SHIFT(n) (2 + 2 * (n))
#define LT_CTRL_UFOA_MASK 0x3u
#define LT_UFOA_NONE 0u
#define LT_UFOA_TOGGLE 1u
#define LT_UFOA_PULSE 2u
#define LT_UFOA_PWM 3u
#define LT_CTRL_OPOL(n) (1u << (6 + (n)))
#define LT_CTRL_BUFTOP (1u << 8)
#define LT_CTRL_CNTTOPEN (1u << 9)
#define LT_CTRL_CNTPRESC_SHIFT 16
#define LT_CTRL_CNTPRESC_MASK 0xfu
#define LT_CMD_START (1u << 0)
#define LT_CMD_STOP (1u << 1)
#define LT_CMD_CLEAR (1u << 2)
#define LT_CMD_CTO(n) (1u << (3 + (n)))
#define LT_CMD_MASK 0x1fu
#define LT_STATUS_RUNNING (1u << 0)
#define LT_COUNT_MASK 0xffffffu
#define LT_REP_MASK 0xffu
#define LT_IF_COMP(n) (1u << (n))
#define LT_IF_UF (1u << 2)
#define LT_IF_REP0 (1u << 3)
#define LT_IF_REP1 (1u << 4)
#define LT_IF_MASK 0x1fu
#define LT_SYNCBUSY_CNT (1u << 0)
#define LT_SYNCBUSY_TOP (1u << 2)
#define LT_SYNCBUSY_REP0 (1u << 3)
#define LT_SYNCBUSY_REP1 (1u << 4)
/* SYNCBUSY bits 5-9: the CMD bits START, STOP, CLEAR, CTO0, CTO1. */
#define LT_SYNCBUSY_CMD_SHIFT 5
#define LT_PRSMODE_MASK 0x0ccc0000u
#define LT_PRSMODE_START_SHIFT 18
#define LT_PRSMODE_STOP_SHIFT 22
#define LT_PRSMODE_CLEAR_SHIFT 26
#define LT_PRSMODE_RISING 1u
#define LT_PRSMODE_FALLING 2u
#define LT_SYNC_LF_EDGES 3
#define LT_CLKEN0_LETIMER0 (1u << 12)
#define LT_OUTPUTS 2

/* LFSYNC registers with a pending write (efr32_letimer_pend). */
enum { LT_PEND_CMD, LT_PEND_CNT, LT_PEND_TOP, LT_PEND_REP0, LT_PEND_REP1,
       LT_PEND_COUNT };
static const uint32_t lt_pend_offset[LT_PEND_COUNT] = {
    LT_CMD, LT_CNT, LT_TOP, LT_REP0, LT_REP1,
};

struct Efr32LetimerState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    MemoryRegion mmio_ns;
    qemu_irq irq;
    qemu_irq out_line[LT_OUTPUTS];
    qemu_irq prs_line[LT_OUTPUTS];
    const Efr32SocLink *link;
    hwaddr base;
    QEMUTimer *timer;

    uint32_t en, ctrl, comp[2], top, topbuff, rep0, rep1, if_, ien, prsmode;
    uint32_t cnt;
    bool running;
    /* START/STOP/CLEAR seen, applied at the next prescaled edge. */
    bool start, stop;
    /* A REP1 write not yet loaded into REP0 (Figure 18.4's !REP1USED). */
    bool rep1_fresh;
    /* The prescaled edge the state above is at. */
    int64_t edge;
    /* The output stays active for this edge (UFOA = PULSE). */
    bool pulse[LT_OUTPUTS];
    bool out_active[LT_OUTPUTS];
    bool out_sent[LT_OUTPUTS];
    struct {
        bool valid;
        uint32_t value;
        int64_t lf_edge;
    } pend[LT_PEND_COUNT];
    bool prs_level[3];
};

static uint64_t lt_lf_hz(Efr32LetimerState *s)
{
    return efr32_link_tree_hz(s->link, EFR32_TREE_EM23GRPACLK);
}

static unsigned lt_presc(Efr32LetimerState *s)
{
    return (s->ctrl >> LT_CTRL_CNTPRESC_SHIFT) & LT_CTRL_CNTPRESC_MASK;
}

static int64_t lt_lf_edge_at(Efr32LetimerState *s, int64_t ns)
{
    uint64_t hz = lt_lf_hz(s);

    return hz ? muldiv64(ns, hz, NANOSECONDS_PER_SECOND) : 0;
}

static int64_t lt_lf_edge_ns(Efr32LetimerState *s, int64_t lf_edge)
{
    uint64_t hz = lt_lf_hz(s);

    return hz ? muldiv64_round_up(lf_edge, NANOSECONDS_PER_SECOND, hz)
              : INT64_MAX;
}

static int64_t lt_now_edge(Efr32LetimerState *s)
{
    return lt_lf_edge_at(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)) >>
           lt_presc(s);
}

static unsigned lt_repmode(Efr32LetimerState *s)
{
    return s->ctrl & LT_CTRL_REPMODE_MASK;
}

static unsigned lt_ufoa(Efr32LetimerState *s, unsigned n)
{
    return (s->ctrl >> LT_CTRL_UFOA_SHIFT(n)) & LT_CTRL_UFOA_MASK;
}

static void lt_update_outputs(Efr32LetimerState *s)
{
    for (unsigned n = 0; n < LT_OUTPUTS; n++) {
        bool active = lt_ufoa(s, n) == LT_UFOA_PULSE ? s->pulse[n]
                                                      : s->out_active[n];
        bool level = active ^ !!(s->ctrl & LT_CTRL_OPOL(n));

        if (level != s->out_sent[n]) {
            s->out_sent[n] = level;
            qemu_set_irq(s->out_line[n], level);
            qemu_set_irq(s->prs_line[n], level);
        }
    }
    qemu_set_irq(s->irq, !!(s->if_ & s->ien));
}

/* Output n acts on events: in DOUBLE mode only while REPn != 0. */
static bool lt_output_live(Efr32LetimerState *s, unsigned n)
{
    if (lt_repmode(s) != LT_REPMODE_DOUBLE) {
        return true;
    }
    return n ? s->rep1 != 0 : s->rep0 != 0;
}

static void lt_underflow(Efr32LetimerState *s)
{
    s->if_ |= LT_IF_UF;
    for (unsigned n = 0; n < LT_OUTPUTS; n++) {
        if (!lt_output_live(s, n)) {
            continue;
        }
        switch (lt_ufoa(s, n)) {
        case LT_UFOA_TOGGLE:
            s->out_active[n] = !s->out_active[n];
            break;
        case LT_UFOA_PULSE:
            s->pulse[n] = true;
            break;
        case LT_UFOA_PWM:
            s->out_active[n] = false;
            break;
        }
    }
}

static void lt_compare(Efr32LetimerState *s)
{
    for (unsigned n = 0; n < 2; n++) {
        if (s->cnt == (s->comp[n] & LT_COUNT_MASK)) {
            s->if_ |= LT_IF_COMP(n);
            if (lt_ufoa(s, n) == LT_UFOA_PWM && lt_output_live(s, n)) {
                s->out_active[n] = true;
            }
        }
    }
}

static uint32_t lt_top(Efr32LetimerState *s)
{
    return s->ctrl & LT_CTRL_CNTTOPEN ? s->top & LT_COUNT_MASK
                                      : LT_COUNT_MASK;
}

/* Load REP1 into REP0 (and TOPBUFF into TOP with BUFTOP), buffered mode. */
static void lt_load_buffers(Efr32LetimerState *s)
{
    s->rep0 = s->rep1;
    s->rep1_fresh = false;
    s->if_ |= LT_IF_REP0;
    if (s->ctrl & LT_CTRL_BUFTOP) {
        s->top = s->topbuff;
    }
}

/* The counter at 0 on an edge that is not the start edge (Figs 18.3-5). */
static void lt_wrap(Efr32LetimerState *s)
{
    switch (lt_repmode(s)) {
    case LT_REPMODE_FREE:
        s->cnt = lt_top(s);
        return;
    case LT_REPMODE_ONESHOT:
        if (s->rep0 < 2) {
            s->running = false;
            if (s->rep0) {
                s->if_ |= LT_IF_REP0;
            }
            s->rep0 = 0;
            return;
        }
        s->rep0--;
        s->cnt = lt_top(s);
        return;
    case LT_REPMODE_BUFFERED:
        if (s->rep0 < 2) {
            if (s->rep1_fresh && s->rep1) {
                lt_load_buffers(s);
                s->cnt = lt_top(s);
                return;
            }
            s->running = false;
            if (s->rep0) {
                s->if_ |= LT_IF_REP0;
            }
            s->rep0 = 0;
            return;
        }
        s->rep0--;
        s->cnt = lt_top(s);
        return;
    default: /* DOUBLE */
        if (s->rep0 < 2 && s->rep1 < 2) {
            s->running = false;
            s->if_ |= (s->rep0 ? LT_IF_REP0 : 0) | (s->rep1 ? LT_IF_REP1 : 0);
            s->rep0 = 0;
            s->rep1 = 0;
            return;
        }
        if (s->rep0) {
            s->if_ |= --s->rep0 ? 0 : LT_IF_REP0;
        }
        if (s->rep1) {
            s->if_ |= --s->rep1 ? 0 : LT_IF_REP1;
        }
        s->cnt = lt_top(s);
        return;
    }
}

/* One prescaled clock edge. */
static void lt_edge(Efr32LetimerState *s)
{
    bool start_edge = false;

    for (unsigned n = 0; n < LT_OUTPUTS; n++) {
        s->pulse[n] = false;
    }
    if (s->stop) {
        s->running = false;
    } else if (s->start) {
        s->running = true;
        start_edge = true;
    }
    s->start = s->stop = false;
    if (!s->running) {
        return;
    }
    if (s->cnt == 0) {
        if (start_edge) {
            if (lt_repmode(s) == LT_REPMODE_BUFFERED && !s->rep0 &&
                s->rep1) {
                lt_load_buffers(s);
            }
            s->cnt = lt_top(s);
        } else {
            lt_wrap(s);
            if (!s->running) {
                return;
            }
        }
    } else {
        s->cnt--;
        if (s->cnt == 0) {
            lt_underflow(s);
        }
    }
    lt_compare(s);
}

/* Commit LFSYNC writes whose third LF edge has come. */
static void lt_commit_pending(Efr32LetimerState *s, int64_t lf_edge)
{
    for (unsigned i = 0; i < LT_PEND_COUNT; i++) {
        uint32_t v;

        if (!s->pend[i].valid || s->pend[i].lf_edge > lf_edge) {
            continue;
        }
        s->pend[i].valid = false;
        v = s->pend[i].value;
        switch (i) {
        case LT_PEND_CMD:
            if (v & LT_CMD_CLEAR) {
                s->cnt = 0;
            }
            if (v & LT_CMD_STOP) {
                s->stop = true;
            } else if (v & LT_CMD_START) {
                s->start = true;
            }
            for (unsigned n = 0; n < LT_OUTPUTS; n++) {
                if (v & LT_CMD_CTO(n)) {
                    s->out_active[n] = false;
                    s->pulse[n] = false;
                }
            }
            break;
        case LT_PEND_CNT:
            s->cnt = v & LT_COUNT_MASK;
            break;
        case LT_PEND_TOP:
            s->top = v & LT_COUNT_MASK;
            break;
        case LT_PEND_REP0:
            s->rep0 = v & LT_REP_MASK;
            break;
        case LT_PEND_REP1:
            s->rep1 = v & LT_REP_MASK;
            s->rep1_fresh = true;
            break;
        }
    }
}

static int64_t lt_next_pending_lf_edge(Efr32LetimerState *s)
{
    int64_t next = INT64_MAX;

    for (unsigned i = 0; i < LT_PEND_COUNT; i++) {
        if (s->pend[i].valid) {
            next = MIN(next, s->pend[i].lf_edge);
        }
    }
    return next;
}

/*
 * The next prescaled edge after s->edge at which something other than a
 * plain decrement happens.
 */
static int64_t lt_next_event_edge(Efr32LetimerState *s)
{
    int64_t next = INT64_MAX;
    int64_t pend = lt_next_pending_lf_edge(s);

    if (pend != INT64_MAX) {
        next = pend >> lt_presc(s);
        if ((next << lt_presc(s)) < pend) {
            next++;
        }
        next = MAX(next, s->edge + 1);
    }
    if (s->start || s->stop || s->pulse[0] || s->pulse[1]) {
        return s->edge + 1;
    }
    if (!s->running) {
        return next;
    }
    if (s->cnt == 0) {
        return s->edge + 1;
    }
    next = MIN(next, s->edge + s->cnt);
    for (unsigned n = 0; n < 2; n++) {
        uint32_t comp = s->comp[n] & LT_COUNT_MASK;

        if (comp < s->cnt) {
            next = MIN(next, s->edge + (s->cnt - comp));
        }
    }
    return next;
}

/* Bring the state up to the current prescaled edge. */
static void lt_advance(Efr32LetimerState *s)
{
    int64_t now;

    if (!(s->en & 1) || !lt_lf_hz(s)) {
        return;
    }
    now = lt_now_edge(s);
    while (s->edge < now) {
        int64_t event = lt_next_event_edge(s);

        if (event > now) {
            /* Plain decrements only, up to now. */
            if (s->running) {
                s->cnt -= now - s->edge;
            }
            s->edge = now;
            break;
        }
        if (s->running) {
            s->cnt -= event - 1 - s->edge;
        }
        s->edge = event;
        lt_commit_pending(s, event << lt_presc(s));
        lt_edge(s);
    }
}

static void lt_schedule(Efr32LetimerState *s)
{
    int64_t next;

    timer_del(s->timer);
    if (!(s->en & 1) || !lt_lf_hz(s)) {
        return;
    }
    next = lt_next_event_edge(s);
    if (next != INT64_MAX) {
        timer_mod(s->timer, lt_lf_edge_ns(s, next << lt_presc(s)));
    }
}

static void lt_sync(Efr32LetimerState *s)
{
    lt_advance(s);
    lt_update_outputs(s);
    lt_schedule(s);
}

static void lt_timer_expired(void *opaque)
{
    lt_sync(opaque);
}

static uint32_t lt_syncbusy(Efr32LetimerState *s)
{
    static const uint32_t bit[LT_PEND_COUNT] = {
        0, LT_SYNCBUSY_CNT, LT_SYNCBUSY_TOP, LT_SYNCBUSY_REP0,
        LT_SYNCBUSY_REP1,
    };
    uint32_t busy = 0;

    for (unsigned i = 0; i < LT_PEND_COUNT; i++) {
        if (!s->pend[i].valid) {
            continue;
        }
        busy |= i == LT_PEND_CMD
                    ? (s->pend[i].value & LT_CMD_MASK) << LT_SYNCBUSY_CMD_SHIFT
                    : bit[i];
    }
    return busy;
}

static bool lt_gated(Efr32LetimerState *s)
{
    return !efr32_link_bus_clock(s->link, EFR32_LINK_CLKEN0,
                                 LT_CLKEN0_LETIMER0);
}

static MemTxResult lt_read(void *opaque, hwaddr addr, uint64_t *data,
                           unsigned size, MemTxAttrs attrs)
{
    Efr32LetimerState *s = opaque;
    hwaddr offset = addr & ~(hwaddr)EFR32_ALIAS_MASK;
    uint32_t v = 0;

    efr32_link_bus_access(s->link, s->base + addr, size, false);
    *data = 0;
    if (lt_gated(s)) {
        return MEMTX_ERROR;
    }
    lt_sync(s);
    for (unsigned i = 1; i < LT_PEND_COUNT; i++) {
        if (s->pend[i].valid && offset == lt_pend_offset[i]) {
            *data = s->pend[i].value;
            return MEMTX_OK;
        }
    }
    switch (offset) {
    case LT_IPVERSION:
        v = LT_IPVERSION_VALUE;
        break;
    case LT_EN:
        v = s->en;
        break;
    case LT_CTRL:
        v = s->ctrl;
        break;
    case LT_STATUS:
        v = s->running || s->start ? LT_STATUS_RUNNING : 0;
        break;
    case LT_CNT:
        v = s->cnt;
        break;
    case LT_COMP0:
    case LT_COMP1:
        v = s->comp[(offset - LT_COMP0) / 4];
        break;
    case LT_TOP:
        v = s->top;
        break;
    case LT_TOPBUFF:
        v = s->topbuff;
        break;
    case LT_REP0:
        v = s->rep0;
        break;
    case LT_REP1:
        v = s->rep1;
        break;
    case LT_IF:
        v = s->if_;
        break;
    case LT_IEN:
        v = s->ien;
        break;
    case LT_SYNCBUSY:
        v = lt_syncbusy(s);
        break;
    case LT_PRSMODE:
        v = s->prsmode;
        break;
    }
    *data = v;
    return MEMTX_OK;
}

/* An LFSYNC write: pending for LT_SYNC_LF_EDGES LF edges while enabled. */
static bool lt_pend(Efr32LetimerState *s, unsigned which, uint32_t value)
{
    if (s->pend[which].valid) {
        return false;
    }
    s->pend[which].valid = true;
    s->pend[which].value = value;
    s->pend[which].lf_edge = (s->en & 1) && lt_lf_hz(s)
        ? lt_lf_edge_at(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)) +
              LT_SYNC_LF_EDGES
        : INT64_MIN;
    if (s->pend[which].lf_edge == INT64_MIN) {
        lt_commit_pending(s, INT64_MAX);
    }
    return true;
}

static MemTxResult lt_write(void *opaque, hwaddr addr, uint64_t data,
                            unsigned size, MemTxAttrs attrs)
{
    Efr32LetimerState *s = opaque;
    hwaddr offset = addr & ~(hwaddr)EFR32_ALIAS_MASK;
    unsigned alias = (addr & EFR32_ALIAS_MASK) >> EFR32_ALIAS_SHIFT;
    uint32_t value = data;
    bool ok = true;

    efr32_link_bus_access(s->link, s->base + addr, size, true);
    if (lt_gated(s)) {
        return MEMTX_ERROR;
    }
    lt_sync(s);
    switch (offset) {
    case LT_EN:
        if (!(efr32_alias_apply(addr, s->en, value) & 1) &&
            lt_next_pending_lf_edge(s) != INT64_MAX) {
            /* RM SS4.2.4.4.5: disabling with an LFSYNC write pending. */
            return MEMTX_ERROR;
        }
        s->en = efr32_alias_apply(addr, s->en, value) & 1;
        s->edge = lt_now_edge(s);
        if (!(s->en & 1)) {
            s->running = false;
            s->start = s->stop = false;
        }
        break;
    case LT_CTRL:
        s->ctrl = efr32_alias_apply(addr, s->ctrl, value) & LT_CTRL_MASK;
        s->edge = lt_now_edge(s);
        break;
    case LT_CMD:
        ok = lt_pend(s, LT_PEND_CMD, value & LT_CMD_MASK);
        break;
    case LT_CNT:
        ok = lt_pend(s, LT_PEND_CNT, efr32_alias_apply(addr, s->cnt, value) &
                                         LT_COUNT_MASK);
        break;
    case LT_TOP:
        ok = lt_pend(s, LT_PEND_TOP, efr32_alias_apply(addr, s->top, value) &
                                         LT_COUNT_MASK);
        break;
    case LT_REP0:
        ok = lt_pend(s, LT_PEND_REP0,
                     efr32_alias_apply(addr, s->rep0, value) & LT_REP_MASK);
        break;
    case LT_REP1:
        ok = lt_pend(s, LT_PEND_REP1,
                     efr32_alias_apply(addr, s->rep1, value) & LT_REP_MASK);
        break;
    case LT_COMP0:
    case LT_COMP1:
        s->comp[(offset - LT_COMP0) / 4] =
            efr32_alias_apply(addr, s->comp[(offset - LT_COMP0) / 4], value) &
            LT_COUNT_MASK;
        break;
    case LT_TOPBUFF:
        s->topbuff = efr32_alias_apply(addr, s->topbuff, value) &
                     LT_COUNT_MASK;
        break;
    case LT_IF:
        /* INTFLAG: only the SET and CLR aliases act (RM SS4.2.4.4.6). */
        if (alias == 1 || alias == 2) {
            s->if_ = efr32_alias_apply(addr, s->if_, value) & LT_IF_MASK;
        }
        break;
    case LT_IEN:
        s->ien = efr32_alias_apply(addr, s->ien, value) & LT_IF_MASK;
        break;
    case LT_PRSMODE:
        s->prsmode = efr32_alias_apply(addr, s->prsmode, value) &
                     LT_PRSMODE_MASK;
        break;
    default:
        break;
    }
    lt_sync(s);
    return ok ? MEMTX_OK : MEMTX_ERROR;
}

static const MemoryRegionOps lt_ops = {
    .read_with_attrs = lt_read,
    .write_with_attrs = lt_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {.min_access_size = 4, .max_access_size = 4},
};

/* PRS consumers START (0), STOP (1), CLEAR (2), with PRSMODE's edges. */
static void lt_prs_in(void *opaque, int n, int level)
{
    static const unsigned shift[3] = {
        LT_PRSMODE_START_SHIFT, LT_PRSMODE_STOP_SHIFT, LT_PRSMODE_CLEAR_SHIFT,
    };
    Efr32LetimerState *s = opaque;
    unsigned mode = (s->prsmode >> shift[n]) & 0x3u;
    bool rising = level && !s->prs_level[n];
    bool falling = !level && s->prs_level[n];

    s->prs_level[n] = level;
    if (!(s->en & 1) || !((rising && (mode & LT_PRSMODE_RISING)) ||
                          (falling && (mode & LT_PRSMODE_FALLING)))) {
        return;
    }
    lt_advance(s);
    switch (n) {
    case 0:
        s->start = true;
        break;
    case 1:
        s->stop = true;
        break;
    default:
        s->cnt = 0;
        break;
    }
    lt_sync(s);
}

static void lt_reset_hold(Object *obj, ResetType type)
{
    Efr32LetimerState *s = EFR32_LETIMER(obj);

    timer_del(s->timer);
    s->en = s->ctrl = s->top = s->topbuff = s->rep0 = s->rep1 = 0;
    s->comp[0] = s->comp[1] = 0;
    s->if_ = s->ien = s->prsmode = 0;
    s->cnt = 0;
    s->running = s->start = s->stop = s->rep1_fresh = false;
    s->edge = 0;
    memset(s->pend, 0, sizeof(s->pend));
    memset(s->pulse, 0, sizeof(s->pulse));
    memset(s->out_active, 0, sizeof(s->out_active));
}

static void lt_reset_exit(Object *obj, ResetType type)
{
    lt_update_outputs(EFR32_LETIMER(obj));
}

static void lt_init(Object *obj)
{
    Efr32LetimerState *s = EFR32_LETIMER(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->mmio, obj, &lt_ops, s, TYPE_EFR32_LETIMER,
                          0x4000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_out_named(dev, s->out_line, EFR32_LETIMER_OUT, LT_OUTPUTS);
    qdev_init_gpio_out_named(dev, s->prs_line, EFR32_LETIMER_PRS, LT_OUTPUTS);
    qdev_init_gpio_in_named(dev, lt_prs_in, EFR32_LETIMER_PRS_IN, 3);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, lt_timer_expired, s);
}

static void lt_class_init(ObjectClass *klass, void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = lt_reset_hold;
    rc->phases.exit = lt_reset_exit;
}

static const TypeInfo lt_info = {
    .name = TYPE_EFR32_LETIMER,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Efr32LetimerState),
    .instance_init = lt_init,
    .class_init = lt_class_init,
};

static void lt_register_types(void)
{
    type_register_static(&lt_info);
}
type_init(lt_register_types)

DeviceState *efr32_letimer_create(Object *soc, MemoryRegion *memory,
                                  const Efr32SocLink *link, hwaddr base,
                                  hwaddr ns_offset, qemu_irq irq)
{
    DeviceState *dev = qdev_new(TYPE_EFR32_LETIMER);
    Efr32LetimerState *s = EFR32_LETIMER(dev);

    object_property_add_child(soc, "letimer0", OBJECT(dev));
    s->link = link;
    s->base = base;
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    memory_region_add_subregion_overlap(memory, base, &s->mmio, 1);
    memory_region_init_alias(&s->mmio_ns, OBJECT(dev), "efr32-letimer-ns",
                             &s->mmio, 0, 0x4000);
    memory_region_add_subregion_overlap(memory, base + ns_offset, &s->mmio_ns,
                                        1);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, irq);
    return dev;
}
