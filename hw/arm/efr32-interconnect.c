/*
 * EFR32xG22 bus interconnect for the per-CPU clocks: which manager meets
 * which other where, and what an access costs.
 *
 * Topology.  RM rev. 1.2 SS4.2.1 (Figure 4.2): a multilayer AHB matrix
 * between the managers (the Cortex-M33's Code and System interfaces, LDMA)
 * and the subordinates flash, RAM, SEQRAM, RDMAILBOX and two AHB-to-APB
 * bridges (high and low frequency); round-robin arbitration, and "one wait
 * state is inserted for manager accesses occurring after a prolonged
 * inactive time" (SS4.2.1.1).  The radio subsystem's bus, the M0+'s
 * manager port and the radio register bridge are not in the RM; from the
 * cross-core measurements (owo-open-fw docs/radio-tables.md SS7.74 [F]):
 *  - the M0+ reaches main SRAM and the 0x5xxxxxxx peripherals (the same
 *    SRAM and peripherals the M33 uses) and has its own SEQRAM, radio
 *    registers and BUFC;
 *  - SEQRAM has one port shared by the M33 and the M0+'s instruction fetch
 *    and data; RAC/FRC/SYNTH/MODEM/AGC are one target (any of them contends
 *    with any other); BUFC is a separate target; TIMER1/CMU/PRS contend with
 *    each other (one HF APB bridge); flash and ICACHE traffic touches none
 *    of the M0+'s paths.
 * Modelled ports: SEQRAM, RADIO, BUFC, SRAM, HF APB, LF APB ([I] the
 * 0x48/0x4A groups, RTCC, LETIMER0, IADC0, I2C0, WDOG0, EUART0, the EM2
 * power domain, sit behind the LF bridge).  Managers: M33, M0+ (LDMA and
 * the debug AP generate no traffic in this model).
 *
 * An access costs: the manager's own path (core cycles), an idle wait
 * state, waiting for the port, and the port tenure (the time it holds the
 * port).  Per manager and target (stage-3 clocks, owo-open-fw SS7.74 and
 * SS7.75 single-core tables, [F]):
 *  M0+, cycles per access back to back (one isolated): RAC/FRC/SYNTH/
 *    MODEM 4.25 (5.0), BUFC 3.25 (4.0), SRAM 3.25, TIMER1 8.75, CMU 4.75.
 *    Isolated accesses cost 0.75 more: the RM's idle wait state, one
 *    radio-clock cycle here (8 accesses back to back pay it once per
 *    loop pass: 4.125 + 1/8 = 4.25; isolated 5.125).
 *  M33 from flash: SEQRAM 1.88, BUFC 1.88, radio 3.88, CMU/GPIO 3.88,
 *    TIMER1 11.88; from SRAM-resident code each costs 1.75 (SEQRAM, BUFC,
 *    radio) or 1.25 (CMU, TIMER1) more: the System interface carries the
 *    instruction fetch too (RM SS4.2).  An isolated M33 TIMER0 read costs
 *    the same as back to back (0.156 us, SS7.77), so the M33 shows no idle
 *    wait state.
 * The tenures split those totals: the port time the other manager loses
 * per access.  [I] fitted to SS7.74: the M33 loses 52 ns (2 radio cycles)
 * per M0+ radio access when it streams the radio (3.88 -> 4.86 cycles per
 * access with the M0+ streaming too, 4 M33 accesses per M0+ access), so
 * the M0+'s radio tenure is 2 radio cycles; the M33's is its access less
 * the core's own cycle.
 *
 * Arbitration.  Under per-CPU clocks the M33 runs its slice of a round
 * first, then the M0+.  The M33's tenures are recorded per port; an M0+
 * access resolves against them: if the port is held by the M33 (a tenure
 * covers the request, or ended less than one tenure before it: the port is
 * parked on its last manager) the M0+ is granted after a handover of
 * k_port M33 tenures, at the end of the M33 tenure in progress then; its
 * own tenure then pushes the M33's later accesses back (round-robin), and
 * the M33 is charged the push.  [I] k_port fitted to SS7.74: the M0+ waits
 * 2-3 M33 tenures on the radio (9.2-9.8 cycles vs 4.25 under M33 radio
 * streaming, whether the M33's code is in flash or SRAM), 1.5-2.5 on the
 * HF APB (TIMER1 2.46x, CMU 1.7x; it waits on the M33's tenure whatever
 * its own target), about 1.5 on BUFC (3.25 -> 4.77).  The M33 meets the
 * M0+'s tenures recorded in earlier rounds the same way.
 *
 * SEQRAM.  The M0+'s fetches and its own SEQRAM data are not bus accesses
 * in QEMU (RAM), so its demand is a stream while it executes: each M33
 * SEQRAM access costs it 0.5 cycle more (1.88 -> 2.38) and costs the M0+
 * 0.9 of its cycles (SS7.74 [F]); M0+ exception entry, which stacks to and
 * fetches from SEQRAM, grows from 16 to a median 45-51 cycles while the M33
 * streams SEQRAM ([F]), scaled here by the M33's SEQRAM tenures inside the
 * entry itself.  An entry also ends the M0+'s sleep: SEQRAM tenures of the
 * M33 before it cost the M0+ nothing.
 *
 * Not modelled: M33 SRAM and flash accesses are RAM to QEMU (no bus
 * access), so the M33's SRAM traffic does not slow the M0+'s SRAM and
 * peripheral accesses (SS7.74 SS2, +0.7..1.1 and +0.5 cycle); LDMA; the
 * debugger's accesses.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/timer.h"
#include "sysemu/cpu-timers.h"
#include "hw/arm/efr32-interconnect.h"

#define PS_PER_NS 1000
#define FS_PER_PS 1000
#define MILLI 1000

enum { CLK_HCLK, CLK_RHCLK };

typedef struct Efr32IcTiming {
    uint8_t port;
    /* Beyond what the core charges for the instruction itself. */
    uint32_t core_mc;
    uint8_t tenure_clock;
    uint32_t tenure_mc;
} Efr32IcTiming;

/*
 * Beyond the instruction's own count: the Armv6-M translator charges the
 * M0+ 2 cycles per load or store, the Armv8-M one the M33 1.
 */
static const Efr32IcTiming timing[EFR32_IC_MANAGERS][EFR32_IC_TARGETS] = {
    [EFR32_IC_SEQ] = {
        [EFR32_IC_SEQRAM] = { EFR32_IC_PORT_SEQRAM, 0, CLK_RHCLK, 0 },
        /* 4.125 = 2 + 0.125 + 2 radio cycles */
        [EFR32_IC_RADIO] = { EFR32_IC_PORT_RADIO, 125, CLK_RHCLK, 2000 },
        /* 3.125 = 2 + 0.125 + 1 */
        [EFR32_IC_BUFC] = { EFR32_IC_PORT_BUFC, 125, CLK_RHCLK, 1000 },
        /* 3.125 = 2 + 0.625 + 1 HCLK (0.5) */
        [EFR32_IC_SRAM] = { EFR32_IC_PORT_SRAM, 625, CLK_HCLK, 1000 },
        /* 8.625 = 2 + 1.125 + 11 HCLK (5.5) */
        [EFR32_IC_TIMER] = { EFR32_IC_PORT_HFAPB, 1125, CLK_HCLK, 11000 },
        /* 4.625 = 2 + 1.025 + 3.2 HCLK (1.6) */
        [EFR32_IC_HFPERIPH] = { EFR32_IC_PORT_HFAPB, 1025, CLK_HCLK, 3200 },
        [EFR32_IC_LFPERIPH] = { EFR32_IC_PORT_LFAPB, 1025, CLK_HCLK, 3200 },
    },
    [EFR32_IC_M33] = {
        /* 1.875 = 1 + 0.875 */
        [EFR32_IC_SEQRAM] = { EFR32_IC_PORT_SEQRAM, 0, CLK_HCLK, 875 },
        /* 3.875 = 1 + 2.875 */
        [EFR32_IC_RADIO] = { EFR32_IC_PORT_RADIO, 0, CLK_HCLK, 2875 },
        [EFR32_IC_BUFC] = { EFR32_IC_PORT_BUFC, 0, CLK_HCLK, 875 },
        [EFR32_IC_SRAM] = { EFR32_IC_PORT_SRAM, 0, CLK_HCLK, 0 },
        /* 11.875 = 1 + 10.875 */
        [EFR32_IC_TIMER] = { EFR32_IC_PORT_HFAPB, 0, CLK_HCLK, 10875 },
        [EFR32_IC_HFPERIPH] = { EFR32_IC_PORT_HFAPB, 0, CLK_HCLK, 2875 },
        [EFR32_IC_LFPERIPH] = { EFR32_IC_PORT_LFAPB, 0, CLK_HCLK, 2875 },
    },
};

/* M33 code in SRAM: the fetch shares the System interface. */
static const uint32_t m33_sram_code_mc[EFR32_IC_TARGETS] = {
    [EFR32_IC_SEQRAM] = 1750, [EFR32_IC_RADIO] = 1750,
    [EFR32_IC_BUFC] = 1750, [EFR32_IC_TIMER] = 1250,
    [EFR32_IC_HFPERIPH] = 1250, [EFR32_IC_LFPERIPH] = 1250,
};

/* Handover to the M0+ while the M33 holds a port, in M33 tenures (x1000). */
static const uint32_t handover_milli[EFR32_IC_PORTS] = {
    [EFR32_IC_PORT_RADIO] = 6000,
    [EFR32_IC_PORT_BUFC] = 4200,
    [EFR32_IC_PORT_SRAM] = 1000,
    [EFR32_IC_PORT_HFAPB] = 6200,
    [EFR32_IC_PORT_LFAPB] = 6200,
};

/* The idle wait state: one cycle of the M0+, after more than one idle. */
#define SEQ_IDLE_WAIT_MC 1000
/*
 * Back to back: the next access issues right after the last one completes,
 * its own load or store (2 cycles) plus at most the one idle cycle the RM
 * allows; a loop pass (subs, bne) or literal loads in between make it idle.
 */
#define SEQ_BACK_TO_BACK_GAP_MC 3000
#define M33_SEQRAM_SHARED_MC 500
/*
 * [I] fitted to SS7.74: under M33 SEQRAM streaming (flash code, one access
 * per 2.38 cycles) the M0+ ALU loop runs 3.46x slower and SEQRAM loads cost
 * 6.0 instead of 2: 1.4 cycles per M33 access while the M0+ runs from
 * SEQRAM; its loops of bus accesses (radio, BUFC, SRAM, CMU, TIMER1) lose
 * only 0.45-1.8 cycles per access: in the few cycles between two bus
 * accesses its prefetch covers most of it, 0.25 per M33 access.
 */
#define SEQ_SEQRAM_STEAL_SHORT_MC 350
#define SEQ_PREFETCH_COVER_MC 6000
#define SEQ_SEQRAM_STEAL_MC 1200
#define SEQ_EXCEPTION_ENTRY_MC 14000
/* 47 (median of 45-51) - 16 cycles at full M33 SEQRAM streaming. */
#define SEQ_EXCEPTION_SEQRAM_MC 31000
/* An M33 SEQRAM access every 2.375 cycles is full streaming. */
#define M33_SEQRAM_STREAM_PERIOD_MC 2375
/* Tenures older than this before a round start are forgotten. */
#define HISTORY_PS (2000 * PS_PER_NS)

static int64_t mc_ps(uint64_t hz, uint64_t millicycles)
{
    return hz ? (int64_t)muldiv64(millicycles, 1000000000ULL, hz) : 0;
}

static uint64_t ps_mc(uint64_t hz, int64_t ps)
{
    return ps > 0 ? muldiv64(ps, hz, 1000000000ULL) : 0;
}

static uint64_t manager_hz(Efr32Interconnect *ic, unsigned m)
{
    return m == EFR32_IC_M33 ? ic->hclk_hz : ic->rhclk_hz;
}

static int64_t now_ps(CPUState *cpu)
{
    int64_t ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    return ns * PS_PER_NS + (cpu ? cpu->icount_time_fs / FS_PER_PS : 0);
}

static Efr32IcTenure *tl_at(Efr32IcTimeline *tl, unsigned i)
{
    return &tl->t[(tl->head + i) % EFR32_IC_RING];
}

static void tl_add(Efr32IcTimeline *tl, int64_t start, int64_t end)
{
    if (tl->count == EFR32_IC_RING) {
        tl->head = (tl->head + 1) % EFR32_IC_RING;
        tl->count--;
    }
    *tl_at(tl, tl->count++) = (Efr32IcTenure){ start, end };
}

/* The tenure covering @t, else the last one ending before it, or NULL. */
static Efr32IcTenure *tl_find(Efr32IcTimeline *tl, int64_t t)
{
    Efr32IcTenure *last = NULL;

    for (unsigned i = 0; i < tl->count; i++) {
        Efr32IcTenure *e = tl_at(tl, i);

        if (e->start_ps <= t && t < e->end_ps) {
            return e;
        }
        if (e->end_ps <= t && (!last || e->end_ps > last->end_ps)) {
            last = e;
        }
    }
    return last;
}

/* The end of the chain of tenures covering @t (t itself if none). */
static int64_t tl_free_from(Efr32IcTimeline *tl, int64_t t)
{
    bool moved;

    do {
        moved = false;
        for (unsigned i = 0; i < tl->count; i++) {
            Efr32IcTenure *e = tl_at(tl, i);

            if (e->start_ps <= t && t < e->end_ps) {
                t = e->end_ps;
                moved = true;
            }
        }
    } while (moved);
    return t;
}

/*
 * The M0+ took [g, end): the M33's first tenure starting inside it, and all
 * the M33 did after (an in-order core blocked on its load), move back.
 */
static void push_m33(Efr32Interconnect *ic, unsigned port, int64_t g,
                     int64_t end)
{
    Efr32IcTimeline *tl = &ic->tl[EFR32_IC_M33][port];
    int64_t first = INT64_MAX, delta;

    for (unsigned i = 0; i < tl->count; i++) {
        Efr32IcTenure *e = tl_at(tl, i);

        if (e->start_ps >= g && e->start_ps < end && e->start_ps < first) {
            first = e->start_ps;
        }
    }
    if (first == INT64_MAX) {
        return;
    }
    delta = end - first;
    for (unsigned p = 0; p < EFR32_IC_PORTS; p++) {
        Efr32IcTimeline *o = &ic->tl[EFR32_IC_M33][p];

        for (unsigned i = 0; i < o->count; i++) {
            Efr32IcTenure *e = tl_at(o, i);

            if (e->start_ps >= first) {
                e->start_ps += delta;
                e->end_ps += delta;
            }
        }
    }
    ic->last_end_ps[EFR32_IC_M33] += delta;
    icount_charge_cycles(ic->cpu[EFR32_IC_M33],
                         ps_mc(ic->hclk_hz, delta));
}

/*
 * The port is held by the M33: one of its tenures covers @r, or it is
 * streaming the port (its last tenure ended no longer before @r than the
 * gap before that tenure, or than the tenure itself).
 */
static bool m33_holds(Efr32Interconnect *ic, unsigned port, int64_t r)
{
    Efr32IcTimeline *tl = &ic->tl[EFR32_IC_M33][port];
    Efr32IcTenure *c = tl_find(tl, r), *prev = NULL;
    int64_t window;

    if (!c) {
        return false;
    }
    if (c->start_ps <= r) {
        if (r < c->end_ps) {
            return true;
        }
    }
    for (unsigned i = 0; i < tl->count; i++) {
        Efr32IcTenure *e = tl_at(tl, i);

        if (e->end_ps <= c->start_ps &&
            (!prev || e->end_ps > prev->end_ps)) {
            prev = e;
        }
    }
    window = c->end_ps - c->start_ps;
    if (prev) {
        window = MAX(window, c->start_ps - prev->end_ps);
    }
    return r - c->end_ps <= window;
}

/*
 * Each M33 SEQRAM tenure that fell while the M0+ executed from SEQRAM (since
 * its last bus access ended, before @t) cost it SEQ_SEQRAM_STEAL_MC.
 */
static int64_t seq_seqram_steal(Efr32Interconnect *ic, int64_t t)
{
    Efr32IcTimeline *tl = &ic->tl[EFR32_IC_M33][EFR32_IC_PORT_SEQRAM];
    uint64_t n = 0, steal;

    for (unsigned i = 0; i < tl->count; i++) {
        Efr32IcTenure *e = tl_at(tl, i);

        if (e->start_ps >= ic->seq_local_from_ps && e->start_ps < t) {
            n++;
        }
    }
    steal = t - ic->seq_local_from_ps >
            mc_ps(ic->rhclk_hz, SEQ_PREFETCH_COVER_MC)
                ? SEQ_SEQRAM_STEAL_MC : SEQ_SEQRAM_STEAL_SHORT_MC;
    ic->seq_local_from_ps = t;
    return mc_ps(ic->rhclk_hz, n * steal);
}

uint64_t efr32_ic_access(Efr32Interconnect *ic, unsigned m,
                         Efr32IcTarget target, bool m33_code_in_sram,
                         unsigned *port_out)
{
    const Efr32IcTiming *tm = &timing[m][target];
    unsigned port = tm->port;
    uint64_t hz = manager_hz(ic, m);
    uint64_t tenure_hz = tm->tenure_clock == CLK_HCLK ? ic->hclk_hz
                                                       : ic->rhclk_hz;
    int64_t t = now_ps(ic->cpu[m]);
    int64_t tenure = mc_ps(tenure_hz, tm->tenure_mc);
    int64_t core = mc_ps(hz, tm->core_mc);
    int64_t r, g;
    unsigned other = m == EFR32_IC_M33 ? EFR32_IC_SEQ : EFR32_IC_M33;

    *port_out = port;
    if (m == EFR32_IC_M33 && m33_code_in_sram) {
        core += mc_ps(hz, m33_sram_code_mc[target]);
    }
    r = t + core;
    if (m == EFR32_IC_SEQ) {
        /* The M0+ ran from SEQRAM since its last bus access. */
        core += seq_seqram_steal(ic, t);
        r = t + core;
        if (t - ic->last_end_ps[m] > mc_ps(hz, SEQ_BACK_TO_BACK_GAP_MC)) {
            r += mc_ps(hz, SEQ_IDLE_WAIT_MC);
        }
    }
    g = r;
    if (m == EFR32_IC_SEQ && tenure) {
        if (m33_holds(ic, port, r)) {
            Efr32IcTenure *c = tl_find(&ic->tl[other][port], r);

            /* Parked on the M33: hand over after k of its tenures. */
            g = r + (c->end_ps - c->start_ps) * handover_milli[port] / MILLI;
        }
        g = tl_free_from(&ic->tl[other][port], g);
        push_m33(ic, port, g, g + tenure);
    } else if (tenure) {
        /* The M0+'s tenures from earlier rounds hold the port. */
        g = tl_free_from(&ic->tl[other][port], r);
    }
    if (tenure) {
        tl_add(&ic->tl[m][port], g, g + tenure);
    }
    ic->last_end_ps[m] = g + tenure;
    if (m == EFR32_IC_SEQ) {
        ic->seq_local_from_ps = g + tenure;
    }
    if (port == EFR32_IC_PORT_SEQRAM && m == EFR32_IC_M33) {
        /* From flash only: with its code in SRAM the M33 waits on its own
         * fetch and SS7.74 saw no change (3.87 either way). */
        if (ic->seq_executing && !m33_code_in_sram) {
            tenure += mc_ps(hz, M33_SEQRAM_SHARED_MC);
        }
    }
    return ps_mc(hz, g - t + tenure);
}

void efr32_ic_round_end(Efr32Interconnect *ic, int64_t start_ns,
                        int64_t end_ns)
{
    int64_t horizon = start_ns * PS_PER_NS - HISTORY_PS;

    /* SEQRAM stolen from the M0+'s local execution in the rest of it. */
    if (ic->cpu[EFR32_IC_SEQ]) {
        /* Up to the M0+'s own time: it may have run past the round end. */
        CPUState *seq = ic->cpu[EFR32_IC_SEQ];
        int64_t upto = MIN(end_ns, seq->icount_time_ns) * PS_PER_NS;

        if (!ic->seq_executing) {
            ic->seq_local_from_ps = MAX(ic->seq_local_from_ps, upto);
        } else if (upto > ic->seq_local_from_ps) {
            icount_charge_cycles(seq, ps_mc(ic->rhclk_hz,
                                            seq_seqram_steal(ic, upto)));
        }
    }
    for (unsigned m = 0; m < EFR32_IC_MANAGERS; m++) {
        for (unsigned p = 0; p < EFR32_IC_PORTS; p++) {
            Efr32IcTimeline *tl = &ic->tl[m][p];

            while (tl->count && tl_at(tl, 0)->end_ps < horizon) {
                tl->head = (tl->head + 1) % EFR32_IC_RING;
                tl->count--;
            }
        }
    }
}

uint64_t efr32_ic_seq_exception_millicycles(Efr32Interconnect *ic)
{
    Efr32IcTimeline *tl = &ic->tl[EFR32_IC_M33][EFR32_IC_PORT_SEQRAM];
    int64_t t = now_ps(ic->cpu[EFR32_IC_SEQ]);
    int64_t window = mc_ps(ic->rhclk_hz, SEQ_EXCEPTION_ENTRY_MC);
    int64_t period = mc_ps(ic->hclk_hz, M33_SEQRAM_STREAM_PERIOD_MC);
    uint64_t n = 0, busy_ps, mc = SEQ_EXCEPTION_ENTRY_MC;

    for (unsigned i = 0; window && i < tl->count; i++) {
        Efr32IcTenure *e = tl_at(tl, i);

        if (e->start_ps >= t && e->start_ps < t + window) {
            n++;
        }
    }
    if (window) {
        busy_ps = n * period;
        mc += SEQ_EXCEPTION_SEQRAM_MC * MIN(busy_ps, window) / window;
    }
    ic->seq_local_from_ps = t + mc_ps(ic->rhclk_hz, mc);
    return mc;
}
