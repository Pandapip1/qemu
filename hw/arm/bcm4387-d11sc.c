/*
 * BCM4387 d11 template RAM and SampleCollect (SC) engine, one per slice.
 * Register map and stock's use: notes/31-macsmpl.md in the bcm4387 project
 * (static RE of stock's FTM/TOF capture: arm RAM 0x790dd4 / ROM 0x469eb8,
 * readback ROM 0x469f40; names from bcmdhd 101.10 d11.h).
 *
 * Modelled:
 *  - Template RAM, TPL_RAM_SIZE bytes, behind the port +0x130 (byte address)
 *    / +0x134 (32-bit data). Every data access, read or write, moves the
 *    address on by 4. [F] stock reads (0x4d6004, 0x469fbe) and writes
 *    (5115 template words at boot) through it after one address write.
 *    The size is not known; 64 KiB covers stock's TOF window (bytes up to
 *    ~0x9678). Addresses wrap at the size. [I]
 *  - SampleCollect: PlayCtrl +0xade (bit 0 START, 1 PAUSE, 2 STOP, [4:3]
 *    source, 5-8 TRIG/TRANS/MATCH/STORE, 9 play) [B], StartPtr +0xaa6,
 *    StopPtr +0xaa8 (32-bit word pointers, bits 19:16 in +0xaa0 [3:0] and
 *    [7:4]) [F], CurPtr +0xaa2 [F], SCPortalSel/Data +0xac6/+0xac8 [F]
 *    (plain storage, mask words; effect U).
 *  - A write that sets START with source 0 (PHY samples) starts a capture
 *    at StartPtr. [F] stock's arm: 0xade = 0, PHY 0x467, pointers, 0xade =
 *    5 (ROM 0x469f36). While running, one 32-bit word per sample at 20 MHz
 *    is stored at CurPtr and CurPtr advances. It stops after StopPtr
 *    (inclusive [F]) with CurPtr = StopPtr + 1, so stock's readback loop
 *    (base + k < CurPtr) reads every word. [I] CurPtr saturates (no wrap),
 *    START is not cleared by the hardware, bit 2 has no effect, and the
 *    0x467 bit-0 strobe is not needed (the ROM path starts captures with
 *    the ARM write alone). [I] Writing 0 to PlayCtrl, or clearing START,
 *    freezes CurPtr. PAUSE holds it. [B] names, [I] effect.
 *  - Samples are taken only while the slice's PHY/radio ports are live
 *    (see port_live in bcm4387.c) and PHY register 0x467 has its enable
 *    (bit 13) set. Otherwise the engine runs but CurPtr does not move. [I]
 *    RX being enabled, the tap select (0x467 [6:2], stock: 7) and the core
 *    select ([15:14]) have no other effect: every tap and core sees the
 *    same I/Q stream. [I]
 *  - Word format: I in bits 12:0, Q in bits 28:16, 13-bit two's
 *    complement. [F] layout, [I] which half is I (tap 7).
 *  - I/Q source (machine property iq-source=<file>, iq-format=s16|cf32):
 *    a 20 MHz complex stream, interleaved I/Q, looped. Sample n is the one
 *    the receiver sees at virtual time n / 20 MHz since reset, so a capture
 *    reads the stream "live". s16 is scaled to 13 bits by >> 3, cf32 by
 *    * 4095 and clamped. With no source, deterministic low-level noise.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/error-report.h"
#include "qemu/bswap.h"
#include <math.h>
#include "hw/arm/bcm4387.h"
#include "trace.h"

#define TPL_RAM_SIZE    0x10000
#define SC_RATE_HZ      20000000

#define TPL_ADDR        0x130
#define TPL_DATA        0x134
#define SC_PTR_HI       0xaa0
#define SC_CUR_PTR      0xaa2
#define SC_START_PTR    0xaa6
#define SC_STOP_PTR     0xaa8
#define SC_PORTAL_SEL   0xac6
#define SC_PORTAL_DATA  0xac8
#define SC_PLAYCTRL     0xade

#define SC_START        0x0001
#define SC_PAUSE        0x0002
#define SC_SRC_MASK     0x0018

#define PHY_SC_TAP      0x467
#define TAP_ENABLE      0x2000

/* ---- I/Q source ----------------------------------------------------------- */

/*
 * The I/Q source: interleaved I/Q at 13 bits. Shared by the slices; loaded
 * once by the machine.
 */
static int16_t *iq;
static size_t iq_n;             /* complex samples */

static int16_t clamp13(double v)
{
    return v > 4095 ? 4095 : v < -4096 ? -4096 : (int16_t)lrint(v);
}

void bcm4387_iq_load(const char *path, const char *fmt)
{
    g_autofree char *buf = NULL;
    gsize len;
    bool f32 = fmt && !strcmp(fmt, "cf32");

    if (fmt && !f32 && strcmp(fmt, "s16")) {
        error_report("bcm4387: iq-format must be s16 or cf32");
        exit(1);
    }
    if (!g_file_get_contents(path, &buf, &len, NULL)) {
        error_report("bcm4387: cannot read iq-source %s", path);
        exit(1);
    }
    iq_n = len / (f32 ? 8 : 4);
    if (!iq_n) {
        error_report("bcm4387: iq-source %s is empty", path);
        exit(1);
    }
    iq = g_new(int16_t, 2 * iq_n);
    for (size_t i = 0; i < 2 * iq_n; i++) {
        if (f32) {
            union { uint32_t u; float f; } c = { .u = ldl_le_p(buf + 4 * i) };

            iq[i] = clamp13(c.f * 4095.0);
        } else {
            iq[i] = (int16_t)lduw_le_p(buf + 2 * i) >> 3;
        }
    }
    info_report("bcm4387: iq-source %s: %zu samples (%s, 20 MHz)", path,
                iq_n, f32 ? "cf32" : "s16");
}

/* complex sample n of the stream as a template word: Q[28:16] | I[12:0] */
static uint32_t iq_word(uint64_t n)
{
    int16_t i, q;

    if (iq) {
        i = iq[2 * (n % iq_n)];
        q = iq[2 * (n % iq_n) + 1];
    } else {
        /* deterministic noise, +-15 */
        uint64_t x = n * 0x9e3779b97f4a7c15ull;

        x ^= x >> 29;
        x *= 0xbf58476d1ce4e5b9ull;
        x ^= x >> 32;
        i = (int16_t)(x & 31) - 16;
        q = (int16_t)((x >> 8) & 31) - 16;
    }
    return ((uint32_t)(q & 0x1fff) << 16) | (i & 0x1fff);
}

static uint64_t sample_now(void)
{
    return muldiv64(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL), SC_RATE_HZ,
                    NANOSECONDS_PER_SECOND);
}

/* ---- per-slice engine -------------------------------------------------------- */

struct Bcm4387Sc {
    int slice;
    Bcm4387ScHooks hooks;
    uint8_t *tpl;
    uint32_t tpl_addr;
    uint16_t playctrl, ptr_hi, start_lo, stop_lo, portal_sel;
    uint16_t portal[0x100];
    uint32_t cur;               /* next word to store */
    bool running;
    uint64_t next_sample;       /* stream index of the next stored sample */
};

static uint32_t sc_start(Bcm4387Sc *s)
{
    return (uint32_t)(s->ptr_hi & 0xf) << 16 | s->start_lo;
}

static uint32_t sc_stop(Bcm4387Sc *s)
{
    return (uint32_t)((s->ptr_hi >> 4) & 0xf) << 16 | s->stop_lo;
}

static bool sc_sampling(Bcm4387Sc *s)
{
    return s->hooks.phy_live(s->hooks.opaque, s->slice) &&
           (s->hooks.phyreg(s->hooks.opaque, s->slice, PHY_SC_TAP) & TAP_ENABLE);
}

/* bring the capture up to the current time */
static void sc_advance(Bcm4387Sc *s)
{
    uint64_t now;

    if (!s->running || (s->playctrl & SC_PAUSE)) {
        return;
    }
    now = sample_now();
    if (!sc_sampling(s)) {
        s->next_sample = now;
        return;
    }
    while (s->next_sample < now && s->cur <= sc_stop(s)) {
        stl_le_p(s->tpl + ((s->cur * 4) & (TPL_RAM_SIZE - 1)),
                 iq_word(s->next_sample));
        s->cur++;
        s->next_sample++;
    }
    if (s->cur > sc_stop(s)) {
        s->running = false;
        trace_bcm4387_sc_done(s->slice, s->cur);
    }
}

static void sc_playctrl(Bcm4387Sc *s, uint16_t v)
{
    uint16_t old = s->playctrl;

    sc_advance(s);
    s->playctrl = v;
    trace_bcm4387_sc_playctrl(s->slice, old, v);
    if ((v & SC_START) && !(old & SC_START) && !(v & SC_SRC_MASK)) {
        s->cur = sc_start(s);
        s->running = true;
        s->next_sample = sample_now();
        trace_bcm4387_sc_start(s->slice, sc_start(s), sc_stop(s), sc_sampling(s));
    } else if (!(v & SC_START)) {
        s->running = false;
    } else if ((v & SC_START) && (v & SC_SRC_MASK)) {
        qemu_log_mask(LOG_UNIMP, "bcm4387: d11.%d SampleCollect source %u not "
                      "modelled (only PHY samples)\n", s->slice,
                      (v & SC_SRC_MASK) >> 3);
    }
}

bool bcm4387_sc_access(Bcm4387Sc *s, hwaddr off, unsigned size, bool wr,
                       uint64_t *val)
{
    uint32_t a;

    switch (off) {
    case TPL_ADDR:
        if (wr) {
            s->tpl_addr = *val;
        } else {
            *val = s->tpl_addr;
        }
        return true;
    case TPL_DATA:
        sc_advance(s);
        a = s->tpl_addr & (TPL_RAM_SIZE - 4);
        if (wr) {
            stl_le_p(s->tpl + a, *val);
        } else {
            *val = ldl_le_p(s->tpl + a);
        }
        s->tpl_addr += 4;
        return true;
    case SC_PTR_HI:
        if (wr) {
            s->ptr_hi = *val;
        } else {
            *val = s->ptr_hi;
        }
        return true;
    case SC_CUR_PTR:
        sc_advance(s);
        if (!wr) {
            *val = s->cur & 0xffff;
        }
        return true;
    case SC_START_PTR:
        if (wr) {
            s->start_lo = *val;
        } else {
            *val = s->start_lo;
        }
        return true;
    case SC_STOP_PTR:
        if (wr) {
            s->stop_lo = *val;
        } else {
            *val = s->stop_lo;
        }
        return true;
    case SC_PORTAL_SEL:
        if (wr) {
            s->portal_sel = *val;
        } else {
            *val = s->portal_sel;
        }
        return true;
    case SC_PORTAL_DATA: {
        uint16_t *p = &s->portal[s->portal_sel & 0xff];

        if (wr) {
            *p = *val;
        } else {
            *val = *p;
        }
        if (s->portal_sel & 0x8000) {   /* AUTO_INCR [B] */
            s->portal_sel = (s->portal_sel & 0xff00) |
                            ((s->portal_sel + 1) & 0xff);
        }
        return true;
    }
    case SC_PLAYCTRL:
        if (wr) {
            sc_playctrl(s, *val);
        } else {
            sc_advance(s);
            *val = s->playctrl;
        }
        return true;
    }
    return false;
}

Bcm4387Sc *bcm4387_sc_new(int slice, const Bcm4387ScHooks *hooks)
{
    Bcm4387Sc *s = g_new0(Bcm4387Sc, 1);

    s->slice = slice;
    s->hooks = *hooks;
    s->tpl = g_malloc0(TPL_RAM_SIZE);
    return s;
}

void bcm4387_sc_reset(Bcm4387Sc *s)
{
    memset(s->tpl, 0, TPL_RAM_SIZE);
    s->tpl_addr = 0;
    s->playctrl = s->ptr_hi = s->start_lo = s->stop_lo = s->portal_sel = 0;
    memset(s->portal, 0, sizeof(s->portal));
    s->cur = 0;
    s->running = false;
}
