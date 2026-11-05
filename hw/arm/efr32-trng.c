/*
 * EFR32xG22 CRYPTOACC TRNG (see efr32-trng.h).
 *
 * Register behaviour follows RM SS11.5.2: RNGCTRL (reset 0x00040000,
 * NB128BITBLOCKS 4), FIFOLEVEL, FIFOTHRESH (read-only 0x3F), FIFODEPTH
 * (0x40), KEY0-3, TESTDATA, RNGSTATUS (STATE, FULLIF, ...), INITWAITVAL
 * and SWOFFTMRVAL (0xFFFF), CLKDIV, AIS31CONF0-2 (0x43401040, 0x03C00680,
 * 0x04400340), AIS31STATUS.
 *
 * The noise source is QEMU's guest random source (qemu_guest_getrandom:
 * the host's RNG, or a reproducible stream under -seed).  The model has no
 * failing noise source, so the health and AIS31 tests always pass and their
 * flags stay clear; test mode (TESTEN, TESTDATA) and the conditioning
 * bypass are not modelled (logged).
 *
 * Timing, measured on silicon (docs/radio-tables.md SS7.89:
 * tests/periph-timing-port, DWT against the LFXO, HCLK 76.8 MHz, CLKDIV 0-3,
 * NB128BITBLOCKS 1-8, INITWAITVAL 16-16384, with and without BYPNIST):
 *  - The rings are sampled at Fs = HCLK / (CLKDIV + 1) (the RM's SS11.5.2.10
 *    names PCLK; the block period follows HCLK, PCLK being HCLK / 2 there).
 *  - After ENABLE the module waits INITWAITVAL HCLK cycles (SS11.5.2.8), then
 *    the NIST start-up test takes 512 samples unless BYPNIST, then the first
 *    block 512 samples whatever NB128BITBLOCKS, and its first word is in the
 *    FIFO 1.41 us later [F fit: residual within 0.42 us; NB128BITBLOCKS 1
 *    and 8 come 1.0 and 0.4 us early].
 *  - Each further 128-bit block takes NB128BITBLOCKS x 128 samples
 *    (SS11.5.2.1) plus 0.55 us [F fit at CLKDIV 0; 0.13 us at CLKDIV 1 and 3,
 *    0.95 us at NB128BITBLOCKS 1].
 *  - Silicon writes a block's four words over 1.0-2.1 us; the model writes
 *    them together at the first word's time.
 * A full FIFO stops the rings (FIFOFULLOFF) unless FORCERUN; they restart
 * when a word is read (FIFOTHRESH, 0x3F, is at least the FIFO's depth however
 * it is counted) [I: the restart's timing is unmeasured].
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/guest-random.h"
#include "hw/arm/efr32-trng.h"

#define RNGCTRL 0x000u
#define FIFOLEVEL 0x004u
#define FIFOTHRESH 0x008u
#define FIFODEPTH 0x00cu
#define KEY0 0x010u
#define KEY3 0x01cu
#define TESTDATA 0x020u
#define RNGSTATUS 0x030u
#define INITWAITVAL 0x034u
#define SWOFFTMRVAL 0x040u
#define CLKDIV 0x044u
#define AIS31CONF0 0x048u
#define AIS31CONF2 0x050u
#define AIS31STATUS 0x054u

#define CTRL_RESET 0x00040000u
#define CTRL_MASK 0x001ffffdu
#define CTRL_ENABLE (1u << 0)
#define CTRL_TESTEN (1u << 2)
#define CTRL_CONDBYPASS (1u << 3)
#define CTRL_SOFTRESET (1u << 8)
#define CTRL_FORCERUN (1u << 11)
#define CTRL_BYPNIST (1u << 12)
#define CTRL_NB128BITBLOCKS_SHIFT 16
#define CTRL_NB128BITBLOCKS_MASK 0xfu

#define STATUS_PREIF (1u << 8)
#define STATUS_FULLIF (1u << 7)
#define STATUS_STATE_SHIFT 1
#define STATE_RESET 0u
#define STATE_STARTUP 1u
#define STATE_FIFOFULLON 2u
#define STATE_FIFOFULLOFF 3u
#define STATE_RUNNING 4u

#define FIFOTHRESH_RESET 0x3fu
#define INITWAITVAL_RESET 0xffffu
#define SWOFFTMRVAL_RESET 0xffffu
#define WAITVAL_MASK 0xffffu
#define CLKDIV_MASK 0xffu
#define AIS31CONF_MASK 0x7fff7fffu
#define AIS31STATUS_MASK 0x3ffffu
static const uint32_t ais31conf_reset[3] = {
    0x43401040u, 0x03c00680u, 0x04400340u,
};

#define NIST_STARTUP_SAMPLES 512u
#define FIRST_BLOCK_SAMPLES 512u
#define FIRST_WORD_LATENCY_NS 1410
#define BLOCK_LATENCY_NS 550
#define SAMPLES_PER_BLOCK 128u
#define WORDS_PER_BLOCK 4u

static int64_t now_ns(void)
{
    return qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static bool producing(Efr32Trng *t)
{
    return (t->ctrl & CTRL_ENABLE) && !(t->ctrl & CTRL_SOFTRESET);
}

static int64_t sample_ns(Efr32Trng *t, uint64_t hclk_hz, uint64_t samples)
{
    return muldiv64(samples * (t->clkdiv + 1), NANOSECONDS_PER_SECOND,
                    hclk_hz ? hclk_hz : 1);
}

static int64_t block_ns(Efr32Trng *t, uint64_t hclk_hz)
{
    uint32_t nb = (t->ctrl >> CTRL_NB128BITBLOCKS_SHIFT) &
                  CTRL_NB128BITBLOCKS_MASK;

    return sample_ns(t, hclk_hz, (uint64_t)(nb ? nb : 1) * SAMPLES_PER_BLOCK) +
           BLOCK_LATENCY_NS;
}

/* Write every block the noise source has finished by now. */
static void trng_update(Efr32Trng *t, uint64_t hclk_hz)
{
    int64_t now = now_ns();

    if (!producing(t)) {
        return;
    }
    while (t->next_block_ns <= now &&
           t->level + WORDS_PER_BLOCK <= EFR32_TRNG_FIFO_WORDS) {
        for (unsigned i = 0; i < WORDS_PER_BLOCK; i++) {
            unsigned slot = (t->head + t->level) % EFR32_TRNG_FIFO_WORDS;

            qemu_guest_getrandom_nofail(&t->fifo[slot], sizeof(t->fifo[0]));
            t->level++;
        }
        t->next_block_ns += block_ns(t, hclk_hz);
        t->starting = false;
    }
    if (t->level + WORDS_PER_BLOCK > EFR32_TRNG_FIFO_WORDS) {
        /* Full: the rings stop (FIFOFULLOFF) and restart on a read. */
        t->status |= STATUS_FULLIF;
        t->next_block_ns = INT64_MAX;
    }
}

static void trng_start(Efr32Trng *t, uint64_t hclk_hz)
{
    uint64_t samples = FIRST_BLOCK_SAMPLES +
                       ((t->ctrl & CTRL_BYPNIST) ? 0 : NIST_STARTUP_SAMPLES);

    t->next_block_ns = now_ns() +
        muldiv64(t->initwaitval, NANOSECONDS_PER_SECOND,
                 hclk_hz ? hclk_hz : 1) +
        sample_ns(t, hclk_hz, samples) + FIRST_WORD_LATENCY_NS;
    t->starting = true;
}

static void trng_clear_fifo(Efr32Trng *t)
{
    t->head = t->level = 0;
    t->status &= ~STATUS_FULLIF;
}

static uint32_t trng_state(Efr32Trng *t, uint64_t hclk_hz)
{
    int64_t now = now_ns();

    if (!producing(t)) {
        return STATE_RESET;
    }
    if (t->next_block_ns == INT64_MAX) {
        return (t->ctrl & CTRL_FORCERUN) ? STATE_FIFOFULLON
                                         : STATE_FIFOFULLOFF;
    }
    if (t->starting && t->next_block_ns > now) {
        return STATE_STARTUP;
    }
    return STATE_RUNNING;
}

void efr32_trng_reset(Efr32Trng *t)
{
    memset(t, 0, sizeof(*t));
    t->ctrl = CTRL_RESET;
    t->initwaitval = INITWAITVAL_RESET;
    t->swofftmrval = SWOFFTMRVAL_RESET;
    memcpy(t->ais31conf, ais31conf_reset, sizeof(t->ais31conf));
    t->next_block_ns = INT64_MAX;
}

bool efr32_trng_read(Efr32Trng *t, uint32_t offset, uint64_t hclk_hz,
                     uint32_t *value)
{
    trng_update(t, hclk_hz);
    switch (offset) {
    case RNGCTRL:
        *value = t->ctrl;
        return true;
    case FIFOLEVEL:
        *value = t->level;
        return true;
    case FIFOTHRESH:
        *value = FIFOTHRESH_RESET;
        return true;
    case FIFODEPTH:
        *value = EFR32_TRNG_FIFO_WORDS;
        return true;
    case KEY0 ... KEY3:
        *value = t->key[(offset - KEY0) / 4];
        return true;
    case TESTDATA:
        *value = 0;
        return true;
    case RNGSTATUS:
        *value = t->status | (trng_state(t, hclk_hz) << STATUS_STATE_SHIFT);
        return true;
    case INITWAITVAL:
        *value = t->initwaitval;
        return true;
    case SWOFFTMRVAL:
        *value = t->swofftmrval;
        return true;
    case CLKDIV:
        *value = t->clkdiv;
        return true;
    case AIS31CONF0 ... AIS31CONF2:
        *value = t->ais31conf[(offset - AIS31CONF0) / 4];
        return true;
    case AIS31STATUS:
        *value = t->ais31status;
        return true;
    default:
        return false;
    }
}

bool efr32_trng_write(Efr32Trng *t, uint32_t offset, uint64_t hclk_hz,
                      uint32_t value)
{
    uint32_t old = t->ctrl;

    trng_update(t, hclk_hz);
    switch (offset) {
    case RNGCTRL:
        t->ctrl = value & CTRL_MASK;
        if (t->ctrl & (CTRL_TESTEN | CTRL_CONDBYPASS)) {
            qemu_log_mask(LOG_UNIMP, "efr32 TRNG: test mode / conditioning "
                          "bypass not modelled (RNGCTRL 0x%x)\n", t->ctrl);
        }
        if (t->ctrl & CTRL_SOFTRESET) {
            /* "The continuous test, the conditioning function and the FIFO
             * are reset" while the bit is set. */
            trng_clear_fifo(t);
            t->next_block_ns = INT64_MAX;
        } else if (producing(t) &&
                   (!(old & CTRL_ENABLE) || (old & CTRL_SOFTRESET))) {
            trng_start(t, hclk_hz);
        } else if (!(t->ctrl & CTRL_ENABLE)) {
            t->next_block_ns = INT64_MAX;
        }
        return true;
    case KEY0 ... KEY3:
        t->key[(offset - KEY0) / 4] = value;
        return true;
    case TESTDATA:
        /* "Writes ... are ignored if the TESTEN bit ... is 0." */
        if (t->ctrl & CTRL_TESTEN) {
            qemu_log_mask(LOG_UNIMP, "efr32 TRNG: TESTDATA not modelled\n");
        }
        return true;
    case RNGSTATUS:
        /* PREIF is the only RW flag (RM SS11.5.2.7). */
        t->status = (t->status & ~STATUS_PREIF) | (value & STATUS_PREIF);
        return true;
    case INITWAITVAL:
        t->initwaitval = value & WAITVAL_MASK;
        return true;
    case SWOFFTMRVAL:
        t->swofftmrval = value & WAITVAL_MASK;
        return true;
    case CLKDIV:
        t->clkdiv = value & CLKDIV_MASK;
        return true;
    case AIS31CONF0 ... AIS31CONF2:
        t->ais31conf[(offset - AIS31CONF0) / 4] = value & AIS31CONF_MASK;
        return true;
    case AIS31STATUS:
        t->ais31status = value & AIS31STATUS_MASK;
        return true;
    case FIFOLEVEL:
    case FIFOTHRESH:
    case FIFODEPTH:
        return true;
    default:
        return false;
    }
}

uint32_t efr32_trng_pop(Efr32Trng *t, uint64_t hclk_hz)
{
    uint32_t word;

    trng_update(t, hclk_hz);
    if (!t->level) {
        /* [U] what silicon returns from an empty FIFO is unmeasured. */
        t->underruns++;
        qemu_log_mask(LOG_GUEST_ERROR, "efr32 TRNG: read of an empty FIFO\n");
        return 0;
    }
    word = t->fifo[t->head];
    t->head = (t->head + 1) % EFR32_TRNG_FIFO_WORDS;
    t->level--;
    if (t->next_block_ns == INT64_MAX && producing(t)) {
        /* The rings restart below the threshold. */
        t->status &= ~STATUS_FULLIF;
        t->next_block_ns = now_ns() + block_ns(t, hclk_hz);
    }
    return word;
}
