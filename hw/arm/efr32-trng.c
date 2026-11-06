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
 * flags stay clear.
 *
 * Test mode (TESTEN): TESTDATA words replace the noise source, ENABLE or not.
 * Every four words are a 128-bit block (word 0 first, bytes little-endian)
 * and the conditioning function is AES-128 CBC-MAC under KEY0-3 (same byte
 * order) over NB128BITBLOCKS blocks, whose MAC is written to the FIFO as one
 * block [I: the stock firmware's start-up known-answer test writes the key
 * and the input blocks of NIST SP 800-38A F.2.1 (the first block XORed with
 * its IV) and expects that example's last CBC block, and passes on silicon].
 * That test writes the field as 0 and four blocks and reads one block back,
 * so 0 conditions four blocks here [I]. CONDBYPASS writes the blocks to the
 * FIFO unconditioned [I]. TESTDATABUSY stays clear and a MAC is in the FIFO
 * when its last word is written [U: unmeasured].
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
#include "crypto/aes.h"
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
    return (t->ctrl & CTRL_ENABLE) &&
           !(t->ctrl & (CTRL_SOFTRESET | CTRL_TESTEN));
}

/* The stock known-answer test's field value 0 conditions four blocks. */
#define TEST_NB_ZERO_BLOCKS 4u

static void test_reset(Efr32Trng *t)
{
    t->test_words = t->test_blocks = 0;
    memset(t->test_mac, 0, sizeof(t->test_mac));
}

static void fifo_push_block(Efr32Trng *t, const uint32_t *words)
{
    for (unsigned i = 0; i < WORDS_PER_BLOCK; i++) {
        if (t->level == EFR32_TRNG_FIFO_WORDS) {
            t->status |= STATUS_FULLIF;
            return;
        }
        t->fifo[(t->head + t->level) % EFR32_TRNG_FIFO_WORDS] = words[i];
        t->level++;
    }
}

/* One TESTDATA word into the conditioning function (or around it). */
static void test_data(Efr32Trng *t, uint32_t value)
{
    uint32_t nb = (t->ctrl >> CTRL_NB128BITBLOCKS_SHIFT) &
                  CTRL_NB128BITBLOCKS_MASK;
    uint8_t block[16], key[16];
    uint32_t out[WORDS_PER_BLOCK];
    AES_KEY ek;

    t->test_block[t->test_words++] = value;
    if (t->test_words < WORDS_PER_BLOCK) {
        return;
    }
    t->test_words = 0;
    if (t->ctrl & CTRL_CONDBYPASS) {
        fifo_push_block(t, t->test_block);
        return;
    }
    for (unsigned i = 0; i < WORDS_PER_BLOCK; i++) {
        stl_le_p(&block[4 * i], t->test_block[i]);
        stl_le_p(&key[4 * i], t->key[i]);
    }
    for (unsigned i = 0; i < sizeof(block); i++) {
        t->test_mac[i] ^= block[i];
    }
    AES_set_encrypt_key(key, 128, &ek);
    AES_encrypt(t->test_mac, t->test_mac, &ek);
    if (++t->test_blocks < (nb ? nb : TEST_NB_ZERO_BLOCKS)) {
        return;
    }
    for (unsigned i = 0; i < WORDS_PER_BLOCK; i++) {
        out[i] = ldl_le_p(&t->test_mac[4 * i]);
    }
    fifo_push_block(t, out);
    test_reset(t);
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
        if ((t->ctrl & CTRL_TESTEN) && !(old & CTRL_TESTEN)) {
            test_reset(t);
        }
        if (t->ctrl & CTRL_SOFTRESET) {
            /* "The continuous test, the conditioning function and the FIFO
             * are reset" while the bit is set. */
            trng_clear_fifo(t);
            test_reset(t);
            t->next_block_ns = INT64_MAX;
        } else if (producing(t) &&
                   (!(old & CTRL_ENABLE) ||
                    (old & (CTRL_SOFTRESET | CTRL_TESTEN)))) {
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
        if ((t->ctrl & CTRL_TESTEN) && !(t->ctrl & CTRL_SOFTRESET)) {
            test_data(t, value);
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
