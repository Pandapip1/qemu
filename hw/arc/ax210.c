/*
 * QEMU machine model for the Intel Wi-Fi 6E AX210 (Typhoon Peak / "Ty")
 * on-die ARCompact (ARC700) MAC processor.
 *
 * This is a reverse-engineering / security-research scaffold, not a faithful
 * silicon model. It provides:
 *   - RAM over the firmware's LMAC/UMAC/uncached address windows,
 *   - traced MMIO stubs over the known peripheral windows (crypto coprocessor,
 *     control registers), and
 *   - a low-priority catch-all tracer so any unmodelled access is logged
 *     (with the faulting PC) and returns a programmable value instead of
 *     aborting, so firmware init can be driven and its hardware dependencies
 *     discovered iteratively.
 *
 * Firmware sections are loaded by the harness over the gdbstub; this board
 * lays out the shared memory map and brings up both cores: the ARC700 UMAC
 * and the ARCv2EM LMAC, over one flat physical address space.
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "cpu.h"
#include "hw/hw.h"
#include "hw/boards.h"
#include "hw/irq.h"
#include "qemu/guest-random.h"
#include "ax210-crypto.h"
#include "hw/qdev-properties.h"
#include "exec/memory.h"
#include "exec/address-spaces.h"
#include "sysemu/reset.h"
#include "sysemu/sysemu.h"
#include "hw/arc/cpudevs.h"
#include "boot.h"
#include "qemu/timer.h"
#include "crypto/aes.h"
#include "hw/core/cpu.h"

/* ---- traced MMIO -------------------------------------------------------- */

/* Cross-core address view (finding K1). The IML/UMAC side converts LMAC
 * addresses with (a & 0xff0fffff) | 0xc0e20000 (IML 0x804441a6, 0x80447082):
 * LMAC 0x00axxxxx == UMAC 0xc0exxxxx [F, from that code]. UMAC 0xc0d0xxxx ==
 * LMAC 0x00d0xxxx [I]: UMAC's semaphore set sits at the same block +0x20.
 * UMAC 0xc0a0xxxx == LMAC 0x00a0xxxx (K3) [F]: UMAC 0x804a8b80 converts
 * the LMAC-view 0xa0a404 + 16 * ch with (a & 0xff0fffff) | 0xc0a00000.
 * Every shared-register model is keyed by the LMAC-view address. */
static hwaddr lmac_view(hwaddr a)
{
    if (a >= 0xc0a00000 && a < 0xc0a10000) {
        return a - 0xc0000000;
    }
    if (a >= 0xc0d00000 && a < 0xc0e00000) {
        return a - 0xc0000000;
    }
    if (a >= 0xc0e00000 && a < 0xc0f00000) {
        return a - 0xc0e00000 + 0x00a00000;
    }
    return a;
}

static uint32_t rs_get(hwaddr a);
static void trace_log(const char *tag, hwaddr addr, uint64_t val, unsigned size);


typedef struct {
    const char *name;
    hwaddr base;
} TraceRegion;

static FILE *trace_fp;
static unsigned long trace_count;
static unsigned long trace_cap = 200000;

/* Override table: reads from these addresses return the given value instead of
 * 0. Populated as we learn which status/"ready" registers firmware polls. */
#define MAX_OVERRIDES 64
static struct { hwaddr addr; uint32_t val; } overrides[MAX_OVERRIDES];
static int n_overrides;

static uint32_t override_lookup(hwaddr addr, int *hit)
{
    for (int i = 0; i < n_overrides; i++) {
        if (overrides[i].addr == addr) { *hit = 1; return overrides[i].val; }
    }
    *hit = 0;
    return 0;
}

/* Sparse write-readback store: registers not otherwise modelled return the
 * last value written (0 if never written). Overrides take priority. */
static GHashTable *regstore;

/* Free-running counter registers (timers): each read returns the previous value
 * plus one. Default: the 0xc0d02c68 timebase used by IML delay loops. */
#define MAX_CTRS 16
static struct { hwaddr addr; uint32_t val; } ctrs[MAX_CTRS] = { { 0x00d02c68, 0 } }; /* UMAC 0xc0d02c68 */
static int n_ctrs = 1;

/* Host-CSR mailbox at 0xc0d0001c/20/24/28 (inferred from IML code + driver
 * csr offsets): write index to +0x20, command to +0x1c (top byte 0x6a = read,
 * 0x69 = write, data at +0x24), result at +0x28. Backing array models the PCIe
 * CSR block the host driver programs (e.g. 0x118 CTXT_INFO_ADDR). */
/* Descriptor DMA engine at 0xc0a0a500 (from IML routine 0x804458a4): +0x34
 * command (0x81000001 = host->device copy, kick), +0x38 device addr, +0x3c/0x40
 * host addr lo/hi, +0x44 length. Completion = bit 29 of status reg 0xc0d02e24
 * (polled by IML 0x804458c8; written back to acknowledge). Host "physical"
 * addresses are served from system memory (host RAM window at 0x20000000).
 * The driver's IMR copy uses channel 0 of the same block with the same field
 * order, CTRL last with bit 31 DMA_EN (iwl-fh.h IMR_TFH_SRV_DMA_CHNL0_* at
 * 0xa0a51c, pcie/trans.c iwl_trans_pcie_copy_imr_fh): +0x34 is channel 1. */
#define DMA_BASE  0x00a0a500u      /* UMAC 0xc0a0a500 (K3) */
#define DMA_STAT  0x00d02e24u   /* UMAC 0xc0d02e24 */
static unsigned dma_count;

/* SMEM range watcher and the entropy engine it watches (finding W3, [F]).
 * After LMAC answers 0xfd11, UMAC moves data in 512-byte chunks:
 * 0x8049da44 clears 0xd02e24 bit 7, arms channel 0 with a range at
 * 0xd0c154 and bit 31 of 0xd0c150, then 0xc0084a0c starts an engine through
 * the W1 window (0xb044 start, 0xb00c end, 0xb008 control 0x61e000) and
 * 0x80486638 waits, without timeout, for 0xd02e24 bit 7 (0x80485fd4).
 * The samples are entropy: 0x8047f564 packs bit 0 of each into a 0x840-bit
 * map at 0x660c00, or, on platforms where [0xd038c8] & 0xf2 is 0x32, bit 0
 * of the software PRNG 0x804a0bc0 (Park-Miller, x 0x41a7) instead;
 * 0x8049da98 feeds the first 2048 bits to the 0xd04800 engine (fatal
 * 0x10023f if they XOR to 0 or start with 0xbadc0fee) and 0x80493974 seeds
 * the PRNG from word 0x660d04.
 * Model: starting the engine with channel 0 armed fills the watched SMEM
 * range (0x400000 | range) with random words and sets bit 7 at once. */
#define WATCH_CTL   0x00d0c150u
#define WATCH_ARM   (1u << 31)
#define WATCH_EVT   (1u << 7)
#define ENG_CTL     (0x00a20000u + 0xb008)
#define WATCH_RANGE 0x00d0c154u
#define SMEM_BASE   0x00400000u
static uint32_t watch_evt;

/* Paging engine at 0xd04800 (finding W4, [F]): UMAC loads the 2048
 * entropy bits into +0x200 (0x8049da98), sets the page length at +0xc
 * (0x1000) and per page writes +8/+4 then a command to +0: 1 in the initial
 * page loop (0xc0084dac, then waits for status bit 2), 2 when paging a page
 * in from host DRAM (0x8047f0fc, waits for bit 0). 0x8047efd4 polls
 * [+0x20] for the bit up to 0x32 times; bit 3 is an error (fatal 0x100230).
 * Model: transparent (pages are not transformed or checked); a command sets
 * its done bit at once and bit 3 is never set. */
#define PGE_CMD   0x00d04800u
#define PGE_STAT  0x00d04820u
static uint32_t pge_stat;

/* Paging manager at 0xd05000 (finding W7, [F] interface, [I] policy):
 * set up by 0x8049de3c (+0/+4 SMEM tables 0x635c00/0x635e00, +0xc 0xf,
 * +8 control 0x40025008, bit 31 pauses it). Each page-in registers the page
 * with (n << 16) | page at +0x40 (0x8049dbc4) and unregisters an evicted
 * page by writing its number to +0x44 (0x8049e022). To evict, 0x8049dcc0 writes
 * n << 16 to +0x48 and polls +0x80 for bit 31 (10 tries, fatal 0x100253):
 * bit 30 set means a candidate in bits 9:0, which UMAC rejects if its bit is
 * set in the bitmap at +0x100 (then retries); +0x98 must read 0 (fatal
 * 0x100251). The hardware ranks pages by access (its SMEM tables); the model
 * cannot see accesses and answers with the oldest registered page whose
 * +0x100 bit is clear. */
#define PGM_BASE  0x00d05000u
#define PGM_REG   (PGM_BASE + 0x40)
#define PGM_UNREG (PGM_BASE + 0x44)
#define PGM_QUERY (PGM_BASE + 0x48)
#define PGM_RES   (PGM_BASE + 0x80)
#define PGM_LOCK  (PGM_BASE + 0x100)
#define PGM_MAX   1024
static uint16_t pgm_order[PGM_MAX];
static unsigned pgm_n;
static uint32_t pgm_res;

static void pgm_register(uint32_t page, bool add)
{
    unsigned i;

    page &= PGM_MAX - 1;
    for (i = 0; i < pgm_n && pgm_order[i] != page; i++) {
    }
    if (i < pgm_n) {
        memmove(&pgm_order[i], &pgm_order[i + 1],
                (pgm_n - i - 1) * sizeof(pgm_order[0]));
        pgm_n--;
    }
    if (add && pgm_n < PGM_MAX) {
        pgm_order[pgm_n++] = page;
    }
}

static void pgm_query(void)
{
    pgm_res = 1u << 31;
    for (unsigned i = 0; i < pgm_n; i++) {
        uint32_t page = pgm_order[i];
        if (!(rs_get(PGM_LOCK + 4 * (page >> 5)) & (1u << (page & 31)))) {
            pgm_res |= (1u << 30) | page;
            break;
        }
    }
    trace_log("PGM.Q", PGM_QUERY, pgm_res, 4);
}

/* LMAC block at 0xa0d000 (finding Q6, [F]; addresses V): LMAC writes 1 to
 * +8 to start it and 0 to stop it, then waits without timeout for +0xc
 * bit 31 (0x4d9a9c). Model: +0xc bit 31 reads set while +8 is 0. */
#define BLK_CTL   0x00a0d008u
#define BLK_STAT  0x00a0d00cu

/* SHA-256/384 engine at 0xa25000 (finding W5, [F]): 0x804a2170 writes the
 * mode to 0xa26410 (1 = SHA-384, 32-word blocks; 0 = SHA-256, 16) and 1 to
 * 0xa2506c and 0xa25064; 0x804a2290 reads the word count at 0xa25068, writes
 * big-endian block words to 0xa25000 + 4i (SHA-384 words 16-31 at 0xa25080)
 * and per block waits for 0xa25060 bit 0 clear (fatal 0x308); 0x804a2228
 * waits for (0xa25060 & 3) == 2 (fatal 0x30c); 0x804a2138 reads the digest
 * from 0xa25040 + 4i (SHA-384 words 8-11 at 0xa250c0), least significant
 * word first: the firmware's ECDSA check (W5 below) verifies Intel's
 * signature only with that order. Padding is done in software (0x804a2054).
 * Model: FIPS 180-4 compression per full block. */
#define SHA_BASE  0x00a25000u
#define SHA_MODE  0x00a26410u
#define SHA_INIT  (SHA_BASE + 0x64)
#define SHA_STAT  (SHA_BASE + 0x60)
#define SHA_CNT   (SHA_BASE + 0x68)
static AX210Sha sha;
static uint8_t sha_buf[128];
static unsigned sha_cnt;
static bool sha_done;

static int sha_in_word(hwaddr a)
{
    if (a >= SHA_BASE && a < SHA_BASE + 0x40) {
        return (a - SHA_BASE) / 4;
    }
    if (sha.wide && a >= SHA_BASE + 0x80 && a < SHA_BASE + 0xc0) {
        return 16 + (a - SHA_BASE - 0x80) / 4;
    }
    return -1;
}

static int sha_out_word(hwaddr a)
{
    if (a >= SHA_BASE + 0x40 && a < SHA_BASE + 0x60) {
        return (a - SHA_BASE - 0x40) / 4;
    }
    if (sha.wide && a >= SHA_BASE + 0xc0 && a < SHA_BASE + 0xe0) {
        return 8 + (a - SHA_BASE - 0xc0) / 4;
    }
    return -1;
}

static uint32_t sha_digest(int i)
{
    i = (sha.wide ? 11 : 7) - i;
    if (sha.wide) {
        uint64_t h = sha.h64[i / 2];
        return i & 1 ? (uint32_t)h : (uint32_t)(h >> 32);
    }
    return sha.h32[i];
}

static void sha_write(hwaddr a, uint32_t v)
{
    int i;

    if (a == SHA_INIT && (v & 1)) {
        ax210_sha_init(&sha, rs_get(SHA_MODE) & 1);
        sha_cnt = 0;
        sha_done = false;
        return;
    }
    i = sha_in_word(a);
    if (i < 0) {
        return;
    }
    stl_be_p(sha_buf + 4 * i, v);
    if (++sha_cnt == (sha.wide ? 32 : 16)) {
        ax210_sha_block(&sha, sha_buf);
        sha_cnt = 0;
        sha_done = true;
    }
}

/* Public-key engine at 0xa24c00 (finding W5, [F]): ECDSA P-384 verify in
 * 0x804a1978. 0xc008c0e4 selects operand slot n with 0x100 | 2n at +0 and
 * writes 12 words to 0xa24cac down to 0xa24c80 (most significant first);
 * slots 1/2 G, 3/4 Q, 5 p, 6 a, 0x10 n, 0x11 r, 0x12 SHA-384 digest,
 * 0x13 s (constants identify the curve). Command 0x11e (286) runs the
 * verify; 0xc008c0a0 requires +4 bit 0 (busy) set at once, then
 * 0xc008c018 waits for it to clear (fatal 0x30f); 0xc008bf70 accepts if
 * all 24 words at 0xa24d00-2c and 0xa24d80-ac are zero. 0x11a (282) is a
 * set-up command with G loaded. Model: FIPS 186-4 verify of the slots;
 * the result words are zero on success, 1 otherwise. */
#define PKA_CMD   0x00a24c00u
#define PKA_STAT  0x00a24c04u
#define PKA_DATA  0x00a24c80u
#define PKA_RES0  0x00a24d00u
#define PKA_RES1  0x00a24d80u
#define PKA_SLOTS 0x20
static uint32_t pka_slot[PKA_SLOTS][P384_WORDS];
static int pka_sel = -1;
static unsigned pka_busy;
static uint32_t pka_res;

static void pka_cmd(uint32_t v)
{
    if (v >= 0x100 && v < 0x100 + 2 * PKA_SLOTS && !(v & 1) &&
        v != 0x11a && v != 0x11e) {
        pka_sel = (v - 0x100) / 2;
        return;
    }
    pka_sel = -1;
    if (v == 0x11a || v == 0x11e) {
        pka_busy = 2;
    }
    if (v == 0x11e) {
        bool ok = ax210_p384_verify(pka_slot[3], pka_slot[4], pka_slot[0x11],
                                    pka_slot[0x13], pka_slot[0x12]);
        pka_res = ok ? 0 : 1;
        for (int n = 3; n < PKA_SLOTS; n++) {
            for (int i = 0; i < P384_WORDS; i++) {
                trace_log(n == 3 && i == 0 ? "PKA.S" : "PKA.s",
                          PKA_DATA + 4 * i, pka_slot[n][i], n);
            }
            if (n == 4) {
                n = 0x10;
            }
            if (n == 0x13) {
                break;
            }
        }
        trace_log("PKA.V", PKA_CMD, ok, 4);
    }
}

static void dma_kick(void);

/* LMAC AES-128 engine at 0xa01800 (finding A1). [F]: LMAC's boot self-test
 * (0x4d9ba0) runs FIPS-197 Appendix C.1 through it (key 000102..0f, pt
 * 00112233..ff, expected ct 69c4e0d8..c55a, table at 0x4e1828). Packing,
 * pinned by that vector: key FIFO +0x08 takes the key's low word first, each
 * word byte-swapped; data +0x98..+0xa4 holds word j = big-endian bytes
 * [12-4j..15-4j], result written back in place. [I]: ctrl +0x94 bit0 =
 * start, bit1 = encrypt (decrypt otherwise, unverified), status bit2 = done;
 * the mode write 0x8 to 0xa01c10 is not interpreted. */
#define AES_BASE 0x00a01800u
#define AES_KEYF (AES_BASE + 0x08)
#define AES_CTRL (AES_BASE + 0x94)
#define AES_DATA (AES_BASE + 0x98)
static uint32_t aes_key[4];
static unsigned aes_kidx;

static void aes_run(uint32_t ctrl)
{
    uint8_t key[16], in[16], out[16];
    AES_KEY k;

    for (int i = 0; i < 4; i++) {          /* key bytes: LE32(K[3]),..,LE32(K[0]) */
        stl_le_p(key + 4 * i, aes_key[3 - i]);
    }
    for (int j = 0; j < 4; j++) {          /* block bytes: BE32(D[3]),..,BE32(D[0]) */
        stl_be_p(in + 4 * (3 - j), rs_get(AES_DATA + 4 * j));
    }
    if (ctrl & 2) {
        AES_set_encrypt_key(key, 128, &k);
        AES_encrypt(in, out, &k);
    } else {
        AES_set_decrypt_key(key, 128, &k);
        AES_decrypt(in, out, &k);
    }
    for (int j = 0; j < 4; j++) {
        g_hash_table_insert(regstore, GUINT_TO_POINTER(AES_DATA + 4 * j),
                            GUINT_TO_POINTER(ldl_be_p(out + 4 * (3 - j))));
    }
    trace_log((ctrl & 2) ? "AES.E" : "AES.D", AES_CTRL, ldl_be_p(out), 4);
}

/* LMAC power/clock request FSM at 0xd03380 (finding F1, [I]): +0x04 clears
 * and +0x08 sets bits of the control word (SET/CLR alias convention). LMAC
 * 0x4d525c sets bit 4 (with bit 26) and waits for state [3:0] == 5;
 * 0x4d520c clears bit 4 and waits for state 0. Model: state follows bit 4
 * immediately (the real transition latency is [?]). */
#define FSM_BASE 0x00d03380u

/* Configure-and-start unit at 0xd03f00 (finding G1, [I]): UMAC 0x8044acd0
 * writes three config words to +0x0/+0x4/+0x8, then 0x804a6c94 writes 1 to
 * +0x10 and waits ~20 timebase ticks for [+0x14] bit 0 (else fatal 0x101035).
 * No firmware on either core writes +0x14. Model: done follows start at once. */
#define CFGU_BASE 0x00d03f00u
static uint32_t fsm_ctl;

static uint32_t fsm_read(void)
{
    return (fsm_ctl & ~0xfu) | ((fsm_ctl & 0x10) ? 5 : 0);
}

/* LMAC outgoing message FIFO at 0xa00400 (finding Q1, [I]): LMAC send
 * routine 0x13140(ptr, len, id) computes free = max([+0x30] - [+0x38] - 8, 0),
 * asserts with the message id if len does not fit (the 0x3601 assert seen
 * when both read 0), else pushes len/4 words to +0x40. Model: the consumer
 * (UMAC or host, [?]) drains at once, so the fill level +0x38 reads 0;
 * capacity +0x30 = 0x400 bytes is a guess. Pushed words are traced as MSG. */
#define MFIFO_BASE 0x00a00400u
#define MFIFO_CAP  0x400u

/* UMAC outgoing message FIFO at 0xc0a00500 (finding Q2, [I]): the layout of
 * Q1 in UMAC's own 0xc0a0xxxx block. UMAC 0xc008d0a0 selects channel 0xe at
 * +0x3c (LMAC does the same at 0xa0043c), takes free = max([+0x30] - [+0x38]
 * - 8, 0) and pushes through 0x804a8d48 to +0x40; error 0x100efa if the
 * message does not fit. Modelled as Q1. */
#define MFIFO_UMAC 0x00a00500u     /* UMAC 0xc0a00500 (K3) */

static bool is_mfifo(hwaddr a, unsigned off)
{
    return a == MFIFO_BASE + off || a == MFIFO_UMAC + off;
}

/* Hardware semaphore bank at 0xd03800 (finding S1, [I]): LMAC 0x4ba960
 * writes a bit mask to +0x0 (request) and returns whether [+0x4] (granted)
 * has it; 0x4d1c88 writes 1<<n to +0x8 (release). Both mirror mask<<8 into
 * 0xa01e30 (not interpreted). Presumably arbitrates LMAC vs UMAC; with one
 * core modelled, any request for a free bit is granted at once. */
#define SEM_BASE 0x00d03800u
/* Per-core register sets: LMAC at +0x00, UMAC at +0x20 (UMAC 0xc0d03820/24/28
 * observed at its boot, 0xc008d258/0xc008d278). Each bit is owned by at most
 * one core. */
static int8_t sem_owner[32] = { [0 ... 31] = -1 };

static uint32_t sem_granted(int core)
{
    uint32_t m = 0;
    for (int i = 0; i < 32; i++) {
        if (sem_owner[i] == core) {
            m |= 1u << i;
        }
    }
    return m;
}

static void sem_update(int core, uint32_t mask, bool request)
{
    for (int i = 0; i < 32; i++) {
        if (!(mask & (1u << i))) {
            continue;
        }
        if (request && sem_owner[i] < 0) {
            sem_owner[i] = core;
        } else if (!request && sem_owner[i] == core) {
            sem_owner[i] = -1;
        }
    }
}

/* LMAC indirect register window (finding W1): CTRL at 0xa2ac04 holds a
 * target address in [16:0], bit 17 = write, bit 18 = GO/BUSY; DATA at
 * 0xa2ac00. LMAC helpers 0x4be6f8 (read) / 0x4be714 (write) / 0x4be818 (wait
 * while BUSY) define the protocol. The target is device address 0xa20000 +
 * addr: UMAC reads 0xb800 as the CNVR ID, which iwl-prph.h names
 * CNVR_AUX_MISC_CHIP at 0xa2b800, and the values firmware writes through
 * 0x9890 / 0x9938 are what silicon reads at 0xa29890 / 0xa29938 (dump in
 * ~/Projects/ax210/ref-silicon/). BUSY self-clears: accesses complete at
 * once. Traced IND.R / IND.W with the window address. */
#define IND_DATA 0x00a2ac00u
#define IND_CTRL 0x00a2ac04u
#define IND_GO   (1u << 18)
#define IND_WR   (1u << 17)
#define IND_ADDR 0x1ffffu
#define IND_BASE 0x00a20000u
/* Request/grant pair behind the window (finding W2, [F]): LMAC 0x4bdf90
 * writes 1 to 0x8c38 (0x4bdfb8) and spins on 0x8c50 bit 0 with no
 * timeout. Model: the grant follows the request bit. */
#define IND_REQ  (IND_BASE + 0x8c38)
#define IND_GNT  (IND_BASE + 0x8c50)
/* Second request/acknowledge pair (W2, [F]; addresses V): UMAC 0x804a1898
 * sets 0xa20818 bit 0 (refcounted, 0x804a193c clears it) and waits for
 * 0xa27cec bit 8 before the image signature check (fatal 0x313). */
#define REQ2     0x00a20818u
#define ACK2     0x00a27cecu
/* Remote register read behind the window (finding W6, [F]): LMAC 0x4b7b40
 * and UMAC write (addr & 0x3ffffff) | bit 29 to 0xc804, read it back
 * (bit 29 still set: fatal 0x93) and take the value from 0xc800. Model:
 * the read completes at once. */
#define RRD_CTL  (IND_BASE + 0xc804)
#define RRD_DATA (IND_BASE + 0xc800)
#define RRD_GO   (1u << 29)

/* Chip identification (finding K2): read-only, silicon values. AX210 dmesg
 * reads "Detected crf-id 0x400410, cnv-id 0x400410 wfpm id 0x80000000"; the
 * driver (pcie/trans.c) takes them from SD_REG_VER_GEN2, CNVI_AUX_MISC_CHIP
 * and WFPM_OTP_CFG1_ADDR (iwl-prph.h; the last at UMAC periphery offset
 * 0x300000). UMAC 0x80448ca4 compares the first two with FSEQ_CNVR_ID /
 * FSEQ_CNVI_ID (0xa34090 / 0xa3408c), fatal 0x1013f1 on a mismatch.
 * FSEQ_OTP_VERSION is OTP content the firmware only reads (UMAC 0x80448cae
 * branches on >= 0xa440); 0xa652 on all three saved AX210 dumps. Sources:
 * the error dumps indexed in ~/Projects/ax210/ref-silicon/README.md, one of
 * them from a card running firmware API 89 (build 89.6b44fa0b; the model
 * runs 89.b20dcd1f). */
static const struct {
    hwaddr a;
    uint32_t v;
} chip_id[] = {
    { 0x00a200b0, 0x00400410 },     /* CNVI_AUX_MISC_CHIP */
    { 0x00a2b800, 0x00400410 },     /* SD_REG_VER_GEN2 = CNVR_AUX_MISC_CHIP */
    { 0x00d03098, 0x80000000 },     /* WFPM_OTP_CFG1_ADDR */
    { 0x00a340fc, 0x0000a652 },     /* FSEQ_OTP_VERSION (per card) */
    /* LMPM_NIC_ISR0-4 (finding R2): an idle AX210's values (LKML dump, the
     * firmware booted and waiting on the host); the Framework card agrees
     * on ISR0/ISR2. LMAC's writes (0xffffffff, ...) never show on silicon.
     * Event sources that would set or clear bits are not modelled. */
    { 0x00a01e20, 0x00020000 },     /* ISR0 */
    { 0x00a01e24, 0x00000000 },     /* ISR1 */
    { 0x00a01e28, 0x48f00002 },     /* ISR2 */
    { 0x00a01e2c, 0x00c0001c },     /* ISR3 */
    { 0x00a01e30, 0x00000000 },     /* ISR4 */
};

static bool chip_id_read(hwaddr a, uint32_t *v)
{
    for (int i = 0; i < ARRAY_SIZE(chip_id); i++) {
        if (chip_id[i].a == a) {
            *v = chip_id[i].v;
            return true;
        }
    }
    return false;
}

/* LMAC timer/event unit at 0xa02000 (finding V1, [I]). From LMAC 0x166e4
 * (enable bit n at +0x0c, count at +0xac+4n, start bit n at +0x08) and
 * 0x16668 (wevt, then return +0x20). +0x20 is a free-running timestamp, not a
 * status: LMAC 0x4d5222/0x4d5280 compute deadline = [+0x20] + N and loop
 * until it passes. Model: a start-bit write arms channel n for `count` ticks
 * (relative; absolute compare is [?]); expiry wakes the core from wevt/sleep
 * as an event (no interrupt delivered: the IRQ line is [?]). 1 tick = 1 us
 * for both, a guess (ch10 = 200000 -> 200 ms; delays of 50/2250 us). */
#define TMR_BASE   0x00a02000u
#define TMR_START  (TMR_BASE + 0x08)
#define TMR_TSTAMP (TMR_BASE + 0x20)
#define TMR_COUNT0 (TMR_BASE + 0xac)
#define TMR_NCH    32
static QEMUTimer *tmr[TMR_NCH];
static CPUState *umac_cs;   /* ARC700 (UMAC) */
static CPUState *lmac_cs;   /* ARCv2EM (LMAC), NULL until dual-core */


static void tmr_expire(void *opaque)
{
    int ch = (int)(uintptr_t)opaque;
    trace_log("TMR.X", TMR_COUNT0 + 4 * ch, ch, 4);
    if (lmac_cs && lmac_cs->halted) {
        lmac_cs->halted = 0;          /* event wake: resume after wevt */
        qemu_cpu_kick(lmac_cs);
    }
}


static void tmr_start(uint32_t mask)
{
    for (int ch = 0; ch < TMR_NCH; ch++) {
        uint32_t cnt;
        if (!(mask & (1u << ch))) {
            continue;
        }
        cnt = rs_get(TMR_COUNT0 + 4 * ch);
        if (!cnt) {
            continue;
        }
        if (!tmr[ch]) {
            tmr[ch] = timer_new_us(QEMU_CLOCK_VIRTUAL, tmr_expire,
                                   (void *)(uintptr_t)ch);
        }
        timer_mod(tmr[ch], qemu_clock_get_us(QEMU_CLOCK_VIRTUAL) + cnt);
        trace_log("TMR.S", TMR_COUNT0 + 4 * ch, cnt, 4);
    }
}

/* UMAC timeout timer, IRQ6 (finding G2, [I]): UMAC's idle loop
 * (0x804a66a0) writes 0x30d40 to 0xd02c78 before each sleep, and its IRQ6
 * handler (0xc0080b30) writes the same value again and 0x10 to 0xd02e2c.
 * Model: a write of N arms a one-shot that, N us later (the tick rate is
 * [?]), sets 0xd02e2c bit 4 and raises IRQ6; writing bit 4 clears both. */
#define TOUT_ARM  0x00d02c78u
#define ISTAT     0x00d02e2cu
#define TOUT_BIT  (1u << 4)
#define TOUT_IRQ  6
static QEMUTimer *tout;
static bool tout_fired;

static void tout_set(bool fired)
{
    CPUARCState *env = &ARC_CPU(umac_cs)->env;

    tout_fired = fired;
    qemu_set_irq(env->irq[TOUT_IRQ], fired);
}

static void tout_expire(void *opaque)
{
    trace_log("TOUT.X", TOUT_ARM, TOUT_IRQ, 4);
    tout_set(true);
}

static void tout_arm(uint32_t n)
{
    if (!umac_cs || !(ARC_CPU(umac_cs)->family & ARC_OPCODE_ARC700)) {
        return;
    }
    if (!tout) {
        tout = timer_new_us(QEMU_CLOCK_VIRTUAL, tout_expire, NULL);
    }
    timer_mod(tout, qemu_clock_get_us(QEMU_CLOCK_VIRTUAL) + n);
}

/* LMAC -> UMAC command path (finding Q3, [F]): LMAC selects TX FIFO 13 at
 * TXF_LARC_NUM and pushes {cmd header, byte length, payload}. UMAC's IRQ5
 * cause 14 handler (0xc00823d0) pops {len_n_flags, cmd header, payload} from
 * 0xd00e68: bit 27 required, (len & 0xfffc) - 4 payload bytes, and
 * 0xd05498 bit 31 required (0x80497898). Model: reframe each complete LMAC
 * frame into that layout; IRQ5 summary bit 14 stays set while words remain.
 * The flag bits and the FIFO number are read off the firmware, not silicon. */
#define TXF_LARC_SEL (MFIFO_BASE + 0x3c)
#define TXF_TO_UMAC  0xd
#define URXF_POP     0x00d00e68u
#define URXF_STAT    0x00d05498u
#define IRQ5_SUM     0x00d02e64u
#define URXF_CAUSE   (1u << 14)
#define URXF_IRQ     5
#define URXF_CAP     0x400u          /* words */
static uint32_t txf_sel;
static uint32_t txf_frame[URXF_CAP];
static unsigned txf_n;
static uint32_t urxf[URXF_CAP];
static unsigned urxf_rd, urxf_wr;

/* Cross-core signals end the sender's round-robin timeslice, so the
 * receiver runs before the sender's busy-wait timeouts (counted in reads of
 * the 0xd02c68 counter, not time) expire. */
static void yield_to_peer(void)
{
    if (current_cpu) {
        cpu_exit(current_cpu);
    }
}

/* DMA completion on IRQ5 (finding G3, [F]): once UMAC enables 0xd02e04
 * bits 29/30 (0x68020000), the IRQ5 handler 0xc008214c takes
 * [0xd02e24] & [0xd02e04] and passes bit 29 (channel 2) or 30 (channel 1)
 * to 0x8049db8c, which posts the completion that a page-in (0x8047f0fc)
 * waits for. Summary bits 2, 7, 23 and 24 all dispatch to that handler;
 * which one the hardware sets is [?], the model uses bit 2. */
#define DMA_IEN    0x00d02e04u
#define DMA_CAUSE  (1u << 2)
static uint32_t dma_done;

static bool dma_irq(void)
{
    return dma_done && (rs_get(DMA_IEN) & (1u << 29));
}

static void urxf_update(void)
{
    CPUARCState *env;

    if (!umac_cs) {
        return;
    }
    env = &ARC_CPU(umac_cs)->env;
    qemu_set_irq(env->irq[URXF_IRQ], urxf_rd != urxf_wr || dma_irq());
}

static void urxf_push(uint32_t w)
{
    if (urxf_wr - urxf_rd < URXF_CAP) {
        urxf[urxf_wr++ % URXF_CAP] = w;
    }
}

static uint32_t urxf_pop(void)
{
    uint32_t w = 0;

    if (urxf_rd != urxf_wr) {
        w = urxf[urxf_rd++ % URXF_CAP];
    }
    urxf_update();
    return w;
}

static void txf_lmac_push(uint32_t w)
{
    if (txf_sel != TXF_TO_UMAC) {
        return;
    }
    if (txf_n < URXF_CAP) {
        txf_frame[txf_n++] = w;
    }
    if (txf_n < 2 || txf_n < 2 + ((txf_frame[1] + 3) >> 2)) {
        return;
    }
    urxf_push((1u << 27) | ((txf_frame[1] + 4) & 0xfffc));
    urxf_push(txf_frame[0]);
    for (unsigned i = 2; i < txf_n; i++) {
        urxf_push(txf_frame[i]);
    }
    trace_log("TXF.U", TXF_LARC_SEL, txf_frame[0], txf_frame[1]);
    txf_n = 0;
    urxf_update();
    yield_to_peer();
}

/* LMAC NIC interrupt, IRQ22 (finding Q4, [F]): LMAC's handler (0xdac)
 * takes cause = [0xa01e80] & [0xa01e84]; LMAC builds the mask at 0xa01e80
 * bit by bit at boot. UMAC writes 0x10 to 0xa01e88 right after pushing a
 * message for LMAC on TXF_CPU2. Model: +0x84 status (write 1 to clear),
 * +0x88 sets status bits; IRQ22 follows status & mask. */
#define NIC_MASK 0x00a01e80u
#define NIC_STAT 0x00a01e84u
#define NIC_SET  0x00a01e88u
#define NIC_IRQ  22
static uint32_t nic_stat;

static void nic_update(void)
{
    if (!lmac_cs) {
        return;
    }
    qemu_set_irq(ARC_CPU(lmac_cs)->env.irq[NIC_IRQ],
                 (nic_stat & rs_get(NIC_MASK)) != 0);
}

/* UMAC -> LMAC command path (finding Q5, [F]): UMAC selects FIFO 14 at
 * TXF_CPU2_NUM (0xa0053c) and pushes {cmd header, byte length, payload} to
 * 0xa00540. LMAC's IRQ19 (0x1bbc) takes cause [0xa01e60] & 0x58902167;
 * bit 28's handler (0x4b78d0) selects FIFO 14 at TXF_LARC_NUM, waits for
 * TXF_FIFO_ITEM_CNT (0xa00438) >= 8 bytes, reads header and length from
 * 0xa00408, then dispatches by group/ID (0x4cd49c). UMAC configured FIFO 14
 * as one SMEM ring (0x23000), so both blocks see the same queue. Model: one
 * word queue that takes whole frames; bit 28 and IRQ19 follow its fill
 * level. */
#define TXF_CPU2_SEL  (MFIFO_UMAC + 0x3c)
#define TXF_TO_LMAC   0xe
#define TXF_POP       (MFIFO_BASE + 0x08)
#define TXF_CNT       (MFIFO_BASE + 0x38)
#define LEV_CAUSE     0x00a01e60u
#define LEV_CMD       (1u << 28)
#define LEV_IRQ       19
static uint32_t txf2_sel;
static uint32_t f14_frame[URXF_CAP];
static unsigned f14_n;
static uint32_t f14[URXF_CAP];
static unsigned f14_rd, f14_wr;

static void f14_update(void)
{
    if (!lmac_cs) {
        return;
    }
    qemu_set_irq(ARC_CPU(lmac_cs)->env.irq[LEV_IRQ], f14_rd != f14_wr);
}

static void f14_push(uint32_t w)
{
    if (f14_n < URXF_CAP) {
        f14_frame[f14_n++] = w;
    }
    if (f14_n < 2 || f14_n < 2 + ((f14_frame[1] + 3) >> 2)) {
        return;
    }
    for (unsigned i = 0; i < f14_n && f14_wr - f14_rd < URXF_CAP; i++) {
        f14[f14_wr++ % URXF_CAP] = f14_frame[i];
    }
    trace_log("TXF.L", TXF_CPU2_SEL, f14_frame[0], f14_frame[1]);
    f14_n = 0;
    f14_update();
    yield_to_peer();
}

static uint32_t f14_pop(void)
{
    uint32_t w = 0;

    if (f14_rd != f14_wr) {
        w = f14[f14_rd++ % URXF_CAP];
    }
    f14_update();
    return w;
}

/* PHY register sync read and PHY engines (finding P1, [F]; the addresses
 * are in the ucode's PHY periphery list, V). LMAC 0x1697c reads a PHY
 * register by writing (addr & 0xfffff) | bit 28 to 0xadfc0c and loading
 * 0xadfc1c; UMAC 0x804a9d70 uses a second port: address to 0xadfc14, wait
 * for 0xadfc24 bit 15, data at 0xadfc20. An engine is started by a pulse on control bit 0 or 1 and
 * polled on its status register for bit 30 or 23 (code site, assert):
 *   0xad131c -> 0xad13c0: b0 (0x4d58a0, 0x13c1; 0xad1324 b29 clears),
 *                         b1 (0x4d57d0 via 0x4bbd40, 0x28a9)
 *   0xad32d0 + 4i -> 0xad3680 + 4i: b0 (0x4d7a64, 0x13c0),
 *                         b1 (0x4d5f38 via 0x4d75a4, 0x28aa)
 * A capture block counts to the length written at 0xac0450 and reports
 * progress at 0xac0458: LMAC 0x4d39a4 reads it twice 5 ticks apart, stores
 * it once stable and non-zero, and 0x4c9988 requires it to equal the 0x1000
 * it programmed (assert 0x14b5).
 * Model: the window reads the register at 0xa00000 | addr; a pulse
 * completes at once and a capture is complete when first read. */
#define PHY_SYNC_ADDR 0x00adfc0cu
#define PHY_SYNC_DATA 0x00adfc1cu
#define PHY_SYNC2_ADDR 0x00adfc14u
#define PHY_SYNC2_DATA 0x00adfc20u
#define PHY_SYNC2_RDY  0x00adfc24u
#define PHY_ENG_CLR   0x00ad1324u
#define PHY_CAP_LEN   0x00ac0450u
#define PHY_CAP_CNT   0x00ac0458u
static const struct { hwaddr ctl, stat; } phy_eng[] = {
    { 0x00ad131c, 0x00ad13c0 },
    { 0x00ad32d0, 0x00ad3680 }, { 0x00ad32d4, 0x00ad3684 },
    { 0x00ad32d8, 0x00ad3688 }, { 0x00ad32dc, 0x00ad368c },
};
static const uint32_t phy_done_bit[2] = { 1u << 30, 1u << 23 };
static uint8_t phy_done[ARRAY_SIZE(phy_eng)];

static bool phy_is_stat(hwaddr a)
{
    if (a == PHY_CAP_CNT) {
        return true;
    }
    for (int i = 0; i < ARRAY_SIZE(phy_eng); i++) {
        if (phy_eng[i].stat == a) {
            return true;
        }
    }
    return false;
}

static uint32_t phy_reg(hwaddr a)
{
    uint32_t v = a == PHY_CAP_CNT ? rs_get(PHY_CAP_LEN) : rs_get(a);

    for (int i = 0; i < ARRAY_SIZE(phy_eng); i++) {
        for (int b = 0; b < 2; b++) {
            if (phy_eng[i].stat == a && (phy_done[i] & (1u << b))) {
                v |= phy_done_bit[b];
            }
        }
    }
    return v;
}

static void phy_write(hwaddr a, uint32_t val)
{
    if (a == PHY_ENG_CLR && (val & (1u << 29))) {
        phy_done[0] = 0;
    }
    for (int i = 0; i < ARRAY_SIZE(phy_eng); i++) {
        if (phy_eng[i].ctl == a) {
            phy_done[i] |= val & 3;
        }
    }
}

#define MBX_BASE 0x00d00000u    /* UMAC 0xc0d00000 */
static uint32_t host_csr[0x1000];   /* 0x0000-0x3fff */
#define HRX_FREE  0x00d00e34u
#define HRX_SIZE  0x00d00e88u
#define HRX_HDR   0x00d00ec0u
#define HRX_DATA  0x00d00e04u
#define HRX_LAST  0x00d00e08u
static uint32_t hrx_buf[0x1000];
static unsigned hrx_n;
static void host_rx_deliver(void);
/* Before a direct send UMAC requests FIFO space with (bytes << 16) | 2 at
 * 0xd05488 and requires 0xd02e28 & 0x30000000 == 0x10000000 (0x804a168c);
 * 0xd02e28 is write-1-to-clear status (0x80493156 clears bit 28, the RX
 * handler bit 29). Model: the request is granted at once. */
#define HRX_REQ   0x00d05488u
#define ISTAT2    0x00d02e28u
#define HRX_GRANT (1u << 28)
static uint32_t istat2;
static void host_alive_restock(void);
static uint32_t mbx_idx, mbx_data;

static target_ulong cur_pc(void)
{
    if (current_cpu) {
        ARCCPU *c = ARC_CPU(current_cpu);
        return c->env.pc;
    }
    return 0;
}

static void trace_log(const char *tag, hwaddr addr, uint64_t val, unsigned size)
{
    if (!trace_fp) {
        const char *p = getenv("AX210_TRACE");
        trace_fp = p ? fopen(p, "w") : stderr;
        if (!trace_fp) trace_fp = stderr;
    }
    if (trace_count++ < trace_cap) {
        fprintf(trace_fp, "%-5s pc=%08x addr=%08x val=%08x sz=%u\n",
                tag, (uint32_t)cur_pc(), (uint32_t)addr, (uint32_t)val, size);
        fflush(trace_fp);
    }
}

static uint64_t trace_read(void *opaque, hwaddr addr, unsigned size)
{
    hwaddr base = (hwaddr)(uintptr_t)opaque;
    hwaddr ra = base + addr;            /* as issued by the core */
    hwaddr a = lmac_view(ra);           /* model key */
    int hit; uint32_t v = override_lookup(a, &hit);
    if (a == DMA_STAT && !hit) {
        v = (dma_done ? (1u << 29) : 0) | watch_evt;
        trace_log("R.dma", ra, v, size);
        return v;
    }
    if (a == TMR_TSTAMP && !hit) {
        v = (uint32_t)qemu_clock_get_us(QEMU_CLOCK_VIRTUAL);
        trace_log("R.ts", ra, v, size);
        return v;
    }
    if (is_mfifo(a, 0x30) && !hit) {
        trace_log("R.mfifo", ra, MFIFO_CAP, size);
        return MFIFO_CAP;
    }
    if (a == TXF_CNT && txf_sel == TXF_TO_LMAC && !hit) {
        v = 4 * (f14_wr - f14_rd);
        trace_log("R.f14n", ra, v, size);
        return v;
    }
    if (a == TXF_POP && txf_sel == TXF_TO_LMAC && !hit) {
        v = f14_pop();
        trace_log("R.f14", ra, v, size);
        return v;
    }
    if (a == PHY_SYNC_DATA && !hit) {
        hwaddr t = 0x00a00000u | (rs_get(PHY_SYNC_ADDR) & 0xfffffu);
        v = phy_reg(t);
        trace_log("R.phy", t, v, size);
        return v;
    }
    if (a == PHY_SYNC2_DATA && !hit) {
        hwaddr t = 0x00a00000u | (rs_get(PHY_SYNC2_ADDR) & 0xfffffu);
        v = phy_reg(t);
        trace_log("R.phy", t, v, size);
        return v;
    }
    if (a == PHY_SYNC2_RDY && !hit) {
        v = rs_get(a) | (1u << 15);
        trace_log("R.phy", ra, v, size);
        return v;
    }
    if (phy_is_stat(a) && !hit) {
        v = phy_reg(a);
        trace_log("R.phy", ra, v, size);
        return v;
    }
    if (sha_out_word(a) >= 0 && !hit) {
        v = sha_digest(sha_out_word(a));
        trace_log("R.sha", ra, v, size);
        return v;
    }
    if ((a == SHA_STAT || a == SHA_CNT) && !hit) {
        v = a == SHA_CNT ? sha_cnt : (sha_done ? 2 : 0);
        trace_log("R.sha", ra, v, size);
        return v;
    }
    if (a == ISTAT2 && !hit) {
        trace_log("R.ist2", ra, istat2, size);
        return istat2;
    }
    if (a == HRX_FREE && !hit) {
        v = rs_get(HRX_SIZE);
        trace_log("R.hrx", ra, v, size);
        return v;
    }
    if (a == BLK_STAT && !hit) {
        v = (rs_get(BLK_STAT) & ~(1u << 31)) |
            ((rs_get(BLK_CTL) & 1) ? 0 : 1u << 31);
        trace_log("R.blk", ra, v, size);
        return v;
    }
    if (a == PGM_RES && !hit) {
        trace_log("R.pgm", ra, pgm_res, size);
        return pgm_res;
    }
    if (a == PKA_STAT && !hit) {
        v = pka_busy ? 1 : 0;
        if (pka_busy) {
            pka_busy--;
        }
        trace_log("R.pka", ra, v, size);
        return v;
    }
    if (((a >= PKA_RES0 && a < PKA_RES0 + 0x30) ||
         (a >= PKA_RES1 && a < PKA_RES1 + 0x30)) && !hit) {
        trace_log("R.pka", ra, pka_res, size);
        return pka_res;
    }
    if (a == PGE_STAT && !hit) {
        trace_log("R.pge", ra, pge_stat, size);
        return pge_stat;
    }
    if (a == ACK2 && !hit) {
        v = (rs_get(ACK2) & ~(1u << 8)) | ((rs_get(REQ2) & 1) << 8);
        trace_log("R.ack", ra, v, size);
        return v;
    }
    if (a == LEV_CAUSE && !hit) {
        v = rs_get(LEV_CAUSE) | (f14_rd != f14_wr ? LEV_CMD : 0);
        trace_log("R.lev", ra, v, size);
        return v;
    }
    if (is_mfifo(a, 0x38) && !hit) {
        trace_log("R.mfifo", ra, 0, size);
        return 0;
    }
    if ((a == SEM_BASE + 0x04 || a == SEM_BASE + 0x24) && !hit) {
        v = sem_granted(a == SEM_BASE + 0x24);
        trace_log("R.sem", ra, v, size);
        return v;
    }
    if (a == CFGU_BASE + 0x14 && !hit) {
        v = rs_get(CFGU_BASE + 0x10) & 1;
        trace_log("R.cfgu", ra, v, size);
        return v;
    }
    if (a == ISTAT && !hit) {
        v = (rs_get(ISTAT) & ~TOUT_BIT) | (tout_fired ? TOUT_BIT : 0);
        trace_log("R.istat", ra, v, size);
        return v;
    }
    if (a == NIC_STAT && !hit) {
        trace_log("R.nic", ra, nic_stat, size);
        return nic_stat;
    }
    if (a == URXF_POP && !hit) {
        v = urxf_pop();
        trace_log("R.rxf", ra, v, size);
        return v;
    }
    if (a == IRQ5_SUM && !hit) {
        v = rs_get(IRQ5_SUM) | (urxf_rd != urxf_wr ? URXF_CAUSE : 0) |
            (dma_irq() ? DMA_CAUSE : 0);
        trace_log("R.sum", ra, v, size);
        return v;
    }
    if (a == URXF_STAT && !hit) {
        v = rs_get(URXF_STAT) | (1u << 31);
        trace_log("R.rxfs", ra, v, size);
        return v;
    }
    if (a == FSM_BASE && !hit) {
        v = fsm_read();
        trace_log("R.fsm", ra, v, size);
        return v;
    }
    if (a == MBX_BASE + 0x28 && !hit) {
        v = host_csr[(mbx_idx >> 2) & 0xfff];
        trace_log("R.csr", mbx_idx, v, size);
        return v;
    }
    for (int i = 0; i < n_ctrs; i++) {
        if (ctrs[i].addr == a) {
            v = ++ctrs[i].val;
            trace_log("R.ctr", ra, v, size);
            return v;
        }
    }
    if (!hit && chip_id_read(a, &v)) {
        trace_log("R.id", ra, v, size);
        return v;
    }
    if (!hit && regstore) {
        gpointer p = g_hash_table_lookup(regstore,
                                         GUINT_TO_POINTER((guint)(a)));
        v = (uint32_t)(uintptr_t)p;
    }
    trace_log(hit ? "R.ovr" : "R", ra, v, size);
    return v;
}

static void trace_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    hwaddr base = (hwaddr)(uintptr_t)opaque;
    hwaddr ra = base + addr;            /* as issued by the core */
    hwaddr a = lmac_view(ra);           /* model key */
    trace_log("W", ra, val, size);
    if (a == MBX_BASE + 0x20) {
        mbx_idx = val;
    } else if (a == MBX_BASE + 0x24) {
        mbx_data = val;
    } else if (a == MBX_BASE + 0x1c && (val >> 24) == 0x69) {
        host_csr[(mbx_idx >> 2) & 0xfff] = mbx_data;
        if (mbx_idx == 0x008 && (mbx_data & 1)) {   /* CSR_INT_BIT_ALIVE */
            host_alive_restock();
        }
        trace_log("W.csr", mbx_idx, mbx_data, size);
    }
    if (a == DMA_STAT && (val & (1u << 29))) {
        dma_done = 0;
    }
    if (a == DMA_STAT || a == DMA_IEN) {
        urxf_update();
    }
    if (a == DMA_STAT) {
        watch_evt &= ~((uint32_t)val & WATCH_EVT);
    }
    sha_write(a, (uint32_t)val);
    if (a == ISTAT2) {
        istat2 &= ~(uint32_t)val;
    } else if (a == HRX_REQ && (val & 2)) {
        istat2 |= HRX_GRANT;
    }
    if (a == HRX_HDR) {
        hrx_buf[0] = (uint32_t)val;
        hrx_n = 1;
    } else if ((a == HRX_DATA || a == HRX_LAST) && hrx_n &&
               hrx_n < ARRAY_SIZE(hrx_buf)) {
        hrx_buf[hrx_n++] = (uint32_t)val;
        if (a == HRX_LAST) {
            host_rx_deliver();
        }
    }
    if (a == PGM_REG) {
        pgm_register((uint32_t)val & 0x3ff, true);
    } else if (a == PGM_UNREG) {
        pgm_register((uint32_t)val & 0x3ff, false);
    } else if (a == PGM_QUERY) {
        pgm_query();
    }
    if (a == PKA_CMD) {
        pka_cmd((uint32_t)val);
    } else if (a >= PKA_DATA && a < PKA_DATA + 0x30 && pka_sel >= 0) {
        pka_slot[pka_sel][(a - PKA_DATA) / 4] = (uint32_t)val;
    }
    if (a == PGE_CMD) {
        pge_stat = val == 1 ? 4 : val == 2 ? 1 : 0;
    }
    if (!regstore) {
        regstore = g_hash_table_new(g_direct_hash, g_direct_equal);
    }
    if (a == AES_KEYF) {
        aes_key[aes_kidx++ & 3] = (uint32_t)val;
    }
    if (a == AES_CTRL && (val & 1)) {
        g_hash_table_insert(regstore, GUINT_TO_POINTER((guint)(a)),
                            (gpointer)(uintptr_t)(uint32_t)val);
        aes_run((uint32_t)val);
        val = (val & ~1ull) | 4;           /* done */
    }
    if (is_mfifo(a, 0x40)) {
        trace_log("MSG", ra, val, size);
    }
    if (a == TXF_LARC_SEL) {
        txf_sel = (uint32_t)val;
        txf_n = 0;
    } else if (a == MFIFO_BASE + 0x40) {
        txf_lmac_push((uint32_t)val);
    } else if (a == TXF_CPU2_SEL) {
        txf2_sel = (uint32_t)val;
        f14_n = 0;
    } else if (a == MFIFO_UMAC + 0x40 && txf2_sel == TXF_TO_LMAC) {
        f14_push((uint32_t)val);
    }
    if (a == SEM_BASE || a == SEM_BASE + 0x20) {
        sem_update(a == SEM_BASE + 0x20, (uint32_t)val, true);
    } else if (a == SEM_BASE + 0x08 || a == SEM_BASE + 0x28) {
        sem_update(a == SEM_BASE + 0x28, (uint32_t)val, false);
    }
    if (a == FSM_BASE) {
        fsm_ctl = (uint32_t)val & ~0xfu;
    } else if (a == FSM_BASE + 4) {
        fsm_ctl &= ~(uint32_t)val;
    } else if (a == FSM_BASE + 8) {
        fsm_ctl |= (uint32_t)val & ~0xfu;
    }
    if (a == TOUT_ARM) {
        tout_arm((uint32_t)val);
    } else if (a == ISTAT && (val & TOUT_BIT) && tout_fired) {
        tout_set(false);
    }
    if (a == TMR_START) {
        /* store first so tmr_start sees counts already written */
        tmr_start((uint32_t)val);
    }
    if (a == IND_CTRL && (val & IND_GO)) {
        guint ia = val & IND_ADDR;
        hwaddr ta = IND_BASE + ia;
        if (val & IND_WR) {
            uint32_t d = rs_get(IND_DATA);
            g_hash_table_insert(regstore, GUINT_TO_POINTER((guint)ta),
                                GUINT_TO_POINTER(d));
            trace_log("IND.W", ia, d, 4);
            if (ta == RRD_CTL && (d & RRD_GO)) {
                hwaddr t = d & 0x3ffffffu;
                uint32_t r;
                if (!chip_id_read(t, &r)) {
                    r = phy_reg(t);
                }
                g_hash_table_insert(regstore, GUINT_TO_POINTER(RRD_DATA),
                                    GUINT_TO_POINTER(r));
                g_hash_table_insert(regstore, GUINT_TO_POINTER((guint)ta),
                                    GUINT_TO_POINTER(d & ~RRD_GO));
                trace_log("RRD", t, r, 4);
            }
            if (ta == ENG_CTL && d && (rs_get(WATCH_CTL) & WATCH_ARM)) {
                hwaddr lo = SMEM_BASE | ((rs_get(WATCH_RANGE) & 0xfff) << 8);
                hwaddr hi = SMEM_BASE | (rs_get(WATCH_CTL) & 0xfffff);
                for (hwaddr w = lo; w + 4 <= hi; w += 4) {
                    uint32_t r;
                    qemu_guest_getrandom_nofail(&r, sizeof(r));
                    address_space_write(&address_space_memory, w,
                                        MEMTXATTRS_UNSPECIFIED, &r, 4);
                }
                watch_evt |= WATCH_EVT;
                trace_log("WATCH", WATCH_CTL, d, 4);
            }
        } else {
            uint32_t d;
            if (!chip_id_read(ta, &d)) {
                d = rs_get(ta);
            }
            if (ta == IND_GNT) {
                d = (d & ~1u) | (rs_get(IND_REQ) & 1);
            }
            g_hash_table_insert(regstore, GUINT_TO_POINTER(IND_DATA),
                                GUINT_TO_POINTER(d));
            trace_log("IND.R", ia, d, 4);
        }
        val &= ~(uint64_t)IND_GO;   /* access completes at once: not busy */
    }
    g_hash_table_insert(regstore, GUINT_TO_POINTER((guint)(a)),
                        (gpointer)(uintptr_t)(uint32_t)val);
    if (a == DMA_BASE + 0x34) {
        dma_kick();
    }
    phy_write(a, (uint32_t)val);
    if (a == NIC_STAT) {
        nic_stat &= ~(uint32_t)val;
    } else if (a == NIC_SET) {
        nic_stat |= (uint32_t)val;
        trace_log("NIC.S", ra, nic_stat, size);
        yield_to_peer();
    }
    if (a == NIC_MASK || a == NIC_STAT || a == NIC_SET) {
        nic_update();
    }
}

static uint32_t rs_get(hwaddr a)
{
    return regstore ? (uint32_t)(uintptr_t)g_hash_table_lookup(regstore,
                          GUINT_TO_POINTER((guint)a)) : 0;
}

static void dma_kick(void)
{
    uint32_t cmd = rs_get(DMA_BASE + 0x34);
    uint32_t dev = rs_get(DMA_BASE + 0x38);
    uint64_t host = ((uint64_t)rs_get(DMA_BASE + 0x40) << 32) |
                    rs_get(DMA_BASE + 0x3c);
    uint32_t len = rs_get(DMA_BASE + 0x44);
    uint8_t *buf;
    MemTxResult r1, r2;

    dma_count++;
    if (cmd != 0x81000001 || len == 0 || len > 0x100000) {
        trace_log("DMA?", cmd, len, 4);
        dma_done = 1;
        return;
    }
    buf = g_malloc(len);
    r1 = address_space_read(&address_space_memory, host,
                            MEMTXATTRS_UNSPECIFIED, buf, len);
    r2 = address_space_write(&address_space_memory, dev,
                             MEMTXATTRS_UNSPECIFIED, buf, len);
    fprintf(trace_fp ? trace_fp : stderr,
            "DMA#%u host=%llx -> dev=%08x len=%x rd=%d wr=%d\n", dma_count,
            (unsigned long long)host, dev, len, r1, r2);
    g_free(buf);
    dma_done = 1;
    urxf_update();
}

/* Host driver model (finding H1): what iwlwifi leaves in host DRAM and the
 * CSRs before the device boots, built from a .ucode file (AX210_HOSTFW) the
 * way iwl_pcie_ctxt_info_v2_alloc() / _kick() do it (pcie/ctxt-info-v2.c,
 * pcie/ctxt-info.c iwl_pcie_init_fw_sec(), iwl-drv.c iwl_store_ucode_sec()).
 * Allocation is a page-granular bump allocator over the host RAM window, in
 * the driver's order. Rings are allocated at their driver sizes but left
 * empty; the debug destination (hwm_cfg) is not configured. */
#define HOST_PAGE        0x1000u
#define CPU1_CPU2_SEP    0xffffccccu
#define PAGING_SEP       0xaaaabbbbu
#define TLV_SEC_RT       19
#define TLV_IML          52
static hwaddr host_brk = 0x20000000;

static hwaddr host_alloc(uint32_t len)
{
    hwaddr a = host_brk;
    host_brk += ROUND_UP(len, HOST_PAGE);
    return a;
}

static hwaddr host_alloc_copy(const void *data, uint32_t len)
{
    hwaddr a = host_alloc(len);
    address_space_write(&address_space_memory, a, MEMTXATTRS_UNSPECIFIED,
                        data, len);
    return a;
}

static void host_csr_write64(unsigned off, uint64_t v)
{
    host_csr[off >> 2] = (uint32_t)v;
    host_csr[(off + 4) >> 2] = v >> 32;
}

/* The driver's answer to the ALIVE interrupt (H1): on gen2 parts it does not
 * restock RX before the kick ("Restock will be done at alive, after firmware
 * configured the RFH", pcie/rx.c iwl_pcie_gen2_rx_init); on CSR_INT_BIT_ALIVE
 * iwl_pcie_rxmq_restock posts every free buffer as a 16-byte transfer
 * descriptor {rbid, addr} (iwl_pcie_restock_bd) and writes the index, rounded
 * down to 8, to RFH_Q0_FRBDCB_WIDX_TRG (CSR 0x1c80). 4096 buffers, one kept
 * back (iwl_trans_get_num_rbds: 2 * IWL_NUM_RBDS_HE); RB IDs follow the
 * pool (the allocator keeps IDs 1-6). The device-side RFH_Q0_FRBDCB_WIDX
 * (0xa08080, iwl-fh.h) is given the same value [I]. */
#define HOST_RX_NBUF    4095
#define HOST_RX_RBID0   7
static hwaddr host_rx_bd, host_rx_used, host_rx_stts;
static uint32_t host_rx_closed;

/* Device -> host RX (finding H2, [F] device side, V host side): UMAC's
 * sender 0x8049305c checks the free space at 0xd00e34 (else it arms the
 * space interrupt via 0xd0549c and waits), writes a packet's len_n_flags to
 * 0xd00ec0, its words to 0xd00e04 and the last to 0xd00e08. The RFH then
 * DMAs it into the next free host RB, as iwlwifi expects on AX210
 * (pcie/rx.c): payload at the RB address from the free descriptor
 * {rbid, addr}, a 32-byte iwl_rx_completion_desc with the rbid at offset 4
 * in the used ring, closed_rb_num (le16) in the status area, then
 * CSR_INT_BIT_FH_RX. Model: the FIFO is drained at once (free space = its
 * configured size, 0xd00e88), and each packet is delivered whole. */

static void host_rx_deliver(void)
{
    uint8_t d[16], cd[32] = { 0 };
    uint32_t k = host_rx_closed % HOST_RX_NBUF;
    hwaddr rb;

    if (!host_rx_bd || !hrx_n) {
        return;
    }
    address_space_read(&address_space_memory, host_rx_bd + 16 * k,
                       MEMTXATTRS_UNSPECIFIED, d, sizeof(d));
    rb = ldq_le_p(d + 8);
    for (unsigned i = 0; i < hrx_n; i++) {
        uint32_t w = cpu_to_le32(hrx_buf[i]);
        address_space_write(&address_space_memory, rb + 4 * i,
                            MEMTXATTRS_UNSPECIFIED, &w, 4);
        trace_log(i ? "HRX.w" : "HRX.P", rb + 4 * i, hrx_buf[i], 4);
    }
    stw_le_p(cd + 4, lduw_le_p(d));
    address_space_write(&address_space_memory, host_rx_used + 32 * k,
                        MEMTXATTRS_UNSPECIFIED, cd, sizeof(cd));
    host_rx_closed++;
    stw_le_p(d, host_rx_closed & 0xfff);
    address_space_write(&address_space_memory, host_rx_stts,
                        MEMTXATTRS_UNSPECIFIED, d, 2);
    host_csr[0x008 >> 2] |= 1u << 31;                   /* CSR_INT_BIT_FH_RX */
    hrx_n = 0;
}

static void host_alive_restock(void)
{
    static bool done;
    uint8_t d[16] = { 0 };
    uint32_t widx = HOST_RX_NBUF & ~7u;

    if (!host_rx_bd || done) {
        return;
    }
    done = true;
    for (uint32_t k = 0; k < HOST_RX_NBUF; k++) {
        stw_le_p(d, HOST_RX_RBID0 + k);
        stq_le_p(d + 8, host_alloc(HOST_PAGE));
        address_space_write(&address_space_memory, host_rx_bd + 16 * k,
                            MEMTXATTRS_UNSPECIFIED, d, sizeof(d));
    }
    host_csr[0x1c80 >> 2] = widx;
    if (!regstore) {
        regstore = g_hash_table_new(g_direct_hash, g_direct_equal);
    }
    g_hash_table_insert(regstore, GUINT_TO_POINTER(0x00a08080u),
                        GUINT_TO_POINTER(widx));
    trace_log("HOST.restock", host_rx_bd, widx, 4);
}

static void ax210_host_driver_init(const char *path)
{
    gchar *buf;
    gsize blen;
    size_t off = 88;                       /* sizeof(iwl_tlv_ucode_header) */
    struct { uint32_t offset; const uint8_t *data; uint32_t len; } sec[256];
    int nsec = 0, i, n[3] = { 0 }, part = 0;
    const uint8_t *iml = NULL;
    uint32_t iml_len = 0;
    uint8_t ps[0x6bc] = { 0 };             /* sizeof(iwl_prph_scratch) */
    uint8_t ci[0x68] = { 0 };              /* sizeof(iwl_context_info_v2) */
    hwaddr rx_bd, rx_used, rx_stts, cmdq, ps_a, pi_a, ci_a, iml_a;

    if (!g_file_get_contents(path, &buf, &blen, NULL)) {
        error_report("ax210: cannot read %s", path);
        exit(1);
    }
    while (off + 8 <= blen) {
        uint32_t t = ldl_le_p(buf + off), l = ldl_le_p(buf + off + 4);
        const uint8_t *d = (const uint8_t *)buf + off + 8;
        off += 8 + ROUND_UP(l, 4);
        if (off > ROUND_UP(blen, 4)) {
            break;
        }
        if (t == TLV_SEC_RT && l >= 4 && nsec < ARRAY_SIZE(sec)) {
            sec[nsec].offset = ldl_le_p(d);
            sec[nsec].data = d + 4;
            sec[nsec++].len = l - 4;
        } else if (t == TLV_IML) {
            iml = d;
            iml_len = l;
        }
    }

    /* iwl_pcie_gen2_nic_init(): RX queue and command queue rings. */
    rx_bd = host_alloc(4096 * 16);         /* 2 * IWL_NUM_RBDS_HE transfer descs */
    host_rx_bd = rx_bd;
    host_rx_used = rx_used;
    host_rx_stts = rx_stts;
    rx_used = host_alloc(4096 * 32);       /* completion descs */
    rx_stts = host_alloc(HOST_PAGE);
    cmdq = host_alloc(128 * 256);          /* min_txq_size TFDs, 256B format */

    ps_a = host_alloc(sizeof(ps));
    stw_le_p(ps + 0x00, host_csr[0x028 >> 2]);          /* version.mac_id */
    stw_le_p(ps + 0x04, sizeof(ps) / 4);                /* version.size */
    stl_le_p(ps + 0x08, (1u << 16) | (1u << 17) | 0xc0000); /* control_flags */
    stq_le_p(ps + 0x30, rx_bd);                         /* rbd_cfg.free_rbd_addr */
    for (i = 0; i < nsec; i++) {
        hwaddr a;
        if (sec[i].offset == CPU1_CPU2_SEP || sec[i].offset == PAGING_SEP) {
            if (++part > 2) {
                break;
            }
            continue;
        }
        if (n[part] >= 64) {
            continue;
        }
        a = host_alloc_copy(sec[i].data, sec[i].len);
        /* dram.common: umac_img[64] at 0x7c, lmac_img[64], virtual_img[64] */
        stq_le_p(ps + 0x7c + (part == 0 ? 0x200 : part == 1 ? 0 : 0x400) +
                 8 * n[part]++, a);
    }
    pi_a = host_alloc(HOST_PAGE);
    ci_a = host_alloc(sizeof(ci));
    stq_le_p(ci + 0x08, pi_a);                          /* prph_info_base_addr */
    stq_le_p(ci + 0x10, rx_stts);                       /* cr_head_idx_arr */
    stq_le_p(ci + 0x18, pi_a + HOST_PAGE / 2);          /* tr_tail_idx_arr */
    stq_le_p(ci + 0x20, pi_a + 3 * HOST_PAGE / 4);      /* cr_tail_idx_arr */
    stq_le_p(ci + 0x34, cmdq);                          /* mtr_base_addr */
    stq_le_p(ci + 0x3c, rx_used);                       /* mcr_base_addr */
    stw_le_p(ci + 0x44, 4);                             /* TFD_QUEUE_CB_SIZE(128) */
    stw_le_p(ci + 0x46, 12);                            /* RX_QUEUE_CB_SIZE(4096) */
    stq_le_p(ci + 0x58, ps_a);                          /* prph_scratch_base_addr */
    stl_le_p(ci + 0x60, 0x67c);         /* offsetofend(prph_scratch, dram.common) */
    address_space_write(&address_space_memory, ps_a, MEMTXATTRS_UNSPECIFIED,
                        ps, sizeof(ps));
    address_space_write(&address_space_memory, ci_a, MEMTXATTRS_UNSPECIFIED,
                        ci, sizeof(ci));
    iml_a = iml ? host_alloc_copy(iml, iml_len) : 0;

    /* iwl_pcie_ctxt_info_v2_kick() */
    host_csr_write64(0x118, ci_a);                      /* CSR_CTXT_INFO_ADDR */
    host_csr_write64(0x120, iml_a);                     /* CSR_IML_DATA_ADDR */
    host_csr[0x128 >> 2] = iml_len;                     /* CSR_IML_SIZE_ADDR */
    host_csr[0x000 >> 2] |= 1u << 1;                    /* CSR_AUTO_FUNC_BOOT_ENA */
    fprintf(trace_fp ? trace_fp : stderr,
            "HOST ctxt=%llx scratch=%llx info=%llx iml=%llx/%x lmac=%d umac=%d "
            "paging=%d\n", (unsigned long long)ci_a, (unsigned long long)ps_a,
            (unsigned long long)pi_a, (unsigned long long)iml_a, iml_len,
            n[0], n[1], n[2]);
    g_free(buf);
}

static const MemoryRegionOps trace_ops = {
    .read = trace_read,
    .write = trace_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
};

static void add_ram(MemoryRegion *parent, const char *name, hwaddr base,
                    uint64_t size)
{
    MemoryRegion *r = g_new(MemoryRegion, 1);
    memory_region_init_ram(r, NULL, name, size, &error_fatal);
    memory_region_add_subregion(parent, base, r);
}

static void add_trace(MemoryRegion *parent, const char *name, hwaddr base,
                      uint64_t size, int prio)
{
    MemoryRegion *r = g_new(MemoryRegion, 1);
    memory_region_init_io(r, NULL, &trace_ops, (void *)(uintptr_t)base,
                          name, size);
    memory_region_add_subregion_overlap(parent, base, r, prio);
}

static void add_alias(const char *name, MemoryRegion *target, hwaddr base,
                      uint64_t size)
{
    MemoryRegion *r = g_new(MemoryRegion, 1);
    memory_region_init_alias(r, NULL, name, target, 0, size);
    memory_region_add_subregion(get_system_memory(), base, r);
}

/* ---- vendor aux registers ---------------------------------------------- */

/* Aux registers the CPU model does not define (e.g. LMAC reset writes
 * 0xa0000000 to aux 0x20a). Policy matches unmodelled MMIO (finding R1):
 * read back the last value written, and trace every access. */
static GHashTable *auxstore;

static void ax210_aux_write(CPUARCState *env, target_ulong aux,
                            target_ulong val)
{
    if (!auxstore) {
        auxstore = g_hash_table_new(g_direct_hash, g_direct_equal);
    }
    g_hash_table_insert(auxstore, GUINT_TO_POINTER((guint)aux),
                        GUINT_TO_POINTER((guint)val));
    trace_log("AUX.W", aux, val, 4);
}

static bool ax210_aux_read(CPUARCState *env, target_ulong aux,
                           target_ulong *val)
{
    *val = auxstore ? GPOINTER_TO_UINT(g_hash_table_lookup(auxstore,
                                       GUINT_TO_POINTER((guint)aux))) : 0;
    trace_log("AUX.R", aux, *val, 4);
    return true;
}

/* ---- machine ------------------------------------------------------------ */

static struct arc_boot_info ax210_boot_info;   /* zeroed: no kernel/cmdline */

/* LMAC extension core register r32 (finding X2, [I]): a hardware status word
 * the LMAC only reads (btst/bbit/tst/xbfu). Its code waits for bit 29 to be
 * set after kicking an engine (0x4bc4b0, 0xc202) and for bits 12/18/25 to be
 * clear (0x4bc48a, 0xc5be). Model: constant, bit 29 ("idle/done") set and
 * busy bits clear; AX210_R32 overrides. Only applied to the ARCv2 (LMAC)
 * core; the ARC700 (UMAC) core has no such use. */
static uint32_t r32_init = 1u << 29;

static void ax210_reset(void *opaque)
{
    ARCCPU *cpu = opaque;

    arc_cpu_reset(cpu);
    if (cpu->family & ARC_OPCODE_ARCv2EM) {
        cpu->env.r[32] = r32_init;
    }
}

static void ax210_init(MachineState *machine)
{
    ARCCPU *umac = ARC_CPU(object_new(TYPE_ARC_CPU_ARC700));
    ARCCPU *lmac = ARC_CPU(object_new(TYPE_ARC_CPU_ARCEM));

    (void)machine;   /* both cores are fixed; -cpu/-smp are not consulted */

    /* LMAC programs 13 MPU region pairs (aux 0x422..0x43b, at
     * 0x4be574); ARCv2 MPUs come in power-of-two sizes. Finding M2. */
    object_property_set_uint(OBJECT(lmac), "mpu-numreg", 16, &error_fatal);
    /* UMAC pages 4 KB blocks (TLB PD0 VPN from bit 12, 0xc0085010;
     * page-in in 0x1000 units, 0x8047f0fc). Finding A12. */
    object_property_set_uint(OBJECT(umac), "mmu-pagesize0", 12, &error_fatal);
    /* LMAC (ARC EM) has an MPU and no MMU (finding M2). */
    object_property_set_bool(OBJECT(lmac), "has-mmu", false, &error_fatal);

    object_property_set_uint(OBJECT(umac), "freq_hz", 1000000, &error_fatal);
    object_property_set_uint(OBJECT(lmac), "freq_hz", 1000000, &error_fatal);
    object_property_set_bool(OBJECT(umac), "rtc-opt", true, &error_fatal);
    object_property_set_bool(OBJECT(lmac), "rtc-opt", true, &error_fatal);
    object_property_set_bool(OBJECT(umac), "realized", true, &error_fatal);
    object_property_set_bool(OBJECT(lmac), "realized", true, &error_fatal);

    cpu_arc_pic_init(umac);
    cpu_arc_pic_init(lmac);
    cpu_arc_clock_init(umac);
    cpu_arc_clock_init(lmac);

    umac->env.boot_info = &ax210_boot_info;
    lmac->env.boot_info = &ax210_boot_info;

    if (getenv("AX210_R32")) {
        r32_init = strtoul(getenv("AX210_R32"), 0, 0);
    }

    qemu_register_reset(ax210_reset, umac);
    qemu_register_reset(ax210_reset, lmac);

    umac_cs = CPU(umac);
    lmac_cs = CPU(lmac);

    arc_unknown_aux_write = ax210_aux_write;
    arc_unknown_aux_read = ax210_aux_read;

    MemoryRegion *sys = get_system_memory();
    MemoryRegion *dev = g_new(MemoryRegion, 1);

    /* Catch-all tracer over the whole 32-bit space (lowest priority). */
    add_trace(sys, "ax210.catchall", 0x00000000, 0x100000000ULL, -1000);

    /* Device address space (finding M3, [F]): what LMAC addresses directly
     * and the driver's debug dumps name (the .ucode's region TLVs: hw_smem
     * 0x400000, sw_smem 0x440000, chmem 0x620000, dccm_lmac1 0x800000,
     * dccm_umac 0x880000, UMAC error table 0x471488). UMAC sees it through
     * its cached 0x80000000 and uncached 0xc0000000 windows: UMAC names its
     * error table 0x80471488, its bss 0xc0880000.
     *
     * Both cores share this one address space. The UMAC/IML runs from
     * a contiguous low window (incl. the paged image); the LMAC's own MPU
     * programming carves named regions out of that same window (finding M2,
     * [F] for the firmware's view; read live at the main loop, decoded with
     * the ARCv2 RDB/RDP encoding of Zephyr arc_mpu_v2_internal.h). Adding the
     * flat window first lets the LMAC regions overlay it at equal priority so
     * the MPU view stays visible in `info mtree`; both are plain RAM, so the
     * two cores see the same bytes either way. Peripheral regions
     * 0xa00000-0xbfffff and 0xd00000-0xefffff are the lmac_periph traces
     * below; everything else unmapped reaches the catch-all tracer. */
    memory_region_init(dev, NULL, "ax210.dev", 0x01100000);
    add_ram(dev, "ax210.lmac",  0x00000000, 0x01100000); /* UMAC/IML flat low window */
    add_ram(dev, "ax210.lmac.iccm",  0x00000000, 0x00018000); /* MPU 0-1, X   */
    add_ram(dev, "ax210.lmac.romwin",0x00440000, 0x00074000); /* MPU 12, R    */
    add_ram(dev, "ax210.lmac.code",  0x004b4000, 0x0004c000); /* MPU 7-11, RWX*/
    add_ram(dev, "ax210.lmac.sram",  0x00600000, 0x00040000); /* MPU 5, RW    */
    add_ram(dev, "ax210.lmac.shbuf", 0x00660800, 0x00000800); /* MPU 6, RW    */
    add_ram(dev, "ax210.lmac.dccm",  0x00800000, 0x00008000); /* MPU 2, RW    */

    /* Traced peripheral windows (overlay RAM; higher priority). */
    /* LMAC peripheral space 0xa00000-0xafffff (finding P1, [I]): LMAC code
     * uses it only as MMIO (e.g. assert mailbox 0xa0200c, sleep control
     * 0xa01ec0, indirect window 0xa2ac00, crypto 0xa248xx, power FSM
     * 0xd03380, markers 0xd0c110); no firmware section loads in
     * 0xa00000-0xffffff. */
    add_trace(dev, "ax210.lmac_periph",  0x00a00000, 0x00200000, 10); /* MPU 3 */
    add_trace(dev, "ax210.lmac_periph2", 0x00d00000, 0x00200000, 10); /* MPU 4 */
    memory_region_add_subregion(sys, 0, dev);
    add_alias("ax210.umac", dev, 0x80000000, 0x01000000); /* cached */
    add_alias("ax210.unc",  dev, 0xc0000000, 0x01000000); /* uncached */
    add_trace(sys, "ax210.umac_d",   0xc0d00000, 0x00100000, 10); /* == LMAC 0xd0xxxx (K1) */
    add_trace(sys, "ax210.umac_e",   0xc0e00000, 0x00100000, 10); /* == LMAC 0xaxxxxx (K1) */
    add_ram(sys, "ax210.host",  0x20000000, 0x10000000);   /* modelled host DRAM */
    add_trace(sys, "ax210.umac_a",   0xc0a00000, 0x00010000, 10); /* UMAC MMIO: Q2, DMA */
    add_trace(sys, "ax210.csr",      0xf0000000, 0x01000000, 10); /* chip CSRs */

    if (getenv("AX210_TRACECAP")) trace_cap = strtoul(getenv("AX210_TRACECAP"), 0, 0);

    const char *ctr = getenv("AX210_CTR");
    if (ctr) {
        char *c = g_strdup(ctr), *t2, *sv = NULL;
        for (t2 = strtok_r(c, ",", &sv); t2 && n_ctrs < MAX_CTRS;
             t2 = strtok_r(NULL, ",", &sv)) {
            ctrs[n_ctrs].addr = lmac_view(strtoull(t2, 0, 16));
            ctrs[n_ctrs++].val = 0;
        }
        g_free(c);
    }

    /* Host CSRs before firmware load (finding C2). CSR_HW_REV (0x028) is
     * 0x420 on silicon ("rev=0x420", "hw version 0x00000420": the dumps in
     * ~/Projects/ax210/ref-silicon/). IML 0x80445a2c and UMAC 0x8044a702 read
     * mailbox index 0 (CSR_HW_IF_CONFIG_REG if M1 holds) and require bits
     * [1:0] == 2 (== 1 when on-die [0xd03400] & 7 == 2), else error 0x2fc.
     * iwl_mvm_nic_config writes HW_REV[3:0] = 0 there, yet real firmware
     * boots, so M1 or that derivation is wrong for this register; 2 is [I]. */
    host_csr[0x028 >> 2] = 0x420;
    host_csr[0x000 >> 2] = 0x2;

    /* Host CSR initial contents: AX210_CSR="offs=val,..." (offs hex, val num) */
    const char *csr = getenv("AX210_CSR");
    if (csr) {
        char *c = g_strdup(csr), *t3, *sv = NULL;
        for (t3 = strtok_r(c, ",", &sv); t3; t3 = strtok_r(NULL, ",", &sv)) {
            char *eq = strchr(t3, '=');
            if (!eq) continue;
            *eq = 0;
            host_csr[(strtoul(t3, 0, 16) >> 2) & 0xfff] = strtoul(eq + 1, 0, 0);
        }
        g_free(c);
    }

    /* Indirect-window seeds (W1), e.g. hardware status bits the firmware
     * polls that no firmware writes: AX210_IND="940c=1,...". */
    const char *ind = getenv("AX210_IND");
    if (ind) {
        char *c = g_strdup(ind), *t4, *sv = NULL;
        if (!regstore) {
            regstore = g_hash_table_new(g_direct_hash, g_direct_equal);
        }
        for (t4 = strtok_r(c, ",", &sv); t4; t4 = strtok_r(NULL, ",", &sv)) {
            char *eq = strchr(t4, '=');
            if (!eq) continue;
            *eq = 0;
            g_hash_table_insert(regstore,
                                GUINT_TO_POINTER(IND_BASE +
                                                 (strtoul(t4, 0, 16) & IND_ADDR)),
                                GUINT_TO_POINTER(strtoul(eq + 1, 0, 0)));
        }
        g_free(c);
    }

    if (getenv("AX210_HOSTFW")) {
        ax210_host_driver_init(getenv("AX210_HOSTFW"));
    }

    /* MMIO read overrides, discovered iteratively, as
     * AX210_OVR="c0d02c68=0x100,c0d02c7c=0xffffffff,...". */
    const char *ovr = getenv("AX210_OVR");
    if (ovr) {
        char *s = g_strdup(ovr), *tok, *save = NULL;
        for (tok = strtok_r(s, ",", &save); tok && n_overrides < MAX_OVERRIDES;
             tok = strtok_r(NULL, ",", &save)) {
            char *eq = strchr(tok, '=');
            if (!eq) continue;
            *eq = 0;
            overrides[n_overrides].addr = lmac_view(strtoull(tok, 0, 16));
            overrides[n_overrides].val  = strtoul(eq + 1, 0, 0);
            n_overrides++;
        }
        g_free(s);
    }
}

static void ax210_machine_init(MachineClass *mc)
{
    mc->desc = "Intel AX210 Wi-Fi MAC (ARC700 UMAC + ARCv2EM LMAC) RE scaffold";
    mc->init = ax210_init;
    mc->max_cpus = 2;
    mc->default_cpus = 2;   /* both cores always instantiated by ax210_init */
    mc->no_serial = 1;
    mc->default_cpu_type = TYPE_ARC_CPU_ARC700;
}

DEFINE_MACHINE("ax210", ax210_machine_init)
