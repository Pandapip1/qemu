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
static uint32_t dma_done;
static unsigned dma_count;

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

/* LMAC indirect register window (finding W1, [I]): CTRL at 0xa2ac04 holds a
 * 16-bit target address in [15:0], bit 17 = write, bit 18 = GO/BUSY; DATA at
 * 0xa2ac00. LMAC helpers 0x4be6f8 (read) / 0x4be714 (write) / 0x4be818 (wait
 * while BUSY) define the protocol. BUSY self-clears: accesses complete
 * immediately against a backing store whose contents are unknown ([?]),
 * so it starts zeroed and is traced (IND.R / IND.W). */
#define IND_DATA 0x00a2ac00u
#define IND_CTRL 0x00a2ac04u
#define IND_GO   (1u << 18)
#define IND_WR   (1u << 17)
#define IND_ADDR 0x1ffffu
static GHashTable *indstore;

/* Register aliases (finding K2, [I]): UMAC 0x80448ca4 writes 0x400410 to
 * 0xa3408c and requires 0xa200b0 to read it back (masked 0xff0fffff), and
 * writes it to 0xa34090 and requires indirect 0xb800 to read it back exactly;
 * either mismatch is fatal 0x1013f1. Modelled as one register per pair. */
#define IND_ALIAS_ADDR 0xb800u
#define IND_ALIAS_REG  0x00a34090u

static hwaddr alias_key(hwaddr a)
{
    return a == 0x00a200b0 ? 0x00a3408c : a;
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

#define MBX_BASE 0x00d00000u    /* UMAC 0xc0d00000 */
static uint32_t host_csr[0x400];
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
    hwaddr a = alias_key(lmac_view(ra)); /* model key */
    int hit; uint32_t v = override_lookup(a, &hit);
    if (a == DMA_STAT && !hit) {
        v = dma_done ? (1u << 29) : 0;
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
    if (a == FSM_BASE && !hit) {
        v = fsm_read();
        trace_log("R.fsm", ra, v, size);
        return v;
    }
    if (a == MBX_BASE + 0x28 && !hit) {
        v = host_csr[(mbx_idx >> 2) & 0x3ff];
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
    hwaddr a = alias_key(lmac_view(ra)); /* model key */
    trace_log("W", ra, val, size);
    if (a == MBX_BASE + 0x20) {
        mbx_idx = val;
    } else if (a == MBX_BASE + 0x24) {
        mbx_data = val;
    } else if (a == MBX_BASE + 0x1c && (val >> 24) == 0x69) {
        host_csr[(mbx_idx >> 2) & 0x3ff] = mbx_data;
        trace_log("W.csr", mbx_idx, mbx_data, size);
    }
    if (a == DMA_STAT && (val & (1u << 29))) {
        dma_done = 0;
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
        if (!indstore) {
            indstore = g_hash_table_new(g_direct_hash, g_direct_equal);
        }
        if (val & IND_WR) {
            guint d = GPOINTER_TO_UINT(g_hash_table_lookup(regstore,
                                       GUINT_TO_POINTER(IND_DATA)));
            g_hash_table_insert(ia == IND_ALIAS_ADDR ? regstore : indstore,
                                GUINT_TO_POINTER(ia == IND_ALIAS_ADDR ?
                                                 IND_ALIAS_REG : ia),
                                GUINT_TO_POINTER(d));
            trace_log("IND.W", ia, d, 4);
        } else {
            guint d = ia == IND_ALIAS_ADDR ? rs_get(IND_ALIAS_REG) :
                      GPOINTER_TO_UINT(g_hash_table_lookup(indstore,
                                       GUINT_TO_POINTER(ia)));
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

    /* Host CSRs as the driver leaves them before firmware load (finding C2):
     * CSR_HW_REV (0x028) type CSR_HW_REV_TYPE_TY = 0x420 [F, iwl-csr.h:358];
     * iwl_mvm_nic_config copies HW_REV[3:0] (step/dash) into
     * CSR_HW_IF_CONFIG_REG[3:0] [F, mvm/ops.c:100, called from
     * pcie/trans-gen2.c:273]. IML 0x80445a2c and UMAC 0x8044a702 require
     * dash == 2 when on-die [0xd03400] & 7 != 2 (else dash == 1), error
     * 0x2fc otherwise; the real step/dash is [?], so step A (0), dash 2,
     * with on-die 0xd03400 left at 0, is a consistent guess. */
    host_csr[0x028 >> 2] = 0x420 | 0x2;
    host_csr[0x000 >> 2] = 0x2;

    /* Host CSR initial contents: AX210_CSR="offs=val,..." (offs hex, val num) */
    const char *csr = getenv("AX210_CSR");
    if (csr) {
        char *c = g_strdup(csr), *t3, *sv = NULL;
        for (t3 = strtok_r(c, ",", &sv); t3; t3 = strtok_r(NULL, ",", &sv)) {
            char *eq = strchr(t3, '=');
            if (!eq) continue;
            *eq = 0;
            host_csr[(strtoul(t3, 0, 16) >> 2) & 0x3ff] = strtoul(eq + 1, 0, 0);
        }
        g_free(c);
    }

    /* Indirect-window seeds (W1), e.g. hardware status bits the firmware
     * polls that no firmware writes: AX210_IND="940c=1,...". */
    const char *ind = getenv("AX210_IND");
    if (ind) {
        char *c = g_strdup(ind), *t4, *sv = NULL;
        indstore = g_hash_table_new(g_direct_hash, g_direct_equal);
        for (t4 = strtok_r(c, ",", &sv); t4; t4 = strtok_r(NULL, ",", &sv)) {
            char *eq = strchr(t4, '=');
            if (!eq) continue;
            *eq = 0;
            g_hash_table_insert(indstore,
                                GUINT_TO_POINTER(strtoul(t4, 0, 16) & IND_ADDR),
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
