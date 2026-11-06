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
 * only lays out memory and peripherals and brings up one ARC700 core.
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "cpu.h"
#include "hw/hw.h"
#include "hw/boards.h"
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
static struct { hwaddr addr; uint32_t val; } ctrs[MAX_CTRS] = { { 0xc0d02c68, 0 } };
static int n_ctrs = 1;

/* Host-CSR mailbox at 0xc0d0001c/20/24/28 (inferred from IML code + driver
 * csr offsets): write index to +0x20, command to +0x1c (top byte 0x6a = read,
 * 0x69 = write, data at +0x24), result at +0x28. Backing array models the PCIe
 * CSR block the host driver programs (e.g. 0x118 CTXT_INFO_ADDR). */
/* Descriptor DMA engine at 0xc0a0a500 (from IML routine 0x804458a4): +0x34
 * command (0x81000001 = host->device copy, kick), +0x38 device addr, +0x3c/0x40
 * host addr lo/hi, +0x44 length. Completion = bit 29 of status reg 0xc0d02e24
 * (polled by IML 0x804458c8; written back to acknowledge). Host "physical"
 * addresses are served from system memory (host RAM window at 0x20000000). */
#define DMA_BASE  0xc0a0a500u
#define DMA_STAT  0xc0d02e24u
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

/* Hardware semaphore bank at 0xd03800 (finding S1, [I]): LMAC 0x4ba960
 * writes a bit mask to +0x0 (request) and returns whether [+0x4] (granted)
 * has it; 0x4d1c88 writes 1<<n to +0x8 (release). Both mirror mask<<8 into
 * 0xa01e30 (not interpreted). Presumably arbitrates LMAC vs UMAC; with one
 * core modelled, any request for a free bit is granted at once. */
#define SEM_BASE 0x00d03800u
static uint32_t sem_owned;

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
static GHashTable *indstore;

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
static CPUState *ax210_cs;


static void tmr_expire(void *opaque)
{
    int ch = (int)(uintptr_t)opaque;
    trace_log("TMR.X", TMR_COUNT0 + 4 * ch, ch, 4);
    if (ax210_cs && ax210_cs->halted) {
        ax210_cs->halted = 0;          /* event wake: resume after wevt */
        qemu_cpu_kick(ax210_cs);
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

#define MBX_BASE 0xc0d00000u
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
    int hit; uint32_t v = override_lookup(base + addr, &hit);
    if (base + addr == DMA_STAT && !hit) {
        v = dma_done ? (1u << 29) : 0;
        trace_log("R.dma", base + addr, v, size);
        return v;
    }
    if (base + addr == TMR_TSTAMP && !hit) {
        v = (uint32_t)qemu_clock_get_us(QEMU_CLOCK_VIRTUAL);
        trace_log("R.ts", base + addr, v, size);
        return v;
    }
    if (base + addr == MFIFO_BASE + 0x30 && !hit) {
        trace_log("R.mfifo", base + addr, MFIFO_CAP, size);
        return MFIFO_CAP;
    }
    if (base + addr == MFIFO_BASE + 0x38 && !hit) {
        trace_log("R.mfifo", base + addr, 0, size);
        return 0;
    }
    if (base + addr == SEM_BASE + 4 && !hit) {
        trace_log("R.sem", base + addr, sem_owned, size);
        return sem_owned;
    }
    if (base + addr == FSM_BASE && !hit) {
        v = fsm_read();
        trace_log("R.fsm", base + addr, v, size);
        return v;
    }
    if (base + addr == MBX_BASE + 0x28 && !hit) {
        v = host_csr[(mbx_idx >> 2) & 0x3ff];
        trace_log("R.csr", mbx_idx, v, size);
        return v;
    }
    for (int i = 0; i < n_ctrs; i++) {
        if (ctrs[i].addr == base + addr) {
            v = ++ctrs[i].val;
            trace_log("R.ctr", base + addr, v, size);
            return v;
        }
    }
    if (!hit && regstore) {
        gpointer p = g_hash_table_lookup(regstore,
                                         GUINT_TO_POINTER((guint)(base + addr)));
        v = (uint32_t)(uintptr_t)p;
    }
    trace_log(hit ? "R.ovr" : "R", base + addr, v, size);
    return v;
}

static void trace_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    hwaddr base = (hwaddr)(uintptr_t)opaque;
    trace_log("W", base + addr, val, size);
    if (base + addr == MBX_BASE + 0x20) {
        mbx_idx = val;
    } else if (base + addr == MBX_BASE + 0x24) {
        mbx_data = val;
    } else if (base + addr == MBX_BASE + 0x1c && (val >> 24) == 0x69) {
        host_csr[(mbx_idx >> 2) & 0x3ff] = mbx_data;
        trace_log("W.csr", mbx_idx, mbx_data, size);
    }
    if (base + addr == DMA_STAT && (val & (1u << 29))) {
        dma_done = 0;
    }
    if (!regstore) {
        regstore = g_hash_table_new(g_direct_hash, g_direct_equal);
    }
    if (base + addr == AES_KEYF) {
        aes_key[aes_kidx++ & 3] = (uint32_t)val;
    }
    if (base + addr == AES_CTRL && (val & 1)) {
        g_hash_table_insert(regstore, GUINT_TO_POINTER((guint)(base + addr)),
                            (gpointer)(uintptr_t)(uint32_t)val);
        aes_run((uint32_t)val);
        val = (val & ~1ull) | 4;           /* done */
    }
    if (base + addr == MFIFO_BASE + 0x40) {
        trace_log("MSG", base + addr, val, size);
    }
    if (base + addr == SEM_BASE) {
        sem_owned |= (uint32_t)val;
    } else if (base + addr == SEM_BASE + 8) {
        sem_owned &= ~(uint32_t)val;
    }
    if (base + addr == FSM_BASE) {
        fsm_ctl = (uint32_t)val & ~0xfu;
    } else if (base + addr == FSM_BASE + 4) {
        fsm_ctl &= ~(uint32_t)val;
    } else if (base + addr == FSM_BASE + 8) {
        fsm_ctl |= (uint32_t)val & ~0xfu;
    }
    if (base + addr == TMR_START) {
        /* store first so tmr_start sees counts already written */
        tmr_start((uint32_t)val);
    }
    if (base + addr == IND_CTRL && (val & IND_GO)) {
        guint ia = val & 0xffff;
        if (!indstore) {
            indstore = g_hash_table_new(g_direct_hash, g_direct_equal);
        }
        if (val & IND_WR) {
            guint d = GPOINTER_TO_UINT(g_hash_table_lookup(regstore,
                                       GUINT_TO_POINTER(IND_DATA)));
            g_hash_table_insert(indstore, GUINT_TO_POINTER(ia),
                                GUINT_TO_POINTER(d));
            trace_log("IND.W", ia, d, 4);
        } else {
            guint d = GPOINTER_TO_UINT(g_hash_table_lookup(indstore,
                                       GUINT_TO_POINTER(ia)));
            g_hash_table_insert(regstore, GUINT_TO_POINTER(IND_DATA),
                                GUINT_TO_POINTER(d));
            trace_log("IND.R", ia, d, 4);
        }
        val &= ~(uint64_t)IND_GO;   /* access completes at once: not busy */
    }
    g_hash_table_insert(regstore, GUINT_TO_POINTER((guint)(base + addr)),
                        (gpointer)(uintptr_t)(uint32_t)val);
    if (base + addr == DMA_BASE + 0x34) {
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

static const MemoryRegionOps trace_ops = {
    .read = trace_read,
    .write = trace_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
};

static void add_ram(const char *name, hwaddr base, uint64_t size)
{
    MemoryRegion *r = g_new(MemoryRegion, 1);
    memory_region_init_ram(r, NULL, name, size, &error_fatal);
    memory_region_add_subregion(get_system_memory(), base, r);
}

static void add_trace(const char *name, hwaddr base, uint64_t size, int prio)
{
    MemoryRegion *r = g_new(MemoryRegion, 1);
    memory_region_init_io(r, NULL, &trace_ops, (void *)(uintptr_t)base,
                          name, size);
    memory_region_add_subregion_overlap(get_system_memory(), base, r, prio);
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
static bool is_lmac_core;

static void ax210_reset(void *opaque)
{
    ARCCPU *cpu = opaque;

    arc_cpu_reset(cpu);
    if (is_lmac_core) {
        cpu->env.r[32] = r32_init;
    }
}

static void ax210_init(MachineState *machine)
{
    ARCCPU *cpu = ARC_CPU(object_new(machine->cpu_type));
    if (!strcmp(machine->cpu_type, TYPE_ARC_CPU_ARCEM)) {
        /* LMAC programs 13 MPU region pairs (aux 0x422..0x43b, at
         * 0x4be574); ARCv2 MPUs come in power-of-two sizes. Finding M2. */
        object_property_set_uint(OBJECT(cpu), "mpu-numreg", 16, &error_fatal);
    }
    object_property_set_uint(OBJECT(cpu), "freq_hz", 1000000, &error_fatal);
    object_property_set_bool(OBJECT(cpu), "rtc-opt", true, &error_fatal);
    object_property_set_bool(OBJECT(cpu), "realized", true, &error_fatal);
    cpu_arc_pic_init(cpu);
    cpu_arc_clock_init(cpu);
    cpu->env.boot_info = &ax210_boot_info;
    is_lmac_core = !strcmp(machine->cpu_type, TYPE_ARC_CPU_ARCEM);
    if (getenv("AX210_R32")) {
        r32_init = strtoul(getenv("AX210_R32"), 0, 0);
    }
    qemu_register_reset(ax210_reset, cpu);
    ax210_cs = CPU(cpu);
    arc_unknown_aux_write = ax210_aux_write;
    arc_unknown_aux_read = ax210_aux_read;

    /* Catch-all tracer over the whole 32-bit space (lowest priority). */
    add_trace("ax210.catchall", 0x00000000, 0x100000000ULL, -1000);

    /* RAM windows (firmware load addresses). ARC cache aliases 0x8.. and 0xc..
     * are modelled as independent RAM here; coherence between aliases is a
     * known simplification to revisit if firmware relies on it. */
    add_ram("ax210.lmac",  0x00000000, 0x01100000); /* LMAC code/data, paged, DCCM-low, CSS */
    add_ram("ax210.umac",  0x80000000, 0x00800000); /* UMAC code/data (0x80400000+) */
    add_ram("ax210.unc",   0xc0000000, 0x01000000); /* uncached window: code @0xc0080000, bss @0xc0880000 */

    /* Traced peripheral windows (overlay RAM; higher priority). */
    /* LMAC peripheral space 0xa00000-0xafffff (finding P1, [I]): LMAC code
     * uses it only as MMIO (e.g. assert mailbox 0xa0200c, sleep control
     * 0xa01ec0, indirect window 0xa2ac00, crypto 0xa248xx, power FSM
     * 0xd03380, markers 0xd0c110); no firmware section loads in
     * 0xa00000-0xffffff. */
    add_trace("ax210.lmac_periph", 0x00a00000, 0x00600000, 10);
    add_trace("ax210.ctl_d",    0xc0d00000, 0x00010000, 10); /* control regs polled by IML */
    add_trace("ax210.crypto_c", 0xc0e20000, 0x00010000, 10); /* remapped crypto window */
    add_ram("ax210.host",  0x20000000, 0x10000000);   /* modelled host DRAM */
    add_trace("ax210.dma",      0xc0a0a000, 0x00001000, 10);
    add_trace("ax210.csr",      0xf0000000, 0x01000000, 10); /* chip CSRs */

    if (getenv("AX210_TRACECAP")) trace_cap = strtoul(getenv("AX210_TRACECAP"), 0, 0);

    const char *ctr = getenv("AX210_CTR");
    if (ctr) {
        char *c = g_strdup(ctr), *t2, *sv = NULL;
        for (t2 = strtok_r(c, ",", &sv); t2 && n_ctrs < MAX_CTRS;
             t2 = strtok_r(NULL, ",", &sv)) {
            ctrs[n_ctrs].addr = strtoull(t2, 0, 16);
            ctrs[n_ctrs++].val = 0;
        }
        g_free(c);
    }

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
            overrides[n_overrides].addr = strtoull(tok, 0, 16);
            overrides[n_overrides].val  = strtoul(eq + 1, 0, 0);
            n_overrides++;
        }
        g_free(s);
    }
}

static void ax210_machine_init(MachineClass *mc)
{
    mc->desc = "Intel AX210 Wi-Fi MAC (ARC700) RE scaffold";
    mc->init = ax210_init;
    mc->max_cpus = 1;
    mc->no_serial = 1;
    mc->default_cpu_type = TYPE_ARC_CPU_ARC700;
}

DEFINE_MACHINE("ax210", ax210_machine_init)
