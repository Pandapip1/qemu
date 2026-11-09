/*
 * QTest for the AX210 Wi-Fi MAC (ARC700) board model, hw/arc/ax210.c.
 *
 * These assert only mechanisms the model itself defines (RAM windows, the
 * MMIO trace/override machinery, the write-readback register store, the
 * host-CSR proxy). They do NOT assert any claim about real AX210 silicon
 * behaviour: there is no SWD-equivalent debug access to this chip, so
 * nothing here is validated against hardware. See
 * docs/ax210-model-fidelity.md for the evidence status of each mechanism
 * ([F]/[I]/[?]) and tests/qtest/ax210-test.c vs. the ad hoc GDB-RSP scripts
 * in that doc's "Harness" section for which claims are qtest-grade.
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */
#include "qemu/osdep.h"
#include "libqos/libqtest.h"
#include "qemu/bswap.h"

/* RAM windows from hw/arc/ax210.c: ax210_init(). A write/read round-trip at
 * each window's base confirms it is backed by writable RAM, not an alias
 * or hole. */
static void test_ram_windows(void)
{
    QTestState *s = qtest_init("-M ax210 -m 32M -nographic -monitor none "
                               "-serial none -S");

    qtest_writel(s, 0x00000000, 0x11223344); /* LMAC */
    g_assert_cmpuint(qtest_readl(s, 0x00000000), ==, 0x11223344);

    qtest_writel(s, 0x80000000, 0x22334455); /* UMAC base */
    g_assert_cmpuint(qtest_readl(s, 0x80000000), ==, 0x22334455);

    qtest_writel(s, 0x804ff000, 0x33445566); /* UMAC near top of window */
    g_assert_cmpuint(qtest_readl(s, 0x804ff000), ==, 0x33445566);

    qtest_writel(s, 0xc0000000, 0x44556677); /* uncached window */
    g_assert_cmpuint(qtest_readl(s, 0xc0000000), ==, 0x44556677);

    qtest_quit(s);
}

/* Sparse write-readback store (ax210.c: regstore): a word written to any
 * address NOT covered by RAM or by a named trace region (crypto, ctl_d,
 * crypto_c, csr, dma, host) must read back unchanged. This is the "answer
 * your own last write" behaviour the IML's set/confirm handshakes rely on
 * (see doc, finding R1). */
static void test_write_readback(void)
{
    QTestState *s = qtest_init("-M ax210 -m 32M -nographic -monitor none "
                               "-serial none -S");

    g_assert_cmpuint(qtest_readl(s, 0xc0d09000), ==, 0); /* unwritten: 0 */
    qtest_writel(s, 0xc0d09000, 0xdeadbeef);
    g_assert_cmpuint(qtest_readl(s, 0xc0d09000), ==, 0xdeadbeef);

    qtest_quit(s);
}

/* AX210_OVR fixed-value overrides win over the write-readback store, as
 * documented: they model a status/ready bit the real device would drive
 * independently of anything the firmware has written. */
static void test_override_wins_over_store(void)
{
    QTestState *s;

    g_setenv("AX210_OVR", "c0d02c7c=0xffffffff", true);
    s = qtest_init("-M ax210 -m 32M -nographic -monitor none "
                   "-serial none -S");
    g_unsetenv("AX210_OVR");

    qtest_writel(s, 0xc0d02c7c, 0); /* firmware writes 0 ... */
    g_assert_cmpuint(qtest_readl(s, 0xc0d02c7c), ==, 0xffffffff); /* ... override still wins */

    qtest_quit(s);
}

/* 0xc0d02c68 free-running counter (ax210.c: ctrs[], default entry):
 * each read must return a strictly increasing value, independent of any
 * write. This models a timebase register the IML's delay loops poll
 * (finding T1); it is NOT a claim about the real timer's rate or any
 * other address being a counter. */
static void test_default_counter_increments(void)
{
    QTestState *s = qtest_init("-M ax210 -m 32M -nographic -monitor none "
                               "-serial none -S");

    uint32_t a = qtest_readl(s, 0xc0d02c68);
    uint32_t b = qtest_readl(s, 0xc0d02c68);
    uint32_t c = qtest_readl(s, 0xc0d02c68);
    g_assert_cmpuint(b, ==, a + 1);
    g_assert_cmpuint(c, ==, b + 1);

    qtest_quit(s);
}

/* Host-CSR proxy (ax210.c: host_csr[], AX210_CSR=): an index written to
 * MBX_BASE+0x20 followed by a read-command at MBX_BASE+0x1c must make the
 * value seeded (or later written) at that index visible at MBX_BASE+0x28.
 * This is the *inferred* mapping (finding M1, [I]) from control-flow
 * evidence onto the driver's documented CSR offsets (0x118
 * CSR_CTXT_INFO_ADDR, drivers/net/wireless/intel/iwlwifi/fw/api/
 * ctxt-info-v2.h); the 0x1c/0x20/0x24/0x28 scheme and the 0x69/0x6a command
 * bytes are not independently confirmed, only self-consistent with the
 * IML's observed behaviour under this model. */
static void test_host_csr_proxy_seed_and_read(void)
{
    QTestState *s;

    g_setenv("AX210_CSR", "118=0x20000000,11c=0", true);
    s = qtest_init("-M ax210 -m 32M -nographic -monitor none "
                   "-serial none -S");
    g_unsetenv("AX210_CSR");

    qtest_writel(s, 0xc0d00020, 0x118);        /* index = CSR_CTXT_INFO_ADDR lo */
    qtest_writel(s, 0xc0d0001c, 0x6a000000);   /* command byte 0x6a = "read" [I] */
    g_assert_cmpuint(qtest_readl(s, 0xc0d00028), ==, 0x20000000);

    qtest_writel(s, 0xc0d00020, 0x11c);        /* index = CSR_CTXT_INFO_ADDR hi */
    qtest_writel(s, 0xc0d0001c, 0x6a000000);
    g_assert_cmpuint(qtest_readl(s, 0xc0d00028), ==, 0);

    qtest_quit(s);
}

/* DMA engine (ax210.c: dma_kick(), finding D1 [I]): a command 0x81000001
 * kicked at DMA_BASE+0x34 must copy `len` bytes from the modelled host
 * window (DMA_BASE+0x3c/+0x40, split lo/hi) to the device address at
 * DMA_BASE+0x38, and DMA_STAT (0xc0d02e24) must then report bit 29 set.
 * This is the model's only testable claim about the engine; the real
 * register layout and command encoding are unconfirmed (doc, finding D1). */
static void test_dma_copies_host_to_device(void)
{
    QTestState *s = qtest_init("-M ax210 -m 32M -nographic -monitor none "
                               "-serial none -S");
    const uint8_t pattern[16] = "0123456789abcde";
    uint8_t out[16] = { 0 };

    qtest_memwrite(s, 0x20001000, pattern, sizeof(pattern));

    qtest_writel(s, 0xc0a0a538, 0x80410000);        /* device dest: UMAC */
    qtest_writel(s, 0xc0a0a53c, 0x20001000);        /* host addr lo */
    qtest_writel(s, 0xc0a0a540, 0x00000000);        /* host addr hi */
    qtest_writel(s, 0xc0a0a544, sizeof(pattern));   /* length */
    g_assert_cmpuint(qtest_readl(s, 0xc0d02e24), ==, 0); /* not done yet */
    qtest_writel(s, 0xc0a0a534, 0x81000001);        /* kick */

    g_assert_cmpuint(qtest_readl(s, 0xc0d02e24) & (1u << 29), ==, (1u << 29));
    qtest_memread(s, 0x80410000, out, sizeof(out));
    g_assert_cmpmem(out, sizeof(out), pattern, sizeof(pattern));

    qtest_quit(s);
}

#define LMAC_ARGS "-M ax210 -cpu arcem -m 32M -nographic -monitor none " \
                  "-serial none -S"

/* LMAC AES-128 engine (finding A1, [F]): reproduce the LMAC boot self-test
 * (0x4d9ba0) exactly: FIPS-197 Appendix C.1, with the register packing that
 * test uses. */
static void test_lmac_aes_fips197(void)
{
    QTestState *s = qtest_init(LMAC_ARGS);
    static const uint32_t key[4] = { 0x00010203, 0x04050607,
                                     0x08090a0b, 0x0c0d0e0f };
    static const uint32_t pt[4]  = { 0x00112233, 0x44556677,
                                     0x8899aabb, 0xccddeeff };
    static const uint32_t ct[4]  = { 0x69c4e0d8, 0x6a7b0430,
                                     0xd8cdb780, 0x70b4c55a };

    for (int i = 0; i < 4; i++) {       /* low word first, byte-swapped */
        qtest_writel(s, 0xa01808, bswap32(key[3 - i]));
    }
    for (int j = 0; j < 4; j++) {       /* word j = pt[3 - j] */
        qtest_writel(s, 0xa01898 + 4 * j, pt[3 - j]);
    }
    qtest_writel(s, 0xa01894, 3);       /* start, encrypt */
    g_assert_cmpuint(qtest_readl(s, 0xa01894) & 4, ==, 4);   /* done */
    for (int j = 0; j < 4; j++) {
        g_assert_cmphex(qtest_readl(s, 0xa01898 + 4 * j), ==, ct[3 - j]);
    }
    qtest_quit(s);
}

/* Power/clock request FSM (finding F1, [I]): state [3:0] follows bit 4,
 * which is set via +0x8 and cleared via +0x4 (LMAC 0x4d525c / 0x4d520c). */
static void test_lmac_power_fsm(void)
{
    QTestState *s = qtest_init(LMAC_ARGS);

    g_assert_cmpuint(qtest_readl(s, 0xd03380) & 0xf, ==, 0);
    qtest_writel(s, 0xd03388, 0x04000010);
    g_assert_cmpuint(qtest_readl(s, 0xd03380) & 0xf, ==, 5);
    qtest_writel(s, 0xd03384, 0x10);
    g_assert_cmpuint(qtest_readl(s, 0xd03380) & 0xf, ==, 0);
    qtest_quit(s);
}

/* Hardware semaphore bank (finding S1, [I]). */
static void test_lmac_semaphore(void)
{
    QTestState *s = qtest_init(LMAC_ARGS);

    qtest_writel(s, 0xd03800, 1u << 3);
    g_assert_cmpuint(qtest_readl(s, 0xd03804) & (1u << 3), ==, 1u << 3);
    qtest_writel(s, 0xd03808, 1u << 3);
    g_assert_cmpuint(qtest_readl(s, 0xd03804) & (1u << 3), ==, 0);
    qtest_quit(s);
}

/* LMAC indirect register window (finding W1, [I]): a write then a read of
 * the same target address round-trip, and BUSY (bit 18) reads clear. */
static void test_lmac_indirect_window(void)
{
    QTestState *s = qtest_init(LMAC_ARGS);

    qtest_writel(s, 0xa2ac00, 0x12345678);
    qtest_writel(s, 0xa2ac04, 0x8ef4 | (1u << 17) | (1u << 18));
    g_assert_cmpuint(qtest_readl(s, 0xa2ac04) & (1u << 18), ==, 0);
    qtest_writel(s, 0xa2ac00, 0);
    qtest_writel(s, 0xa2ac04, 0x8ef4 | (1u << 18));
    g_assert_cmpuint(qtest_readl(s, 0xa2ac04) & (1u << 18), ==, 0);
    g_assert_cmphex(qtest_readl(s, 0xa2ac00), ==, 0x12345678);
    qtest_quit(s);
}

/* Indirect window (W1): the address is [16:0] (UMAC reads 0x18028, which
 * must not alias 0x8028) and targets device 0xa20000 + addr. Chip IDs (K2):
 * read-only silicon values, also through the window (0xb800 = 0xa2b800). */
static void test_lmac_indirect_target_and_chip_ids(void)
{
    QTestState *s = qtest_init(LMAC_ARGS);

    qtest_writel(s, 0xa2ac00, 0x11111111);
    qtest_writel(s, 0xa2ac04, 0x18028 | (1u << 17) | (1u << 18));
    qtest_writel(s, 0xa2ac04, 0x08028 | (1u << 18));
    g_assert_cmphex(qtest_readl(s, 0xa2ac00), ==, 0);
    g_assert_cmphex(qtest_readl(s, 0xa38028), ==, 0x11111111);

    qtest_writel(s, 0xa200b0, 0);           /* read-only */
    g_assert_cmphex(qtest_readl(s, 0xa200b0), ==, 0x400410);
    qtest_writel(s, 0xa2ac04, 0xb800 | (1u << 18));
    g_assert_cmphex(qtest_readl(s, 0xa2ac00), ==, 0x400410);
    g_assert_cmphex(qtest_readl(s, 0xd03098), ==, 0x80000000);
    g_assert_cmphex(qtest_readl(s, 0xa340fc), ==, 0xa652);
    qtest_writel(s, 0xa01e20, 0xffffffff);  /* ISR0 (R2): writes do not show */
    g_assert_cmphex(qtest_readl(s, 0xa01e20), ==, 0x00020000);
    g_assert_cmphex(qtest_readl(s, 0xa01e28), ==, 0x48f00002);
    qtest_quit(s);
}

/* Timestamp (finding V1, [I]): 0xa02020 counts virtual microseconds. */
static void test_lmac_timestamp(void)
{
    QTestState *s = qtest_init(LMAC_ARGS);
    uint32_t t0 = qtest_readl(s, 0xa02020);

    qtest_clock_step(s, 1000 * 1000);   /* 1 ms of virtual time, in ns */
    g_assert_cmpuint(qtest_readl(s, 0xa02020) - t0, ==, 1000);
    qtest_quit(s);
}

/* Message FIFO space (finding Q1, [I]): drained consumer, 1 KiB capacity. */
static void test_lmac_msg_fifo_space(void)
{
    QTestState *s = qtest_init(LMAC_ARGS);

    g_assert_cmphex(qtest_readl(s, 0xa00430), ==, 0x400);
    g_assert_cmphex(qtest_readl(s, 0xa00438), ==, 0);
    g_assert_cmphex(qtest_readl(s, 0xc0a00530), ==, 0x400); /* Q2 */
    g_assert_cmphex(qtest_readl(s, 0xc0a00538), ==, 0);
    qtest_quit(s);
}

/* LMAC memory map (finding M2): every RAM region LMAC's own MPU declares is
 * backed by RAM in the arcem configuration. */
static void test_lmac_mpu_map_ram(void)
{
    QTestState *s = qtest_init(LMAC_ARGS);
    static const uint32_t ram[] = { 0x0, 0x17ffc, 0x440000, 0x4b4000,
                                    0x4ffffc, 0x600000, 0x63fffc, 0x660800,
                                    0x660ffc, 0x800000, 0x807ffc };

    for (unsigned i = 0; i < ARRAY_SIZE(ram); i++) {
        qtest_writel(s, ram[i], 0xa5a50000 | i);
        g_assert_cmphex(qtest_readl(s, ram[i]), ==, 0xa5a50000 | i);
    }
    qtest_quit(s);
}

/* Host CSR defaults (finding C2): as iwl_mvm_nic_config leaves them, read
 * back through the UMAC-side CSR proxy. */
static void test_host_csr_driver_defaults(void)
{
    QTestState *s = qtest_init("-M ax210 -m 32M -nographic -monitor none "
                               "-serial none -S");

    qtest_writel(s, 0xc0d00020, 0x028);       /* CSR_HW_REV */
    qtest_writel(s, 0xc0d0001c, 0x6a000000);
    g_assert_cmphex(qtest_readl(s, 0xc0d00028), ==, 0x420); /* silicon */
    qtest_writel(s, 0xc0d00020, 0x000);       /* CSR_HW_IF_CONFIG_REG */
    qtest_writel(s, 0xc0d0001c, 0x6a000000);
    g_assert_cmphex(qtest_readl(s, 0xc0d00028) & 0xf, ==, 0x2);
    qtest_quit(s);
}

/* Cross-core view + two-core semaphore (findings K1, S1): UMAC's set at
 * 0xc0d03820 and LMAC's at 0xd03800 arbitrate the same bits. */
/* Configure-and-start unit (finding G1, [I]): UMAC's start at 0xc0d03f10
 * reads back done at 0xc0d03f14 bit 0. */
static void test_cfg_unit_start_done(void)
{
    QTestState *s = qtest_init("-M ax210 -m 32M -nographic -monitor none "
                               "-serial none -S");

    g_assert_cmpuint(qtest_readl(s, 0xc0d03f14) & 1, ==, 0);
    qtest_writel(s, 0xc0d03f10, 1);
    g_assert_cmpuint(qtest_readl(s, 0xc0d03f14) & 1, ==, 1);
    qtest_quit(s);
}

static void test_lmac_to_umac_frame(void)
{
    QTestState *s = qtest_init("-M ax210 -m 32M -nographic -monitor none "
                               "-serial none -S");
    static const uint32_t want[] = { 0x0800000c, 0x0000ffff, 0x11, 0x22 };

    g_assert_cmpuint(qtest_readl(s, 0xc0d02e64) & (1u << 14), ==, 0);
    qtest_writel(s, 0xc0a0043c, 0xd);           /* TXF_LARC_NUM 13 */
    qtest_writel(s, 0xc0a00440, 0x0000ffff);    /* cmd header */
    qtest_writel(s, 0xc0a00440, 8);             /* byte length */
    qtest_writel(s, 0xc0a00440, 0x11);
    g_assert_cmpuint(qtest_readl(s, 0xc0d02e64) & (1u << 14), ==, 0);
    qtest_writel(s, 0xc0a00440, 0x22);
    g_assert_cmpuint(qtest_readl(s, 0xc0d02e64) & (1u << 14), !=, 0);
    g_assert_cmpuint(qtest_readl(s, 0xc0d05498) >> 31, ==, 1);
    for (int i = 0; i < 4; i++) {
        g_assert_cmphex(qtest_readl(s, 0xc0d00e68), ==, want[i]);
    }
    g_assert_cmpuint(qtest_readl(s, 0xc0d02e64) & (1u << 14), ==, 0);
    qtest_quit(s);
}

static void test_lmac_nic_doorbell(void)
{
    QTestState *s = qtest_init("-M ax210 -m 32M -nographic -monitor none "
                               "-serial none -S");

    qtest_writel(s, 0xc0a01e80, 0x10);          /* mask */
    g_assert_cmphex(qtest_readl(s, 0xc0a01e84), ==, 0);
    qtest_writel(s, 0xc0a01e88, 0x10);          /* UMAC sets cause bit 4 */
    g_assert_cmphex(qtest_readl(s, 0xc0a01e84), ==, 0x10);
    qtest_writel(s, 0xc0a01e84, 0x10);          /* write 1 to clear */
    g_assert_cmphex(qtest_readl(s, 0xc0a01e84), ==, 0);
    qtest_quit(s);
}

static void test_semaphore_two_cores(void)
{
    QTestState *s = qtest_init("-M ax210 -m 32M -nographic -monitor none "
                               "-serial none -S");

    qtest_writel(s, 0xc0d03820, 1);           /* UMAC requests bit 0 */
    g_assert_cmpuint(qtest_readl(s, 0xc0d03824) & 1, ==, 1);
    qtest_writel(s, 0xc0d03800, 1);           /* LMAC view: busy */
    g_assert_cmpuint(qtest_readl(s, 0xc0d03804) & 1, ==, 0);
    qtest_writel(s, 0xc0d03828, 1);           /* UMAC releases */
    qtest_writel(s, 0xc0d03800, 1);           /* LMAC now gets it */
    g_assert_cmpuint(qtest_readl(s, 0xc0d03804) & 1, ==, 1);
    qtest_quit(s);
}

static uint32_t csr_read(QTestState *s, uint32_t off)
{
    qtest_writel(s, 0xc0d00020, off);
    qtest_writel(s, 0xc0d0001c, 0x6a000000);
    return qtest_readl(s, 0xc0d00028);
}

static void put_tlv(GByteArray *b, uint32_t type, uint32_t off,
                    const void *data, uint32_t len)
{
    uint8_t hdr[12];
    static const uint8_t pad[4];

    stl_le_p(hdr, type);
    stl_le_p(hdr + 4, len + (type == 19 ? 4 : 0));
    stl_le_p(hdr + 8, off);
    g_byte_array_append(b, hdr, type == 19 ? 12 : 8);
    g_byte_array_append(b, data, len);
    g_byte_array_append(b, pad, ROUND_UP(len, 4) - len);
}

/* Host driver model (finding H1): from a .ucode, the context info, prph
 * scratch and DRAM map laid out as iwl_pcie_ctxt_info_v2_alloc() does, and
 * the CSRs as iwl_pcie_ctxt_info_v2_kick() leaves them. */
static void test_host_driver_ctxt_info(void)
{
    static const uint8_t lmac[8] = "LMACsec0", umac[8] = "UMACsec0";
    static const uint8_t page[4] = "PAGE", iml[12] = "IMLimage0123";
    GByteArray *b = g_byte_array_new();
    uint8_t hdr[88] = { 0 }, sep[4] = { 0 };
    QTestState *s;
    uint64_t ci, ps;
    uint32_t ctl;
    char *path;
    int fd;

    stl_le_p(hdr + 4, 0x0a4c5749);            /* IWL_TLV_UCODE_MAGIC */
    g_byte_array_append(b, hdr, sizeof(hdr));
    put_tlv(b, 19, 0x00440000, lmac, sizeof(lmac));
    put_tlv(b, 19, 0xffffcccc, sep, sizeof(sep));
    put_tlv(b, 19, 0x80440000, umac, sizeof(umac));
    put_tlv(b, 19, 0xaaaabbbb, sep, sizeof(sep));
    put_tlv(b, 19, 0x00000000, page, sizeof(page));
    put_tlv(b, 52, 0, iml, sizeof(iml));
    fd = g_file_open_tmp("ax210-XXXXXX.ucode", &path, NULL);
    g_assert(fd >= 0);
    g_assert(write(fd, b->data, b->len) == b->len);
    close(fd);
    g_byte_array_free(b, true);

    g_setenv("AX210_HOSTFW", path, true);
    s = qtest_init("-M ax210 -m 32M -nographic -monitor none "
                   "-serial none -S");
    g_unsetenv("AX210_HOSTFW");
    unlink(path);
    g_free(path);

    ci = csr_read(s, 0x118) | (uint64_t)csr_read(s, 0x11c) << 32;
    g_assert_cmphex(ci, !=, 0);
    ps = qtest_readq(s, ci + 0x58);           /* prph_scratch_base_addr */
    g_assert_cmphex(qtest_readl(s, ci + 0x60), ==, 0x67c);
    g_assert_cmphex(qtest_readw(s, ps + 0x00), ==, csr_read(s, 0x028) & 0xffff);
    ctl = qtest_readl(s, ps + 0x08);          /* RB_SIZE_4K|MTR_MODE|256B */
    g_assert_cmphex(ctl, ==, 0x000f0000);
    g_assert_cmphex(qtest_readq(s, ps + 0x27c) & 0xfff, ==, 0); /* lmac_img[0] */
    g_assert_cmphex(qtest_readq(s, qtest_readq(s, ps + 0x27c)), ==,
                    ldq_le_p(lmac));
    g_assert_cmphex(qtest_readq(s, ps + 0x27c + 8), ==, 0);
    g_assert_cmphex(qtest_readq(s, qtest_readq(s, ps + 0x7c)), ==,
                    ldq_le_p(umac));                        /* umac_img[0] */
    g_assert_cmphex(qtest_readl(s, qtest_readq(s, ps + 0x47c)), ==,
                    ldl_le_p(page));                        /* virtual_img[0] */
    g_assert_cmphex(qtest_readl(s, csr_read(s, 0x120) + 8), ==,
                    ldl_le_p(iml + 8));                     /* CSR_IML_DATA_ADDR */
    g_assert_cmphex(csr_read(s, 0x128), ==, sizeof(iml));
    g_assert_cmphex(csr_read(s, 0x000) & 0x2, ==, 0x2);     /* AUTO_FUNC_BOOT_ENA */
    qtest_quit(s);
}

/* UMAC timeout timer (finding G2, [I]): a write of N to 0xc0d02c78 sets
 * 0xc0d02e2c bit 4 N us of virtual time later; writing bit 4 clears it. */
static void test_umac_timeout_timer(void)
{
    /* No -S: virtual-clock timers only run while the VM runs. */
    QTestState *s = qtest_init("-M ax210 -m 32M -nographic -monitor none "
                               "-serial none");

    qtest_writel(s, 0xc0d02c78, 1000);
    qtest_clock_step(s, 999 * 1000);
    g_assert_cmpuint(qtest_readl(s, 0xc0d02e2c) & 0x10, ==, 0);
    qtest_clock_step(s, 2 * 1000);
    g_assert_cmpuint(qtest_readl(s, 0xc0d02e2c) & 0x10, ==, 0x10);
    qtest_writel(s, 0xc0d02e2c, 0x10);
    g_assert_cmpuint(qtest_readl(s, 0xc0d02e2c) & 0x10, ==, 0);
    qtest_quit(s);
}

/* Device address space (finding M3): UMAC's 0x80000000 (cached) and
 * 0xc0000000 (uncached) windows are views of the device addresses LMAC
 * uses; dccm_umac at 0x880000 is UMAC's 0xc0880000. */
static void test_umac_windows_alias_device(void)
{
    QTestState *s = qtest_init("-M ax210 -m 32M -nographic -monitor none "
                               "-serial none -S");

    qtest_writel(s, 0x00880000, 0x5a5a1234);
    g_assert_cmphex(qtest_readl(s, 0xc0880000), ==, 0x5a5a1234);
    g_assert_cmphex(qtest_readl(s, 0x80880000), ==, 0x5a5a1234);
    qtest_writel(s, 0x80471488, 0x0badf00d);
    g_assert_cmphex(qtest_readl(s, 0x00471488), ==, 0x0badf00d);
    qtest_quit(s);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/ax210/ram-windows", test_ram_windows);
    qtest_add_func("/ax210/write-readback", test_write_readback);
    qtest_add_func("/ax210/override-wins-over-store",
                   test_override_wins_over_store);
    qtest_add_func("/ax210/default-counter-increments",
                   test_default_counter_increments);
    qtest_add_func("/ax210/host-csr-proxy-seed-and-read",
                   test_host_csr_proxy_seed_and_read);
    qtest_add_func("/ax210/dma-copies-host-to-device",
                   test_dma_copies_host_to_device);

    qtest_add_func("/ax210/lmac/aes-fips197-c1", test_lmac_aes_fips197);
    qtest_add_func("/ax210/lmac/power-fsm", test_lmac_power_fsm);
    qtest_add_func("/ax210/lmac/semaphore", test_lmac_semaphore);
    qtest_add_func("/ax210/lmac/indirect-window", test_lmac_indirect_window);
    qtest_add_func("/ax210/lmac/indirect-target-chip-ids",
                   test_lmac_indirect_target_and_chip_ids);
    qtest_add_func("/ax210/lmac/timestamp", test_lmac_timestamp);
    qtest_add_func("/ax210/lmac/msg-fifo-space", test_lmac_msg_fifo_space);

    qtest_add_func("/ax210/lmac/mpu-map-ram", test_lmac_mpu_map_ram);

    qtest_add_func("/ax210/host-csr-driver-defaults",
                   test_host_csr_driver_defaults);
    qtest_add_func("/ax210/semaphore-two-cores", test_semaphore_two_cores);
    qtest_add_func("/ax210/host-driver-ctxt-info", test_host_driver_ctxt_info);
    qtest_add_func("/ax210/cfg-unit-start-done", test_cfg_unit_start_done);
    qtest_add_func("/ax210/umac-timeout-timer", test_umac_timeout_timer);
    qtest_add_func("/ax210/lmac-to-umac-frame", test_lmac_to_umac_frame);
    qtest_add_func("/ax210/lmac-nic-doorbell", test_lmac_nic_doorbell);
    qtest_add_func("/ax210/umac-windows-alias-device",
                   test_umac_windows_alias_device);

    return g_test_run();
}
