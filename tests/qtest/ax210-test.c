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

    return g_test_run();
}
