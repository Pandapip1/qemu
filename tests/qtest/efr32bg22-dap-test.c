/*
 * QTest for the EFR32BG22 debug port as a probe sees it: SWD over the
 * swd-remote-bitbang probe (OpenOCD's remote_bitbang protocol), the SW-DP,
 * the AHB-AP into the M33 bus, the DCI AP, the M33 debug unit (DCB, FPB,
 * DWT, ROM table) and MSC flash programming.
 *
 * Identification values are silicon captures (owo-open-fw
 * docs/debug-port-model.md).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <sys/socket.h>
#include <sys/un.h>
#include "qemu/cutils.h"
#include "libqtest.h"

/* remote_bitbang commands */
#define RBB_DRIVE 'O'
#define RBB_RELEASE 'o'
#define RBB_SAMPLE 'c'
#define RBB_WRITE(clk, dio) ('d' + ((clk) ? 2 : 0) + ((dio) ? 1 : 0))

/* SWD */
#define SWD_ACK_OK 0x1
#define SWD_ACK_FAULT 0x4
#define SWD_NO_ACK 0x7          /* nobody drives: the pull-up */
#define SWD_LINE_RESET_ONES 56
#define SWD_JTAG_TO_SWD 0xe79e
#define SWD_IDLE_CYCLES 8

/* DP */
#define DP_DPIDR 0x0
#define DP_ABORT 0x0
#define DP_CTRL_STAT 0x4
#define DP_SELECT 0x8
#define DP_RDBUFF 0xc
#define CTRL_CSYSPWRUPREQ (1u << 30)
#define CTRL_CDBGPWRUPREQ (1u << 28)
#define CTRL_CSYSPWRUPACK (1u << 31)
#define CTRL_CDBGPWRUPACK (1u << 29)
#define CTRL_STICKYERR (1u << 5)
#define ABORT_STKERRCLR (1u << 2)
#define EFR32_DPIDR 0x6ba02477u

/* MEM-AP */
#define AP_CSW 0x00
#define AP_TAR 0x04
#define AP_DRW 0x0c
#define AP_BASE 0xf8
#define AP_IDR 0xfc
#define CSW_32BIT_AUTOINC 0x23000012u
#define EFR32_AHBAP_IDR 0x84770001u
#define EFR32_AHBAP_BASE 0xe00ff003u
#define AP_M33 0
#define AP_DCI 1

/* DCI */
#define DCI_WDATA 0x1000
#define DCI_RDATA 0x1004
#define DCI_STATUS 0x1008
#define DCI_ID 0x10fc
#define DCI_ID_VALUE 0xdc11du
#define DCI_STATUS_RDATAVALID (1u << 8)
#define DCI_CMD_GET_STATUS 0xfe010000u
#define DCI_DEBUGLOCK_DEVICE_ERASE 0x2u

/* M33 debug */
#define DHCSR 0xe000edf0
#define DCRSR 0xe000edf4
#define DCRDR 0xe000edf8
#define AIRCR 0xe000ed0c
#define CPUID 0xe000ed00
#define FP_CTRL 0xe0002000
#define FP_COMP0 0xe0002008
#define DWT_CTRL 0xe0001000
#define ROM_TABLE 0xe00ff000
#define DBGKEY_C_DEBUGEN 0xa05f0001u
#define DBGKEY_C_HALT 0xa05f0003u
#define DHCSR_S_HALT (1u << 17)
#define DHCSR_S_REGRDY (1u << 16)
#define DHCSR_S_RESET_ST (1u << 25)
#define DHCSR_S_SDE (1u << 20)
#define AIRCR_SYSRESETREQ 0x05fa0004u
#define REGSEL_R0 0
#define EFR32_M33_CPUID 0x410fd214u

/* MSC */
#define MSC_NS 0x50030000
#define MSC_WRITECTRL 0x00c
#define MSC_WRITECMD 0x010
#define MSC_ADDRB 0x014
#define MSC_WDATA 0x018
#define MSC_STATUS 0x01c
#define MSC_WRITECMD_ERASEPAGE 0x2
#define FLASH_TEST_PAGE 0x00040000
#define SRAM_BASE 0x20000000

typedef struct Swd {
    QTestState *qts;
    int fd;
    bool drive;
    GString *out;
} Swd;

static void swd_flush(Swd *s)
{
    ssize_t off = 0;

    while (off < s->out->len) {
        ssize_t n = write(s->fd, s->out->str + off, s->out->len - off);

        g_assert_cmpint(n, >, 0);
        off += n;
    }
    g_string_truncate(s->out, 0);
}

static bool swd_cycle(Swd *s, bool drive, bool bit, bool sample)
{
    bool level = false;

    if (drive != s->drive) {
        g_string_append_c(s->out, drive ? RBB_DRIVE : RBB_RELEASE);
        s->drive = drive;
    }
    g_string_append_c(s->out, RBB_WRITE(0, bit));
    if (sample) {
        char c;

        g_string_append_c(s->out, RBB_SAMPLE);
        swd_flush(s);
        g_assert_cmpint(read(s->fd, &c, 1), ==, 1);
        level = c == '1';
    }
    g_string_append_c(s->out, RBB_WRITE(1, bit));
    return level;
}

static void swd_send_bits(Swd *s, uint64_t value, unsigned n)
{
    for (unsigned i = 0; i < n; i++) {
        swd_cycle(s, true, (value >> i) & 1, false);
    }
}

static void swd_line_reset(Swd *s)
{
    for (unsigned i = 0; i < SWD_LINE_RESET_ONES; i++) {
        swd_cycle(s, true, true, false);
    }
    swd_send_bits(s, 0, SWD_IDLE_CYCLES);
}

static void swd_connect(Swd *s)
{
    for (unsigned i = 0; i < SWD_LINE_RESET_ONES; i++) {
        swd_cycle(s, true, true, false);
    }
    swd_send_bits(s, SWD_JTAG_TO_SWD, 16);
    swd_line_reset(s);
}

static unsigned swd_transfer(Swd *s, bool apndp, bool rnw, unsigned a,
                             uint32_t *data)
{
    unsigned field = apndp | (rnw << 1) | (((a >> 2) & 3) << 2);
    unsigned req = 1 | (field << 1) | (__builtin_parity(field) << 5) |
                   (1 << 7);
    unsigned ack = 0;

    swd_send_bits(s, req, 8);
    swd_cycle(s, false, true, false);
    for (unsigned i = 0; i < 3; i++) {
        ack |= swd_cycle(s, false, true, true) << i;
    }
    if (ack == SWD_ACK_OK && rnw) {
        uint64_t v = 0;

        for (unsigned i = 0; i < 33; i++) {
            v |= (uint64_t)swd_cycle(s, false, true, true) << i;
        }
        g_assert_cmpint((v >> 32) & 1, ==, __builtin_parity((uint32_t)v));
        *data = v;
        swd_cycle(s, false, true, false);
    } else if (ack == SWD_ACK_OK) {
        swd_cycle(s, false, true, false);
        swd_send_bits(s, *data | ((uint64_t)__builtin_parity(*data) << 32),
                      33);
    } else {
        swd_cycle(s, false, true, false);
    }
    swd_send_bits(s, 0, SWD_IDLE_CYCLES);
    swd_flush(s);
    return ack;
}

static uint32_t dp_read(Swd *s, unsigned a)
{
    uint32_t v = 0;

    g_assert_cmpuint(swd_transfer(s, false, true, a, &v), ==, SWD_ACK_OK);
    return v;
}

static void dp_write(Swd *s, unsigned a, uint32_t v)
{
    g_assert_cmpuint(swd_transfer(s, false, false, a, &v), ==, SWD_ACK_OK);
}

static void ap_select(Swd *s, unsigned ap, unsigned reg)
{
    dp_write(s, DP_SELECT, (ap << 24) | (reg & 0xf0));
}

static uint32_t ap_read(Swd *s, unsigned ap, unsigned reg)
{
    uint32_t v = 0;

    ap_select(s, ap, reg);
    g_assert_cmpuint(swd_transfer(s, true, true, reg & 0xc, &v), ==,
                     SWD_ACK_OK);
    return dp_read(s, DP_RDBUFF);
}

static void ap_write(Swd *s, unsigned ap, unsigned reg, uint32_t v)
{
    ap_select(s, ap, reg);
    g_assert_cmpuint(swd_transfer(s, true, false, reg & 0xc, &v), ==,
                     SWD_ACK_OK);
}

static uint32_t mem_read(Swd *s, unsigned ap, uint32_t addr)
{
    ap_write(s, ap, AP_CSW, CSW_32BIT_AUTOINC);
    ap_write(s, ap, AP_TAR, addr);
    return ap_read(s, ap, AP_DRW);
}

static void mem_write(Swd *s, unsigned ap, uint32_t addr, uint32_t v)
{
    ap_write(s, ap, AP_CSW, CSW_32BIT_AUTOINC);
    ap_write(s, ap, AP_TAR, addr);
    ap_write(s, ap, AP_DRW, v);
    /*
     * A write's data phase gets no reply; a read makes sure QEMU has
     * consumed it before the test looks through the qtest socket.
     */
    dp_read(s, DP_CTRL_STAT);
}

static Swd *swd_start(void)
{
    Swd *s = g_new0(Swd, 1);
    g_autofree char *dir = g_dir_make_tmp("efr32-dap-XXXXXX", NULL);
    g_autofree char *path = g_build_filename(dir, "swd", NULL);
    struct sockaddr_un addr = { .sun_family = AF_UNIX };

    s->qts = qtest_initf("-machine efr32bg22 "
                         "-chardev socket,id=swd,path=%s,server=on,wait=off "
                         "-device swd-remote-bitbang,chardev=swd", path);
    s->fd = socket(AF_UNIX, SOCK_STREAM, 0);
    g_assert_cmpint(s->fd, >=, 0);
    pstrcpy(addr.sun_path, sizeof(addr.sun_path), path);
    g_assert_cmpint(connect(s->fd, (struct sockaddr *)&addr, sizeof(addr)),
                    ==, 0);
    unlink(path);
    rmdir(dir);
    s->out = g_string_new(NULL);
    s->drive = false;   /* the probe has not driven SWDIO yet */
    swd_connect(s);
    return s;
}

static void swd_power_up(Swd *s)
{
    uint32_t ctrl;

    g_assert_cmphex(dp_read(s, DP_DPIDR), ==, EFR32_DPIDR);
    dp_write(s, DP_CTRL_STAT, CTRL_CSYSPWRUPREQ | CTRL_CDBGPWRUPREQ);
    ctrl = dp_read(s, DP_CTRL_STAT);
    g_assert_true(ctrl & CTRL_CSYSPWRUPACK);
    g_assert_true(ctrl & CTRL_CDBGPWRUPACK);
}

static void swd_end(Swd *s)
{
    close(s->fd);
    qtest_quit(s->qts);
    g_string_free(s->out, true);
    g_free(s);
}

/* After a line reset only a DPIDR read is answered (ADIv5.2 B4.3.3). */
static void test_dp_identity(void)
{
    Swd *s = swd_start();
    uint32_t v = 0;

    g_assert_cmpuint(swd_transfer(s, false, true, DP_CTRL_STAT, &v), ==,
                     SWD_NO_ACK);
    swd_line_reset(s);
    g_assert_cmphex(dp_read(s, DP_DPIDR), ==, EFR32_DPIDR);
    g_assert_cmpuint(swd_transfer(s, false, true, DP_CTRL_STAT, &v), ==,
                     SWD_ACK_OK);
    swd_end(s);
}

static void test_ap_identity_and_memory(void)
{
    Swd *s = swd_start();

    swd_power_up(s);
    g_assert_cmphex(ap_read(s, AP_M33, AP_IDR), ==, EFR32_AHBAP_IDR);
    g_assert_cmphex(ap_read(s, AP_M33, AP_BASE), ==, EFR32_AHBAP_BASE);
    g_assert_cmphex(mem_read(s, AP_M33, CPUID), ==, EFR32_M33_CPUID);

    /* The AHB-AP and the core see the same SRAM */
    qtest_writel(s->qts, SRAM_BASE, 0x12345678);
    g_assert_cmphex(mem_read(s, AP_M33, SRAM_BASE), ==, 0x12345678);
    mem_write(s, AP_M33, SRAM_BASE + 4, 0xcafef00d);
    g_assert_cmphex(qtest_readl(s->qts, SRAM_BASE + 4), ==, 0xcafef00d);

    /* ROM table: SCS, DWT, FPB, then the end marker */
    g_assert_cmphex(mem_read(s, AP_M33, ROM_TABLE), ==, 0xfff0f003);
    g_assert_cmphex(mem_read(s, AP_M33, ROM_TABLE + 4), ==, 0xfff02003);
    g_assert_cmphex(mem_read(s, AP_M33, ROM_TABLE + 8), ==, 0xfff03003);
    g_assert_cmphex(mem_read(s, AP_M33, ROM_TABLE + 12), ==, 0);

    /* 8 code comparators, FPB version 2; 4 DWT comparators */
    g_assert_cmphex(mem_read(s, AP_M33, FP_CTRL), ==, 0x10000080);
    g_assert_cmphex(mem_read(s, AP_M33, DWT_CTRL) >> 28, ==, 4);
    swd_end(s);
}

/* A bus error sets STICKYERR; AP accesses then FAULT until ABORT. */
static void test_sticky_error(void)
{
    Swd *s = swd_start();
    uint32_t v = 0;
    const uint32_t unmapped = 0x30000000;

    swd_power_up(s);
    ap_write(s, AP_M33, AP_CSW, CSW_32BIT_AUTOINC);
    ap_write(s, AP_M33, AP_TAR, unmapped);
    ap_select(s, AP_M33, AP_DRW);
    g_assert_cmpuint(swd_transfer(s, true, true, AP_DRW, &v), ==,
                     SWD_ACK_OK);
    g_assert_true(dp_read(s, DP_CTRL_STAT) & CTRL_STICKYERR);
    g_assert_cmpuint(swd_transfer(s, true, true, AP_DRW, &v), ==,
                     SWD_ACK_FAULT);
    dp_write(s, DP_ABORT, ABORT_STKERRCLR);
    g_assert_false(dp_read(s, DP_CTRL_STAT) & CTRL_STICKYERR);
    g_assert_cmphex(ap_read(s, AP_M33, AP_IDR), ==, EFR32_AHBAP_IDR);
    swd_end(s);
}

static void test_dci_status(void)
{
    Swd *s = swd_start();
    uint32_t len, word[4];

    swd_power_up(s);
    g_assert_cmphex(mem_read(s, AP_DCI, DCI_ID), ==, DCI_ID_VALUE);
    mem_write(s, AP_DCI, DCI_WDATA, 8);
    mem_write(s, AP_DCI, DCI_WDATA, DCI_CMD_GET_STATUS);
    g_assert_true(mem_read(s, AP_DCI, DCI_STATUS) & DCI_STATUS_RDATAVALID);
    len = mem_read(s, AP_DCI, DCI_RDATA);
    g_assert_cmphex(len & 0xffff0000, ==, 0);
    g_assert_cmpuint(len, ==, 4 + sizeof(word));
    for (unsigned i = 0; i < ARRAY_SIZE(word); i++) {
        word[i] = mem_read(s, AP_DCI, DCI_RDATA);
    }
    /* Suit and EK: only "Device erase" enabled in the debug-lock word */
    g_assert_cmphex(word[3], ==, DCI_DEBUGLOCK_DEVICE_ERASE);
    g_assert_false(mem_read(s, AP_DCI, DCI_STATUS) & DCI_STATUS_RDATAVALID);
    swd_end(s);
}

static uint32_t wait_dhcsr(Swd *s, uint32_t mask)
{
    uint32_t v = 0;

    for (int i = 0; i < 1000; i++) {
        v = mem_read(s, AP_M33, DHCSR);
        if ((v & mask) == mask) {
            break;
        }
        qtest_clock_step(s->qts, 1000);
    }
    return v;
}

static void test_halt_registers_and_reset(void)
{
    Swd *s = swd_start();
    uint32_t v;

    swd_power_up(s);
    /* The power-on reset is reported once */
    v = mem_read(s, AP_M33, DHCSR);
    g_assert_true(v & DHCSR_S_RESET_ST);
    g_assert_true(v & DHCSR_S_SDE);
    g_assert_false(mem_read(s, AP_M33, DHCSR) & DHCSR_S_RESET_ST);

    mem_write(s, AP_M33, DHCSR, DBGKEY_C_HALT);
    v = wait_dhcsr(s, DHCSR_S_HALT);
    g_assert_true(v & DHCSR_S_HALT);
    mem_write(s, AP_M33, DCRDR, 0x5a5aa5a5);
    mem_write(s, AP_M33, DCRSR, (1u << 16) | REGSEL_R0);
    g_assert_true(mem_read(s, AP_M33, DHCSR) & DHCSR_S_REGRDY);
    mem_write(s, AP_M33, DCRDR, 0);
    mem_write(s, AP_M33, DCRSR, REGSEL_R0);
    g_assert_cmphex(mem_read(s, AP_M33, DCRDR), ==, 0x5a5aa5a5);

    /* FPB comparators survive SYSRESETREQ; the halt does not */
    mem_write(s, AP_M33, FP_COMP0, 0x00001235);
    mem_write(s, AP_M33, FP_CTRL, 3);
    mem_write(s, AP_M33, AIRCR, AIRCR_SYSRESETREQ);
    v = wait_dhcsr(s, DHCSR_S_RESET_ST);
    g_assert_true(v & DHCSR_S_RESET_ST);
    g_assert_false(v & DHCSR_S_HALT);
    g_assert_cmphex(mem_read(s, AP_M33, FP_COMP0), ==, 0x00001235);
    g_assert_cmphex(mem_read(s, AP_M33, FP_CTRL) & 1, ==, 1);
    swd_end(s);
}

/* OpenOCD's efm32 driver sequence: erase a page, then program a word. */
static void test_msc_program(void)
{
    Swd *s = swd_start();

    swd_power_up(s);
    qtest_writel(s->qts, FLASH_TEST_PAGE, 0);
    mem_write(s, AP_M33, MSC_NS + MSC_WRITECTRL, 1);
    mem_write(s, AP_M33, MSC_NS + MSC_ADDRB, FLASH_TEST_PAGE);
    g_assert_cmphex(mem_read(s, AP_M33, MSC_NS + MSC_STATUS) & 0x6, ==, 0);
    mem_write(s, AP_M33, MSC_NS + MSC_WRITECMD, MSC_WRITECMD_ERASEPAGE);
    g_assert_cmphex(qtest_readl(s->qts, FLASH_TEST_PAGE), ==, 0xffffffff);
    mem_write(s, AP_M33, MSC_NS + MSC_ADDRB, FLASH_TEST_PAGE);
    mem_write(s, AP_M33, MSC_NS + MSC_WDATA, 0xdeadbeef);
    g_assert_cmphex(mem_read(s, AP_M33, FLASH_TEST_PAGE), ==, 0xdeadbeef);
    /* Programming only clears bits */
    mem_write(s, AP_M33, MSC_NS + MSC_ADDRB, FLASH_TEST_PAGE);
    mem_write(s, AP_M33, MSC_NS + MSC_WDATA, 0xffff0000);
    g_assert_cmphex(qtest_readl(s->qts, FLASH_TEST_PAGE), ==, 0xdead0000);
    /* Without WREN nothing changes */
    mem_write(s, AP_M33, MSC_NS + MSC_WRITECTRL, 0);
    mem_write(s, AP_M33, MSC_NS + MSC_WRITECMD, MSC_WRITECMD_ERASEPAGE);
    g_assert_cmphex(qtest_readl(s->qts, FLASH_TEST_PAGE), ==, 0xdead0000);
    swd_end(s);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/efr32bg22/dap/dp-identity", test_dp_identity);
    qtest_add_func("/efr32bg22/dap/ap-identity-memory",
                   test_ap_identity_and_memory);
    qtest_add_func("/efr32bg22/dap/sticky-error", test_sticky_error);
    qtest_add_func("/efr32bg22/dap/dci-status", test_dci_status);
    qtest_add_func("/efr32bg22/dap/halt-registers-reset",
                   test_halt_registers_and_reset);
    qtest_add_func("/efr32bg22/dap/msc-program", test_msc_program);
    return g_test_run();
}
