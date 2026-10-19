/*
 * QTest coverage for the EFR32BG22 native peripheral model.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "libqtest-single.h"

#define FRC_BASE             0xa8004000
#define AGC_BASE             0xa800c000
#define FRC_WHITEPOLY        0x034
#define MODEM_BASE           0xa8014000
#define MODEM_SHAPING1       0x098
#define MODEM_LONGRANGE5     0x14c
#define RFCRC_BASE           0xa8010000
#define RFCRC_EN             0x004
#define RFCRC_CTRL           0x008
#define PROTIMER_BASE         0xa801c000
#define PROTIMER_CTRL         0x008
#define PROTIMER_STATE        0x030
#define PROTIMER_PERIOD       0x038
#define PROTIMER_CMP0         0x100
#define SYNTH_BASE            0xa8018000
#define RAC_BASE              0xa8020000
#define RAC_STATUS            0x00c
#define RAC_CMD               0x010
#define RAC_RXENSRCEN         0x008
#define RAC_IEN               0x020
#define RAC_SEQIF             0x028
#define RAC_CMD_TXEN          (1u << 0)
#define RAC_CMD_TXDIS         (1u << 5)
#define RAC_STATUS_STATE_MASK 0x0f000000
#define RAC_STATUS_STATE_TX   (9u << 24)
#define RAC_SEQIF_STATEOFF    (1u << 16)
#define RAC_SEQIF_STATETX     (1u << 25)
#define BUFC_BASE            0xaa000000
#define BUFC_CH0             0x00c
#define BUFC_CH1             0x04c
#define BUFC_IF              0x114
#define BUFC_IEN             0x118
#define BUFC_WRITEOFFSET     0x008
#define BUFC_READOFFSET      0x00c
#define BUFC_READDATA        0x014
#define BUFC_WRITEDATA       0x018
#define BUFC_STATUS          0x020
#define BUFC_THRESHOLD       0x024
#define BUFC_CMD             0x028

#define RADIO_NONSECURE_ALIAS 0x10000000

#define REGISTER_SET_WINDOW  0x1000
#define REGISTER_CLEAR_WINDOW 0x2000
#define REGISTER_TOGGLE_WINDOW 0x3000

static void test_register_aliases(void)
{
    uint32_t address = FRC_BASE + FRC_WHITEPOLY;

    writel(address, 0);
    writel(address + REGISTER_SET_WINDOW, 0x12);
    g_assert_cmphex(readl(address), ==, 0x12);

    writel(address + REGISTER_CLEAR_WINDOW, 0x02);
    g_assert_cmphex(readl(address), ==, 0x10);

    writel(address + REGISTER_TOGGLE_WINDOW, 0x01);
    g_assert_cmphex(readl(address), ==, 0x11);
}

static void test_modem_masks(void)
{
    writel(MODEM_BASE + MODEM_SHAPING1, 0xdeadbeef);
    g_assert_cmphex(readl(MODEM_BASE + MODEM_SHAPING1), ==, 0xdeadbeef);

    writel(MODEM_BASE + MODEM_LONGRANGE5, UINT32_MAX);
    g_assert_cmphex(readl(MODEM_BASE + MODEM_LONGRANGE5), ==, 0x0fffffff);
}

static void test_rfcrc_masks(void)
{
    writel(RFCRC_BASE + RFCRC_EN, UINT32_MAX);
    writel(RFCRC_BASE + RFCRC_CTRL, UINT32_MAX);

    g_assert_cmphex(readl(RFCRC_BASE + RFCRC_EN), ==, 1);
    g_assert_cmphex(readl(RFCRC_BASE + RFCRC_CTRL), ==, 0x1fef);
}

static void test_radio_enable_masks(void)
{
    const uint32_t blocks[] = {
        FRC_BASE,
        AGC_BASE,
        RFCRC_BASE,
        MODEM_BASE,
        SYNTH_BASE,
        RAC_BASE,
    };

    for (size_t i = 0; i < G_N_ELEMENTS(blocks); i++) {
        writel(blocks[i] + 0x004, UINT32_MAX);
        g_assert_cmphex(readl(blocks[i] + 0x004), ==, 1);
    }
}

static void test_radio_nonsecure_alias(void)
{
    uint32_t secure_address = FRC_BASE + FRC_WHITEPOLY;
    uint32_t nonsecure_address = secure_address + RADIO_NONSECURE_ALIAS;

    writel(secure_address, 0);
    writel(nonsecure_address, UINT32_MAX);

    g_assert_cmphex(readl(secure_address), ==, 0xffff);
    g_assert_cmphex(readl(nonsecure_address), ==, 0xffff);
}

static void test_bufc_data_port(void)
{
    uint32_t channel = BUFC_BASE + BUFC_CH0;

    writel(channel + BUFC_WRITEDATA, 0x5a);
    writel(channel + BUFC_WRITEDATA, 0xa5);
    writel(channel + BUFC_READOFFSET, 0);

    g_assert_cmphex(readl(channel + BUFC_READDATA), ==, 0x5a);
    g_assert_cmphex(readl(channel + BUFC_READDATA), ==, 0xa5);
}

static void test_bufc_tx_fifo_setup_latches(void)
{
    uint32_t channel = BUFC_BASE + BUFC_CH0;

    /* A vendor 512-byte FIFO encodes as log2(512) - 6 = 3. */
    writel(channel, 3);
    writel(channel + 0x004, 0x20001000);
    writel(channel + BUFC_WRITEOFFSET, 18);

    g_assert_cmphex(readl(channel), ==, 3);
    g_assert_cmphex(readl(channel + 0x004), ==, 0x20001000);
    g_assert_cmphex(readl(channel + BUFC_WRITEOFFSET), ==, 18);

    writel(channel + BUFC_CMD, 1);
    writel(channel + BUFC_CMD, 2);

    g_assert_cmphex(readl(channel), ==, 3);
    g_assert_cmphex(readl(channel + 0x004), ==, 0x20001000);
    g_assert_cmphex(readl(channel + BUFC_CMD), ==, 2);
}

static void test_bufc_sparse_threshold_flag(void)
{
    uint32_t channel = BUFC_BASE + BUFC_CH1;
    uint32_t threshold_flag = 1u << 10;

    writel(BUFC_BASE + BUFC_IEN, threshold_flag);
    writel(channel + BUFC_THRESHOLD, 0);
    writel(channel + BUFC_WRITEDATA, 0x5a);

    g_assert_cmphex(readl(BUFC_BASE + BUFC_IF) & threshold_flag, ==,
                    threshold_flag);
    g_assert_cmphex(readl(channel + BUFC_STATUS), ==, (1u << 20) | 1u);
}

static void test_protimer_trace_latches(void)
{
    writel(PROTIMER_BASE + PROTIMER_CTRL, 0x00011102);
    writel(PROTIMER_BASE + PROTIMER_STATE, 0x00001233);
    writel(PROTIMER_BASE + PROTIMER_PERIOD, 0x80015558);
    writel(PROTIMER_BASE + PROTIMER_CMP0, 0x01000003);

    g_assert_cmphex(readl(PROTIMER_BASE + PROTIMER_CTRL), ==, 0x00011102);
    g_assert_cmphex(readl(PROTIMER_BASE + PROTIMER_STATE), ==, 0x00001233);
    g_assert_cmphex(readl(PROTIMER_BASE + PROTIMER_PERIOD), ==, 0x80015558);
    g_assert_cmphex(readl(PROTIMER_BASE + PROTIMER_CMP0), ==, 0x01000003);
}

static void test_rac_tx_commands(void)
{
    writel(RAC_BASE + RAC_CMD, RAC_CMD_TXEN);
    g_assert_cmphex(readl(RAC_BASE + RAC_STATUS) & RAC_STATUS_STATE_MASK, ==,
                    RAC_STATUS_STATE_TX);
    g_assert_cmphex(readl(RAC_BASE + RAC_SEQIF) & RAC_SEQIF_STATETX, ==,
                    RAC_SEQIF_STATETX);

    writel(RAC_BASE + RAC_CMD, RAC_CMD_TXDIS);
    g_assert_cmphex(readl(RAC_BASE + RAC_STATUS) & RAC_STATUS_STATE_MASK, ==,
                    0);
    g_assert_cmphex(readl(RAC_BASE + RAC_SEQIF) & RAC_SEQIF_STATEOFF, ==,
                    RAC_SEQIF_STATEOFF);
}

static void test_rac_masks(void)
{
    writel(RAC_BASE + RAC_RXENSRCEN, UINT32_MAX);
    writel(RAC_BASE + RAC_IEN, UINT32_MAX);

    g_assert_cmphex(readl(RAC_BASE + RAC_RXENSRCEN), ==, 0x00003fff);
    g_assert_cmphex(readl(RAC_BASE + RAC_IEN), ==, 0x00ff000f);
}

int main(int argc, char **argv)
{
    int result;

    g_test_init(&argc, &argv, NULL);
    qtest_start("-machine efr32bg22 -S");

    qtest_add_func("/efr32bg22/register-aliases", test_register_aliases);
    qtest_add_func("/efr32bg22/modem-masks", test_modem_masks);
    qtest_add_func("/efr32bg22/rfcrc-masks", test_rfcrc_masks);
    qtest_add_func("/efr32bg22/radio-enable-masks", test_radio_enable_masks);
    qtest_add_func("/efr32bg22/radio-nonsecure-alias",
                   test_radio_nonsecure_alias);
    qtest_add_func("/efr32bg22/bufc-data-port", test_bufc_data_port);
    qtest_add_func("/efr32bg22/bufc-tx-fifo-setup-latches",
                   test_bufc_tx_fifo_setup_latches);
    qtest_add_func("/efr32bg22/bufc-sparse-threshold-flag",
                   test_bufc_sparse_threshold_flag);
    qtest_add_func("/efr32bg22/protimer-trace-latches",
                   test_protimer_trace_latches);
    qtest_add_func("/efr32bg22/rac-tx-commands", test_rac_tx_commands);
    qtest_add_func("/efr32bg22/rac-masks", test_rac_masks);

    result = g_test_run();
    qtest_end();
    return result;
}
