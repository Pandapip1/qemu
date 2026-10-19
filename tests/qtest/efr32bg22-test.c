/*
 * QTest coverage for the EFR32BG22 native peripheral model.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "libqtest-single.h"

#define FRC_BASE             0xa8004000
#define FRC_WHITEPOLY        0x034
#define MODEM_BASE           0xa8014000
#define MODEM_SHAPING1       0x098
#define MODEM_LONGRANGE5     0x14c
#define BUFC_BASE            0xaa000000
#define BUFC_CH0             0x00c
#define BUFC_READOFFSET      0x00c
#define BUFC_READDATA        0x014
#define BUFC_WRITEDATA       0x018

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

static void test_bufc_data_port(void)
{
    uint32_t channel = BUFC_BASE + BUFC_CH0;

    writel(channel + BUFC_WRITEDATA, 0x5a);
    writel(channel + BUFC_WRITEDATA, 0xa5);
    writel(channel + BUFC_READOFFSET, 0);

    g_assert_cmphex(readl(channel + BUFC_READDATA), ==, 0x5a);
    g_assert_cmphex(readl(channel + BUFC_READDATA), ==, 0xa5);
}

int main(int argc, char **argv)
{
    int result;

    g_test_init(&argc, &argv, NULL);
    qtest_start("-machine efr32bg22 -S");

    qtest_add_func("/efr32bg22/register-aliases", test_register_aliases);
    qtest_add_func("/efr32bg22/modem-masks", test_modem_masks);
    qtest_add_func("/efr32bg22/bufc-data-port", test_bufc_data_port);

    result = g_test_run();
    qtest_end();
    return result;
}
