/*
 * QTest coverage for the EFR32BG22 native peripheral model.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "libqtest-single.h"

static char *tx_snapshot_trace_path;

#define FRC_BASE             0xa8004000
#define FRC_DATABUFFER       0x018
#define FRC_RXRAWDATA        0x090
#define FRC_IF               0x068
#define FRC_IEN              0x06c
#define FRC_IF_TXDONE        (1u << 0)
#define FRC_IF_RXDONE        (1u << 4)
#define AGC_BASE             0xa800c000
#define AGC_RSSI             0x018
#define AGC_FRAMERSSI        0x01c
#define AGC_IF               0x044
#define AGC_IEN              0x048
#define AGC_CMD              0x04c
#define FRC_WHITEPOLY        0x034
#define FRC_BUFFERMODE       0x078
#define FRC_SNIFFCTRL        0x084
#define FRC_RAWCTRL          0x08c
#define FRC_TRAILRXDATA       0x054
#define FRC_RXCTRL            0x04c
#define FRC_SEQIF            0x0b4
#define FRC_SEQIEN           0x0b8
#define MODEM_BASE           0xa8014000
#define MODEM_SHAPING1       0x098
#define MODEM_LONGRANGE5     0x14c
#define MODEM_IF             0x208
#define MODEM_IEN            0x20c
#define MODEM_IRCAL          0x228
#define RFCRC_BASE           0xa8010000
#define RFCRC_EN             0x004
#define RFCRC_CTRL           0x008
#define PROTIMER_BASE         0xa801c000
#define PROTIMER_NS_BASE      0xb801c000
#define PROTIMER_EN           0x004
#define PROTIMER_CTRL         0x008
#define PROTIMER_CMD          0x00c
#define PROTIMER_COUNTER      0x020
#define PROTIMER_STATE        0x030
#define PROTIMER_PERIOD       0x038
#define PROTIMER_IF           0x064
#define PROTIMER_IEN          0x070
#define PROTIMER_CMP0         0x100
#define PROTIMER_CMP0_TIME    0x10c
#define SYNTH_BASE            0xa8018000
#define RAC_BASE              0xa8020000
#define RAC_STATUS            0x00c
#define RAC_CMD               0x010
#define RAC_CTRL              0x014
#define RAC_SR0               0x040
#define RAC_SR3               0x04c
#define RAC_RXENSRCEN         0x008
#define RAC_IEN               0x020
#define RAC_SEQIF             0x028
#define RAC_CMD_TXEN          (1u << 0)
#define RAC_CMD_TXDIS         (1u << 5)
#define RAC_CMD_RXDIS         (1u << 8)
#define RAC_CTRL_FORCEDISABLE (1u << 0)
#define RAC_RXENSRCEN_SWRXEN  (1u << 1)
#define RAC_STATUS_STATE_MASK 0x0f000000
#define RAC_STATUS_STATE_RXSEARCH (2u << 24)
#define RAC_STATUS_STATE_TX   (9u << 24)
#define RAC_STATUS_TXENS      (1u << 30)
#define RAC_STATUS_RXENS      (1u << 31)
#define RAC_SEQIF_STATEOFF    (1u << 16)
#define RAC_SEQIF_STATERXSEARCH (1u << 18)
#define RAC_SEQIF_STATETX     (1u << 25)
#define DCDC_BASE             0x40094000
#define DCDC_NONSECURE_BASE   0x50094000
#define DCDC_CTRL             0x008
#define DCDC_IF               0x024
#define DCDC_STATUS           0x02c
#define DCDC_IF_REGULATION    (1u << 5)
#define EMU_BASE              0x40004000
#define EMU_STATUS            0x084
#define MSC_BASE              0x40030000
#define MSC_STATUS            0x01c
#define HFXO_BASE             0x4000c000
#define HFXO_NONSECURE_BASE   0x5000c000
#define HFXO_CTRL             0x028
#define HFXO_CMD              0x050
#define HFXO_STATUS           0x058
#define HFXO_STATUS_READY     0x000b0003
#define HFXO_STATUS_FSMLOCK   (1u << 30)
#define HFXO_CTRL_DISONDEMAND (1u << 1)
#define CMU_BASE              0x40008000
#define CMU_NONSECURE_BASE    0x50008000
#define CMU_SYSCLKCTRL        0x070
#define DPLL0_BASE            0x4001c000
#define DPLL0_NONSECURE_BASE  0x5001c000
#define DPLL_EN               0x004
#define DPLL_CFG              0x008
#define DPLL_CFG1             0x00c
#define DPLL_IF               0x010
#define DPLL_STATUS           0x018
#define DPLL_EN_EN            (1u << 0)
#define DPLL_IF_LOCK          (1u << 0)
#define DPLL_STATUS_READY_LOCKED 0x80000003
#define USART0_BASE           0x4005c000
#define USART0_NONSECURE_BASE 0x5005c000
#define USART1_BASE           0x40060000
#define USART_CMD             0x014
#define USART_STATUS          0x018
#define USART_TXDATA          0x03c
#define USART_CMD_TXEN        (1u << 2)
#define USART_CMD_TXDIS       (1u << 3)
#define USART_STATUS_TXENS    (1u << 1)
#define USART_STATUS_TXC      (1u << 5)
#define USART_STATUS_TXBL     (1u << 6)
#define BURTC_BASE             0x40064000
#define BURTC_EN               0x004
#define BURTC_CFG              0x008
#define BURTC_CMD              0x00c
#define BURTC_STATUS           0x010
#define BURTC_IF               0x014
#define BURTC_IEN              0x018
#define BURTC_CNT              0x020
#define BURTC_COMP             0x030
#define BURTC_IF_COMP          (1u << 1)
#define BURTC_EN_EN            (1u << 0)
#define BURTC_CMD_START        (1u << 0)
#define BURTC_CMD_STOP         (1u << 1)
#define BURTC_STATUS_RUNNING   (1u << 0)
#define RADIO_RAM_NS_BASE     0xb0000000
#define RADIO_RAM_RHCLKPRESC  0x4fe0
#define BUFC_BASE            0xaa000000
#define BUFC_EN              0x004
#define BUFC_CH0             0x00c
#define BUFC_CH1             0x04c
#define BUFC_CH2             0x08c
#define BUFC_IF              0x114
#define BUFC_IEN             0x118
#define BUFC_WRITEOFFSET     0x008
#define BUFC_READOFFSET      0x00c
#define BUFC_READDATA        0x014
#define BUFC_WRITEDATA       0x018
#define BUFC_STATUS          0x020
#define BUFC_THRESHOLD       0x024
#define BUFC_CMD             0x028
#define BUFC_TX_CHANNEL       BUFC_CH0
#define BUFC_RX_CHANNEL       BUFC_CH1

#define BUFC_SECURE_BASE     0xaa000000
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

static void test_burtc_counter_contract(void)
{
    uint32_t stopped;

    qtest_qmp_assert_success(global_qtest, "{'execute':'system_reset'}");
    /* The LL ticker uses the driver's COMP-backed top-value API. */
    g_assert_cmphex(readl(BURTC_BASE + BURTC_COMP), ==, UINT32_MAX);

    writel(BURTC_BASE + BURTC_CFG, UINT32_MAX);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_CFG), ==, 0xf3);
    writel(BURTC_BASE + BURTC_CFG + REGISTER_CLEAR_WINDOW, 0xf0);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_CFG), ==, 0x03);

    writel(BURTC_BASE + BURTC_EN + REGISTER_SET_WINDOW, BURTC_EN_EN);
    writel(BURTC_BASE + BURTC_CNT, 0x11223344);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_CNT), ==, 0x11223344);
    writel(BURTC_BASE + BURTC_CMD, BURTC_CMD_START);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_STATUS), ==,
                    BURTC_STATUS_RUNNING);
    qtest_clock_step(global_qtest, 1000000);
    g_assert_cmpuint(readl(BURTC_BASE + BURTC_CNT), >, 0x11223344);

    writel(BURTC_BASE + BURTC_CMD + REGISTER_SET_WINDOW, BURTC_CMD_STOP);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_STATUS), ==, 0);
    stopped = readl(BURTC_BASE + BURTC_CNT);
    qtest_clock_step(global_qtest, 1000000);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_CNT), ==, stopped);

    writel(BURTC_BASE + BURTC_EN + REGISTER_CLEAR_WINDOW, BURTC_EN_EN);
    writel(BURTC_BASE + BURTC_CMD, BURTC_CMD_START);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_STATUS), ==, 0);
}

/* The Zephyr counter driver uses COMP as a one-shot alarm and dispatches
 * only enabled pending flags from its IRQ 18 handler. */
static void test_burtc_compare_pending(void)
{
    qtest_qmp_assert_success(global_qtest, "{'execute':'system_reset'}");

    writel(BURTC_BASE + BURTC_EN + REGISTER_SET_WINDOW, BURTC_EN_EN);
    writel(BURTC_BASE + BURTC_CNT, 10);
    writel(BURTC_BASE + BURTC_IF + REGISTER_CLEAR_WINDOW, BURTC_IF_COMP);
    writel(BURTC_BASE + BURTC_COMP, 42);
    writel(BURTC_BASE + BURTC_IEN + REGISTER_SET_WINDOW, BURTC_IF_COMP);
    writel(BURTC_BASE + BURTC_CMD, BURTC_CMD_START);
    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    qtest_clock_step(global_qtest, 2 * 1000 * 1000);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");

    g_assert_cmphex(readl(BURTC_BASE + BURTC_IF) & BURTC_IF_COMP, ==,
                    BURTC_IF_COMP);
    writel(BURTC_BASE + BURTC_IF + REGISTER_CLEAR_WINDOW, BURTC_IF_COMP);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_IF) & BURTC_IF_COMP, ==, 0);
}

static void test_cmu_nonsecure_alias(void)
{
    qtest_qmp_assert_success(global_qtest, "{'execute':'system_reset'}");
    g_assert_cmphex(readl(CMU_BASE + CMU_SYSCLKCTRL) & 7, ==, 1);
    g_assert_cmphex(readl(CMU_NONSECURE_BASE + CMU_SYSCLKCTRL) & 7, ==, 1);

    writel(CMU_NONSECURE_BASE + CMU_SYSCLKCTRL, 3);
    g_assert_cmphex(readl(CMU_BASE + CMU_SYSCLKCTRL) & 7, ==, 3);

    writel(CMU_BASE + CMU_SYSCLKCTRL, 1);
    g_assert_cmphex(readl(CMU_NONSECURE_BASE + CMU_SYSCLKCTRL) & 7, ==, 1);
}

/* Zephyr CMU_DPLLLock() clears IF, enables DPLL0, then polls LOCK/FAIL. */
static void test_dpll_lock(void)
{
    qtest_qmp_assert_success(global_qtest, "{'execute':'system_reset'}");
    g_assert_cmphex(readl(DPLL0_BASE + DPLL_STATUS), ==, 0);
    g_assert_cmphex(readl(DPLL0_NONSECURE_BASE + DPLL_IF) & 7, ==, 0);

    writel(DPLL0_NONSECURE_BASE + DPLL_CFG, UINT32_MAX);
    writel(DPLL0_NONSECURE_BASE + DPLL_CFG1, UINT32_MAX);
    g_assert_cmphex(readl(DPLL0_BASE + DPLL_CFG), ==, 0x47);
    g_assert_cmphex(readl(DPLL0_BASE + DPLL_CFG1), ==, 0x0fff0fff);

    writel(DPLL0_NONSECURE_BASE + DPLL_IF + REGISTER_CLEAR_WINDOW, 7);
    writel(DPLL0_NONSECURE_BASE + DPLL_EN + REGISTER_SET_WINDOW,
           DPLL_EN_EN);
    g_assert_cmphex(readl(DPLL0_BASE + DPLL_STATUS), ==,
                    DPLL_STATUS_READY_LOCKED);
    g_assert_cmphex(readl(DPLL0_NONSECURE_BASE + DPLL_IF) & 7, ==,
                    DPLL_IF_LOCK);

    writel(DPLL0_BASE + DPLL_EN + REGISTER_CLEAR_WINDOW, DPLL_EN_EN);
    g_assert_cmphex(readl(DPLL0_NONSECURE_BASE + DPLL_STATUS), ==, 0);
}

/* Zephyr's USART_Tx() polls TXBL, then writes TXDATA through this sequence. */
static void test_usart_tx_polling(void)
{
    qtest_qmp_assert_success(global_qtest, "{'execute':'system_reset'}");

    g_assert_cmphex(readl(USART0_BASE + USART_STATUS), ==,
                    USART_STATUS_TXBL);
    writel(USART0_NONSECURE_BASE + USART_CMD + REGISTER_SET_WINDOW,
           USART_CMD_TXEN);
    g_assert_cmphex(readl(USART0_BASE + USART_STATUS), ==,
                    USART_STATUS_TXENS | USART_STATUS_TXBL);

    writel(USART0_NONSECURE_BASE + USART_TXDATA, 0x1a5);
    g_assert_cmphex(readl(USART0_BASE + USART_TXDATA), ==, 0xa5);
    g_assert_cmphex(readl(USART0_BASE + USART_STATUS), ==,
                    USART_STATUS_TXENS | USART_STATUS_TXBL | USART_STATUS_TXC);

    writel(USART0_BASE + USART_CMD, USART_CMD_TXDIS);
    g_assert_cmphex(readl(USART0_NONSECURE_BASE + USART_STATUS), ==,
                    USART_STATUS_TXBL | USART_STATUS_TXC);
    g_assert_cmphex(readl(USART1_BASE + USART_STATUS), ==,
                    USART_STATUS_TXBL);
}

static void test_modem_masks(void)
{
    writel(MODEM_BASE + MODEM_SHAPING1, 0xdeadbeef);
    g_assert_cmphex(readl(MODEM_BASE + MODEM_SHAPING1), ==, 0xdeadbeef);

    writel(MODEM_BASE + MODEM_LONGRANGE5, UINT32_MAX);
    writel(MODEM_BASE + MODEM_IF, UINT32_MAX);
    writel(MODEM_BASE + MODEM_IEN, UINT32_MAX);
    writel(MODEM_BASE + MODEM_IRCAL, UINT32_MAX);

    g_assert_cmphex(readl(MODEM_BASE + MODEM_LONGRANGE5), ==, 0x0fffffff);
    g_assert_cmphex(readl(MODEM_BASE + MODEM_IF), ==, 0x0007ffff);
    g_assert_cmphex(readl(MODEM_BASE + MODEM_IEN), ==, 0x0007ffff);
    g_assert_cmphex(readl(MODEM_BASE + MODEM_IRCAL), ==, 0x0000ffbf);
}

static void test_rfcrc_masks(void)
{
    writel(RFCRC_BASE + RFCRC_EN, UINT32_MAX);
    writel(RFCRC_BASE + RFCRC_CTRL, UINT32_MAX);

    g_assert_cmphex(readl(RFCRC_BASE + RFCRC_EN), ==, 1);
    g_assert_cmphex(readl(RFCRC_BASE + RFCRC_CTRL), ==, 0x1fef);
}

static void test_agc_rssi_status_registers(void)
{
    writel(AGC_BASE + AGC_RSSI, UINT32_MAX);
    writel(AGC_BASE + AGC_FRAMERSSI, UINT32_MAX);

    g_assert_cmphex(readl(AGC_BASE + AGC_RSSI), ==, 0x8000);
    g_assert_cmphex(readl(AGC_BASE + AGC_FRAMERSSI), ==, 0x8000);
}

static void test_agc_rssi_valid_latches(void)
{
    writel(AGC_BASE + AGC_IF, UINT32_MAX);
    writel(AGC_BASE + AGC_IEN, UINT32_MAX);

    g_assert_cmphex(readl(AGC_BASE + AGC_IF), ==, 0x37d);
    g_assert_cmphex(readl(AGC_BASE + AGC_IEN), ==, 0x37d);

    writel(AGC_BASE + REGISTER_CLEAR_WINDOW + AGC_IF, 1);
    g_assert_cmphex(readl(AGC_BASE + AGC_IF), ==, 0x37c);
}

static void test_agc_rssi_start_command(void)
{
    writel(AGC_BASE + AGC_CMD, 1);

    /* The public command register is write-only; no RF source is modeled. */
    g_assert_cmphex(readl(AGC_BASE + AGC_CMD), ==, 0);
    g_assert_cmphex(readl(AGC_BASE + AGC_IF) & 1, ==, 0);
}

static void test_frc_buffer_register_masks(void)
{
    writel(FRC_BASE + FRC_DATABUFFER, UINT32_MAX);
    writel(FRC_BASE + FRC_BUFFERMODE, UINT32_MAX);
    writel(FRC_BASE + FRC_TRAILRXDATA, UINT32_MAX);
    writel(FRC_BASE + FRC_RXRAWDATA, UINT32_MAX);

    g_assert_cmphex(readl(FRC_BASE + FRC_DATABUFFER), ==, 0xff);
    g_assert_cmphex(readl(FRC_BASE + FRC_BUFFERMODE), ==, 0xf);
    g_assert_cmphex(readl(FRC_BASE + FRC_TRAILRXDATA), ==, 0x3f);
    g_assert_cmphex(readl(FRC_BASE + FRC_RXRAWDATA), ==, 0);
}

static void test_frc_sniffctrl_mask(void)
{
    uint32_t address = FRC_BASE + FRC_SNIFFCTRL;

    writel(address, UINT32_MAX);
    g_assert_cmphex(readl(address), ==, 0x0003ffff);

    /* Stock clears this register then sets 0x17f8 through alias windows. */
    writel(address + REGISTER_CLEAR_WINDOW, UINT32_MAX);
    writel(address + REGISTER_SET_WINDOW, 0x17f8);
    g_assert_cmphex(readl(address), ==, 0x17f8);
}

static void test_frc_rawctrl_mask(void)
{
    writel(FRC_BASE + FRC_RAWCTRL, UINT32_MAX);

    g_assert_cmphex(readl(FRC_BASE + FRC_RAWCTRL), ==, 0x000021bf);
}

static void test_rail_initialization_latches(void)
{
    /* RAIL's RF initializer writes RXCTRL=0x60 and clears RAC.SR0 bit 6. */
    writel(FRC_BASE + FRC_RXCTRL, 0x60);
    writel(RAC_BASE + RAC_SR0, UINT32_MAX);
    writel(RAC_BASE + RAC_SR0 + REGISTER_CLEAR_WINDOW, 1u << 6);

    g_assert_cmphex(readl(FRC_BASE + FRC_RXCTRL), ==, 0x60);
    g_assert_cmphex(readl(RAC_BASE + RAC_SR0) & (1u << 6), ==, 0);
}

static void test_rac_sr3_clear_alias(void)
{
    uint32_t address = RAC_BASE + RAC_SR3;

    writel(address, UINT32_MAX);
    writel(address + REGISTER_CLEAR_WINDOW, 0x40000000);

    /* Stock clears this exact bit through RAC.SR3's CLR window. */
    g_assert_cmphex(readl(address), ==, 0xbfffffff);
}

static void test_frc_interrupt_register_masks(void)
{
    const uint32_t registers[] = {
        FRC_IF,
        FRC_IEN,
        FRC_SEQIF,
        FRC_SEQIEN,
    };

    for (size_t i = 0; i < G_N_ELEMENTS(registers); i++) {
        writel(FRC_BASE + registers[i], UINT32_MAX);
        g_assert_cmphex(readl(FRC_BASE + registers[i]), ==, 0x7f7fffff);
    }
}

static void test_frc_completion_interrupt_enable(void)
{
    const uint32_t completion = FRC_IF_TXDONE | FRC_IF_RXDONE;

    writel(FRC_BASE + FRC_IEN + REGISTER_CLEAR_WINDOW, completion);
    writel(FRC_BASE + FRC_IEN + REGISTER_SET_WINDOW, completion);

    g_assert_cmphex(readl(FRC_BASE + FRC_IEN) & completion, ==, completion);
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

static void test_stock_setup_aliases(void)
{
    /* Stock writes one at +0x004 through these two radio windows. */
    writel(BUFC_BASE + RADIO_NONSECURE_ALIAS + BUFC_EN, 1);
    writel(PROTIMER_NS_BASE + PROTIMER_EN, 1);

    g_assert_cmphex(readl(BUFC_BASE + BUFC_EN), ==, 1);
    g_assert_cmphex(readl(PROTIMER_BASE + PROTIMER_EN), ==, 1);
}

static void test_dcdc_regulation(void)
{
    qtest_qmp_assert_success(global_qtest, "{'execute':'system_reset'}");
    g_assert_cmphex(readl(DCDC_NONSECURE_BASE + DCDC_IF) & DCDC_IF_REGULATION,
                    ==, 0);

    writel(DCDC_NONSECURE_BASE + DCDC_CTRL + REGISTER_SET_WINDOW, 1);

    g_assert_cmphex(readl(DCDC_NONSECURE_BASE + DCDC_IF) & DCDC_IF_REGULATION,
                    ==,
                    DCDC_IF_REGULATION);
    g_assert_cmphex(readl(DCDC_BASE + DCDC_IF) & DCDC_IF_REGULATION, ==,
                    DCDC_IF_REGULATION);
    g_assert_cmphex(readl(DCDC_BASE + DCDC_STATUS), ==, 6);
}

static void test_emu_status_reset(void)
{
    g_assert_cmphex(readl(EMU_BASE + EMU_STATUS), ==, 0x80);
}

static void test_msc_status_reset(void)
{
    g_assert_cmphex(readl(MSC_BASE + MSC_STATUS), ==, 0x08000008);
}

static void test_hfxo_ready_status(void)
{
    qtest_qmp_assert_success(global_qtest, "{'execute':'system_reset'}");
    g_assert_cmphex(readl(CMU_BASE + CMU_SYSCLKCTRL) & 7, ==, 1);
    g_assert_cmphex(readl(HFXO_BASE + HFXO_CTRL), ==, HFXO_CTRL_DISONDEMAND);
    g_assert_cmphex(readl(HFXO_NONSECURE_BASE + HFXO_STATUS), ==, 0);

    writel(HFXO_NONSECURE_BASE + HFXO_CTRL + REGISTER_SET_WINDOW, 1);
    g_assert_cmphex(readl(HFXO_NONSECURE_BASE + HFXO_STATUS), ==,
                    HFXO_STATUS_READY | HFXO_STATUS_FSMLOCK);
    g_assert_cmphex(readl(HFXO_BASE + HFXO_STATUS), ==,
                    HFXO_STATUS_READY | HFXO_STATUS_FSMLOCK);

    writel(HFXO_NONSECURE_BASE + HFXO_CMD, 1u << 1);
    g_assert_cmphex(readl(HFXO_BASE + HFXO_STATUS), ==, HFXO_STATUS_READY);

    /* Re-enable without reading disabled STATUS: a fresh lock must start. */
    writel(HFXO_NONSECURE_BASE + HFXO_CTRL + REGISTER_CLEAR_WINDOW, 1);
    writel(HFXO_NONSECURE_BASE + HFXO_CTRL + REGISTER_SET_WINDOW, 1);
    g_assert_cmphex(readl(HFXO_BASE + HFXO_STATUS), ==,
                    HFXO_STATUS_READY | HFXO_STATUS_FSMLOCK);

    writel(HFXO_BASE + HFXO_CMD, 1u << 1);
    writel(HFXO_BASE + HFXO_CTRL + REGISTER_CLEAR_WINDOW, 1);
    writel(CMU_BASE + CMU_SYSCLKCTRL, 3);
    g_assert_cmphex(readl(HFXO_BASE + HFXO_STATUS), ==,
                    HFXO_STATUS_READY | HFXO_STATUS_FSMLOCK);

    qtest_qmp_assert_success(global_qtest, "{'execute':'system_reset'}");
    g_assert_cmphex(readl(HFXO_BASE + HFXO_CTRL), ==, HFXO_CTRL_DISONDEMAND);
    g_assert_cmphex(readl(HFXO_BASE + HFXO_STATUS), ==, 0);
}

static void test_bufc_data_port(void)
{
    uint32_t tx_channel = BUFC_BASE + BUFC_CH0;
    uint32_t sequencer_channel = BUFC_SECURE_BASE + BUFC_CH1;
    uint32_t nonsecure_rx_channel = BUFC_BASE + BUFC_CH1;
    const uint32_t rx_buffer = 0x20002000;

    writel(tx_channel + BUFC_WRITEDATA, 0x5a);
    writel(tx_channel + BUFC_WRITEDATA, 0xa5);
    writel(tx_channel + BUFC_READOFFSET, 0);

    g_assert_cmphex(readl(tx_channel + BUFC_READDATA), ==, 0x5a);
    g_assert_cmphex(readl(tx_channel + BUFC_READDATA), ==, 0xa5);

    /* RAIL permits TX FIFO contents to be preloaded in its backing RAM. */
    writel(tx_channel + 0x004, 0x20001000);
    qtest_writeb(global_qtest, 0x20001000, 0x6b);
    writel(tx_channel + BUFC_READOFFSET, 0);
    g_assert_cmphex(readl(tx_channel + BUFC_READDATA), ==, 0x6b);

    /* The recovered sequencer image writes RX buffer 1 via its secure alias. */
    writel(sequencer_channel + BUFC_CMD, 1);
    writel(sequencer_channel + 0x004, rx_buffer);
    writel(sequencer_channel + BUFC_WRITEDATA, 0x3c);
    writel(nonsecure_rx_channel + BUFC_READOFFSET, 0);
    g_assert_cmphex(readl(nonsecure_rx_channel + BUFC_READDATA), ==, 0x3c);
    g_assert_cmphex(qtest_readb(global_qtest, rx_buffer), ==, 0x3c);
    writel(sequencer_channel + BUFC_CMD, 1);
}

static void test_bufc_packet_channels_are_independent(void)
{
    const uint32_t tx_channel = BUFC_BASE + BUFC_TX_CHANNEL;
    const uint32_t rx_channel = BUFC_BASE + BUFC_RX_CHANNEL;
    const uint32_t tx_buffer = 0x20001000;
    const uint32_t rx_buffer = 0x20002000;

    /* RAIL's recovered FIFO helpers use channel 0 for TX and channel 1 for RX. */
    writel(tx_channel + BUFC_CMD, 1);
    writel(rx_channel + BUFC_CMD, 1);
    writel(tx_channel, 3);
    writel(rx_channel, 3);
    writel(tx_channel + 0x004, tx_buffer);
    writel(rx_channel + 0x004, rx_buffer);

    writel(tx_channel + BUFC_WRITEDATA, 0x31);
    writel(rx_channel + BUFC_WRITEDATA, 0x72);

    g_assert_cmphex(qtest_readb(global_qtest, tx_buffer), ==, 0x31);
    g_assert_cmphex(qtest_readb(global_qtest, rx_buffer), ==, 0x72);
    g_assert_cmphex(readl(tx_channel + BUFC_READDATA), ==, 0x31);
    g_assert_cmphex(readl(rx_channel + BUFC_READDATA), ==, 0x72);
}

static void test_bufc_tx_fifo_setup_latches(void)
{
    uint32_t channel = BUFC_BASE + BUFC_CH0;

    /* A vendor 512-byte FIFO encodes as log2(512) - 6 = 3. */
    writel(channel + BUFC_CMD, 1);
    writel(channel, 3);
    writel(channel + 0x004, 0x20001000);
    writel(channel + BUFC_WRITEOFFSET, 18);
    writel(channel + BUFC_CMD, 2);

    g_assert_cmphex(readl(channel), ==, 3);
    g_assert_cmphex(readl(channel + 0x004), ==, 0x20001000);
    g_assert_cmphex(readl(channel + BUFC_CMD), ==, 2);
    g_assert_cmphex(readl(channel + BUFC_STATUS), ==, 18 | (1u << 20));
}

static void test_bufc_backing_memory_wrap(void)
{
    const uint32_t channel = BUFC_BASE + BUFC_CH0;
    const uint32_t buffer = 0x20001000;

    writel(channel + BUFC_CMD, 1);
    writel(channel, 3); /* 512-byte FIFO */
    writel(channel + 0x004, buffer);
    writel(channel + BUFC_WRITEOFFSET, 511);
    writel(channel + BUFC_WRITEDATA, 0x31);
    writel(channel + BUFC_WRITEDATA, 0x32);

    g_assert_cmphex(qtest_readb(global_qtest, buffer + 511), ==, 0x31);
    g_assert_cmphex(qtest_readb(global_qtest, buffer), ==, 0x32);
    g_assert_cmphex(readl(channel + BUFC_WRITEOFFSET), ==, 513);
}

static void test_bufc_overflow_latch(void)
{
    const uint32_t channel = BUFC_BASE + BUFC_CH0;

    writel(channel + BUFC_CMD, 1);
    writel(channel, 0); /* 64-byte FIFO */
    for (unsigned i = 0; i <= 64; i++) {
        writel(channel + BUFC_WRITEDATA, i);
    }

    g_assert_cmphex(readl(BUFC_BASE + BUFC_IF) & 1, ==, 1);
}

static void test_bufc_rx_fifo_setup_latches(void)
{
    uint32_t channel = BUFC_BASE + BUFC_CH1;

    /* RAIL configures the RX FIFO as a 512-byte channel-1 buffer. */
    writel(channel + BUFC_CMD, 1);
    writel(channel, 3);
    writel(channel + 0x004, 0x20002000);
    writel(channel + BUFC_THRESHOLD, 0x0fff);

    g_assert_cmphex(readl(channel), ==, 3);
    g_assert_cmphex(readl(channel + 0x004), ==, 0x20002000);
    g_assert_cmphex(readl(channel + BUFC_THRESHOLD), ==, 0x0fff);
    g_assert_cmphex(readl(channel + BUFC_CMD), ==, 1);
}

static void test_bufc_rx_descriptor_ring_setup_latches(void)
{
    uint32_t channel = BUFC_BASE + BUFC_CH2;

    /*
     * RAIL configures BUFC channel 2 as its eight-byte RX descriptor ring:
     * CMD=1, capacity encoding, backing address, and threshold=7.  This is
     * distinct from the channel-1 sequencer payload-byte FIFO.
     */
    writel(BUFC_BASE + BUFC_IF, 0x00050000);
    writel(channel + BUFC_CMD, 1);
    writel(channel, 3);
    writel(channel + 0x004, 0x20002400);
    writel(channel + BUFC_THRESHOLD, 7);
    writel(BUFC_BASE + BUFC_IF + REGISTER_CLEAR_WINDOW, 0x00050000);

    g_assert_cmphex(readl(channel), ==, 3);
    g_assert_cmphex(readl(channel + 0x004), ==, 0x20002400);
    g_assert_cmphex(readl(channel + BUFC_THRESHOLD), ==, 7);
    g_assert_cmphex(readl(channel + BUFC_CMD), ==, 1);
    g_assert_cmphex(readl(BUFC_BASE + BUFC_IF) & 0x00050000, ==, 0);
}

static void test_bufc_interrupt_register_masks(void)
{
    writel(BUFC_BASE + BUFC_IF, UINT32_MAX);
    writel(BUFC_BASE + BUFC_IEN, UINT32_MAX);

    g_assert_cmphex(readl(BUFC_BASE + BUFC_IF), ==, 0x8f0f0f0f);
    g_assert_cmphex(readl(BUFC_BASE + BUFC_IEN), ==, 0x8f0f0f0f);
}

static void test_bufc_channel_register_masks(void)
{
    uint32_t channel = BUFC_BASE + BUFC_CH2;

    writel(channel + BUFC_CMD, UINT32_MAX);
    writel(channel, UINT32_MAX);
    writel(channel + BUFC_WRITEOFFSET, UINT32_MAX);
    writel(channel + BUFC_READOFFSET, UINT32_MAX);
    writel(channel + BUFC_THRESHOLD, UINT32_MAX);

    g_assert_cmphex(readl(channel), ==, 0x7);
    g_assert_cmphex(readl(channel + BUFC_WRITEOFFSET), ==, 0x1fff);
    g_assert_cmphex(readl(channel + BUFC_READOFFSET), ==, 0x1fff);
    g_assert_cmphex(readl(channel + BUFC_THRESHOLD), ==, 0x2fff);
    g_assert_cmphex(readl(channel + BUFC_CMD), ==, 0xf);
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

static void test_protimer_secure_nonsecure_alias(void)
{
    /* RAIL enables PROTIMER through the non-secure 0xb801c000 window. */
    writel(PROTIMER_NS_BASE + PROTIMER_CTRL, 0x00011102);
    writel(PROTIMER_NS_BASE + PROTIMER_CMD, 1);

    g_assert_cmphex(readl(PROTIMER_BASE + PROTIMER_CTRL), ==, 0x00011102);
    g_assert_cmphex(readl(PROTIMER_NS_BASE + PROTIMER_CTRL), ==, 0x00011102);
}

/*
 * generic_phy.o:FRC_IRQHandler reads PROTIMER.IF (+0x064) and IEN (+0x070)
 * through the non-secure window.  Keep those banks as aliasable latches; a
 * counter or compare match deliberately does not manufacture a pending bit.
 */
static void test_protimer_interrupt_banks_are_aliasable(void)
{
    writel(PROTIMER_NS_BASE + PROTIMER_IF, 0x02000010);
    writel(PROTIMER_BASE + PROTIMER_IEN + REGISTER_SET_WINDOW, 0x00000010);

    g_assert_cmphex(readl(PROTIMER_BASE + PROTIMER_IF), ==, 0x02000010);
    g_assert_cmphex(readl(PROTIMER_NS_BASE + PROTIMER_IEN), ==, 0x00000010);

    writel(PROTIMER_NS_BASE + PROTIMER_IF + REGISTER_CLEAR_WINDOW, 0x10);
    writel(PROTIMER_BASE + PROTIMER_IEN + REGISTER_TOGGLE_WINDOW, 0x10);

    g_assert_cmphex(readl(PROTIMER_NS_BASE + PROTIMER_IF), ==, 0x02000000);
    g_assert_cmphex(readl(PROTIMER_BASE + PROTIMER_IEN), ==, 0);
}

/*
 * rfhal_protimer.o:0x3c140 writes 0x11 to the first indexed slot and, on
 * one branch, clears a bit through 0xb801e100. This only preserves the
 * observed normal/CLR alias access; it does not assign compare semantics.
 */
static void test_protimer_compare_slot_clear_alias(void)
{
    writel(PROTIMER_NS_BASE + PROTIMER_CMP0, 0x11);
    writel(PROTIMER_NS_BASE + PROTIMER_CMP0_TIME, 0x12345678);
    writel(PROTIMER_NS_BASE + PROTIMER_CMP0 + REGISTER_CLEAR_WINDOW, 1);

    g_assert_cmphex(readl(PROTIMER_BASE + PROTIMER_CMP0), ==, 0x10);
    g_assert_cmphex(readl(PROTIMER_BASE + PROTIMER_CMP0_TIME), ==, 0x12345678);

    writel(PROTIMER_BASE + PROTIMER_CMP0 + REGISTER_CLEAR_WINDOW, 0x10);
    g_assert_cmphex(readl(PROTIMER_NS_BASE + PROTIMER_CMP0), ==, 0);
}

static void test_protimer_counter(void)
{
    uint32_t before = readl(PROTIMER_BASE + PROTIMER_COUNTER);

    g_assert_cmphex(readl(PROTIMER_NS_BASE + PROTIMER_COUNTER), ==, before);
    writel(PROTIMER_BASE + PROTIMER_COUNTER, UINT32_MAX);
    g_assert_cmphex(readl(PROTIMER_BASE + PROTIMER_COUNTER), ==, before);

    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    /* Step past the second 500 ns edge so QEMU dispatches both ptimer ticks. */
    clock_step(1001);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");
    g_assert_cmphex(readl(PROTIMER_BASE + PROTIMER_COUNTER), ==, before + 2);
    g_assert_cmphex(readl(PROTIMER_NS_BASE + PROTIMER_COUNTER), ==, before + 2);

    qtest_qmp_assert_success(global_qtest, "{'execute':'system_reset'}");
    g_assert_cmphex(readl(PROTIMER_BASE + PROTIMER_COUNTER), ==, 0);
}

static void test_protimer_counter_is_independent_from_radio_setup(void)
{
    qtest_qmp_assert_success(global_qtest, "{'execute':'system_reset'}");

    /*
     * Stock setup derives state from these inputs, but does not establish
     * them as the RAIL_GetTime counter's source or divisor. Keep the model's
     * documented 2 MHz clock independent until that relationship is known.
     */
    writel(CMU_BASE + CMU_SYSCLKCTRL, 3);
    writel(RADIO_RAM_NS_BASE + RADIO_RAM_RHCLKPRESC, 1u << 16);
    writel(PROTIMER_BASE + PROTIMER_CTRL, 0x00011102);

    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(1001);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");
    g_assert_cmphex(readl(PROTIMER_BASE + PROTIMER_COUNTER), ==, 2);
}

static void test_rac_tx_commands(void)
{
    writel(RAC_BASE + RAC_CMD, RAC_CMD_TXEN);
    g_assert_cmphex(readl(RAC_BASE + RAC_STATUS) & RAC_STATUS_STATE_MASK, ==,
                    RAC_STATUS_STATE_TX);
    g_assert_cmphex(readl(RAC_BASE + RAC_STATUS) &
                        (RAC_STATUS_TXENS | RAC_STATUS_RXENS),
                    ==, RAC_STATUS_TXENS);
    g_assert_cmphex(readl(RAC_BASE + RAC_SEQIF) & RAC_SEQIF_STATETX, ==,
                    RAC_SEQIF_STATETX);

    writel(RAC_BASE + RAC_CMD, RAC_CMD_TXDIS);
    g_assert_cmphex(readl(RAC_BASE + RAC_STATUS) & RAC_STATUS_STATE_MASK, ==,
                    0);
    g_assert_cmphex(readl(RAC_BASE + RAC_SEQIF) & RAC_SEQIF_STATEOFF, ==,
                    RAC_SEQIF_STATEOFF);
}

static void test_rac_tx_completion(void)
{
    writel(FRC_BASE + FRC_IF + REGISTER_CLEAR_WINDOW, FRC_IF_TXDONE);
    writel(RAC_BASE + RAC_CMD, RAC_CMD_TXEN);
    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(150001);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");

    g_assert_cmphex(readl(RAC_BASE + RAC_STATUS) & RAC_STATUS_STATE_MASK, ==,
                    0);
    g_assert_cmphex(readl(FRC_BASE + FRC_IF) & FRC_IF_TXDONE, ==,
                    FRC_IF_TXDONE);
}

/* TXDIS cancels QEMU's synthetic completion; it is not a hardware timing claim. */
static void test_rac_tx_disable_cancels_completion(void)
{
    writel(FRC_BASE + FRC_IF + REGISTER_CLEAR_WINDOW, FRC_IF_TXDONE);
    writel(RAC_BASE + RAC_CMD, RAC_CMD_TXEN);
    g_assert_cmphex(readl(RAC_BASE + RAC_STATUS) & RAC_STATUS_STATE_MASK, ==,
                    RAC_STATUS_STATE_TX);

    writel(RAC_BASE + RAC_CMD, RAC_CMD_TXDIS);
    g_assert_cmphex(readl(RAC_BASE + RAC_STATUS) & RAC_STATUS_STATE_MASK, ==,
                    0);
    g_assert_cmphex(readl(RAC_BASE + RAC_SEQIF) & RAC_SEQIF_STATEOFF, ==,
                    RAC_SEQIF_STATEOFF);

    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(150001);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");
    g_assert_cmphex(readl(FRC_BASE + FRC_IF) & FRC_IF_TXDONE, ==, 0);
}

/*
 * RX supersedes QEMU's synthetic TX operation, so it cannot later raise
 * TXDONE.
 */
static void test_rac_rx_enable_cancels_tx_completion(void)
{
    writel(FRC_BASE + FRC_IF + REGISTER_CLEAR_WINDOW, FRC_IF_TXDONE);
    writel(RAC_BASE + RAC_CMD, RAC_CMD_TXEN);
    g_assert_cmphex(readl(RAC_BASE + RAC_STATUS) & RAC_STATUS_STATE_MASK, ==,
                    RAC_STATUS_STATE_TX);

    writel(RAC_BASE + RAC_RXENSRCEN + REGISTER_SET_WINDOW,
           RAC_RXENSRCEN_SWRXEN);
    g_assert_cmphex(readl(RAC_BASE + RAC_STATUS) & RAC_STATUS_STATE_MASK, ==,
                    RAC_STATUS_STATE_RXSEARCH);

    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(150001);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");

    g_assert_cmphex(readl(RAC_BASE + RAC_STATUS) & RAC_STATUS_STATE_MASK, ==,
                    RAC_STATUS_STATE_RXSEARCH);
    g_assert_cmphex(readl(FRC_BASE + FRC_IF) & FRC_IF_TXDONE, ==, 0);
}

static void test_system_reset_cancels_tx_completion(void)
{
    writel(FRC_BASE + FRC_IF + REGISTER_CLEAR_WINDOW, FRC_IF_TXDONE);
    writel(RAC_BASE + RAC_CMD, RAC_CMD_TXEN);
    g_assert_cmphex(readl(RAC_BASE + RAC_STATUS) & RAC_STATUS_STATE_MASK, ==,
                    RAC_STATUS_STATE_TX);

    qtest_qmp_assert_success(global_qtest, "{'execute':'system_reset'}");
    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(150001);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");

    g_assert_cmphex(readl(RAC_BASE + RAC_STATUS) & RAC_STATUS_STATE_MASK, ==,
                    0);
    g_assert_cmphex(readl(FRC_BASE + FRC_IF) & FRC_IF_TXDONE, ==, 0);
}

static void test_rac_tx_snapshot_captures_staged_bufc(void)
{
    const uint32_t tx_channel = BUFC_BASE + BUFC_CH0;
    gchar *trace;

    writel(tx_channel + BUFC_CMD, 1);
    writel(tx_channel + BUFC_WRITEDATA, 0x91);
    writel(tx_channel + BUFC_WRITEDATA, 0x72);
    writel(RAC_BASE + RAC_CMD, RAC_CMD_TXEN);

    /* This later staging write must not change the first TXEN snapshot. */
    writel(tx_channel + BUFC_WRITEDATA, 0xe3);
    writel(RAC_BASE + RAC_CMD, RAC_CMD_TXDIS);
    writel(RAC_BASE + RAC_CMD, RAC_CMD_TXEN);

    g_assert(g_file_get_contents(tx_snapshot_trace_path, &trace, NULL, NULL));
    g_assert_nonnull(g_strstr_len(trace, -1,
                                  "bytes=2 first=0x00007291"));
    g_assert_nonnull(g_strstr_len(trace, -1,
                                  "bytes=3 first=0x00e37291"));
    g_free(trace);
}

static void test_rac_tx_snapshot_captures_guest_backed_fifo(void)
{
    const uint32_t tx_channel = BUFC_BASE + BUFC_CH0;
    const uint32_t tx_buffer = 0x20001000;
    gchar *trace;

    /* Match the RAIL SetTxFifo staging shape used by the Zephyr HAL. */
    writel(tx_channel + BUFC_CMD, 1);
    writel(tx_channel, 3);
    writel(tx_channel + 0x004, tx_buffer);
    qtest_writeb(global_qtest, tx_buffer, 0x8d);
    qtest_writeb(global_qtest, tx_buffer + 1, 0x5b);
    qtest_writeb(global_qtest, tx_buffer + 2, 0xe1);
    writel(tx_channel + BUFC_WRITEOFFSET, 3);
    writel(tx_channel + BUFC_CMD, 2);
    writel(RAC_BASE + RAC_CMD, RAC_CMD_TXEN);

    g_assert(g_file_get_contents(tx_snapshot_trace_path, &trace, NULL, NULL));
    g_assert_nonnull(g_strstr_len(trace, -1,
                                  "bytes=3 first=0x00e15b8d"));
    g_free(trace);
}

static void test_rac_tx_snapshot_wraps_guest_backed_fifo(void)
{
    const uint32_t tx_channel = BUFC_BASE + BUFC_CH0;
    const uint32_t tx_buffer = 0x20001000;
    gchar *trace;

    writel(tx_channel + BUFC_CMD, 1);
    writel(tx_channel, 3); /* 512-byte circular FIFO */
    writel(tx_channel + 0x004, tx_buffer);
    qtest_writeb(global_qtest, tx_buffer + 511, 0x2a);
    qtest_writeb(global_qtest, tx_buffer, 0x4c);
    qtest_writeb(global_qtest, tx_buffer + 1, 0x7e);
    writel(tx_channel + BUFC_READOFFSET, 511);
    writel(tx_channel + BUFC_WRITEOFFSET, 514);
    writel(tx_channel + BUFC_CMD, 2);
    writel(RAC_BASE + RAC_CMD, RAC_CMD_TXEN);

    g_assert(g_file_get_contents(tx_snapshot_trace_path, &trace, NULL, NULL));
    g_assert_nonnull(g_strstr_len(trace, -1,
                                  "bytes=3 first=0x007e4c2a"));
    g_free(trace);
}

static void test_rac_tx_snapshot_has_no_local_offers(void)
{
    const uint32_t tx_channel = BUFC_BASE + BUFC_CH0;
    gchar *trace;

    writel(tx_channel + BUFC_CMD, 1);
    writel(tx_channel + BUFC_WRITEDATA, 0x4d);
    writel(RAC_BASE + RAC_CMD, RAC_CMD_TXEN);

    g_assert(g_file_get_contents(tx_snapshot_trace_path, &trace, NULL, NULL));
    g_assert_nonnull(g_strstr_len(trace, -1, "RX mailbox offers=0"));
    g_free(trace);
}

static void test_rac_software_rx_enable(void)
{
    writel(RAC_BASE + RAC_RXENSRCEN + REGISTER_SET_WINDOW,
           RAC_RXENSRCEN_SWRXEN);
    g_assert_cmphex(readl(RAC_BASE + RAC_STATUS) & RAC_STATUS_STATE_MASK, ==,
                    RAC_STATUS_STATE_RXSEARCH);
    g_assert_cmphex(readl(RAC_BASE + RAC_STATUS) &
                        (RAC_STATUS_TXENS | RAC_STATUS_RXENS),
                    ==, RAC_STATUS_RXENS);
    g_assert_cmphex(readl(RAC_BASE + RAC_SEQIF) & RAC_SEQIF_STATERXSEARCH, ==,
                    RAC_SEQIF_STATERXSEARCH);

    writel(RAC_BASE + RAC_CMD, RAC_CMD_RXDIS);
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

/* RAIL_StartTx clears RAC.SR0 bit 5 through the Series-2 CLR alias. */
static void test_rac_sr0_clear_alias(void)
{
    writel(RAC_BASE + RAC_SR0, UINT32_MAX);
    writel(RAC_BASE + RAC_SR0 + REGISTER_CLEAR_WINDOW, 1u << 5);

    g_assert_cmphex(readl(RAC_BASE + RAC_SR0), ==, UINT32_MAX & ~(1u << 5));
}

static void test_rac_force_disable(void)
{
    writel(FRC_BASE + FRC_IF + REGISTER_CLEAR_WINDOW, FRC_IF_TXDONE);
    writel(RAC_BASE + RAC_CMD, RAC_CMD_TXEN);
    g_assert_cmphex(readl(RAC_BASE + RAC_STATUS) & RAC_STATUS_STATE_MASK,
                    ==, RAC_STATUS_STATE_TX);

    writel(RAC_BASE + RAC_CTRL, RAC_CTRL_FORCEDISABLE);
    g_assert_cmphex(readl(RAC_BASE + RAC_CTRL), ==, RAC_CTRL_FORCEDISABLE);
    g_assert_cmphex(readl(RAC_BASE + RAC_STATUS) & RAC_STATUS_STATE_MASK,
                    ==, 0);

    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(150001);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");
    g_assert_cmphex(readl(FRC_BASE + FRC_IF) & FRC_IF_TXDONE, ==, 0);

    writel(RAC_BASE + RAC_CTRL, 0);
    g_assert_cmphex(readl(RAC_BASE + RAC_CTRL), ==, 0);
}

int main(int argc, char **argv)
{
    int result;
    gchar *qemu_args;

    g_test_init(&argc, &argv, NULL);
    tx_snapshot_trace_path = g_strdup_printf(
        "%s/efr32bg22-tx-snapshot-%u.trace", g_get_tmp_dir(), g_random_int());
    qemu_args = g_strdup_printf(
        "-machine efr32bg22 -S -trace enable=efr32bg22_*,file=%s",
        tx_snapshot_trace_path);
    qtest_start(qemu_args);
    g_free(qemu_args);

    qtest_add_func("/efr32bg22/register-aliases", test_register_aliases);
    qtest_add_func("/efr32bg22/burtc-counter-contract",
                   test_burtc_counter_contract);
    qtest_add_func("/efr32bg22/burtc-compare-pending",
                   test_burtc_compare_pending);
    qtest_add_func("/efr32bg22/cmu-nonsecure-alias", test_cmu_nonsecure_alias);
    qtest_add_func("/efr32bg22/dpll-lock", test_dpll_lock);
    qtest_add_func("/efr32bg22/usart-tx-polling", test_usart_tx_polling);
    qtest_add_func("/efr32bg22/modem-masks", test_modem_masks);
    qtest_add_func("/efr32bg22/rfcrc-masks", test_rfcrc_masks);
    qtest_add_func("/efr32bg22/agc-rssi-status-registers",
                   test_agc_rssi_status_registers);
    qtest_add_func("/efr32bg22/agc-rssi-valid-latches",
                   test_agc_rssi_valid_latches);
    qtest_add_func("/efr32bg22/agc-rssi-start-command",
                   test_agc_rssi_start_command);
    qtest_add_func("/efr32bg22/frc-buffer-register-masks",
                   test_frc_buffer_register_masks);
    qtest_add_func("/efr32bg22/frc-sniffctrl-mask", test_frc_sniffctrl_mask);
    qtest_add_func("/efr32bg22/frc-rawctrl-mask", test_frc_rawctrl_mask);
    qtest_add_func("/efr32bg22/rail-initialization-latches",
                   test_rail_initialization_latches);
    qtest_add_func("/efr32bg22/rac-sr3-clear-alias", test_rac_sr3_clear_alias);
    qtest_add_func("/efr32bg22/frc-interrupt-register-masks",
                   test_frc_interrupt_register_masks);
    qtest_add_func("/efr32bg22/frc-completion-interrupt-enable",
                   test_frc_completion_interrupt_enable);
    qtest_add_func("/efr32bg22/radio-enable-masks", test_radio_enable_masks);
    qtest_add_func("/efr32bg22/radio-nonsecure-alias",
                   test_radio_nonsecure_alias);
    qtest_add_func("/efr32bg22/stock-setup-aliases",
                   test_stock_setup_aliases);
    qtest_add_func("/efr32bg22/dcdc-regulation", test_dcdc_regulation);
    qtest_add_func("/efr32bg22/emu-status-reset", test_emu_status_reset);
    qtest_add_func("/efr32bg22/msc-status-reset", test_msc_status_reset);
    qtest_add_func("/efr32bg22/hfxo-ready-status", test_hfxo_ready_status);
    qtest_add_func("/efr32bg22/bufc-data-port", test_bufc_data_port);
    qtest_add_func("/efr32bg22/bufc-packet-channels-are-independent",
                   test_bufc_packet_channels_are_independent);
    qtest_add_func("/efr32bg22/bufc-tx-fifo-setup-latches",
                   test_bufc_tx_fifo_setup_latches);
    qtest_add_func("/efr32bg22/bufc-backing-memory-wrap",
                   test_bufc_backing_memory_wrap);
    qtest_add_func("/efr32bg22/bufc-overflow-latch",
                   test_bufc_overflow_latch);
    qtest_add_func("/efr32bg22/bufc-rx-fifo-setup-latches",
                   test_bufc_rx_fifo_setup_latches);
    qtest_add_func("/efr32bg22/bufc-rx-descriptor-ring-setup-latches",
                   test_bufc_rx_descriptor_ring_setup_latches);
    qtest_add_func("/efr32bg22/bufc-interrupt-register-masks",
                   test_bufc_interrupt_register_masks);
    qtest_add_func("/efr32bg22/bufc-channel-register-masks",
                   test_bufc_channel_register_masks);
    qtest_add_func("/efr32bg22/bufc-sparse-threshold-flag",
                   test_bufc_sparse_threshold_flag);
    qtest_add_func("/efr32bg22/protimer-trace-latches",
                   test_protimer_trace_latches);
    qtest_add_func("/efr32bg22/protimer-secure-nonsecure-alias",
                   test_protimer_secure_nonsecure_alias);
    qtest_add_func("/efr32bg22/protimer-interrupt-banks-are-aliasable",
                   test_protimer_interrupt_banks_are_aliasable);
    qtest_add_func("/efr32bg22/protimer-compare-slot-clear-alias",
                   test_protimer_compare_slot_clear_alias);
    qtest_add_func("/efr32bg22/protimer-counter", test_protimer_counter);
    qtest_add_func("/efr32bg22/protimer-counter-is-independent-from-radio-setup",
                   test_protimer_counter_is_independent_from_radio_setup);
    qtest_add_func("/efr32bg22/rac-tx-commands", test_rac_tx_commands);
    qtest_add_func("/efr32bg22/rac-tx-completion", test_rac_tx_completion);
    qtest_add_func("/efr32bg22/rac-tx-disable-cancels-completion",
                   test_rac_tx_disable_cancels_completion);
    qtest_add_func("/efr32bg22/rac-rx-enable-cancels-tx-completion",
                   test_rac_rx_enable_cancels_tx_completion);
    qtest_add_func("/efr32bg22/system-reset-cancels-tx-completion",
                   test_system_reset_cancels_tx_completion);
    qtest_add_func("/efr32bg22/rac-tx-snapshot-captures-staged-bufc",
                   test_rac_tx_snapshot_captures_staged_bufc);
    qtest_add_func("/efr32bg22/rac-tx-snapshot-captures-guest-backed-fifo",
                   test_rac_tx_snapshot_captures_guest_backed_fifo);
    qtest_add_func("/efr32bg22/rac-tx-snapshot-wraps-guest-backed-fifo",
                   test_rac_tx_snapshot_wraps_guest_backed_fifo);
    qtest_add_func("/efr32bg22/rac-tx-snapshot-has-no-local-offers",
                   test_rac_tx_snapshot_has_no_local_offers);
    qtest_add_func("/efr32bg22/rac-software-rx-enable",
                   test_rac_software_rx_enable);
    qtest_add_func("/efr32bg22/rac-masks", test_rac_masks);
    qtest_add_func("/efr32bg22/rac-sr0-clear-alias",
                   test_rac_sr0_clear_alias);
    qtest_add_func("/efr32bg22/rac-force-disable", test_rac_force_disable);

    result = g_test_run();
    qtest_end();
    g_unlink(tx_snapshot_trace_path);
    g_free(tx_snapshot_trace_path);
    return result;
}
