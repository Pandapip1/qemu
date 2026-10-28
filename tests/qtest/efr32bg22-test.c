/*
 * QTest coverage for the EFR32BG22 native peripheral model.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "libqtest-single.h"

static char *tx_snapshot_trace_path;

#define FRC_BASE             0xa8004000
#define RADIO_IPVERSION      0x000
#define FRC_DATABUFFER       0x018
#define FRC_RXRAWDATA        0x090
#define FRC_STATUS           0x008
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
#define FRC_CMD              0x02c
#define FRC_CMD_RXABORT      (1u << 0)
#define FRC_CMD_CONVINIT     (1u << 9)
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
#define RAC_EN                0x004
#define RAC_STATUS            0x00c
#define RAC_CMD               0x010
#define RAC_CTRL              0x014
#define RAC_SR0               0x040
#define RAC_SR3               0x04c
#define RAC_STCTRL            0x050
/* STCAL[23:0] | STSKEW[24] -- the only two documented fields (RAC_S_field.py),
 * confirmed structurally identical in layout to Armv6-M SysTick.CALIB's own
 * TENMS[23:0]/SKEW[24] (see owo-open-fw docs/radio-tables.md SS7.9). */
#define RAC_STCTRL_WRITABLE_MASK 0x01ffffffu
/* The exact literal stock's sequencer reset handler writes to RAC.STCTRL,
 * independently measured live on stock hardware (docs/radio-tables.md
 * SS7.8/SS7.9). Not a derived or invented value. */
#define RAC_STCTRL_STOCK_VALUE   0x01061a7fu
#define RAC_RXENSRCEN         0x008
#define RAC_IEN               0x020
#define RAC_SEQIF             0x028
#define RAC_CMD_TXEN          (1u << 0)
#define RAC_CMD_TXDIS         (1u << 5)
#define RAC_CMD_RXDIS         (1u << 8)
#define RAC_CTRL_FORCEDISABLE (1u << 0)
#define RAC_CTRL_SEQRESET     (1u << 24)
#define RAC_CTRL_CPUWAITDIS   (1u << 26)
#define RAC_CTRL_ACTIVE_HIGH  (1u << 7)
#define RAC_CTRL_PAEN_HIGH    (1u << 8)
#define RAC_CTRL_LNAEN_HIGH   (1u << 9)
#define RAC_CTRL_BOARD_POLARITIES \
    (RAC_CTRL_ACTIVE_HIGH | RAC_CTRL_PAEN_HIGH | RAC_CTRL_LNAEN_HIGH)
#define THUMB16_LDR_R0_PC_8      0x4802u
#define THUMB16_LDR_R1_R0        0x6801u
#define THUMB16_ADDS_R1_1        0x3101u
#define THUMB16_STR_R1_R0        0x6001u
#define THUMB16_BRANCH_SELF      0xe7feu
#define THUMB16_PAIR(first, second) \
    ((uint32_t)(first) | ((uint32_t)(second) << 16))

/*
 * Small Thumb-16 (ARMv6-M) instruction encoders used to assemble one-off
 * probe snippets for the AP3 sequencer core at runtime, rather than hand-
 * computing and transcribing magic opcode words. Each encoder is named
 * after, and implements exactly, the architected encoding in the Armv6-M
 * ARM ARM for that mnemonic; none of this invents or approximates behavior.
 */
static uint16_t thumb_ldr_pc(unsigned rd, uint32_t insn_addr,
                             uint32_t literal_addr)
{
    uint32_t pc = (insn_addr + 4) & ~3u;
    uint32_t imm8;

    g_assert_cmpuint(literal_addr, >=, pc);
    g_assert_cmpuint((literal_addr - pc) % 4, ==, 0);
    imm8 = (literal_addr - pc) / 4;
    g_assert_cmpuint(imm8, <=, 0xff);
    return 0x4800u | (rd << 8) | imm8;
}

static uint16_t thumb_movs_imm8(unsigned rd, uint8_t imm8)
{
    return 0x2000u | (rd << 8) | imm8;
}

static uint16_t thumb_strb_imm(unsigned rt, unsigned rn, unsigned imm5)
{
    return 0x7000u | (imm5 << 6) | (rn << 3) | rt;
}

static uint16_t thumb_ldrb_imm(unsigned rt, unsigned rn, unsigned imm5)
{
    return 0x7800u | (imm5 << 6) | (rn << 3) | rt;
}

static uint16_t thumb_str_imm(unsigned rt, unsigned rn, unsigned imm5_words)
{
    return 0x6000u | (imm5_words << 6) | (rn << 3) | rt;
}

static uint16_t thumb_ldr_imm(unsigned rt, unsigned rn, unsigned imm5_words)
{
    return 0x6800u | (imm5_words << 6) | (rn << 3) | rt;
}

static uint16_t thumb_b(uint32_t insn_addr, uint32_t target_addr)
{
    int32_t offset = (int32_t)(target_addr - (insn_addr + 4));

    g_assert_cmpint(offset, >=, -2048);
    g_assert_cmpint(offset, <=, 2046);
    g_assert_true(offset % 2 == 0);
    return 0xe000u | (((uint32_t)offset >> 1) & 0x7ffu);
}
#define RAC_RXENSRCEN_SWRXEN  (1u << 1)
#define RAC_STATUS_STATE_MASK 0x0f000000
#define RAC_STATUS_STATE_RXSEARCH (2u << 24)
#define RAC_STATUS_STATE_TXWARM (8u << 24)
#define RAC_STATUS_STATE_TX   (9u << 24)
#define RAC_STATUS_STATE_SHUTDOWN (13u << 24)
#define RAC_STATUS_TXENS      (1u << 30)
#define RAC_STATUS_RXENS      (1u << 31)
#define RAC_SEQIF_STATEOFF    (1u << 16)
#define RAC_SEQIF_STATECHANGESEQ (1u << 0)
#define RAC_SEQIF_STATERXSEARCH (1u << 18)
#define RAC_SEQIF_STATETX     (1u << 25)
#define SEQRAM_NS_BASE         0xb0000000
#define SEQ_SPARE0             0x4fe0
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
#define CMU_CLKEN1            0x068
#define CMU_RADIOCLKCTRL      0x280
#define CMU_RADIOCLKCTRL_EN   (1u << 0)
#define CMU_CLKEN1_RADIO_BLOCKS 0x87fu
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
#define BURTC_NONSECURE_BASE   0x50064000
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
#define TIMER4_BASE             0x40058000
#define TIMER4_NS_BASE          0x50058000
#define TIMER2_BASE             0x40050000
#define TIMER2_NS_BASE          0x50050000
#define TIMER_EN                0x030
#define TIMER_CMD               0x00c
#define TIMER_STATUS            0x010
#define TIMER_IF                0x014
#define TIMER_IEN               0x018
#define TIMER_TOP               0x01c
#define TIMER_CNT               0x024
#define TIMER_CC0_OC            0x068
#define TIMER_EN_EN             (1u << 0)
#define TIMER_CMD_START          (1u << 0)
#define TIMER_CMD_STOP           (1u << 1)
#define TIMER_STATUS_RUNNING     (1u << 0)
#define TIMER_IF_CC0             (1u << 4)
#define TIMER0_BASE             0x40048000
#define TIMER0_NS_BASE          0x50048000
#define TIMER_CFG               0x004
#define TIMER_CFG_PRESC_SHIFT   18
#define TIMER_CC_CFG(n)         (0x060 + 0x20 * (n))
#define TIMER_CC_OC(n)          (0x068 + 0x20 * (n))
#define TIMER_CC_CFG_OUTPUTCOMPARE 0x2
#define TIMER_IF_OF             (1u << 0)
#define TIMER_IF_CC(n)          (1u << (4 + (n)))
#define SMU_BASE                0x44008000
#define SMU_PPUSATD1            0x064
#define SMU_PPUSATD1_AHBRADIO   (1u << 15)
#define CMU_BASE_ADDR           0x40008000
#define CMU_DPLLREFCLKCTRL      0x100
#define CMU_EM01GRPACLKCTRL     0x120
#define CMU_EM01GRPACLK_HFXO    2
#define DPLL0_BASE_ADDR         0x4001c000
#define DPLL_EN                 0x004
#define DPLL_CFG1               0x00c
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

#define RFMAILBOX_B_BASE      0xa8028000
#define RFMAILBOX_C_BASE      0xa802c000
#define RFMAILBOX_NS_OFFSET   0x10000000
#define RFMAILBOX_DATA        0x004
#define RFMAILBOX_FLAGS       0x040

#define BUFC_SECURE_BASE     0xaa000000
#define RADIO_NONSECURE_ALIAS 0x10000000

#define REGISTER_SET_WINDOW  0x1000
#define REGISTER_CLEAR_WINDOW 0x2000

static void attribute_radio_nonsecure(void)
{
    writel(SMU_BASE + SMU_PPUSATD1 + REGISTER_CLEAR_WINDOW,
           SMU_PPUSATD1_AHBRADIO);
}

static void attribute_radio_secure(void)
{
    writel(SMU_BASE + SMU_PPUSATD1 + REGISTER_SET_WINDOW,
           SMU_PPUSATD1_AHBRADIO);
}
#define REGISTER_TOGGLE_WINDOW 0x3000
#define DEVINFO_BASE          0x0fe00000
#define DEVINFO_MODULENAME0   0x8130
#define DEVINFO_MODULEINFO    0x814c

static void enable_radio_bus_clocks(void)
{
    writel(CMU_BASE + CMU_RADIOCLKCTRL + REGISTER_SET_WINDOW,
           CMU_RADIOCLKCTRL_EN);
    writel(CMU_BASE + CMU_CLKEN1 + REGISTER_SET_WINDOW,
           CMU_CLKEN1_RADIO_BLOCKS);
}

static void test_bgm220_module_identity(void)
{
    const uint32_t module_name[] = {
        0x324d4742, 0x43503032, 0x4e483232, 0xffff3241,
        UINT32_MAX, UINT32_MAX, UINT32_MAX,
    };

    for (size_t i = 0; i < G_N_ELEMENTS(module_name); i++) {
        g_assert_cmphex(readl(DEVINFO_BASE + DEVINFO_MODULENAME0 + 4 * i),
                        ==, module_name[i]);
    }
    g_assert_cmphex(readl(DEVINFO_BASE + DEVINFO_MODULEINFO), ==, 0x00570100);
}

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

/*
 * Series-2 SET/CLR/TGL pages are alternate write views of the normal
 * register page.  In particular, counter_silabs uses the alias forms for
 * EN, IEN, and IF.  Exercise the observable consequences too: EN gates the
 * free-running counter, and writes through the non-secure view share the
 * same canonical state.
 */
static void test_burtc_register_aliases(void)
{
    qtest_qmp_assert_success(global_qtest, "{'execute':'system_reset'}");

    writel(BURTC_BASE + BURTC_EN + REGISTER_SET_WINDOW, BURTC_EN_EN);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_EN), ==, BURTC_EN_EN);
    writel(BURTC_BASE + BURTC_CMD, BURTC_CMD_START);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_STATUS), ==,
                    BURTC_STATUS_RUNNING);

    writel(BURTC_BASE + BURTC_EN + REGISTER_TOGGLE_WINDOW, BURTC_EN_EN);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_EN), ==, 0);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_STATUS), ==, 0);
    writel(BURTC_BASE + BURTC_EN + REGISTER_TOGGLE_WINDOW, BURTC_EN_EN);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_EN), ==, BURTC_EN_EN);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_STATUS), ==,
                    BURTC_STATUS_RUNNING);

    writel(BURTC_NONSECURE_BASE + BURTC_IEN + REGISTER_SET_WINDOW,
           BURTC_IF_COMP);
    writel(BURTC_NONSECURE_BASE + BURTC_IF + REGISTER_SET_WINDOW,
           BURTC_IF_COMP);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_IEN), ==, BURTC_IF_COMP);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_IF), ==, BURTC_IF_COMP);

    writel(BURTC_BASE + BURTC_IF + REGISTER_TOGGLE_WINDOW, BURTC_IF_COMP);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_IF), ==, 0);

    writel(BURTC_BASE + BURTC_IEN + REGISTER_TOGGLE_WINDOW, BURTC_IF_COMP);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_IEN), ==, 0);
    writel(BURTC_BASE + BURTC_IEN + REGISTER_TOGGLE_WINDOW, BURTC_IF_COMP);
    writel(BURTC_BASE + BURTC_IF + REGISTER_SET_WINDOW, BURTC_IF_COMP);
    writel(BURTC_BASE + BURTC_IEN + REGISTER_CLEAR_WINDOW, BURTC_IF_COMP);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_IEN), ==, 0);
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

/*
 * A compare equal to the current free-running count is the next-wrap edge,
 * rather than an immediate event.  Zephyr's counter cancel path writes
 * COMP = 0 after clearing IF, so this also keeps cancellation from creating
 * a spurious pending flag at reset count zero.
 */
static void test_burtc_current_compare_waits_for_wrap(void)
{
    qtest_qmp_assert_success(global_qtest, "{'execute':'system_reset'}");

    writel(BURTC_BASE + BURTC_EN + REGISTER_SET_WINDOW, BURTC_EN_EN);
    writel(BURTC_BASE + BURTC_CNT, 10);
    writel(BURTC_BASE + BURTC_IF + REGISTER_CLEAR_WINDOW, BURTC_IF_COMP);
    writel(BURTC_BASE + BURTC_COMP, 10);
    writel(BURTC_BASE + BURTC_IEN + REGISTER_SET_WINDOW, BURTC_IF_COMP);
    writel(BURTC_BASE + BURTC_CMD, BURTC_CMD_START);
    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    qtest_clock_step(global_qtest, 2 * 1000 * 1000);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");

    g_assert_cmphex(readl(BURTC_BASE + BURTC_IF) & BURTC_IF_COMP, ==, 0);
}

/* TIMER4 is Zephyr's 16-bit 38.4 MHz scheduling counter on this part. */
static void test_timer4_counter_and_aliases(void)
{
    uint32_t stopped;

    qtest_qmp_assert_success(global_qtest, "{'execute':'system_reset'}");
    g_assert_cmphex(readl(TIMER4_BASE + TIMER_TOP), ==, 0xffff);

    writel(TIMER4_NS_BASE + TIMER_EN + REGISTER_SET_WINDOW, TIMER_EN_EN);
    writel(TIMER4_NS_BASE + TIMER_TOP, 0xffff);
    writel(TIMER4_NS_BASE + TIMER_CNT, 0x1200);
    writel(TIMER4_NS_BASE + TIMER_CMD, TIMER_CMD_START);
    g_assert_cmphex(readl(TIMER4_BASE + TIMER_STATUS), ==,
                    TIMER_STATUS_RUNNING);

    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    qtest_clock_step(global_qtest, 1000);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");
    g_assert_cmpuint(readl(TIMER4_BASE + TIMER_CNT), >, 0x1200);

    writel(TIMER4_BASE + TIMER_CMD + REGISTER_SET_WINDOW, TIMER_CMD_STOP);
    g_assert_cmphex(readl(TIMER4_NS_BASE + TIMER_STATUS), ==, 0);
    stopped = readl(TIMER4_BASE + TIMER_CNT);
    qtest_clock_step(global_qtest, 1000);
    g_assert_cmphex(readl(TIMER4_BASE + TIMER_CNT), ==, stopped);

    writel(TIMER4_NS_BASE + TIMER_EN + REGISTER_CLEAR_WINDOW, TIMER_EN_EN);
    g_assert_cmphex(readl(TIMER4_BASE + TIMER_STATUS), ==, 0);
}

/* The retained stock image starts TIMER2 during application bring-up. */
static void test_timer2_counter_and_aliases(void)
{
    uint32_t stopped;

    qtest_qmp_assert_success(global_qtest, "{'execute':'system_reset'}");
    g_assert_cmphex(readl(TIMER2_BASE + TIMER_TOP), ==, 0xffff);

    writel(TIMER2_NS_BASE + TIMER_EN + REGISTER_SET_WINDOW, TIMER_EN_EN);
    writel(TIMER2_NS_BASE + TIMER_TOP, 0xffff);
    writel(TIMER2_NS_BASE + TIMER_CNT, 0x1200);
    writel(TIMER2_NS_BASE + TIMER_CMD, TIMER_CMD_START);
    g_assert_cmphex(readl(TIMER2_BASE + TIMER_STATUS), ==,
                    TIMER_STATUS_RUNNING);

    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    qtest_clock_step(global_qtest, 1000);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");
    g_assert_cmpuint(readl(TIMER2_BASE + TIMER_CNT), >, 0x1200);

    writel(TIMER2_BASE + TIMER_CMD + REGISTER_SET_WINDOW, TIMER_CMD_STOP);
    g_assert_cmphex(readl(TIMER2_NS_BASE + TIMER_STATUS), ==, 0);
    stopped = readl(TIMER2_BASE + TIMER_CNT);
    qtest_clock_step(global_qtest, 1000);
    g_assert_cmphex(readl(TIMER2_BASE + TIMER_CNT), ==, stopped);

    writel(TIMER2_NS_BASE + TIMER_EN + REGISTER_CLEAR_WINDOW, TIMER_EN_EN);
    g_assert_cmphex(readl(TIMER2_BASE + TIMER_STATUS), ==, 0);
}

static void timer0_step(int64_t ns)
{
    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    qtest_clock_step(global_qtest, ns);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");
}

/* TIMER0 is 32-bit (TIMER0_CNTWIDTH = 0x20) and counts HFXO / (PRESC + 1). */
static void test_timer0_32bit_count_prescale_overflow(void)
{
    qtest_qmp_assert_success(global_qtest, "{'execute':'system_reset'}");
    g_assert_cmphex(readl(TIMER0_BASE + TIMER_TOP), ==, 0xffff);
    writel(CMU_BASE_ADDR + CMU_EM01GRPACLKCTRL, CMU_EM01GRPACLK_HFXO);

    writel(TIMER0_NS_BASE + TIMER_EN + REGISTER_SET_WINDOW, TIMER_EN_EN);
    writel(TIMER0_NS_BASE + TIMER_TOP, 0xffffffff);
    writel(TIMER0_NS_BASE + TIMER_CNT, 0xffffff00);
    writel(TIMER0_NS_BASE + TIMER_CMD, TIMER_CMD_START);
    g_assert_cmphex(readl(TIMER0_BASE + TIMER_STATUS), ==,
                    TIMER_STATUS_RUNNING);

    /* 10 us at 38.4 MHz is exactly 384 ticks: past the 32-bit wrap. */
    timer0_step(10000);
    g_assert_cmphex(readl(TIMER0_BASE + TIMER_CNT), ==, 0x80);
    g_assert_cmphex(readl(TIMER0_BASE + TIMER_IF) & TIMER_IF_OF, ==,
                    TIMER_IF_OF);

    /* PRESC = 3 divides by four from the current count. */
    writel(TIMER0_BASE + TIMER_CFG, 3u << TIMER_CFG_PRESC_SHIFT);
    timer0_step(10000);
    g_assert_cmphex(readl(TIMER0_BASE + TIMER_CNT), ==, 0x80 + 96);

    writel(TIMER0_BASE + TIMER_CMD + REGISTER_SET_WINDOW, TIMER_CMD_STOP);
    g_assert_cmphex(readl(TIMER0_NS_BASE + TIMER_STATUS), ==, 0);
    timer0_step(10000);
    g_assert_cmphex(readl(TIMER0_BASE + TIMER_CNT), ==, 0x80 + 96);
}

/* Each output-compare channel flags only when CNT reaches its own OC. */
static void test_timer0_cc_output_compare(void)
{
    qtest_qmp_assert_success(global_qtest, "{'execute':'system_reset'}");
    writel(CMU_BASE_ADDR + CMU_EM01GRPACLKCTRL, CMU_EM01GRPACLK_HFXO);

    writel(TIMER0_NS_BASE + TIMER_EN, TIMER_EN_EN);
    writel(TIMER0_NS_BASE + TIMER_TOP, 0xffffffff);
    writel(TIMER0_NS_BASE + TIMER_CC_CFG(0), TIMER_CC_CFG_OUTPUTCOMPARE);
    writel(TIMER0_NS_BASE + TIMER_CC_CFG(1), TIMER_CC_CFG_OUTPUTCOMPARE);
    writel(TIMER0_NS_BASE + TIMER_CC_OC(0), 1000);
    writel(TIMER0_NS_BASE + TIMER_CC_OC(1), 2000);
    /* CC2 is left in OFF mode: its OC value must not raise a flag. */
    writel(TIMER0_NS_BASE + TIMER_CC_OC(2), 500);
    writel(TIMER0_NS_BASE + TIMER_IEN, TIMER_IF_CC(0) | TIMER_IF_CC(1));
    writel(TIMER0_NS_BASE + TIMER_CNT, 0);
    writel(TIMER0_NS_BASE + TIMER_CMD, TIMER_CMD_START);

    /* 1000 ticks are 26041.67 ns. */
    timer0_step(26041);
    g_assert_cmphex(readl(TIMER0_BASE + TIMER_IF), ==, 0);
    timer0_step(1);
    g_assert_cmphex(readl(TIMER0_BASE + TIMER_IF), ==, TIMER_IF_CC(0));

    writel(TIMER0_BASE + TIMER_IF + REGISTER_CLEAR_WINDOW, TIMER_IF_CC(0));
    timer0_step(26042);
    g_assert_cmphex(readl(TIMER0_BASE + TIMER_IF), ==, TIMER_IF_CC(1));

    /* A compare moved behind CNT waits for the counter to come around. */
    writel(TIMER0_BASE + TIMER_IF + REGISTER_CLEAR_WINDOW, TIMER_IF_CC(1));
    writel(TIMER0_BASE + TIMER_CC_OC(0), 10);
    timer0_step(100000);
    g_assert_cmphex(readl(TIMER0_BASE + TIMER_IF), ==, 0);
}

/*
 * TIMER0 follows EM01GRPACLK.  The BG22 explorer kit runs it from HFRCODPLL
 * locked to HFXO with M = 1919, N = 3839 (xg22_explorer_kit.dtsi), i.e.
 * 38.4 MHz * 3840 / 1920 = 76.8 MHz.
 */
static void test_timer0_follows_em01grpaclk_dpll(void)
{
    qtest_qmp_assert_success(global_qtest, "{'execute':'system_reset'}");

    writel(CMU_BASE_ADDR + CMU_DPLLREFCLKCTRL, 1); /* HFXO reference */
    writel(DPLL0_BASE_ADDR + DPLL_CFG1, (3839u << 16) | 1919u);
    writel(DPLL0_BASE_ADDR + DPLL_EN, 1);
    writel(TIMER0_NS_BASE + TIMER_EN, TIMER_EN_EN);
    writel(TIMER0_NS_BASE + TIMER_TOP, 0xffffffff);
    writel(TIMER0_NS_BASE + TIMER_CNT, 0);
    writel(TIMER0_NS_BASE + TIMER_CMD, TIMER_CMD_START);
    timer0_step(10000);
    g_assert_cmphex(readl(TIMER0_BASE + TIMER_CNT), ==, 768);

    /* Switching EM01GRPACLK to HFXO halves the rate from the current count. */
    writel(CMU_BASE_ADDR + CMU_EM01GRPACLKCTRL, CMU_EM01GRPACLK_HFXO);
    timer0_step(10000);
    g_assert_cmphex(readl(TIMER0_BASE + TIMER_CNT), ==, 768 + 384);
}

static void test_timer4_cc0_compare_pending(void)
{
    qtest_qmp_assert_success(global_qtest, "{'execute':'system_reset'}");

    writel(TIMER4_NS_BASE + TIMER_EN + REGISTER_SET_WINDOW, TIMER_EN_EN);
    writel(TIMER4_NS_BASE + TIMER_TOP, 0xffff);
    writel(TIMER4_NS_BASE + TIMER_CNT, 10);
    writel(TIMER4_NS_BASE + TIMER_IF + REGISTER_CLEAR_WINDOW, TIMER_IF_CC0);
    writel(TIMER4_NS_BASE + TIMER_CC0_OC, 48);
    writel(TIMER4_NS_BASE + TIMER_IEN + REGISTER_SET_WINDOW, TIMER_IF_CC0);
    writel(TIMER4_NS_BASE + TIMER_CMD, TIMER_CMD_START);

    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    qtest_clock_step(global_qtest, 2000);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");
    g_assert_cmphex(readl(TIMER4_BASE + TIMER_IF) & TIMER_IF_CC0, ==,
                    TIMER_IF_CC0);
    writel(TIMER4_BASE + TIMER_IF + REGISTER_CLEAR_WINDOW, TIMER_IF_CC0);
    g_assert_cmphex(readl(TIMER4_NS_BASE + TIMER_IF) & TIMER_IF_CC0, ==, 0);
}

/*
 * A board reset is also the safe startup baseline for the radio port.  In
 * particular, it must discard both pending timer events and register-latched
 * interrupt enables before firmware begins a new LL schedule.
 */
static void test_system_reset_clears_radio_timer_state(void)
{
    qtest_qmp_assert_success(global_qtest, "{'execute':'system_reset'}");

    writel(BURTC_BASE + BURTC_EN, BURTC_EN_EN);
    writel(BURTC_BASE + BURTC_CNT, 10);
    writel(BURTC_BASE + BURTC_COMP, 42);
    writel(BURTC_BASE + BURTC_IEN, BURTC_IF_COMP);
    writel(BURTC_BASE + BURTC_CMD, BURTC_CMD_START);

    writel(TIMER4_BASE + TIMER_EN, TIMER_EN_EN);
    writel(TIMER4_BASE + TIMER_CNT, 10);
    writel(TIMER4_BASE + TIMER_CC0_OC, 48);
    writel(TIMER4_BASE + TIMER_IEN, TIMER_IF_CC0);
    writel(TIMER4_BASE + TIMER_CMD, TIMER_CMD_START);

    writel(PROTIMER_NS_BASE + PROTIMER_IF, 0x02000010);
    writel(PROTIMER_NS_BASE + PROTIMER_IEN, 0x00000010);

    qtest_qmp_assert_success(global_qtest, "{'execute':'system_reset'}");
    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    qtest_clock_step(global_qtest, 2 * 1000 * 1000);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");

    g_assert_cmphex(readl(BURTC_BASE + BURTC_STATUS), ==, 0);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_IF), ==, 0);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_IEN), ==, 0);
    g_assert_cmphex(readl(TIMER4_BASE + TIMER_STATUS), ==, 0);
    g_assert_cmphex(readl(TIMER4_BASE + TIMER_IF), ==, 0);
    g_assert_cmphex(readl(TIMER4_BASE + TIMER_IEN), ==, 0);
    g_assert_cmphex(readl(PROTIMER_BASE + PROTIMER_IF), ==, 0);
    g_assert_cmphex(readl(PROTIMER_BASE + PROTIMER_IEN), ==, 0);
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
    enable_radio_bus_clocks();
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
    /* RAIL initializes convolution state while aborting stale receive work. */
    writel(FRC_BASE + FRC_CMD, FRC_CMD_RXABORT | FRC_CMD_CONVINIT);
    g_assert_cmphex(readl(FRC_BASE + FRC_CMD), ==, 0);

    /* Its RF initializer then writes RXCTRL=0x60 and clears RAC.SR0 bit 6. */
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

    qtest_qmp_assert_success(global_qtest, "{'execute':'system_reset'}");
    enable_radio_bus_clocks();
    for (size_t i = 0; i < G_N_ELEMENTS(blocks); i++) {
        writel(blocks[i] + 0x004, UINT32_MAX);
        g_assert_cmphex(readl(blocks[i] + 0x004), ==, 1);
    }
}

static void test_radio_enable_requires_bus_clocks(void)
{
    qtest_qmp_assert_success(global_qtest, "{'execute':'system_reset'}");

    writel(AGC_BASE + 0x004, 1);
    g_assert_cmphex(readl(AGC_BASE + 0x004), ==, 0);

    writel(CMU_BASE + CMU_CLKEN1 + REGISTER_SET_WINDOW, 1u << 0);
    writel(AGC_BASE + 0x004, 1);
    g_assert_cmphex(readl(AGC_BASE + 0x004), ==, 0);

    writel(CMU_BASE + CMU_RADIOCLKCTRL + REGISTER_SET_WINDOW,
           CMU_RADIOCLKCTRL_EN);
    writel(AGC_BASE + 0x004, 1);
    g_assert_cmphex(readl(AGC_BASE + 0x004), ==, 1);
}

static void test_radio_ipversions(void)
{
    static const struct {
        uint32_t base;
        uint32_t value;
    } blocks[] = {
        { FRC_BASE, 1 },
        { AGC_BASE, 1 },
        { RFCRC_BASE, 0 },
        { MODEM_BASE, 1 },
        { SYNTH_BASE, 1 },
        { RAC_BASE, 1 },
    };

    qtest_qmp_assert_success(global_qtest, "{'execute':'system_reset'}");
    for (size_t i = 0; i < G_N_ELEMENTS(blocks); i++) {
        uint32_t secure = blocks[i].base + RADIO_IPVERSION;
        uint32_t nonsecure = secure + RADIO_NONSECURE_ALIAS;

        g_assert_cmphex(readl(secure), ==, blocks[i].value);
        g_assert_cmphex(readl(nonsecure), ==, 0);
        writel(secure, UINT32_MAX);
        g_assert_cmphex(readl(secure), ==, blocks[i].value);
    }

    attribute_radio_nonsecure();
    for (size_t i = 0; i < G_N_ELEMENTS(blocks); i++) {
        uint32_t secure = blocks[i].base + RADIO_IPVERSION;
        uint32_t nonsecure = secure + RADIO_NONSECURE_ALIAS;

        g_assert_cmphex(readl(secure), ==, 0);
        g_assert_cmphex(readl(nonsecure), ==, blocks[i].value);
    }
}

/*
 * FRC.STATUS is hardware read-only (public revB2 FRC_S_register.py); no
 * modelled command path sets ACTIVETXFCD or any other bit
 * (docs/radio-tables.md SS7.4 records that open question separately). This
 * only confirms the model cannot be made to disagree with that read-only
 * contract through either address view; it is not a claim about what value
 * real hardware would show at this point.
 */
static void test_frc_status_is_read_only(void)
{
    uint32_t secure = FRC_BASE + FRC_STATUS;
    uint32_t nonsecure = secure + RADIO_NONSECURE_ALIAS;

    qtest_qmp_assert_success(global_qtest, "{'execute':'system_reset'}");
    g_assert_cmphex(readl(secure), ==, 0);
    writel(secure, UINT32_MAX);
    g_assert_cmphex(readl(secure), ==, 0);

    attribute_radio_nonsecure();
    g_assert_cmphex(readl(nonsecure), ==, 0);
    writel(nonsecure, UINT32_MAX);
    g_assert_cmphex(readl(nonsecure), ==, 0);
    attribute_radio_secure();
}

static void test_radio_nonsecure_alias(void)
{
    uint32_t secure_address = FRC_BASE + FRC_WHITEPOLY;
    uint32_t nonsecure_address = secure_address + RADIO_NONSECURE_ALIAS;

    qtest_qmp_assert_success(global_qtest, "{'execute':'system_reset'}");
    writel(secure_address, 0x1234);
    writel(nonsecure_address, UINT32_MAX); /* inactive alias is WI */
    g_assert_cmphex(readl(secure_address), ==, 0x1234);
    g_assert_cmphex(readl(nonsecure_address), ==, 0); /* inactive alias is RAZ */

    attribute_radio_nonsecure();
    g_assert_cmphex(readl(secure_address), ==, 0);
    g_assert_cmphex(readl(nonsecure_address), ==, 0x1234);
    writel(nonsecure_address, UINT32_MAX);
    g_assert_cmphex(readl(nonsecure_address), ==, 0xffff);

    attribute_radio_secure();
    g_assert_cmphex(readl(secure_address), ==, 0xffff);
    g_assert_cmphex(readl(nonsecure_address), ==, 0);
}

static void test_stock_setup_aliases(void)
{
    /* Stock writes one at +0x004 through these two radio windows. */
    enable_radio_bus_clocks();
    attribute_radio_nonsecure();
    writel(BUFC_BASE + RADIO_NONSECURE_ALIAS + BUFC_EN, 1);
    writel(PROTIMER_NS_BASE + PROTIMER_EN, 1);

    g_assert_cmphex(readl(BUFC_BASE + RADIO_NONSECURE_ALIAS + BUFC_EN), ==, 1);
    g_assert_cmphex(readl(PROTIMER_NS_BASE + PROTIMER_EN), ==, 1);
    g_assert_cmphex(readl(BUFC_BASE + BUFC_EN), ==, 0);
    g_assert_cmphex(readl(PROTIMER_BASE + PROTIMER_EN), ==, 0);
}

static void test_rac_enable_requires_dependency_order(void)
{
    qtest_qmp_assert_success(global_qtest, "{'execute':'system_reset'}");
    enable_radio_bus_clocks();

    writel(RAC_BASE + RAC_EN, 1);
    g_assert_cmphex(readl(RAC_BASE + RAC_STATUS) & RAC_STATUS_STATE_MASK,
                    ==, RAC_STATUS_STATE_SHUTDOWN);
    writel(BUFC_BASE + BUFC_EN, 1);
    writel(PROTIMER_BASE + PROTIMER_EN, 1);
    g_assert_cmphex(readl(RAC_BASE + RAC_STATUS) & RAC_STATUS_STATE_MASK,
                    ==, RAC_STATUS_STATE_SHUTDOWN);

    qtest_qmp_assert_success(global_qtest, "{'execute':'system_reset'}");
    enable_radio_bus_clocks();
    writel(BUFC_BASE + BUFC_EN, 1);
    writel(PROTIMER_BASE + PROTIMER_EN, 1);
    writel(RAC_BASE + RAC_EN, 1);
    g_assert_cmphex(readl(RAC_BASE + RAC_STATUS) & RAC_STATUS_STATE_MASK,
                    ==, 0);
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

    /*
     * The recovered sequencer image uses 32-bit STR instructions here, but
     * WRITEDATA is an 8-bit field. Only the low octet is consumed per write.
     */
    writel(sequencer_channel + BUFC_CMD, 1);
    writel(sequencer_channel + 0x004, rx_buffer);
    writel(sequencer_channel + BUFC_WRITEDATA, 0xa5b63c);
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

static void test_bufc_descriptor_ring_overflow_latch(void)
{
    const uint32_t channel = BUFC_BASE + BUFC_CH2;
    const uint32_t overflow_flag = 1u << 16;

    /*
     * RAIL's archive-backed class-4 producer starts from BUF2OF.  The BUFC
     * block supplies that channel-2 condition; software, not this generic
     * FIFO model, decides whether and how to publish the eight-byte record.
     */
    writel(BUFC_BASE + BUFC_IF + REGISTER_CLEAR_WINDOW, overflow_flag);
    writel(channel + BUFC_CMD, 1);
    writel(channel, 0); /* 64-byte descriptor-ring backing FIFO */
    for (unsigned i = 0; i <= 64; i++) {
        writel(channel + BUFC_WRITEDATA, i);
    }

    g_assert_cmphex(readl(BUFC_BASE + BUFC_IF) & overflow_flag, ==,
                    overflow_flag);
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
    /* Channel 1's sparse threshold source is independent from BUF2THR. */
    writel(channel + BUFC_THRESHOLD, 7);
    for (unsigned int i = 0; i < 7; i++) {
        writel(channel + BUFC_WRITEDATA, i);
    }

    g_assert_cmphex(readl(BUFC_BASE + BUFC_IF) & threshold_flag, ==, 0);
    g_assert_cmphex(readl(channel + BUFC_STATUS), ==, 7);

    writel(channel + BUFC_WRITEDATA, 7);

    g_assert_cmphex(readl(BUFC_BASE + BUFC_IF) & threshold_flag, ==,
                    threshold_flag);
    g_assert_cmphex(readl(channel + BUFC_STATUS), ==, (1u << 20) | 8u);
}

static void test_bufc_descriptor_threshold_flag(void)
{
    uint32_t channel = BUFC_BASE + BUFC_CH2;
    uint32_t threshold_flag = 1u << 18;

    /*
     * The archived RAIL descriptor-ring setup uses buffer 2, whose global
     * threshold source is bit 18 (not a compact third threshold bit).  This
     * checks only BUFC's documented counter/flag presentation; it does not
     * claim that this CPU data-port write is a hardware descriptor producer.
     */
    writel(BUFC_BASE + BUFC_IF + REGISTER_CLEAR_WINDOW, threshold_flag);
    writel(BUFC_BASE + BUFC_IEN, threshold_flag);
    writel(channel + BUFC_CMD, 1);
    writel(channel, 3);
    writel(channel + 0x004, 0x20002400);
    writel(channel + BUFC_THRESHOLD, 7);
    for (unsigned int i = 0; i < 7; i++) {
        writel(channel + BUFC_WRITEDATA, i);
    }

    g_assert_cmphex(readl(BUFC_BASE + BUFC_IF) & threshold_flag, ==, 0);
    g_assert_cmphex(readl(channel + BUFC_STATUS), ==, 7);

    writel(channel + BUFC_WRITEDATA, 7);

    g_assert_cmphex(readl(BUFC_BASE + BUFC_IF) & threshold_flag, ==,
                    threshold_flag);
    g_assert_cmphex(readl(channel + BUFC_STATUS), ==, (1u << 20) | 8u);
    g_assert_cmphex(readl(0x20002400), ==, 0x03020100);
    g_assert_cmphex(readl(0x20002404), ==, 0x07060504);
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
    writel(PROTIMER_BASE + PROTIMER_CTRL, 0x00011102);
    attribute_radio_nonsecure();
    /* RAIL enables PROTIMER through the attributed 0xb801c000 window. */
    writel(PROTIMER_NS_BASE + PROTIMER_CTRL, 0x00011102);
    writel(PROTIMER_NS_BASE + PROTIMER_CMD, 1);

    g_assert_cmphex(readl(PROTIMER_BASE + PROTIMER_CTRL), ==, 0);
    g_assert_cmphex(readl(PROTIMER_NS_BASE + PROTIMER_CTRL), ==, 0x00011102);
}

/*
 * generic_phy.o:FRC_IRQHandler reads PROTIMER.IF (+0x064) and IEN (+0x070)
 * through the non-secure window.  Keep those banks as aliasable latches; a
 * counter or compare match deliberately does not manufacture a pending bit.
 */
static void test_protimer_interrupt_banks_are_aliasable(void)
{
    attribute_radio_nonsecure();
    writel(PROTIMER_NS_BASE + PROTIMER_IF, 0x02000010);
    writel(PROTIMER_NS_BASE + PROTIMER_IEN + REGISTER_SET_WINDOW, 0x00000010);

    g_assert_cmphex(readl(PROTIMER_BASE + PROTIMER_IF), ==, 0);
    g_assert_cmphex(readl(PROTIMER_NS_BASE + PROTIMER_IEN), ==, 0x00000010);

    writel(PROTIMER_NS_BASE + PROTIMER_IF + REGISTER_CLEAR_WINDOW, 0x10);
    writel(PROTIMER_NS_BASE + PROTIMER_IEN + REGISTER_TOGGLE_WINDOW, 0x10);

    g_assert_cmphex(readl(PROTIMER_NS_BASE + PROTIMER_IF), ==, 0x02000000);
    g_assert_cmphex(readl(PROTIMER_NS_BASE + PROTIMER_IEN), ==, 0);
}

/*
 * rfhal_protimer.o:0x3c140 writes 0x11 to the first indexed slot and, on
 * one branch, clears a bit through 0xb801e100. This only preserves the
 * observed normal/CLR alias access; it does not assign compare semantics.
 */
static void test_protimer_compare_slot_clear_alias(void)
{
    attribute_radio_nonsecure();
    writel(PROTIMER_NS_BASE + PROTIMER_CMP0, 0x11);
    writel(PROTIMER_NS_BASE + PROTIMER_CMP0_TIME, 0x12345678);
    writel(PROTIMER_NS_BASE + PROTIMER_CMP0 + REGISTER_CLEAR_WINDOW, 1);

    g_assert_cmphex(readl(PROTIMER_NS_BASE + PROTIMER_CMP0), ==, 0x10);
    g_assert_cmphex(readl(PROTIMER_NS_BASE + PROTIMER_CMP0_TIME), ==, 0x12345678);

    writel(PROTIMER_NS_BASE + PROTIMER_CMP0 + REGISTER_CLEAR_WINDOW, 0x10);
    g_assert_cmphex(readl(PROTIMER_NS_BASE + PROTIMER_CMP0), ==, 0);
}

static void test_protimer_counter(void)
{
    uint32_t before = readl(PROTIMER_BASE + PROTIMER_COUNTER);

    attribute_radio_nonsecure();
    g_assert_cmphex(readl(PROTIMER_NS_BASE + PROTIMER_COUNTER), ==, before);
    writel(PROTIMER_NS_BASE + PROTIMER_COUNTER, UINT32_MAX);
    g_assert_cmphex(readl(PROTIMER_NS_BASE + PROTIMER_COUNTER), ==, before);

    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    /* Step past the second 500 ns edge so QEMU dispatches both ptimer ticks. */
    clock_step(1001);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");
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

static void test_protimer_counter_is_independent_from_burtc(void)
{
    uint32_t before;

    qtest_qmp_assert_success(global_qtest, "{'execute':'system_reset'}");
    before = readl(PROTIMER_BASE + PROTIMER_COUNTER);

    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(1001);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");
    g_assert_cmphex(readl(PROTIMER_BASE + PROTIMER_COUNTER), ==, before + 2);

    qtest_qmp_assert_success(global_qtest, "{'execute':'system_reset'}");
    writel(BURTC_BASE + BURTC_EN, BURTC_EN_EN);
    writel(BURTC_BASE + BURTC_CNT, 0x12345678);
    writel(BURTC_BASE + BURTC_CMD, BURTC_CMD_START);
    g_assert_cmphex(readl(BURTC_BASE + BURTC_STATUS), ==,
                    BURTC_STATUS_RUNNING);

    before = readl(PROTIMER_BASE + PROTIMER_COUNTER);
    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(1001);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");
    /* BURTC's separate 32.768 kHz clock must not replace PROTIMERCLK. */
    g_assert_cmphex(readl(PROTIMER_BASE + PROTIMER_COUNTER), ==, before + 2);
}

static void test_rac_tx_commands(void)
{
    writel(RAC_BASE + RAC_CMD, RAC_CMD_TXEN);
    g_assert_cmphex(readl(RAC_BASE + RAC_STATUS) & RAC_STATUS_STATE_MASK, ==,
                    RAC_STATUS_STATE_TX);
    /*
     * TXENS does NOT set here, matching real silicon: this exact
     * TXEN sequence measures RAC.STATUS == 0x09400000 on real hardware
     * (owo-open-fw docs/radio-tables.md SS7.21/SS7.25-27), i.e. STATE=TX
     * with TXENS=0. Confirmed repeatedly, not a simplification gap.
     */
    g_assert_cmphex(readl(RAC_BASE + RAC_STATUS) &
                        (RAC_STATUS_TXENS | RAC_STATUS_RXENS),
                    ==, 0);
    g_assert_cmphex(readl(RAC_BASE + RAC_SEQIF) & RAC_SEQIF_STATETX, ==,
                    RAC_SEQIF_STATETX);
    g_assert_cmphex(readl(RAC_BASE + RAC_SEQIF) & RAC_SEQIF_STATECHANGESEQ, ==,
                    RAC_SEQIF_STATECHANGESEQ);

    writel(RAC_BASE + RAC_CMD, RAC_CMD_TXDIS);
    g_assert_cmphex(readl(RAC_BASE + RAC_STATUS) & RAC_STATUS_STATE_MASK, ==,
                    0);
    g_assert_cmphex(readl(RAC_BASE + RAC_SEQIF) & RAC_SEQIF_STATEOFF, ==,
                    RAC_SEQIF_STATEOFF);
    g_assert_cmphex(readl(RAC_BASE + RAC_SEQIF) & RAC_SEQIF_STATECHANGESEQ, ==,
                    RAC_SEQIF_STATECHANGESEQ);
}

static void test_rac_tx_completion(void)
{
    writel(FRC_BASE + FRC_IF + REGISTER_CLEAR_WINDOW, FRC_IF_TXDONE);
    writel(RAC_BASE + RAC_CMD, RAC_CMD_TXEN);
    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(149999);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");

    /* QEMU's synthetic completion must not become visible before 150 us. */
    g_assert_cmphex(readl(RAC_BASE + RAC_STATUS) & RAC_STATUS_STATE_MASK, ==,
                    RAC_STATUS_STATE_TX);
    g_assert_cmphex(readl(FRC_BASE + FRC_IF) & FRC_IF_TXDONE, ==, 0);

    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(1);
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

/*
 * The recovered M33-side protocol uses endpoint data plus FLAGS bit 1.  QEMU
 * retains the guest data register but deliberately has no peer endpoint, so
 * neither secure nor non-secure FLAGS reads may manufacture a doorbell.
 * This is a known, recorded divergence from real silicon, not an absence of
 * one: bit 0 of this same register gates a real M33-side RAIL calibration
 * busy-wait on real hardware (owo-open-fw docs/radio-tables.md SS7.5.4,
 * lockstepped directly), which this model cannot reproduce. Not fixed here
 * because there is no emulated M0+ peer to ever clear a real latch -- see
 * the matching comment in hw/arm/efr32bg22.c.
 */
static void test_rfmailbox_no_peer(void)
{
    attribute_radio_nonsecure();
    writel(RFMAILBOX_C_BASE + RFMAILBOX_NS_OFFSET + RFMAILBOX_DATA, 0x3412);
    g_assert_cmphex(readl(RFMAILBOX_C_BASE + RFMAILBOX_DATA), ==, 0);
    g_assert_cmphex(readl(RFMAILBOX_C_BASE + RFMAILBOX_NS_OFFSET +
                          RFMAILBOX_DATA), ==, 0x3412);

    writel(RFMAILBOX_C_BASE + RFMAILBOX_NS_OFFSET + RFMAILBOX_FLAGS +
               REGISTER_SET_WINDOW,
           1u << 1);
    g_assert_cmphex(readl(RFMAILBOX_C_BASE + RFMAILBOX_FLAGS), ==, 0);
    g_assert_cmphex(readl(RFMAILBOX_C_BASE + RFMAILBOX_NS_OFFSET +
                          RFMAILBOX_FLAGS), ==, 0);
    g_assert_cmphex(readl(RFMAILBOX_B_BASE + RFMAILBOX_FLAGS), ==, 0);
    attribute_radio_secure();
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
                    ==, RAC_STATUS_STATE_SHUTDOWN);

    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(150001);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");
    g_assert_cmphex(readl(FRC_BASE + FRC_IF) & FRC_IF_TXDONE, ==, 0);

    writel(RAC_BASE + RAC_CTRL, 0);
    g_assert_cmphex(readl(RAC_BASE + RAC_CTRL), ==, 0);
}

/*
 * RAC.STCTRL's only two documented fields -- STCAL[23:0] and STSKEW[24]
 * (RAC_S_field.py) -- are the whole writable mask; everything else must be
 * dropped. Checked against the real literal value (0x01061a7f) stock's own
 * sequencer reset handler writes (docs/radio-tables.md SS7.8/SS7.9): that
 * value already fits the mask exactly, so this also pins the mask against
 * silently regressing to something that would truncate it.
 */
static void test_rac_stctrl_mask(void)
{
    writel(RAC_BASE + RAC_STCTRL, RAC_STCTRL_STOCK_VALUE);
    g_assert_cmphex(readl(RAC_BASE + RAC_STCTRL), ==, RAC_STCTRL_STOCK_VALUE);

    writel(RAC_BASE + RAC_STCTRL, UINT32_MAX);
    g_assert_cmphex(readl(RAC_BASE + RAC_STCTRL), ==,
                    RAC_STCTRL_WRITABLE_MASK);
}

/*
 * Release the AP3 sequencer core exactly as
 * test_radio_sequencer_executes_open_code does, then load an arbitrary
 * caller-assembled Thumb-16 program (instruction count must be even)
 * immediately followed by its literal pool, both at the same
 * SEQRAM_NS_BASE + 0x100 the other sequencer probes use. SEQ.SPARE0 and
 * the seven words after it are zeroed first and are the program's
 * exfiltration channel back to the qtest side. Returns with the core
 * already running ('cont' issued); callers stop/cont it themselves.
 */
static QTestState *seq_release_with_program(const uint16_t *insns,
                                            size_t n_insns,
                                            const uint32_t *literals,
                                            size_t n_literals)
{
    const uint32_t code_base = SEQRAM_NS_BASE + 0x100;
    QTestState *qts;
    size_t i;

    g_assert_cmpuint(n_insns % 2, ==, 0);

    qts = qtest_initf("-machine efr32bg22 -accel tcg -S");

    /* Keep the image-less application M33 in a valid idle loop while TCG
     * executes the independently released auxiliary core. */
    qtest_writel(qts, 0x00000000, 0x20008000);
    qtest_writel(qts, 0x00000004, 0x00000101);
    qtest_writel(qts, 0x00000100, 0x0000e7fe);
    qtest_qmp_assert_success(qts, "{'execute':'system_reset'}");

    qtest_writel(qts, RAC_BASE + RAC_CTRL, RAC_CTRL_SEQRESET);
    qtest_writel(qts, SEQRAM_NS_BASE + 0x000, 0x00003e00);
    qtest_writel(qts, SEQRAM_NS_BASE + 0x004, 0x00000101);

    for (i = 0; i < n_insns; i += 2) {
        qtest_writel(qts, code_base + 2 * i,
                    THUMB16_PAIR(insns[i], insns[i + 1]));
    }
    for (i = 0; i < n_literals; i++) {
        qtest_writel(qts, code_base + 2 * n_insns + 4 * i, literals[i]);
    }
    for (i = 0; i < 8; i++) {
        qtest_writel(qts, SEQRAM_NS_BASE + SEQ_SPARE0 + 4 * i, 0);
    }

    qtest_writel(qts, RAC_BASE + RAC_CTRL, RAC_CTRL_CPUWAITDIS);
    qtest_qmp_assert_success(qts, "{'execute':'cont'}");
    return qts;
}

/*
 * IRQ8/IRQ9 are the two interrupts this project's radio_seq.S actually
 * services (TXWARM and TX). Stock's own sequencer reset handler sets both
 * to priority 1 through NVIC.IPR's byte-addressable aliases specifically
 * so a single `strb` touches only its own IRQ's priority byte, never a
 * neighboring one packed into the same 32-bit IPR word
 * (docs/radio-tables.md SS7.9). This proves that behavior for AP3's own
 * private NVIC instance -- not the M33's, which qtest's own memory
 * access path always reaches first -- by having AP3 itself write and
 * read back IRQ8..IRQ11's priority bytes (IRQ10/IRQ11 sharing NVIC.IPR2
 * with IRQ8/IRQ9) and publish the results to shared RAM.
 *
 * It also pins down that AP3's NVIC is genuinely Armv6-M (2 priority bits
 * implemented, not the M33's 8): every byte written here truncates to one
 * of only four possible stored values (0x00/0x40/0x80/0xc0), which is
 * itself a real, previously-unrecorded consequence of stock writing the
 * literal priority value 1 -- it quantizes away to the same priority 0
 * the core already resets to.
 */
static void test_seq_nvic_ipr_byte_isolation(void)
{
    static const uint32_t ipr2_addr = 0xe000e408; /* NVIC.IPR2: IRQ8..11 */
    const uint32_t code_base = SEQRAM_NS_BASE + 0x100;
    uint16_t insn[22];
    uint32_t literals[3];
    QTestState *qts;
    unsigned n = 0;

    insn[n++] = 0; /* [0] LDR r0, [PC, #...] -> ipr2_addr (patched below) */
    insn[n++] = 0; /* [1] LDR r1, [PC, #...] -> SEQ_SPARE0 (patched below) */
    insn[n++] = thumb_movs_imm8(2, 0x80);        /* [2] sentinel for IRQ10 */
    insn[n++] = thumb_strb_imm(2, 0, 2);         /* [3] */
    insn[n++] = thumb_movs_imm8(2, 0x00);        /* [4] sentinel for IRQ11 */
    insn[n++] = thumb_strb_imm(2, 0, 3);         /* [5] */
    insn[n++] = thumb_movs_imm8(2, 0x40);        /* [6] IRQ8 priority byte */
    insn[n++] = thumb_strb_imm(2, 0, 0);         /* [7] */
    insn[n++] = thumb_movs_imm8(2, 0xc0);        /* [8] IRQ9 priority byte */
    insn[n++] = thumb_strb_imm(2, 0, 1);         /* [9] */
    insn[n++] = thumb_ldrb_imm(3, 0, 0);         /* [10] read IRQ8 back */
    insn[n++] = thumb_str_imm(3, 1, 0);          /* [11] -> SPARE0+0 */
    insn[n++] = thumb_ldrb_imm(3, 0, 1);         /* [12] read IRQ9 back */
    insn[n++] = thumb_str_imm(3, 1, 1);          /* [13] -> SPARE0+4 */
    insn[n++] = thumb_ldrb_imm(3, 0, 2);         /* [14] read IRQ10 back */
    insn[n++] = thumb_str_imm(3, 1, 2);          /* [15] -> SPARE0+8 */
    insn[n++] = thumb_ldrb_imm(3, 0, 3);         /* [16] read IRQ11 back */
    insn[n++] = thumb_str_imm(3, 1, 3);          /* [17] -> SPARE0+12 */
    insn[n++] = 0; /* [18] LDR r4, [PC, #...] -> done marker (patched) */
    insn[n++] = thumb_str_imm(4, 1, 4);          /* [19] -> SPARE0+16 */
    insn[n++] = THUMB16_BRANCH_SELF;             /* [20] park here, done */
    insn[n++] = THUMB16_BRANCH_SELF;             /* [21] padding */
    g_assert_cmpuint(n, ==, 22);

    literals[0] = ipr2_addr;
    literals[1] = SEQ_SPARE0;
    literals[2] = 0xabcd1234u; /* arbitrary completion marker */
    insn[0] = thumb_ldr_pc(0, code_base + 0, code_base + 2 * n);
    insn[1] = thumb_ldr_pc(1, code_base + 2, code_base + 2 * n + 4);
    insn[18] = thumb_ldr_pc(4, code_base + 2 * 18, code_base + 2 * n + 8);

    qts = seq_release_with_program(insn, n, literals, 3);

    for (unsigned attempt = 0; attempt < 1000; attempt++) {
        if (qtest_readl(qts, SEQRAM_NS_BASE + SEQ_SPARE0 + 16) ==
            0xabcd1234u) {
            break;
        }
        g_usleep(1000);
    }
    qtest_qmp_assert_success(qts, "{'execute':'stop'}");

    g_assert_cmphex(qtest_readl(qts, SEQRAM_NS_BASE + SEQ_SPARE0 + 16), ==,
                    0xabcd1234u);
    /* IRQ8: wrote 0x40 (priority bits 01) -- untruncated, since the two
     * implemented priority bits are exactly its top two. */
    g_assert_cmphex(qtest_readl(qts, SEQRAM_NS_BASE + SEQ_SPARE0 + 0), ==,
                    0x40);
    /* IRQ9: wrote 0xc0 (priority bits 11). */
    g_assert_cmphex(qtest_readl(qts, SEQRAM_NS_BASE + SEQ_SPARE0 + 4), ==,
                    0xc0);
    /* IRQ10: never touched after its 0x80 sentinel -- a word-wide or
     * neighbor-clobbering write would corrupt this. */
    g_assert_cmphex(qtest_readl(qts, SEQRAM_NS_BASE + SEQ_SPARE0 + 8), ==,
                    0x80);
    /* IRQ11: never touched after its 0x00 sentinel. */
    g_assert_cmphex(qtest_readl(qts, SEQRAM_NS_BASE + SEQ_SPARE0 + 12), ==,
                    0x00);
    qtest_quit(qts);
}

/*
 * This core's own private Armv6-M SysTick (0xE000E010) is separate from
 * the M33's own SysTick -- stock's reset handler configures it (from
 * SysTick.CALIB, enabled from the processor clock, no interrupt) right
 * after the NVIC.IPR loop above (docs/radio-tables.md SS7.9). This test
 * reads CALIB purely for the record: this model reports NOREF here
 * because no refclk is wired into either core's SysTick, and this
 * project has no confirmed real-hardware CALIB value to assert against
 * (STCAL/CALIB is explicitly recorded as "an opaque empirical value this
 * project has no theory for" -- not invented here either). The actual
 * counting proof below instead drives RELOAD/CSR from a fixed,
 * deliberately-arbitrary test constant, so it is testing the real
 * Armv6-M counting mechanism for AP3's own SysTick instance, not a claimed
 * real-hardware timing value.
 */
static void test_seq_systick_counts(void)
{
    static const uint32_t systick_base = 0xe000e010;
    static const uint32_t test_reload = 0x00ffffffu;
    const uint32_t code_base = SEQRAM_NS_BASE + 0x100;
    uint16_t insn[12];
    uint32_t literals[3];
    QTestState *qts;
    uint32_t val1, val2;

    insn[0] = 0; /* LDR r0, [PC, #...] -> systick_base (patched below) */
    insn[1] = 0; /* LDR r1, [PC, #...] -> SEQ_SPARE0 (patched below) */
    insn[2] = thumb_ldr_imm(4, 0, 3);    /* r4 = CALIB (base+12) */
    insn[3] = thumb_str_imm(4, 1, 4);    /* SPARE0+16 = CALIB, for the record */
    insn[4] = 0; /* LDR r5, [PC, #...] -> test_reload (patched below) */
    insn[5] = thumb_str_imm(5, 0, 1);    /* RELOAD (base+4) = test_reload */
    insn[6] = thumb_movs_imm8(6, 5);     /* r6 = ENABLE | CLKSOURCE */
    insn[7] = thumb_str_imm(6, 0, 0);    /* CSR (base+0) = r6 */
    insn[8] = thumb_ldr_imm(2, 0, 2);    /* loop: r2 = CURRENT (base+8) */
    insn[9] = thumb_str_imm(2, 1, 0);    /* SPARE0 = r2 */
    insn[10] = thumb_b(code_base + 2 * 10, code_base + 2 * 8); /* -> loop */
    insn[11] = THUMB16_BRANCH_SELF;      /* padding, never reached */

    literals[0] = systick_base;
    literals[1] = SEQ_SPARE0;
    literals[2] = test_reload;
    insn[0] = thumb_ldr_pc(0, code_base + 0, code_base + 24);
    insn[1] = thumb_ldr_pc(1, code_base + 2, code_base + 28);
    insn[4] = thumb_ldr_pc(5, code_base + 8, code_base + 32);

    qts = seq_release_with_program(insn, 12, literals, 3);

    /* The virtual clock only advances while the VM is actually running;
     * stop before reading so the two samples below can't race the loop
     * still rewriting SPARE0. */
    g_usleep(5000);
    qtest_qmp_assert_success(qts, "{'execute':'stop'}");
    val1 = qtest_readl(qts, SEQRAM_NS_BASE + SEQ_SPARE0);

    qtest_qmp_assert_success(qts, "{'execute':'cont'}");
    g_usleep(5000);
    qtest_qmp_assert_success(qts, "{'execute':'stop'}");
    val2 = qtest_readl(qts, SEQRAM_NS_BASE + SEQ_SPARE0);

    g_assert_cmpuint(val1, <=, test_reload);
    g_assert_cmpuint(val2, <=, test_reload);
    g_assert_cmphex(val1, !=, val2);

    qtest_quit(qts);
}

static void test_radio_sequencer_executes_open_code(void)
{
    QTestState *qts = qtest_initf("-machine efr32bg22 -accel tcg -S");

    /* Keep the image-less application M33 in a valid idle loop while TCG
     * executes the independently released auxiliary core. */
    qtest_writel(qts, 0x00000000, 0x20008000);
    qtest_writel(qts, 0x00000004, 0x00000101);
    qtest_writel(qts, 0x00000100, 0x0000e7fe);
    qtest_qmp_assert_success(qts, "{'execute':'system_reset'}");

    /* Model the real stage-3 boundary: SEQRAM may retain an unrelated image
     * from an earlier boot.  The M33 asserts the hardware-proven base-register
     * SEQRESET request, overwrites that image, and only then replaces CTRL
     * with CPUWAITDIS.  The replacement is important for the explicit-reset
     * path; RAIL's SET-alias release is a warm path that did not set reset. */
    qtest_writel(qts, SEQRAM_NS_BASE + 0x000, 0xdeadbeef);
    qtest_writel(qts, SEQRAM_NS_BASE + 0x004, 0xcafebabe);
    qtest_writel(qts, RAC_BASE + RAC_CTRL, RAC_CTRL_SEQRESET);

    /* Same independent four-instruction ARMv6-M proof used on silicon:
     * load the local SEQ.SPARE0 address/value, store it, then loop. */
    qtest_writel(qts, SEQRAM_NS_BASE + 0x000, 0x00003e00);
    qtest_writel(qts, SEQRAM_NS_BASE + 0x004, 0x00000101);
    qtest_writel(qts, SEQRAM_NS_BASE + 0x100, 0x49024801);
    qtest_writel(qts, SEQRAM_NS_BASE + 0x104, 0xe7fe6001);
    qtest_writel(qts, SEQRAM_NS_BASE + 0x108, 0x00004fe0);
    qtest_writel(qts, SEQRAM_NS_BASE + 0x10c, 0x12345678);
    qtest_writel(qts, SEQRAM_NS_BASE + SEQ_SPARE0, 0);

    g_assert_cmphex(qtest_readl(qts, SEQRAM_NS_BASE + 0x000), ==,
                    0x00003e00);
    g_assert_cmphex(qtest_readl(qts, SEQRAM_NS_BASE + 0x004), ==,
                    0x00000101);

    qtest_writel(qts, RAC_BASE + RAC_CTRL, RAC_CTRL_CPUWAITDIS);
    qtest_qmp_assert_success(qts, "{'execute':'cont'}");
    for (unsigned attempt = 0; attempt < 1000; attempt++) {
        if (qtest_readl(qts, SEQRAM_NS_BASE + SEQ_SPARE0) == 0x12345678) {
            break;
        }
        g_usleep(1000);
    }
    qtest_qmp_assert_success(qts, "{'execute':'stop'}");

    g_assert_cmphex(qtest_readl(qts, SEQRAM_NS_BASE + SEQ_SPARE0), ==,
                    0x12345678);

    /* This probe never acknowledges the TXWARM state interrupt.  Merely
     * releasing a private core and issuing TXEN must therefore leave RAC at
     * the handoff boundary; the old eager model incorrectly published and
     * completed a frame from the staged FIFO here. */
    qtest_writel(qts, RAC_BASE + RAC_CMD, RAC_CMD_TXEN);
    g_assert_cmphex(qtest_readl(qts, RAC_BASE + RAC_STATUS) &
                        RAC_STATUS_STATE_MASK,
                    ==, RAC_STATUS_STATE_TXWARM);
    qtest_qmp_assert_success(qts, "{'execute':'cont'}");
    g_usleep(10000);
    qtest_qmp_assert_success(qts, "{'execute':'stop'}");
    g_assert_cmphex(qtest_readl(qts, RAC_BASE + RAC_STATUS) &
                        RAC_STATUS_STATE_MASK,
                    ==, RAC_STATUS_STATE_TXWARM);
    qtest_quit(qts);
}

static void test_radio_sequencer_release_is_idempotent(void)
{
    QTestState *qts = qtest_initf("-machine efr32bg22 -accel tcg -S");

    qtest_writel(qts, 0x00000000, 0x20008000);
    qtest_writel(qts, 0x00000004, 0x00000101);
    qtest_writel(qts, 0x00000100, 0x0000e7fe);
    qtest_qmp_assert_success(qts, "{'execute':'system_reset'}");

    /* Increment SPARE0 once at reset entry, then loop.  A second write of
     * CPUWAITDIS must not restart the already-released sequencer. */
    qtest_writel(qts, RAC_BASE + RAC_CTRL, RAC_CTRL_SEQRESET);
    qtest_writel(qts, SEQRAM_NS_BASE + 0x000, 0x00003e00);
    qtest_writel(qts, SEQRAM_NS_BASE + 0x004, 0x00000101);
    qtest_writel(qts, SEQRAM_NS_BASE + 0x100,
                 THUMB16_PAIR(THUMB16_LDR_R0_PC_8, THUMB16_LDR_R1_R0));
    qtest_writel(qts, SEQRAM_NS_BASE + 0x104,
                 THUMB16_PAIR(THUMB16_ADDS_R1_1, THUMB16_STR_R1_R0));
    qtest_writel(qts, SEQRAM_NS_BASE + 0x108, THUMB16_BRANCH_SELF);
    qtest_writel(qts, SEQRAM_NS_BASE + 0x10c, 0x00004fe0);
    qtest_writel(qts, SEQRAM_NS_BASE + SEQ_SPARE0, 0);

    qtest_writel(qts, RAC_BASE + RAC_CTRL, RAC_CTRL_CPUWAITDIS);
    qtest_qmp_assert_success(qts, "{'execute':'cont'}");
    for (unsigned attempt = 0; attempt < 1000; attempt++) {
        if (qtest_readl(qts, SEQRAM_NS_BASE + SEQ_SPARE0) == 1) {
            break;
        }
        g_usleep(1000);
    }
    qtest_qmp_assert_success(qts, "{'execute':'stop'}");
    g_assert_cmphex(qtest_readl(qts, SEQRAM_NS_BASE + SEQ_SPARE0), ==, 1);

    qtest_writel(qts, RAC_BASE + RAC_CTRL,
                 RAC_CTRL_CPUWAITDIS | RAC_CTRL_BOARD_POLARITIES);
    g_assert_cmphex(qtest_readl(qts, RAC_BASE + RAC_CTRL), ==,
                    RAC_CTRL_CPUWAITDIS | RAC_CTRL_BOARD_POLARITIES);
    qtest_qmp_assert_success(qts, "{'execute':'cont'}");
    g_usleep(10000);
    qtest_qmp_assert_success(qts, "{'execute':'stop'}");
    g_assert_cmphex(qtest_readl(qts, SEQRAM_NS_BASE + SEQ_SPARE0), ==, 1);
    qtest_quit(qts);
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
    qtest_add_func("/efr32bg22/bgm220-module-identity",
                   test_bgm220_module_identity);
    qtest_add_func("/efr32bg22/burtc-counter-contract",
                   test_burtc_counter_contract);
    qtest_add_func("/efr32bg22/burtc-register-aliases",
                   test_burtc_register_aliases);
    qtest_add_func("/efr32bg22/burtc-compare-pending",
                   test_burtc_compare_pending);
    qtest_add_func("/efr32bg22/burtc-current-compare-waits-for-wrap",
                   test_burtc_current_compare_waits_for_wrap);
    qtest_add_func("/efr32bg22/timer4-counter-and-aliases",
                   test_timer4_counter_and_aliases);
    qtest_add_func("/efr32bg22/timer2-counter-and-aliases",
                   test_timer2_counter_and_aliases);
    qtest_add_func("/efr32bg22/timer4-cc0-compare-pending",
                   test_timer4_cc0_compare_pending);
    qtest_add_func("/efr32bg22/timer0-32bit-count-prescale-overflow",
                   test_timer0_32bit_count_prescale_overflow);
    qtest_add_func("/efr32bg22/timer0-cc-output-compare",
                   test_timer0_cc_output_compare);
    qtest_add_func("/efr32bg22/timer0-follows-em01grpaclk-dpll",
                   test_timer0_follows_em01grpaclk_dpll);
    qtest_add_func("/efr32bg22/system-reset-clears-radio-timer-state",
                   test_system_reset_clears_radio_timer_state);
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
    qtest_add_func("/efr32bg22/frc-status-is-read-only",
                   test_frc_status_is_read_only);
    qtest_add_func("/efr32bg22/rail-initialization-latches",
                   test_rail_initialization_latches);
    qtest_add_func("/efr32bg22/rac-sr3-clear-alias", test_rac_sr3_clear_alias);
    qtest_add_func("/efr32bg22/frc-interrupt-register-masks",
                   test_frc_interrupt_register_masks);
    qtest_add_func("/efr32bg22/frc-completion-interrupt-enable",
                   test_frc_completion_interrupt_enable);
    qtest_add_func("/efr32bg22/radio-enable-masks", test_radio_enable_masks);
    qtest_add_func("/efr32bg22/radio-enable-requires-bus-clocks",
                   test_radio_enable_requires_bus_clocks);
    qtest_add_func("/efr32bg22/radio-ipversions", test_radio_ipversions);
    qtest_add_func("/efr32bg22/radio-nonsecure-alias",
                   test_radio_nonsecure_alias);
    qtest_add_func("/efr32bg22/stock-setup-aliases",
                   test_stock_setup_aliases);
    qtest_add_func("/efr32bg22/rac-enable-requires-dependency-order",
                   test_rac_enable_requires_dependency_order);
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
    qtest_add_func("/efr32bg22/bufc-descriptor-ring-overflow-latch",
                   test_bufc_descriptor_ring_overflow_latch);
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
    qtest_add_func("/efr32bg22/bufc-descriptor-threshold-flag",
                   test_bufc_descriptor_threshold_flag);
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
    qtest_add_func("/efr32bg22/protimer-counter-is-independent-from-burtc",
                   test_protimer_counter_is_independent_from_burtc);
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
    qtest_add_func("/efr32bg22/rfmailbox-no-peer", test_rfmailbox_no_peer);
    qtest_add_func("/efr32bg22/rac-sr0-clear-alias",
                   test_rac_sr0_clear_alias);
    qtest_add_func("/efr32bg22/rac-force-disable", test_rac_force_disable);
    qtest_add_func("/efr32bg22/rac-stctrl-mask", test_rac_stctrl_mask);
    qtest_add_func("/efr32bg22/seq-nvic-ipr-byte-isolation",
                   test_seq_nvic_ipr_byte_isolation);
    qtest_add_func("/efr32bg22/seq-systick-counts", test_seq_systick_counts);
    qtest_add_func("/efr32bg22/radio-sequencer-executes-open-code",
                   test_radio_sequencer_executes_open_code);
    qtest_add_func("/efr32bg22/radio-sequencer-release-is-idempotent",
                   test_radio_sequencer_release_is_idempotent);

    result = g_test_run();
    qtest_end();
    g_unlink(tx_snapshot_trace_path);
    g_free(tx_snapshot_trace_path);
    return result;
}
