/*
 * QTest for the efr32bg22 bus and pin peripherals with the OWO suit board
 * attached (-machine efr32bg22,board=owo-suit): GPIO, USART synchronous
 * main mode on the board's switch chain, I2C0 on the board's expander,
 * TIMER1 PWM routed to a pin, the IADC scan queue on the board's battery
 * net, and the WDOG0 reset.
 *
 * Register offsets and fields: the public EFR32BG22 device headers
 * (efr32bg22_gpio.h, _usart.h, _i2c.h, _timer.h, _iadc.h, _wdog.h, _emu.h).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest-single.h"

#define SET 0x1000u
#define CLR 0x2000u

#define CMU_BASE 0x40008000u
#define CMU_CLKEN0 0x064u
#define CMU_CLKEN0_TIMERS (0xfu << 4)

#define GPIO_BASE 0x4003c000u
#define GPIO_PORT(p) (GPIO_BASE + 0x30u * (p))
#define GPIO_MODEL 0x04u
#define GPIO_DOUT 0x10u
#define GPIO_DIN 0x14u
#define GPIO_EXTIPSELL 0x400u
#define GPIO_EXTIPINSELL 0x408u
#define GPIO_EXTIFALL 0x414u
#define GPIO_IF 0x420u
#define GPIO_TIMER1_ROUTEEN 0x59cu
#define GPIO_TIMER1_CC0ROUTE 0x5a0u
#define GPIO_MODE_PUSHPULL 4u
#define GPIO_MODE_INPUT 1u
#define PORT_A 0
#define PORT_C 2
#define PORT_D 3

#define USART0_BASE 0x4005c000u
#define USART_EN 0x04u
#define USART_CTRL 0x08u
#define USART_CTRL_SYNC (1u << 0)
#define USART_CTRL_MSBF (1u << 10)
#define USART_CMD 0x14u
#define USART_CMD_RXEN (1u << 0)
#define USART_CMD_TXEN (1u << 2)
#define USART_CMD_MASTEREN (1u << 4)
#define USART_STATUS 0x18u
#define USART_STATUS_MASTER (1u << 2)
#define USART_STATUS_RXDATAV (1u << 7)
#define USART_RXDATA 0x24u
#define USART_TXDATA 0x3cu

#define I2C0_BASE 0x4a010000u
#define I2C_EN 0x04u
#define I2C_CMD 0x0cu
#define I2C_CMD_START (1u << 0)
#define I2C_CMD_STOP (1u << 1)
#define I2C_CMD_ACK (1u << 2)
#define I2C_CMD_NACK (1u << 3)
#define I2C_CMD_ABORT (1u << 5)
#define I2C_RXDATA 0x24u
#define I2C_TXDATA 0x34u
#define I2C_IF 0x3cu
#define I2C_IF_ACK (1u << 6)
#define I2C_IF_NACK (1u << 7)
#define I2C_IF_MSTOP (1u << 8)
#define I2C_IF_RXDATAV (1u << 5)
#define EXPANDER_WRITE 0x40u
#define EXPANDER_READ 0x41u
#define ABSENT_WRITE 0x44u

#define TIMER1_BASE 0x4004c000u
#define TIMER_CMD 0x0cu
#define TIMER_CMD_START (1u << 0)
#define TIMER_TOP 0x1cu
#define TIMER_CNT 0x24u
#define TIMER_EN 0x30u
#define TIMER_CC0_CFG 0x60u
#define TIMER_CC0_OC 0x68u
#define TIMER_CC_MODE_PWM 3u

#define IADC_BASE 0x4a004000u
#define IADC_EN 0x04u
#define IADC_CMD 0x0cu
#define IADC_CMD_SCANSTART (1u << 3)
#define IADC_MASKREQ 0x18u
#define IADC_IF 0x24u
#define IADC_IF_SCANTABLEDONE (1u << 8)
#define IADC_CFG0 0x48u
#define IADC_CFG_GAIN_X1 (2u << 12)
#define IADC_CFG_REF_VDDX (3u << 16)
#define IADC_SCANFIFOCFG 0x80u
#define IADC_SCANFIFODATA 0x84u
#define IADC_SCANTABLE0 0xa0u
#define IADC_PORT_D 11u

#define WDOG0_BASE 0x4a018000u
#define WDOG_EN 0x04u
#define WDOG_CFG 0x08u
#define WDOG_CFG_EM2RUN (1u << 1)
#define WDOG_CFG_EM3RUN (1u << 2)
#define WDOG_CFG_PERSEL(n) ((uint32_t)(n) << 16)
#define WDOG_CFG_WARNSEL(n) ((uint32_t)(n) << 24)
#define WDOG_CMD 0x0cu
#define WDOG_CMD_CLEAR (1u << 0)
#define WDOG_STATUS 0x14u
#define WDOG_STATUS_LOCK (1u << 31)
#define WDOG_IF 0x18u
#define WDOG_IF_WARN (1u << 1)
#define WDOG_LOCK 0x20u
#define WDOG_SYNCBUSY 0x24u
#define WDOG_SYNCBUSY_CMD (1u << 0)
/* RM SS27.3.1: 2^(PERSEL + 3) + 1 cycles; SS4.2.4.4.4: LFSYNC 3-4 cycles. */
#define WDOG_PERSEL_BIAS 3
#define WDOG_LFSYNC_CYCLES 4
/* CMU.WDOG0CLKCTRL resets to the LFRCO (RM SS8.5.21): 32768 Hz. */
#define WDOG_LF_HZ 32768u
#define WDOG_NS_PER_S UINT64_C(1000000000)
#define EMU_CMD 0x40004070u
#define CMU_STATUS 0x008u
#define CMU_STATUS_WDOGLOCK (1u << 30)
#define CMU_WDOGLOCK 0x014u
#define CMU_WDOG0CLKCTRL 0x200u
#define CMU_WDOG0CLKCTRL_LFRCO 1u
#define CMU_WDOG0CLKCTRL_ULFRCO 3u
#define EMU_CMD_RSTCAUSECLR (1u << 17)
#define EMU_RSTCAUSE 0x40004094u
#define EMU_RSTCAUSE_WDOG0 (1u << 3)

/* Board nominals (hw/misc/owo-suit.c): 7600 mV through 33/133. */
#define BOARD_BATTERY_MV 7600
#define BOARD_DIV_NUM 33
#define BOARD_DIV_DEN 133
#define VDDX_MV 3300
#define IADC_FULL 4095

static void clock_run(int64_t ns)
{
    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    qtest_clock_step(global_qtest, ns);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");
}

static void reset(void)
{
    qtest_qmp_assert_success(global_qtest, "{'execute':'system_reset'}");
    writel(CMU_BASE + CMU_CLKEN0 + SET, CMU_CLKEN0_TIMERS);
}

static void pin_mode(unsigned port, unsigned pin, unsigned mode)
{
    uint32_t a = GPIO_PORT(port) + (pin < 8 ? GPIO_MODEL : GPIO_MODEL + 8);
    unsigned shift = (pin % 8) * 4;

    writel(a, (readl(a) & ~(0xfu << shift)) | (mode << shift));
}

static void pin_out(unsigned port, unsigned pin, bool level)
{
    writel(GPIO_PORT(port) + GPIO_DOUT + (level ? SET : CLR), 1u << pin);
}

/* A push-pull pin reads back what it drives; the expander's /INT on PC4. */
static void test_gpio_drive_and_exti(void)
{
    reset();
    pin_mode(PORT_A, 5, GPIO_MODE_PUSHPULL);
    pin_out(PORT_A, 5, 1);
    g_assert_cmphex(readl(GPIO_PORT(PORT_A) + GPIO_DIN) & (1u << 5), ==,
                    1u << 5);
    pin_out(PORT_A, 5, 0);
    g_assert_cmphex(readl(GPIO_PORT(PORT_A) + GPIO_DIN) & (1u << 5), ==, 0);

    /* EXTI4 on port C (pin 4 = base 4 + offset 0), falling edge. */
    pin_mode(PORT_C, 4, GPIO_MODE_INPUT);
    writel(GPIO_BASE + GPIO_EXTIPSELL, PORT_C << 16);
    writel(GPIO_BASE + GPIO_EXTIPINSELL, 0);
    writel(GPIO_BASE + GPIO_EXTIFALL, 1u << 4);
    g_assert_cmphex(readl(GPIO_PORT(PORT_C) + GPIO_DIN) & (1u << 4), ==,
                    1u << 4);
    qtest_qmp_assert_success(global_qtest, "{'execute':'qom-set','arguments':"
                             "{'path':'/machine/suit','property':"
                             "'expander-ext-low','value':1}}");
    clock_run(1000);
    g_assert_cmphex(readl(GPIO_PORT(PORT_C) + GPIO_DIN) & (1u << 4), ==, 0);
    g_assert_cmphex(readl(GPIO_BASE + GPIO_IF) & (1u << 4), ==, 1u << 4);
    qtest_qmp_assert_success(global_qtest, "{'execute':'qom-set','arguments':"
                             "{'path':'/machine/suit','property':"
                             "'expander-ext-low','value':0}}");
}

static uint8_t spi_byte(uint8_t tx)
{
    writel(USART0_BASE + USART_TXDATA, tx);
    clock_run(100000);
    g_assert_cmphex(readl(USART0_BASE + USART_STATUS) & USART_STATUS_RXDATAV,
                    ==, USART_STATUS_RXDATAV);
    return readl(USART0_BASE + USART_RXDATA);
}

/* 32 bits into the switch chain, latched on PC3, then clocked back out. */
static void test_usart_spi_switch_readback(void)
{
    static const uint8_t word[] = { 0xa5, 0x5a, 0x01, 0x80 };

    reset();
    pin_mode(PORT_C, 3, GPIO_MODE_PUSHPULL);
    pin_out(PORT_C, 3, 0);
    writel(USART0_BASE + USART_EN, 1);
    writel(USART0_BASE + USART_CTRL, USART_CTRL_SYNC | USART_CTRL_MSBF);
    writel(USART0_BASE + USART_CMD,
           USART_CMD_RXEN | USART_CMD_TXEN | USART_CMD_MASTEREN);
    g_assert_cmphex(readl(USART0_BASE + USART_STATUS) & USART_STATUS_MASTER,
                    ==, USART_STATUS_MASTER);
    for (unsigned i = 0; i < sizeof(word); i++) {
        spi_byte(word[i]);
    }
    pin_out(PORT_C, 3, 1);
    for (unsigned i = 0; i < sizeof(word); i++) {
        g_assert_cmphex(spi_byte(0), ==, word[i]);
    }
}

static uint32_t i2c_step(void)
{
    uint32_t flags;

    clock_run(2000000);
    flags = readl(I2C0_BASE + I2C_IF);
    writel(I2C0_BASE + I2C_IF + CLR, flags);
    return flags;
}

/* Write the expander's two latch bytes, read the pins back; NACK absent. */
static void test_i2c_expander(void)
{
    reset();
    writel(I2C0_BASE + I2C_EN, 1);
    writel(I2C0_BASE + I2C_CMD, I2C_CMD_ABORT);
    writel(I2C0_BASE + I2C_TXDATA, EXPANDER_WRITE);
    writel(I2C0_BASE + I2C_CMD, I2C_CMD_START);
    g_assert_cmphex(i2c_step() & I2C_IF_ACK, ==, I2C_IF_ACK);
    writel(I2C0_BASE + I2C_TXDATA, 0xf0);
    writel(I2C0_BASE + I2C_TXDATA, 0x0f);
    g_assert_cmphex(i2c_step() & I2C_IF_ACK, ==, I2C_IF_ACK);
    writel(I2C0_BASE + I2C_CMD, I2C_CMD_STOP);
    g_assert_cmphex(i2c_step() & I2C_IF_MSTOP, ==, I2C_IF_MSTOP);

    writel(I2C0_BASE + I2C_TXDATA, EXPANDER_READ);
    writel(I2C0_BASE + I2C_CMD, I2C_CMD_START);
    g_assert_cmphex(i2c_step() & I2C_IF_RXDATAV, ==, I2C_IF_RXDATAV);
    g_assert_cmphex(readl(I2C0_BASE + I2C_RXDATA), ==, 0xf0);
    writel(I2C0_BASE + I2C_CMD, I2C_CMD_ACK);
    g_assert_cmphex(i2c_step() & I2C_IF_RXDATAV, ==, I2C_IF_RXDATAV);
    g_assert_cmphex(readl(I2C0_BASE + I2C_RXDATA), ==, 0x0f);
    writel(I2C0_BASE + I2C_CMD, I2C_CMD_NACK | I2C_CMD_STOP);
    g_assert_cmphex(i2c_step() & I2C_IF_MSTOP, ==, I2C_IF_MSTOP);

    writel(I2C0_BASE + I2C_TXDATA, ABSENT_WRITE);
    writel(I2C0_BASE + I2C_CMD, I2C_CMD_START);
    g_assert_cmphex(i2c_step() & I2C_IF_NACK, ==, I2C_IF_NACK);
    writel(I2C0_BASE + I2C_CMD, I2C_CMD_STOP);
    g_assert_cmphex(i2c_step() & I2C_IF_MSTOP, ==, I2C_IF_MSTOP);
}

/* Up-count PWM on PC7: high while CNT < OC (RM SS19.3.3.9). */
static void test_timer_pwm_pin(void)
{
    const uint32_t top = 999, oc = 500;
    unsigned high = 0, samples = 0;

    reset();
    writel(TIMER1_BASE + TIMER_CC0_CFG, TIMER_CC_MODE_PWM);
    writel(TIMER1_BASE + TIMER_EN, 1);
    writel(TIMER1_BASE + TIMER_TOP, top);
    writel(TIMER1_BASE + TIMER_CC0_OC, oc);
    writel(GPIO_BASE + GPIO_TIMER1_CC0ROUTE, PORT_C | (7u << 16));
    writel(GPIO_BASE + GPIO_TIMER1_ROUTEEN, 1);
    pin_mode(PORT_C, 7, GPIO_MODE_PUSHPULL);
    writel(TIMER1_BASE + TIMER_CMD, TIMER_CMD_START);
    for (unsigned i = 0; i < 40; i++) {
        uint32_t cnt, din;

        clock_run(3700 + 31 * i);
        cnt = readl(TIMER1_BASE + TIMER_CNT);
        din = readl(GPIO_PORT(PORT_C) + GPIO_DIN) & (1u << 7);
        if (cnt == oc || cnt == 0) {
            continue; /* an edge falls on this sample */
        }
        g_assert_cmpuint(!!din, ==, cnt < oc);
        high += !!din;
        samples++;
    }
    g_assert_cmpuint(high, >, 0);
    g_assert_cmpuint(high, <, samples);
}

/* One scan entry on PD3, the board's battery divider. */
static void test_iadc_scan_board_battery(void)
{
    uint32_t mv = BOARD_BATTERY_MV * BOARD_DIV_NUM / BOARD_DIV_DEN;
    uint32_t expect = (mv * 2 * IADC_FULL + VDDX_MV) / (2 * VDDX_MV);
    uint32_t data;

    reset();
    writel(IADC_BASE + IADC_EN, 1);
    writel(IADC_BASE + IADC_CFG0, IADC_CFG_GAIN_X1 | IADC_CFG_REF_VDDX);
    writel(IADC_BASE + IADC_SCANTABLE0, (IADC_PORT_D << 12) | (3u << 8));
    writel(IADC_BASE + IADC_MASKREQ, 1);
    writel(IADC_BASE + IADC_SCANFIFOCFG, 0);
    writel(IADC_BASE + IADC_CMD, IADC_CMD_SCANSTART);
    g_assert_cmphex(readl(IADC_BASE + IADC_IF) & IADC_IF_SCANTABLEDONE, ==,
                    IADC_IF_SCANTABLEDONE);
    data = readl(IADC_BASE + IADC_SCANFIFODATA);
    g_assert_cmpuint(data, >=, expect - 1);
    g_assert_cmpuint(data, <=, expect + 1);
}

/* PERSEL 0: 9 LFRCO cycles, then a reset with RSTCAUSE.WDOG0 (RM 27.3). */
static void test_wdog_reset(void)
{
    reset();
    writel(EMU_RSTCAUSE - 0x94u + 0x70u, 1u << 17); /* EMU_CMD.RSTCAUSECLR */
    writel(WDOG0_BASE + WDOG_CFG, 0);
    writel(WDOG0_BASE + WDOG_EN, 1);
    g_assert_cmphex(readl(WDOG0_BASE + WDOG_EN), ==, 1);
    clock_run(1000000);
    g_assert_cmphex(readl(EMU_RSTCAUSE) & EMU_RSTCAUSE_WDOG0, ==,
                    EMU_RSTCAUSE_WDOG0);
    g_assert_cmphex(readl(WDOG0_BASE + WDOG_EN), ==, 0);
}

/* Virtual nanoseconds of @cycles watchdog-clock cycles, rounded up. */
static int64_t wdog_ns(uint64_t cycles)
{
    return (cycles * WDOG_NS_PER_S + WDOG_LF_HZ - 1) / WDOG_LF_HZ;
}

/*
 * The product's configuration (owo-open-fw docs/watchdog.md): CFG is a
 * CONFIG register (RM SS4.2.4.4.2), so a write while enabled is a bus error
 * and keeps the value; LOCK (SS27.5.8) keeps EN set; IF.WARN at WARNSEL x
 * 25 % (SS27.3.4) stays set across a clear (INTFLAG); CMD is LFSYNC
 * (SS4.2.4.4.4): SYNCBUSY.CMD until it completes, the counter restarts then.
 */
static void test_wdog_config_lock_warning_clear(void)
{
    const unsigned persel = 8, warnsel = 3;
    const uint32_t cfg = WDOG_CFG_PERSEL(persel) | WDOG_CFG_WARNSEL(warnsel) |
                         WDOG_CFG_EM2RUN | WDOG_CFG_EM3RUN;
    const uint64_t period = (UINT64_C(1) << (persel + WDOG_PERSEL_BIAS)) + 1;
    const uint64_t warn = period * warnsel / 4;

    reset();
    writel(EMU_CMD, EMU_CMD_RSTCAUSECLR);
    writel(WDOG0_BASE + WDOG_CFG, cfg);
    writel(WDOG0_BASE + WDOG_EN, 1);
    writel(WDOG0_BASE + WDOG_CFG, 0);
    g_assert_cmphex(readl(WDOG0_BASE + WDOG_CFG), ==, cfg);
    writel(WDOG0_BASE + WDOG_LOCK, 0);
    g_assert_cmphex(readl(WDOG0_BASE + WDOG_STATUS), ==, WDOG_STATUS_LOCK);
    writel(WDOG0_BASE + WDOG_EN, 0);
    g_assert_cmphex(readl(WDOG0_BASE + WDOG_EN), ==, 1);
    /* CMU_WDOGLOCK (RM SS8.5.4) freezes the clock selection. */
    writel(CMU_BASE + CMU_WDOGLOCK, 0);
    g_assert_cmphex(readl(CMU_BASE + CMU_STATUS) & CMU_STATUS_WDOGLOCK, ==,
                    CMU_STATUS_WDOGLOCK);
    writel(CMU_BASE + CMU_WDOG0CLKCTRL, CMU_WDOG0CLKCTRL_ULFRCO);
    g_assert_cmphex(readl(CMU_BASE + CMU_WDOG0CLKCTRL), ==,
                    CMU_WDOG0CLKCTRL_LFRCO);

    clock_run(wdog_ns(warn) - wdog_ns(1));
    g_assert_cmphex(readl(WDOG0_BASE + WDOG_IF) & WDOG_IF_WARN, ==, 0);
    clock_run(wdog_ns(2));
    g_assert_cmphex(readl(WDOG0_BASE + WDOG_IF) & WDOG_IF_WARN, ==,
                    WDOG_IF_WARN);

    writel(WDOG0_BASE + WDOG_CMD, WDOG_CMD_CLEAR);
    g_assert_cmphex(readl(WDOG0_BASE + WDOG_SYNCBUSY), ==, WDOG_SYNCBUSY_CMD);
    clock_run(wdog_ns(WDOG_LFSYNC_CYCLES));
    g_assert_cmphex(readl(WDOG0_BASE + WDOG_SYNCBUSY), ==, 0);
    g_assert_cmphex(readl(WDOG0_BASE + WDOG_IF) & WDOG_IF_WARN, ==,
                    WDOG_IF_WARN);
    /* Past the first period's end: the clear restarted the count. */
    clock_run(wdog_ns(period - warn));
    g_assert_cmphex(readl(EMU_RSTCAUSE) & EMU_RSTCAUSE_WDOG0, ==, 0);
    /* One period after the clear: reset, which also unlocks and disables. */
    clock_run(wdog_ns(warn + WDOG_LFSYNC_CYCLES));
    g_assert_cmphex(readl(EMU_RSTCAUSE) & EMU_RSTCAUSE_WDOG0, ==,
                    EMU_RSTCAUSE_WDOG0);
    g_assert_cmphex(readl(WDOG0_BASE + WDOG_EN), ==, 0);
    g_assert_cmphex(readl(WDOG0_BASE + WDOG_STATUS), ==, 0);
    g_assert_cmphex(readl(CMU_BASE + CMU_STATUS) & CMU_STATUS_WDOGLOCK, ==, 0);
}

int main(int argc, char **argv)
{
    int result;

    g_test_init(&argc, &argv, NULL);
    qtest_start("-machine efr32bg22,board=owo-suit -S");
    qtest_add_func("/efr32bg22-board/gpio-drive-and-exti",
                   test_gpio_drive_and_exti);
    qtest_add_func("/efr32bg22-board/usart-spi-switch-readback",
                   test_usart_spi_switch_readback);
    qtest_add_func("/efr32bg22-board/i2c-expander", test_i2c_expander);
    qtest_add_func("/efr32bg22-board/timer-pwm-pin", test_timer_pwm_pin);
    qtest_add_func("/efr32bg22-board/iadc-scan-board-battery",
                   test_iadc_scan_board_battery);
    qtest_add_func("/efr32bg22-board/wdog-reset", test_wdog_reset);
    qtest_add_func("/efr32bg22-board/wdog-config-lock-warning-clear",
                   test_wdog_config_lock_warning_clear);
    result = g_test_run();
    qtest_end();
    return result;
}
