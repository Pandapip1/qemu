/*
 * End-to-end QTest coverage for the EFR32BG22 in-process RF medium.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "libqtest-single.h"

#define EFR32BG22_PEER_PERIPH_BASE 0x60000000
#define EFR32BG22_PEER_RADIO_BASE  0x80000000

#define RADIO_BASE               0xa8000000
#define FRC_BASE                 0xa8004000
#define FRC_IF                   0x068
#define FRC_RXRAWDATA            0x090
#define FRC_IF_RXDONE            (1u << 4)
#define RAC_BASE                 0xa8020000
#define RAC_CMD                  0x010
#define RAC_RXENSRCEN            0x008
#define RAC_CMD_TXEN             (1u << 0)
#define RAC_RXENSRCEN_SWRXEN     (1u << 1)
#define RAC_STATUS               0x00c
#define RAC_STATUS_STATE_MASK     0x0f000000
#define RAC_STATUS_STATE_RXSEARCH (2u << 24)
#define BUFC_BASE                0xaa000000
#define BUFC_CH0                 0x00c
#define BUFC_CH1                 0x04c
#define BUFC_READOFFSET          0x00c
#define BUFC_READDATA            0x014
#define BUFC_WRITEDATA           0x018
#define BUFC_CMD                 0x028
#define REGISTER_SET_WINDOW      0x1000
#define REGISTER_CLEAR_WINDOW    0x2000

static uint64_t peer(uint32_t address)
{
    if (address >= FRC_BASE) {
        return EFR32BG22_PEER_RADIO_BASE + (address - RADIO_BASE);
    }
    return EFR32BG22_PEER_PERIPH_BASE + (address - 0x40000000);
}

/*
 * This fixture has two native EFR32 SoCs.  qtest only performs their normal
 * MMIO staging/configuration: TX bytes enter the source's BUFC channel 0 and
 * the receiver is placed in RXSEARCH.  No QMP, host Bluetooth, or packet
 * injection path exists in this test.
 */
static void test_tx_offer_reaches_peer_rx_fifo(void)
{
    const uint8_t payload[] = { 0x91, 0x72, 0xe3 };
    const uint64_t peer_rx_channel = peer(BUFC_BASE + BUFC_CH1);
    const uint32_t source_tx_channel = BUFC_BASE + BUFC_CH0;

    qtest_writel(global_qtest, peer(FRC_BASE + FRC_IF + REGISTER_CLEAR_WINDOW),
                 FRC_IF_RXDONE);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_CMD, 1);
    qtest_writel(global_qtest, peer_rx_channel, 3); /* 512-byte peer RX FIFO */
    qtest_writel(global_qtest,
                 peer(RAC_BASE + RAC_RXENSRCEN + REGISTER_SET_WINDOW),
                 RAC_RXENSRCEN_SWRXEN);
    g_assert_cmphex(qtest_readl(global_qtest, peer(RAC_BASE + RAC_RXENSRCEN)) &
                        RAC_RXENSRCEN_SWRXEN,
                    ==, RAC_RXENSRCEN_SWRXEN);
    g_assert_cmphex(qtest_readl(global_qtest, peer(RAC_BASE + RAC_STATUS)) &
                        RAC_STATUS_STATE_MASK,
                    ==, RAC_STATUS_STATE_RXSEARCH);

    qtest_writel(global_qtest, source_tx_channel + BUFC_CMD, 1);
    for (size_t index = 0; index < G_N_ELEMENTS(payload); index++) {
        qtest_writel(global_qtest, source_tx_channel + BUFC_WRITEDATA,
                     payload[index]);
    }
    qtest_writel(global_qtest, RAC_BASE + RAC_CMD, RAC_CMD_TXEN);

    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(1);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");

    g_assert_cmphex(qtest_readl(global_qtest, peer(FRC_BASE + FRC_IF)) &
                        FRC_IF_RXDONE,
                    ==, FRC_IF_RXDONE);
    g_assert_cmphex(qtest_readl(global_qtest, peer(FRC_BASE + FRC_RXRAWDATA)),
                    ==, payload[G_N_ELEMENTS(payload) - 1]);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_READOFFSET, 0);
    for (size_t index = 0; index < G_N_ELEMENTS(payload); index++) {
        g_assert_cmphex(qtest_readl(global_qtest,
                                    peer_rx_channel + BUFC_READDATA),
                        ==, payload[index]);
    }
}

int main(int argc, char **argv)
{
    int result;

    g_test_init(&argc, &argv, NULL);
    qtest_start("-machine efr32bg22-dual -S");
    qtest_add_func("/efr32bg22/rf/tx-offer-reaches-peer-rx-fifo",
                   test_tx_offer_reaches_peer_rx_fifo);
    result = g_test_run();
    qtest_end();
    return result;
}
