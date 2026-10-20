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
#define RAC_CMD_TXDIS            (1u << 5)
#define RAC_CMD_RXDIS            (1u << 8)
#define RAC_RXENSRCEN_SWRXEN     (1u << 1)
#define RAC_STATUS               0x00c
#define RAC_STATUS_STATE_MASK     0x0f000000
#define RAC_STATUS_STATE_RXSEARCH (2u << 24)
#define BUFC_BASE                0xaa000000
#define BUFC_CH0                 0x00c
#define BUFC_CH1                 0x04c
#define BUFC_ADDR                0x004
#define BUFC_WRITEOFFSET         0x008
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
 * The test board intentionally gives both SoCs RAM at 0x20000000.  BUFC is a
 * bus master, so its guest backing store must resolve through the owning
 * SoC's address space, not the global qtest map (which is SoC 0's map).
 */
static void test_peer_bufc_dma_uses_peer_address_space(void)
{
    const uint32_t shared_guest_address = 0x20000100;
    const uint64_t peer_tx_channel = peer(BUFC_BASE + BUFC_CH0);

    qtest_writeb(global_qtest, shared_guest_address, 0xa5);
    qtest_writel(global_qtest, peer_tx_channel + BUFC_CMD, 1);
    qtest_writel(global_qtest, peer_tx_channel + BUFC_ADDR,
                 shared_guest_address);
    qtest_writel(global_qtest, peer_tx_channel, 0); /* 64-byte peer FIFO */
    qtest_writel(global_qtest, peer_tx_channel + BUFC_WRITEDATA, 0x5a);

    g_assert_cmphex(qtest_readb(global_qtest, shared_guest_address), ==, 0xa5);
    qtest_writel(global_qtest, peer_tx_channel + BUFC_READOFFSET, 0);
    g_assert_cmphex(qtest_readl(global_qtest,
                                peer_tx_channel + BUFC_READDATA), ==, 0x5a);
}

/*
 * This fixture has two native EFR32 SoCs.  qtest only performs their normal
 * MMIO staging/configuration: the source's guest-backed BUFC channel 0 holds
 * the TX bytes and the receiver is placed in RXSEARCH with an independent,
 * same-address guest-backed BUFC FIFO. No QMP, host Bluetooth, or packet
 * injection path exists in this test.
 */
static void test_tx_offer_reaches_peer_rx_fifo(void)
{
    const uint8_t payload[] = { 0x91, 0x72, 0xe3 };
    const uint32_t fifo_guest_address = 0x20000180;
    const uint64_t peer_rx_channel = peer(BUFC_BASE + BUFC_CH1);
    const uint32_t source_tx_channel = BUFC_BASE + BUFC_CH0;

    qtest_writel(global_qtest, peer(FRC_BASE + FRC_IF + REGISTER_CLEAR_WINDOW),
                 FRC_IF_RXDONE);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_CMD, 1);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_ADDR, fifo_guest_address);
    qtest_writel(global_qtest, peer_rx_channel, 0); /* 64-byte peer RX FIFO */
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
        qtest_writeb(global_qtest, fifo_guest_address + index, payload[index]);
    }
    qtest_writel(global_qtest, source_tx_channel + BUFC_ADDR, fifo_guest_address);
    qtest_writel(global_qtest, source_tx_channel, 0); /* 64-byte source TX FIFO */
    qtest_writel(global_qtest, source_tx_channel + BUFC_WRITEOFFSET,
                 G_N_ELEMENTS(payload));
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

/*
 * RX offers are deferred out of RAC.CMD.TXEN.  If the receiver leaves
 * RXSEARCH before that virtual-clock callback runs, it must not observe a
 * spurious RXDONE or partially written BUFC channel.  The offer is retained
 * and is delivered only after the normal SWRXEN path enters RXSEARCH again.
 */
static void test_tx_offer_waits_for_receiver_rxsearch(void)
{
    const uint8_t payload[] = { 0x4d, 0xb2 };
    const uint32_t fifo_guest_address = 0x20000200;
    const uint64_t peer_rx_channel = peer(BUFC_BASE + BUFC_CH1);
    const uint32_t source_tx_channel = BUFC_BASE + BUFC_CH0;

    qtest_writel(global_qtest, peer(FRC_BASE + FRC_IF + REGISTER_CLEAR_WINDOW),
                 FRC_IF_RXDONE);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_CMD, 1);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_ADDR, fifo_guest_address);
    qtest_writel(global_qtest, peer_rx_channel, 0); /* 64-byte peer RX FIFO */
    qtest_writel(global_qtest,
                 peer(RAC_BASE + RAC_RXENSRCEN + REGISTER_SET_WINDOW),
                 RAC_RXENSRCEN_SWRXEN);

    qtest_writel(global_qtest, source_tx_channel + BUFC_CMD, 1);
    for (size_t index = 0; index < G_N_ELEMENTS(payload); index++) {
        qtest_writeb(global_qtest, fifo_guest_address + index, payload[index]);
    }
    qtest_writel(global_qtest, source_tx_channel + BUFC_ADDR, fifo_guest_address);
    qtest_writel(global_qtest, source_tx_channel, 0); /* 64-byte source TX FIFO */
    qtest_writel(global_qtest, source_tx_channel + BUFC_WRITEOFFSET,
                 G_N_ELEMENTS(payload));
    qtest_writel(global_qtest, RAC_BASE + RAC_CMD, RAC_CMD_TXEN);

    /* Leave RXSEARCH before the deferred handoff gets a virtual-clock turn. */
    qtest_writel(global_qtest, peer(RAC_BASE + RAC_CMD), RAC_CMD_RXDIS);
    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(1);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");

    g_assert_cmphex(qtest_readl(global_qtest, peer(FRC_BASE + FRC_IF)) &
                        FRC_IF_RXDONE,
                    ==, 0);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_READOFFSET, 0);
    g_assert_cmphex(qtest_readl(global_qtest,
                                peer_rx_channel + BUFC_READDATA),
                    ==, 0);

    qtest_writel(global_qtest,
                 peer(RAC_BASE + RAC_RXENSRCEN + REGISTER_SET_WINDOW),
                 RAC_RXENSRCEN_SWRXEN);
    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(1);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");

    g_assert_cmphex(qtest_readl(global_qtest, peer(FRC_BASE + FRC_IF)) &
                        FRC_IF_RXDONE,
                    ==, FRC_IF_RXDONE);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_READOFFSET, 0);
    for (size_t index = 0; index < G_N_ELEMENTS(payload); index++) {
        g_assert_cmphex(qtest_readl(global_qtest,
                                    peer_rx_channel + BUFC_READDATA),
                        ==, payload[index]);
    }
}

/*
 * TXEN snapshots staged BUFC data, but a subsequent TXDIS cancels that model
 * transaction.  In particular, an offer already deferred for a peer must not
 * leak through when the peer later re-enters RXSEARCH.
 */
static void test_tx_disable_withdraws_deferred_offer(void)
{
    const uint8_t payload[] = { 0x6e, 0x51 };
    const uint32_t fifo_guest_address = 0x20000280;
    const uint64_t peer_rx_channel = peer(BUFC_BASE + BUFC_CH1);
    const uint32_t source_tx_channel = BUFC_BASE + BUFC_CH0;

    qtest_writel(global_qtest, peer(FRC_BASE + FRC_IF + REGISTER_CLEAR_WINDOW),
                 FRC_IF_RXDONE);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_CMD, 1);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_ADDR, fifo_guest_address);
    qtest_writel(global_qtest, peer_rx_channel, 0);
    qtest_writel(global_qtest,
                 peer(RAC_BASE + RAC_RXENSRCEN + REGISTER_SET_WINDOW),
                 RAC_RXENSRCEN_SWRXEN);

    qtest_writel(global_qtest, source_tx_channel + BUFC_CMD, 1);
    for (size_t index = 0; index < G_N_ELEMENTS(payload); index++) {
        qtest_writeb(global_qtest, fifo_guest_address + index, payload[index]);
    }
    qtest_writel(global_qtest, source_tx_channel + BUFC_ADDR, fifo_guest_address);
    qtest_writel(global_qtest, source_tx_channel, 0);
    qtest_writel(global_qtest, source_tx_channel + BUFC_WRITEOFFSET,
                 G_N_ELEMENTS(payload));
    qtest_writel(global_qtest, RAC_BASE + RAC_CMD, RAC_CMD_TXEN);
    qtest_writel(global_qtest, RAC_BASE + RAC_CMD, RAC_CMD_TXDIS);

    qtest_qmp_assert_success(global_qtest, "{'execute':'cont'}");
    clock_step(1);
    qtest_qmp_assert_success(global_qtest, "{'execute':'stop'}");

    g_assert_cmphex(qtest_readl(global_qtest, peer(FRC_BASE + FRC_IF)) &
                        FRC_IF_RXDONE,
                    ==, 0);
    qtest_writel(global_qtest, peer_rx_channel + BUFC_READOFFSET, 0);
    g_assert_cmphex(qtest_readl(global_qtest,
                                peer_rx_channel + BUFC_READDATA), ==, 0);
}

int main(int argc, char **argv)
{
    int result;

    g_test_init(&argc, &argv, NULL);
    qtest_start("-machine efr32bg22-dual -S");
    qtest_add_func("/efr32bg22/rf/peer-bufc-dma-address-space",
                   test_peer_bufc_dma_uses_peer_address_space);
    qtest_add_func("/efr32bg22/rf/tx-offer-reaches-peer-rx-fifo",
                   test_tx_offer_reaches_peer_rx_fifo);
    qtest_add_func("/efr32bg22/rf/tx-offer-waits-for-receiver-rxsearch",
                   test_tx_offer_waits_for_receiver_rxsearch);
    qtest_add_func("/efr32bg22/rf/tx-disable-withdraws-deferred-offer",
                   test_tx_disable_withdraws_deferred_offer);
    result = g_test_run();
    qtest_end();
    return result;
}
