/*
 * EFR32xG22 EUART0 (RM rev. 1.2 chapter 22), a chardev-backed UART.
 *
 * Lines: "prs-txc", "prs-rxfl" (PRS producers EUART0 TXC/RXFL), "dma-req"
 * (0: TXFL, 1: RXFL, the LDMAXBAR requests), "prs-trigger" (PRS consumer
 * EUART0 TRIGGER).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_ARM_EFR32_EUSART_H
#define HW_ARM_EFR32_EUSART_H

#include "hw/irq.h"
#include "hw/arm/efr32-soc-link.h"

#define TYPE_EFR32_EUSART "efr32-eusart"
#define EFR32_EUART0_BASE 0x4a030000u
#define EFR32_EUART0_RX_IRQ 62
#define EFR32_EUART0_TX_IRQ 63
#define EFR32_EUSART_PRS_TXC "prs-txc"
#define EFR32_EUSART_PRS_RXFL "prs-rxfl"
#define EFR32_EUSART_DMA_REQ "dma-req"
#define EFR32_EUSART_PRS_TRIGGER "prs-trigger"
/* Input: the SoC is in EM2 (its EUSART0CLK may have stopped; see the .c). */
#define EFR32_EUSART_EM2 "em2"

DeviceState *efr32_eusart_create(Object *soc, MemoryRegion *memory,
                                 const Efr32SocLink *link, hwaddr base,
                                 hwaddr ns_offset, qemu_irq rx_irq,
                                 qemu_irq tx_irq);

#endif
