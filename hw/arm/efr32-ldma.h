/*
 * EFR32xG22 LDMA and LDMAXBAR (RM rev. 1.2 chapter 26).
 *
 * "req" input line SOURCESEL * EFR32_LDMA_SIGNALS + SIGSEL carries the
 * request LDMAXBAR CHn_REQSEL selects with those fields (efr32bg22_ldmaxbar
 * _defines.h); "prs-sync" are PRS channels 0-7 for SYNCHWEN.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_ARM_EFR32_LDMA_H
#define HW_ARM_EFR32_LDMA_H

#include "hw/irq.h"
#include "hw/arm/efr32-soc-link.h"

#define TYPE_EFR32_LDMA "efr32-ldma"
#define EFR32_LDMA_BASE 0x40040000u
#define EFR32_LDMAXBAR_BASE 0x40044000u
#define EFR32_LDMA_IRQ 21
#define EFR32_LDMA_SIGNALS 16
#define EFR32_LDMA_SOURCES 0x12
#define EFR32_LDMA_REQ_LINES (EFR32_LDMA_SOURCES * EFR32_LDMA_SIGNALS)
#define EFR32_LDMA_REQ_LINE(source, signal) \
    ((source) * EFR32_LDMA_SIGNALS + (signal))
#define EFR32_LDMA_REQ "req"
#define EFR32_LDMA_PRS_SYNC "prs-sync"

/* LDMAXBAR SOURCESEL values. */
enum {
    EFR32_LDMA_SRC_LDMAXBAR = 0x01,
    EFR32_LDMA_SRC_TIMER0 = 0x02,
    EFR32_LDMA_SRC_TIMER1 = 0x03,
    EFR32_LDMA_SRC_USART0 = 0x04,
    EFR32_LDMA_SRC_USART1 = 0x05,
    EFR32_LDMA_SRC_I2C0 = 0x06,
    EFR32_LDMA_SRC_I2C1 = 0x07,
    EFR32_LDMA_SRC_IADC0 = 0x0b,
    EFR32_LDMA_SRC_MSC = 0x0c,
    EFR32_LDMA_SRC_TIMER2 = 0x0d,
    EFR32_LDMA_SRC_TIMER3 = 0x0e,
    EFR32_LDMA_SRC_PDM = 0x0f,
    EFR32_LDMA_SRC_EUART0 = 0x10,
    EFR32_LDMA_SRC_TIMER4 = 0x11,
};
/* TIMERn signals: CC0-2, UFOF. */
#define EFR32_LDMA_SIG_TIMER_CC(n) (n)
#define EFR32_LDMA_SIG_TIMER_UFOF 3
/* USARTn: RXDATAV, RXDATAVRIGHT, TXBL, TXBLRIGHT, TXEMPTY. */
#define EFR32_LDMA_USART_SIGNALS 5
/* EUART0: RXFL 0, TXFL 1. */
#define EFR32_LDMA_SIG_EUART_RXFL 0
#define EFR32_LDMA_SIG_EUART_TXFL 1

DeviceState *efr32_ldma_create(Object *soc, MemoryRegion *memory,
                               const Efr32SocLink *link, hwaddr ns_offset,
                               qemu_irq irq);

#endif
