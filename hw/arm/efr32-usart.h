/*
 * EFR32 Series 2 USART in asynchronous (UART) mode.
 *
 * The peripheral is the silicon-specific half of a host-device passthrough:
 * its registers, buffers, interrupts and line timing are the EFR32's, and
 * its other side is a QEMU character device that owns whatever the host
 * offers (a tty, a pty, a socket, or /dev/vhci for an HCI UART controller).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_ARM_EFR32_USART_H
#define HW_ARM_EFR32_USART_H

#include "exec/memory.h"
#include "hw/irq.h"
#include "hw/clock.h"
#include "sysemu/cpu-timers.h"

#define TYPE_EFR32_USART "efr32-usart"
/* LDMA request lines: RXDATAV, RXDATAVRIGHT, TXBL, TXBLRIGHT, TXEMPTY. */
#define EFR32_USART_DMA_REQ "dma-req"
#define EFR32_USART_DMA_REQS 5

/* The register block, including its SET, CLR and TGL aliases. */
#define EFR32_USART_BLOCK_SIZE 0x4000u

typedef struct Efr32UsartConfig {
    /* Instance name ("usart0"); the chardev "<soc>.<name>" claims it. */
    const char *name;
    /* USARTn: the n of the trace output. */
    unsigned index;
    /* Secure base address; the non-secure alias is @ns_offset above it. */
    hwaddr base;
    hwaddr ns_offset;
    unsigned rx_irq;
    unsigned tx_irq;
    /* Fallback chardev when no "<soc>.<name>" chardev exists, or -1. */
    int serial_index;
} Efr32UsartConfig;

/*
 * Create, realize and map one USART of @soc: overlay its block (priority
 * above the catch-all peripheral window) at both security aliases of
 * @memory, wire its RX/TX lines to @nvic and its clock to @pclk, and route
 * its CPU accesses through @bus_model.
 */
DeviceState *efr32_usart_create(Object *soc, MemoryRegion *memory,
                                DeviceState *nvic, Clock *pclk,
                                IcountBusModel *bus_model,
                                const Efr32UsartConfig *config);

#endif
