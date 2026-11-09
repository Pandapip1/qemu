/*
 * EFR32 Series 2 GPIO: ports A-D, pin modes, data in/out, external
 * interrupts and the TIMER compare-output routes.
 *
 * Pins are the silicon-independent side.  Each pin has three qdev GPIO
 * lines, numbered port * EFR32_GPIO_PINS_PER_PORT + pin:
 *  - "pin-out" (output): the level the pin drives (0/1), meaningful while
 *    "pin-oe" is 1;
 *  - "pin-oe" (output): 1 while the pin drives (push-pull, or wired-and
 *    driving low, or wired-or driving high), 0 while it is high impedance
 *    (possibly with its internal pull, which the model does not report);
 *  - "pin-in" (input): the level something outside drives onto the pin.
 *    A pin nothing drives reads its pull (DOUT for the pull modes), else 0.
 * Whatever is on the other side (a board model, or a host GPIO chip through
 * a passthrough backend) sees pins, never registers.
 *
 * Peripheral outputs that the route registers can put on a pin enter on the
 * "timer-cc" input lines: TIMERn CCm is line n * EFR32_GPIO_TIMER_CCS + m.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_ARM_EFR32_GPIO_H
#define HW_ARM_EFR32_GPIO_H

#include "exec/memory.h"
#include "sysemu/cpu-timers.h"

#define TYPE_EFR32_GPIO "efr32-gpio"

#define EFR32_GPIO_PORT_COUNT 4
#define EFR32_GPIO_PINS_PER_PORT 16
#define EFR32_GPIO_LINES (EFR32_GPIO_PORT_COUNT * EFR32_GPIO_PINS_PER_PORT)
#define EFR32_GPIO_LINE(port, pin) ((port) * EFR32_GPIO_PINS_PER_PORT + (pin))
#define EFR32_GPIO_TIMERS 5
#define EFR32_GPIO_TIMER_CCS 3

#define EFR32_GPIO_PIN_OUT "pin-out"
#define EFR32_GPIO_PIN_OE "pin-oe"
#define EFR32_GPIO_PIN_IN "pin-in"
#define EFR32_GPIO_TIMER_CC "timer-cc"
/* PRS channel outputs (async 0-11, then sync 0-3) for GPIO_PRSROUTE. */
#define EFR32_GPIO_PRS_CH "prs-ch"
#define EFR32_GPIO_PRS_CHANNELS 16
#define EFR32_GPIO_LETIMER_OUT "letimer-out"
#define EFR32_GPIO_LETIMER_OUTS 2
/* Outputs: the PRS producers GPIO PIN0-7. */
#define EFR32_GPIO_PRS_PIN "prs-pin"
#define EFR32_GPIO_PRS_PINS 8
/* Input: the part is in EM2 (ports C and D latched). */
#define EFR32_GPIO_EM2 "em2"

/* The register block, including its SET, CLR and TGL aliases. */
#define EFR32_GPIO_BLOCK_SIZE 0x4000u

DeviceState *efr32_gpio_create(Object *soc, MemoryRegion *memory,
                               DeviceState *nvic, IcountBusModel *bus_model,
                               hwaddr base, hwaddr ns_offset,
                               unsigned odd_irq, unsigned even_irq);

#endif
