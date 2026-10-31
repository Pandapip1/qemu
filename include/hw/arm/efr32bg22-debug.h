/*
 * EFR32BG22 debug blocks: the Cortex-M33 halting debug unit and the SWJ-DP
 * with its AHB-AP and DCI AP.  Flash programming is the MSC in
 * efr32bg22.c, which the AHB-AP reaches like the core does.
 *
 * Copyright (c) 2026 Gavin John
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_ARM_EFR32BG22_DEBUG_H
#define HW_ARM_EFR32BG22_DEBUG_H

#include "hw/arm/armv7m.h"
#include "hw/clock.h"

/*
 * Create the blocks as children of @soc ("m33-debug", "dap") and map the
 * debug unit into the M33's private PPB (@armv7m's container); AP0 of the
 * DAP is the M33 bus.
 */
void efr32bg22_debug_realize(Object *soc, ARMv7MState *armv7m,
                             MemoryRegion *flash, Clock *cpuclk,
                             Error **errp);

/* A system reset (SYSRESETREQ) begins: the debug port's AP window. */
void efr32bg22_debug_system_reset(Object *soc);

#endif
