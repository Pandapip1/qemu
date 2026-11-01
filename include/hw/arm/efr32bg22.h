/*
 * EFR32BG22 SoC services for boards built around it.
 *
 * Copyright (c) 2026 Gavin John
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_ARM_EFR32BG22_H
#define HW_ARM_EFR32BG22_H

#include "qemu/notify.h"
#include "qom/object.h"

/*
 * @n runs after every reset of @soc (an "efr32bg22-soc"): one the part
 * raised itself (SYSRESETREQ, WDOG0, Lockup, the RESETn pin), which resets
 * that part alone, or a reset of the whole machine.
 */
void efr32bg22_soc_add_reset_notifier(Object *soc, Notifier *n);

#endif
