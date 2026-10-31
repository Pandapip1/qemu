/*
 * Analog source interface (see include/hw/misc/analog-source.h).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/misc/analog-source.h"

static const TypeInfo analog_source_info = {
    .name = TYPE_ANALOG_SOURCE,
    .parent = TYPE_INTERFACE,
    .class_size = sizeof(AnalogSourceClass),
};

static void analog_source_register_types(void)
{
    type_register_static(&analog_source_info);
}
type_init(analog_source_register_types)
