/*
 * Analog source: what an ADC model samples.
 *
 * The silicon-independent side of an ADC input.  A board model (or a
 * passthrough to a host IIO device) implements it; the SoC's ADC model,
 * which owns the per-silicon register interface, asks it for the voltage
 * on a named net at the instant of each conversion.  Nets are named by the
 * package pin the ADC reads, e.g. "PD02".
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_ANALOG_SOURCE_H
#define HW_MISC_ANALOG_SOURCE_H

#include "qom/object.h"

#define TYPE_ANALOG_SOURCE "analog-source"

typedef struct AnalogSourceClass AnalogSourceClass;
DECLARE_CLASS_CHECKERS(AnalogSourceClass, ANALOG_SOURCE, TYPE_ANALOG_SOURCE)
#define ANALOG_SOURCE(obj) INTERFACE_CHECK(AnalogSource, (obj), \
                                           TYPE_ANALOG_SOURCE)

typedef struct AnalogSource AnalogSource;

struct AnalogSourceClass {
    InterfaceClass parent;

    /* Microvolts on @net now, or a negative value for an undriven net. */
    int64_t (*read_uv)(AnalogSource *src, const char *net);
};

static inline int64_t analog_source_read_uv(AnalogSource *src,
                                            const char *net)
{
    AnalogSourceClass *k = ANALOG_SOURCE_GET_CLASS(src);

    return k->read_uv ? k->read_uv(src, net) : -1;
}

#endif
