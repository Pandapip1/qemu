/*
 * The OWO suit's control-unit board: the parts on its buses and the
 * analog output stage they drive (hw/misc/owo-suit.c, owo-suit-parts.c).
 *
 * Parts are generic models of the part classes the stock firmware's bus
 * traffic identifies (docs/stock-stim-path.md in owo-open-fw): a quad
 * 12-bit SPI DAC of the LTC2634 family, a 32-bit SPI shift-register chain
 * with serial read-back (two daisy-chained 16-channel switches), and a
 * PCF8575-class 16-bit I2C expander.  They keep their state across an MCU
 * reset, as separate chips on the board do.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_OWO_SUIT_H
#define HW_MISC_OWO_SUIT_H

#include "hw/ssi/ssi.h"
#include "hw/i2c/i2c.h"

#define TYPE_LTC2634 "ltc2634"
#define TYPE_OWO_SWITCH_CHAIN "owo-switch-chain"
#define TYPE_PCF8575 "pcf8575"
#define TYPE_I2C_PLACEHOLDER "i2c-placeholder"
#define TYPE_OWO_SUIT "owo-suit"

#define LTC2634_CHANNELS 4

/* Called by a part whenever an output the board sees changed. */
typedef void OwoSuitPartNotify(void *opaque);

typedef struct LTC2634State {
    SSIPeripheral parent_obj;
    /* Shift register of the current CS-low frame and its bit count. */
    uint32_t shift;
    unsigned bits;
    bool selected;
    uint16_t input_reg[LTC2634_CHANNELS];
    uint16_t dac_reg[LTC2634_CHANNELS];
    bool powered[LTC2634_CHANNELS];
    bool internal_ref;
    /* Properties. */
    uint32_t full_scale_mv;
    uint32_t clear_code;
    bool fault_ignore;
    OwoSuitPartNotify *notify;
    void *opaque;
} LTC2634State;

typedef struct OwoSwitchChainState {
    SSIPeripheral parent_obj;
    uint32_t shift;
    uint32_t latched;
    bool latch_level;
    /* Bytes clocked since the last latch edge (selects the fault byte). */
    unsigned byte_index;
    /* Properties. */
    bool transparent_low;
    uint32_t fault_readback_xor;
    OwoSuitPartNotify *notify;
    void *opaque;
} OwoSwitchChainState;

typedef struct PCF8575State {
    I2CSlave parent_obj;
    uint16_t latch;
    /* Pins something outside pulls low. */
    uint16_t ext_low;
    uint16_t last_read;
    bool second_byte;
    qemu_irq int_out;
    /* Properties. */
    bool fault_nack;
    OwoSuitPartNotify *notify;
    void *opaque;
} PCF8575State;

OBJECT_DECLARE_SIMPLE_TYPE(LTC2634State, LTC2634)
OBJECT_DECLARE_SIMPLE_TYPE(OwoSwitchChainState, OWO_SWITCH_CHAIN)
OBJECT_DECLARE_SIMPLE_TYPE(PCF8575State, PCF8575)

/* The DAC's output voltage of channel @ch in microvolts (0 powered down). */
int64_t ltc2634_output_uv(LTC2634State *s, unsigned ch);
/* The expander's pin levels (quasi-bidirectional: latch AND not pulled). */
uint16_t pcf8575_pins(PCF8575State *s);
void pcf8575_set_ext_low(PCF8575State *s, uint16_t mask);

#endif
