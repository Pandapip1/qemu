/*
 * Parts on the OWO suit's control-unit buses (see include/hw/misc/owo-suit.h).
 *
 * Evidence: the frames are the stock firmware's bus traffic
 * (owo-open-fw docs/stock-stim-path.md); part behaviour is the part class's
 * data sheet as summarised below.  The LTC2634 and PCF8575 data sheets are
 * not stored with the project's reference documents: the command and
 * register summaries here must be checked against them (listed as open
 * items in docs/stim-simulation.md).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/misc/owo-suit.h"

/*
 * ---------------------------------------------------------------- LTC2634
 *
 * Quad 12-bit SPI DAC.  A frame is shifted in MSB first while CS/LD is low;
 * the rising edge of CS/LD executes the last 24 bits: C3-C0 (23:20),
 * A3-A0 (19:16), D11-D0 (15:4), 4 don't-care bits.  Commands: 0 write
 * input register n; 1 update DAC register n (power up); 2 write input n,
 * update all; 3 write and update n; 4 power down n; 5 power down chip;
 * 6 internal reference; 7 external reference; 15 no operation.  Address
 * 0-3 is DAC A-D, 15 all.  /CLR (active low, asynchronous) clears input
 * and DAC registers to the clear code (zero scale on the -Z variants);
 * power-on is zero scale with all DACs powered down until updated.  Output
 * = code / 4096 x full scale.
 * The full scale is a property: the -L parts give 2.5 V, the -H 4.096 V
 * (internal reference).  2.5 V is the nominal here because the stock
 * firmware's scaling then makes the DAC volts per level (0x12FED / 0x9C4
 * codes x 2.5 V / 4096 = 19.0 mV) match its sense volts per level
 * (3.3 V / 4096 / (0xA1220 / 0xEF510A) = 19.15 mV); unmeasured.
 * Fault injection: "fault-ignore" makes the part ignore every frame (a
 * dead DAC or open CS; nothing on the bus can tell).
 */
#define LTC2634_FRAME_BITS 24
#define LTC2634_CMD_SHIFT 20
#define LTC2634_ADDR_SHIFT 16
#define LTC2634_FIELD_MASK 0xfu
#define LTC2634_DATA_SHIFT 4
#define LTC2634_CODE_MASK 0xfffu
#define LTC2634_CODES 4096u
#define LTC2634_ADDR_ALL 0xfu
#define LTC2634_FULL_SCALE_L_MV 2500u
enum {
    LTC2634_WRITE_INPUT = 0,
    LTC2634_UPDATE = 1,
    LTC2634_WRITE_UPDATE_ALL = 2,
    LTC2634_WRITE_UPDATE = 3,
    LTC2634_POWER_DOWN = 4,
    LTC2634_POWER_DOWN_CHIP = 5,
    LTC2634_REF_INTERNAL = 6,
    LTC2634_REF_EXTERNAL = 7,
};

static void ltc2634_notify(LTC2634State *s)
{
    if (s->notify) {
        s->notify(s->opaque);
    }
}

static void ltc2634_execute(LTC2634State *s)
{
    uint32_t word = s->shift;
    unsigned cmd = (word >> LTC2634_CMD_SHIFT) & LTC2634_FIELD_MASK;
    unsigned addr = (word >> LTC2634_ADDR_SHIFT) & LTC2634_FIELD_MASK;
    uint16_t code = (word >> LTC2634_DATA_SHIFT) & LTC2634_CODE_MASK;

    for (unsigned ch = 0; ch < LTC2634_CHANNELS; ch++) {
        bool hit = addr == LTC2634_ADDR_ALL || addr == ch;

        switch (cmd) {
        case LTC2634_WRITE_INPUT:
            if (hit) {
                s->input_reg[ch] = code;
            }
            break;
        case LTC2634_UPDATE:
            if (hit) {
                s->dac_reg[ch] = s->input_reg[ch];
                s->powered[ch] = true;
            }
            break;
        case LTC2634_WRITE_UPDATE_ALL:
            if (hit) {
                s->input_reg[ch] = code;
            }
            s->dac_reg[ch] = s->input_reg[ch];
            s->powered[ch] = true;
            break;
        case LTC2634_WRITE_UPDATE:
            if (hit) {
                s->input_reg[ch] = code;
                s->dac_reg[ch] = code;
                s->powered[ch] = true;
            }
            break;
        case LTC2634_POWER_DOWN:
            if (hit) {
                s->powered[ch] = false;
            }
            break;
        case LTC2634_POWER_DOWN_CHIP:
            s->powered[ch] = false;
            break;
        default:
            break;
        }
    }
    if (cmd == LTC2634_REF_INTERNAL || cmd == LTC2634_REF_EXTERNAL) {
        s->internal_ref = cmd == LTC2634_REF_INTERNAL;
    }
    ltc2634_notify(s);
}

static uint32_t ltc2634_transfer(SSIPeripheral *dev, uint32_t val)
{
    LTC2634State *s = LTC2634(dev);

    if (s->selected && !s->fault_ignore) {
        s->shift = (s->shift << 8) | (val & 0xffu);
        s->bits += 8;
    }
    return 0; /* SDO is not connected on the board */
}

static void ltc2634_cs(void *opaque, int n, int level)
{
    LTC2634State *s = opaque;

    if (!level && !s->selected) {
        s->selected = true;
        s->shift = 0;
        s->bits = 0;
    } else if (level && s->selected) {
        s->selected = false;
        if (s->bits >= LTC2634_FRAME_BITS && !s->fault_ignore) {
            ltc2634_execute(s);
        }
    }
}

static void ltc2634_clr(void *opaque, int n, int level)
{
    LTC2634State *s = opaque;

    if (!level) {
        for (unsigned ch = 0; ch < LTC2634_CHANNELS; ch++) {
            s->input_reg[ch] = s->clear_code;
            s->dac_reg[ch] = s->clear_code;
        }
        ltc2634_notify(s);
    }
}

int64_t ltc2634_output_uv(LTC2634State *s, unsigned ch)
{
    if (ch >= LTC2634_CHANNELS || !s->powered[ch]) {
        return 0;
    }
    return (int64_t)s->dac_reg[ch] * s->full_scale_mv * 1000 /
           LTC2634_CODES;
}

static void ltc2634_realize(SSIPeripheral *dev, Error **errp)
{
    LTC2634State *s = LTC2634(dev);

    qdev_init_gpio_in_named(DEVICE(dev), ltc2634_cs, "cs", 1);
    qdev_init_gpio_in_named(DEVICE(dev), ltc2634_clr, "clr", 1);
    /* Power-on: zero scale, powered down (no reset handler: a board part
     * keeps its state across an MCU reset). */
    memset(s->input_reg, 0, sizeof(s->input_reg));
    memset(s->dac_reg, 0, sizeof(s->dac_reg));
    memset(s->powered, 0, sizeof(s->powered));
    s->internal_ref = true;
}

static Property ltc2634_properties[] = {
    DEFINE_PROP_UINT32("full-scale-mv", LTC2634State, full_scale_mv,
                       LTC2634_FULL_SCALE_L_MV),
    DEFINE_PROP_UINT32("clear-code", LTC2634State, clear_code, 0),
    DEFINE_PROP_BOOL("fault-ignore", LTC2634State, fault_ignore, false),
    DEFINE_PROP_END_OF_LIST(),
};

static void ltc2634_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SSIPeripheralClass *k = SSI_PERIPHERAL_CLASS(klass);

    k->realize = ltc2634_realize;
    k->transfer_raw = ltc2634_transfer;
    device_class_set_props(dc, ltc2634_properties);
    dc->desc = "LTC2634-class quad 12-bit SPI DAC";
}

/*
 * ------------------------------------------------------- switch chain
 *
 * Two 16-bit serial-in switch drivers, daisy-chained: 32 bits shift in MSB
 * first on every clock, whatever the chip select does (the stock firmware
 * clocks its read-back with CS high), and the chain's last output is wired
 * back to MISO, so the byte returned is the one shifted out.  A rising edge
 * on the latch input (the "CS" pin, PC3) copies the chain to the switch
 * outputs (74HC595-style).  "transparent-low" models a latch-enable part
 * (HV2601-style) instead, whose outputs follow the chain while the pin is
 * low.  Which applies is unknown (property, unmeasured).
 * Fault injection: "fault-readback-xor" flips bits of what comes back on
 * MISO (byte k after a latch edge is XORed with byte k mod 4 of the mask,
 * most significant first): a broken read-back path.
 */
#define SWITCH_CHAIN_BITS 32
#define SWITCH_BYTE_BITS 8
#define SWITCH_CHAIN_BYTES (SWITCH_CHAIN_BITS / SWITCH_BYTE_BITS)

static void switch_notify(OwoSwitchChainState *s)
{
    if (s->notify) {
        s->notify(s->opaque);
    }
}

static uint32_t switch_transfer(SSIPeripheral *dev, uint32_t val)
{
    OwoSwitchChainState *s = OWO_SWITCH_CHAIN(dev);
    uint8_t out = s->shift >> (SWITCH_CHAIN_BITS - SWITCH_BYTE_BITS);
    unsigned k = s->byte_index++ % SWITCH_CHAIN_BYTES;

    out ^= s->fault_readback_xor >>
           ((SWITCH_CHAIN_BYTES - 1 - k) * SWITCH_BYTE_BITS);
    s->shift = (s->shift << SWITCH_BYTE_BITS) | (val & 0xffu);
    if (s->transparent_low && !s->latch_level) {
        s->latched = s->shift;
        switch_notify(s);
    }
    return out;
}

static void switch_latch(void *opaque, int n, int level)
{
    OwoSwitchChainState *s = opaque;
    bool rising = level && !s->latch_level;

    s->latch_level = level;
    if (rising || (s->transparent_low && !level)) {
        s->latched = s->shift;
        s->byte_index = 0;
        switch_notify(s);
    }
}

static void switch_realize(SSIPeripheral *dev, Error **errp)
{
    OwoSwitchChainState *s = OWO_SWITCH_CHAIN(dev);

    qdev_init_gpio_in_named(DEVICE(dev), switch_latch, "latch", 1);
    /* [U] Power-on contents of the chain and the switch latches. */
    s->shift = 0;
    s->latched = 0;
    s->latch_level = true;
}

static Property switch_properties[] = {
    DEFINE_PROP_BOOL("transparent-low", OwoSwitchChainState,
                     transparent_low, false),
    DEFINE_PROP_UINT32("fault-readback-xor", OwoSwitchChainState,
                       fault_readback_xor, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void switch_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SSIPeripheralClass *k = SSI_PERIPHERAL_CLASS(klass);

    k->realize = switch_realize;
    k->transfer_raw = switch_transfer;
    device_class_set_props(dc, switch_properties);
    dc->desc = "Two daisy-chained 16-bit serial switch drivers, read-back";
}

/*
 * ---------------------------------------------------------------- PCF8575
 *
 * 16-bit quasi-bidirectional I2C expander.  A write sends P0-P7 then
 * P10-P17 (pairs repeat); a read returns the pin levels in the same order.
 * A pin is high unless its latch is 0 or something outside pulls it low;
 * power-on latches are all 1.  /INT goes low when an input changes from
 * what was last read and returns high on a read.
 * Fault injection: "fault-nack": the part does not acknowledge its address
 * (absent or wedged).
 */
#define PCF8575_PORT_BITS 8
#define PCF8575_POR_LATCH 0xffffu

static void pcf8575_notify(PCF8575State *s)
{
    if (s->notify) {
        s->notify(s->opaque);
    }
}

uint16_t pcf8575_pins(PCF8575State *s)
{
    return s->latch & ~s->ext_low;
}

static void pcf8575_update_int(PCF8575State *s)
{
    /* /INT is open drain, active low. */
    qemu_set_irq(s->int_out, pcf8575_pins(s) == s->last_read);
}

void pcf8575_set_ext_low(PCF8575State *s, uint16_t mask)
{
    s->ext_low = mask;
    pcf8575_update_int(s);
    pcf8575_notify(s);
}

static int pcf8575_event(I2CSlave *i2c, enum i2c_event event)
{
    PCF8575State *s = PCF8575(i2c);

    if ((event == I2C_START_SEND || event == I2C_START_RECV) &&
        s->fault_nack) {
        return -1;
    }
    if (event == I2C_START_SEND || event == I2C_START_RECV) {
        s->second_byte = false;
    }
    return 0;
}

static int pcf8575_send(I2CSlave *i2c, uint8_t data)
{
    PCF8575State *s = PCF8575(i2c);

    if (s->second_byte) {
        s->latch = (s->latch & 0x00ffu) | ((uint16_t)data << PCF8575_PORT_BITS);
    } else {
        s->latch = (s->latch & 0xff00u) | data;
    }
    s->second_byte = !s->second_byte;
    pcf8575_update_int(s);
    pcf8575_notify(s);
    return 0;
}

static uint8_t pcf8575_recv(I2CSlave *i2c)
{
    PCF8575State *s = PCF8575(i2c);
    uint16_t pins = pcf8575_pins(s);
    uint8_t value = s->second_byte ? pins >> PCF8575_PORT_BITS : pins;

    s->second_byte = !s->second_byte;
    s->last_read = pins;
    pcf8575_update_int(s);
    return value;
}

static void pcf8575_realize(DeviceState *dev, Error **errp)
{
    PCF8575State *s = PCF8575(dev);

    qdev_init_gpio_out_named(dev, &s->int_out, "int", 1);
    s->latch = PCF8575_POR_LATCH;
    s->last_read = pcf8575_pins(s);
}

static Property pcf8575_properties[] = {
    DEFINE_PROP_BOOL("fault-nack", PCF8575State, fault_nack, false),
    DEFINE_PROP_END_OF_LIST(),
};

static void pcf8575_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    dc->realize = pcf8575_realize;
    k->event = pcf8575_event;
    k->send = pcf8575_send;
    k->recv = pcf8575_recv;
    device_class_set_props(dc, pcf8575_properties);
    dc->desc = "PCF8575-class 16-bit I2C expander";
}

/*
 * ---------------------------------------------------- unknown I2C part
 *
 * The stock firmware probes 7-bit address 0x69 once at start-up (one-byte
 * read of register 0) and only notes whether it answered.  The part is not
 * identified; this placeholder acknowledges and reads 0 so the probe
 * succeeds as it presumably does on the unit ("present" = false removes
 * it).  [U] Everything about the real part.
 */
typedef struct I2CPlaceholderState {
    I2CSlave parent_obj;
    bool present;
} I2CPlaceholderState;
OBJECT_DECLARE_SIMPLE_TYPE(I2CPlaceholderState, I2C_PLACEHOLDER)

static int placeholder_event(I2CSlave *i2c, enum i2c_event event)
{
    I2CPlaceholderState *s = I2C_PLACEHOLDER(i2c);

    return (event == I2C_START_SEND || event == I2C_START_RECV) &&
                   !s->present ? -1 : 0;
}

static int placeholder_send(I2CSlave *i2c, uint8_t data)
{
    return 0;
}

static uint8_t placeholder_recv(I2CSlave *i2c)
{
    return 0;
}

static Property placeholder_properties[] = {
    DEFINE_PROP_BOOL("present", I2CPlaceholderState, present, true),
    DEFINE_PROP_END_OF_LIST(),
};

static void placeholder_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    k->event = placeholder_event;
    k->send = placeholder_send;
    k->recv = placeholder_recv;
    device_class_set_props(dc, placeholder_properties);
    dc->desc = "Acknowledging placeholder for an unidentified I2C part";
}

static const TypeInfo owo_suit_part_types[] = {
    {
        .name = TYPE_LTC2634,
        .parent = TYPE_SSI_PERIPHERAL,
        .instance_size = sizeof(LTC2634State),
        .class_init = ltc2634_class_init,
    }, {
        .name = TYPE_OWO_SWITCH_CHAIN,
        .parent = TYPE_SSI_PERIPHERAL,
        .instance_size = sizeof(OwoSwitchChainState),
        .class_init = switch_class_init,
    }, {
        .name = TYPE_PCF8575,
        .parent = TYPE_I2C_SLAVE,
        .instance_size = sizeof(PCF8575State),
        .class_init = pcf8575_class_init,
    }, {
        .name = TYPE_I2C_PLACEHOLDER,
        .parent = TYPE_I2C_SLAVE,
        .instance_size = sizeof(I2CPlaceholderState),
        .class_init = placeholder_class_init,
    },
};

DEFINE_TYPES(owo_suit_part_types)
