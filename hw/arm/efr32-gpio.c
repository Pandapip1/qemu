/*
 * EFR32 Series 2 GPIO (EFR32xG22 reference manual rev. 1.2, chapter 25).
 *
 * Register layout and field encodings are the public device header's
 * (hal_silabs .../EFR32BG22/Include/efr32bg22_gpio.h and
 * efr32bg22_gpio_port.h): four GPIO_PORT_TypeDef blocks of 0x30 bytes
 * (CTRL, MODEL, MODEH, DOUT, DIN), LOCK at 0x300, the bus allocations at
 * 0x320-0x328, the external interrupt selects at 0x400-0x414, IF/IEN at
 * 0x420/0x424 and the peripheral routes from 0x450, with the usual SET, CLR
 * and TGL aliases at +0x1000/+0x2000/+0x3000.
 *
 * Modelled:
 *  - pin modes (GPIO_P_MODEL/MODEH, RM Table 25.1): DISABLED, the input
 *    modes, push-pull, wired-or and wired-and; what a pin drives follows
 *    from its mode and DOUT, or from the routed peripheral output;
 *  - DIN: the pin level when its input is enabled (mode not DISABLED and
 *    DINDIS / DINDISALT clear in GPIO_P_CTRL, RM SS25.3.4), else 0;
 *  - external interrupts (RM SS25.3.10.1, Figure 25.6): EXTIn watches pin
 *    4 * int(n / 4) + EXTIPINSELn of port EXTIPSELn, EXTIRISE/EXTIFALL
 *    select the edges, IF & IEN drive GPIO_ODD (odd n) and GPIO_EVEN
 *    (even n), RM "The even interrupt line is triggered by any enabled even
 *    numbered interrupt flag index";
 *  - TIMERn CCm routes (GPIO_TIMERROUTE: ROUTEEN.CCmPEN, CCmROUTE.PORT bits
 *    1:0 and PIN bits 19:16), the PRS channel routes (GPIO_PRSROUTE) and
 *    the LETIMER0 outputs (GPIO_LETIMERROUTE): an enabled route replaces
 *    DOUT as the value an output-mode pin drives;
 *  - the PRS producers GPIO PIN0-7 (the EXTI0-7 pins' levels);
 *  - the configuration lock (RM SS25.3.5): any value but 0xA534 written to
 *    LOCK freezes CTRL, MODEL/MODEH, the bus allocations, the external
 *    interrupt selects and the routes; GPIOLOCKSTATUS reports it.
 *
 * [U] Not modelled: slew rate and drive strength, the glitch filters, EM4
 * retention and wake-up, the debug pins' default pulls, the other
 * peripherals' routes (USART and I2C reach their devices through SSI and
 * I2C buses, not through pins), and an undriven input without a pull, which
 * reads 0 here (on silicon it floats).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/arm/efr32-gpio.h"
#include "trace.h"

OBJECT_DECLARE_SIMPLE_TYPE(Efr32GpioState, EFR32_GPIO)

#define GPIO_PORT_STRIDE 0x30u
#define GPIO_P_CTRL 0x00u
#define GPIO_P_MODEL 0x04u
#define GPIO_P_MODEH 0x0cu
#define GPIO_P_DOUT 0x10u
#define GPIO_P_DIN 0x14u
#define GPIO_P_CTRL_RESET 0x00400040u
#define GPIO_P_CTRL_DINDIS (1u << 12)
#define GPIO_P_CTRL_DINDISALT (1u << 28)
#define GPIO_P_DOUT_MASK 0xffffu
#define GPIO_PORTS_END (EFR32_GPIO_PORT_COUNT * GPIO_PORT_STRIDE)

#define GPIO_LOCK 0x300u
#define GPIO_LOCK_UNLOCK_KEY 0xa534u
#define GPIO_GPIOLOCKSTATUS 0x310u
#define GPIO_ABUSALLOC 0x320u
#define GPIO_CDBUSALLOC 0x328u
#define GPIO_EXTIPSELL 0x400u
#define GPIO_EXTIPSELH 0x404u
#define GPIO_EXTIPINSELL 0x408u
#define GPIO_EXTIPINSELH 0x40cu
#define GPIO_EXTIRISE 0x410u
#define GPIO_EXTIFALL 0x414u
#define GPIO_IF 0x420u
#define GPIO_IEN 0x424u
#define GPIO_IF_MASK 0x0fff0fffu
#define GPIO_EXTI_COUNT 12
#define GPIO_EXTI_PER_SELECT_REG 8
#define GPIO_EXTI_FIELD_STRIDE 4
#define GPIO_EXTI_FIELD_MASK 0x3u
#define GPIO_EXTI_PINS_PER_GROUP 4
/* GPIO_CMUROUTE is the first route block; GPIO_USARTROUTE[1] the last. */
#define GPIO_ROUTES_BEGIN 0x450u
#define GPIO_ROUTES_END 0x65cu
#define GPIO_LETIMERROUTE 0x4b0u
#define GPIO_PRSROUTE 0x534u
#define GPIO_TIMERROUTE 0x57cu
#define GPIO_TIMERROUTE_STRIDE 0x20u
#define GPIO_TIMERROUTE_ROUTEEN 0x00u
#define GPIO_TIMERROUTE_CC0ROUTE 0x04u
#define GPIO_ROUTE_PORT_MASK 0x3u
#define GPIO_ROUTE_PIN_SHIFT 16
#define GPIO_ROUTE_PIN_MASK 0xfu

/* Register words kept (offsets below the first alias). */
#define GPIO_REGISTER_SPAN 0x1000u
#define GPIO_ALIAS_SHIFT 12
#define GPIO_ALIAS_MASK 0x3000u
enum { GPIO_ALIAS_NONE, GPIO_ALIAS_SET, GPIO_ALIAS_CLR, GPIO_ALIAS_TGL };

/* GPIO_P_MODEL/MODEH MODEn encodings (efr32bg22_gpio_port.h). */
enum {
    GPIO_MODE_DISABLED = 0,
    GPIO_MODE_INPUT = 1,
    GPIO_MODE_INPUTPULL = 2,
    GPIO_MODE_INPUTPULLFILTER = 3,
    GPIO_MODE_PUSHPULL = 4,
    GPIO_MODE_PUSHPULLALT = 5,
    GPIO_MODE_WIREDOR = 6,
    GPIO_MODE_WIREDORPULLDOWN = 7,
    GPIO_MODE_WIREDAND = 8,
    GPIO_MODE_WIREDANDFILTER = 9,
    GPIO_MODE_WIREDANDPULLUP = 10,
    GPIO_MODE_WIREDANDPULLUPFILTER = 11,
    GPIO_MODE_WIREDANDALT = 12,
    GPIO_MODE_WIREDANDALTPULLUP = 14,
    GPIO_MODE_WIREDANDALTPULLUPFILTER = 15,
};
#define GPIO_MODE_BITS 4
#define GPIO_MODE_MASK 0xfu
#define GPIO_PINS_PER_MODE_REG 8

struct Efr32GpioState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    MemoryRegion mmio_ns;
    qemu_irq odd_irq;
    qemu_irq even_irq;
    qemu_irq pin_out[EFR32_GPIO_LINES];
    qemu_irq pin_oe[EFR32_GPIO_LINES];
    IcountBusModel *bus_model;
    hwaddr base;

    uint32_t regs[GPIO_REGISTER_SPAN / 4];
    bool locked;
    /* What the outside drives onto each pin. */
    bool ext_driven[EFR32_GPIO_LINES];
    bool ext_level[EFR32_GPIO_LINES];
    /* Peripheral outputs that a route may put on a pin. */
    bool timer_cc[EFR32_GPIO_TIMERS * EFR32_GPIO_TIMER_CCS];
    bool prs_ch[EFR32_GPIO_PRS_CHANNELS];
    bool letimer_out[EFR32_GPIO_LETIMER_OUTS];
    /* EXTI0-7 levels as the PRS producers GPIO PIN0-7 see them. */
    qemu_irq prs_pin[EFR32_GPIO_PRS_PINS];
    bool prs_pin_sent[EFR32_GPIO_PRS_PINS];
    /* Last values sent on the output lines, and the last pin levels. */
    bool out_sent[EFR32_GPIO_LINES];
    bool oe_sent[EFR32_GPIO_LINES];
    bool level[EFR32_GPIO_LINES];
};

static uint32_t gpio_reg(Efr32GpioState *s, uint32_t offset)
{
    return s->regs[offset / 4];
}

static unsigned gpio_mode(Efr32GpioState *s, unsigned line)
{
    unsigned port = line / EFR32_GPIO_PINS_PER_PORT;
    unsigned pin = line % EFR32_GPIO_PINS_PER_PORT;
    uint32_t reg = gpio_reg(s, port * GPIO_PORT_STRIDE +
                                   (pin < GPIO_PINS_PER_MODE_REG
                                        ? GPIO_P_MODEL : GPIO_P_MODEH));

    return (reg >> ((pin % GPIO_PINS_PER_MODE_REG) * GPIO_MODE_BITS)) &
           GPIO_MODE_MASK;
}

static bool gpio_dout(Efr32GpioState *s, unsigned line)
{
    unsigned port = line / EFR32_GPIO_PINS_PER_PORT;

    return extract32(gpio_reg(s, port * GPIO_PORT_STRIDE + GPIO_P_DOUT),
                     line % EFR32_GPIO_PINS_PER_PORT, 1);
}

/* Route register @route (PORT bits 1:0, PIN bits 19:16) names @line. */
static bool gpio_route_is(Efr32GpioState *s, uint32_t route, unsigned line)
{
    uint32_t sel = gpio_reg(s, route);

    return EFR32_GPIO_LINE(sel & GPIO_ROUTE_PORT_MASK,
                           (sel >> GPIO_ROUTE_PIN_SHIFT) &
                               GPIO_ROUTE_PIN_MASK) == line;
}

/* The value an output-mode pin drives: its routed peripheral, else DOUT. */
static bool gpio_out_value(Efr32GpioState *s, unsigned line)
{
    /*
     * GPIO_PRSROUTE (efr32bg22_gpio.h): ROUTEEN bits 0-11 ASYNCHnPEN,
     * 12-15 SYNCHnPEN, then one route word per channel.  GPIO_LETIMERROUTE:
     * ROUTEEN.OUTnPEN, OUTnROUTE.
     */
    for (unsigned ch = 0; ch < EFR32_GPIO_PRS_CHANNELS; ch++) {
        if ((gpio_reg(s, GPIO_PRSROUTE) & (1u << ch)) &&
            gpio_route_is(s, GPIO_PRSROUTE + 4 + 4 * ch, line)) {
            return s->prs_ch[ch];
        }
    }
    for (unsigned n = 0; n < EFR32_GPIO_LETIMER_OUTS; n++) {
        if ((gpio_reg(s, GPIO_LETIMERROUTE) & (1u << n)) &&
            gpio_route_is(s, GPIO_LETIMERROUTE + 4 + 4 * n, line)) {
            return s->letimer_out[n];
        }
    }
    for (unsigned t = 0; t < EFR32_GPIO_TIMERS; t++) {
        uint32_t route = GPIO_TIMERROUTE + t * GPIO_TIMERROUTE_STRIDE;
        uint32_t enable = gpio_reg(s, route + GPIO_TIMERROUTE_ROUTEEN);

        for (unsigned cc = 0; cc < EFR32_GPIO_TIMER_CCS; cc++) {
            uint32_t sel = gpio_reg(s, route + GPIO_TIMERROUTE_CC0ROUTE +
                                           4 * cc);
            unsigned port = sel & GPIO_ROUTE_PORT_MASK;
            unsigned pin = (sel >> GPIO_ROUTE_PIN_SHIFT) & GPIO_ROUTE_PIN_MASK;

            if ((enable & (1u << cc)) &&
                EFR32_GPIO_LINE(port, pin) == line) {
                return s->timer_cc[t * EFR32_GPIO_TIMER_CCS + cc];
            }
        }
    }
    return gpio_dout(s, line);
}

/* Whether the MCU drives the pin, and to which level (RM Table 25.1). */
static bool gpio_drives(Efr32GpioState *s, unsigned line, bool *level)
{
    unsigned mode = gpio_mode(s, line);
    bool out = gpio_out_value(s, line);

    *level = out;
    switch (mode) {
    case GPIO_MODE_PUSHPULL:
    case GPIO_MODE_PUSHPULLALT:
        return true;
    case GPIO_MODE_WIREDOR:
    case GPIO_MODE_WIREDORPULLDOWN:
        return out;
    default:
        if (mode >= GPIO_MODE_WIREDAND) {
            return !out;
        }
        return false;
    }
}

/* The internal pull of an undriven pin, if any. */
static bool gpio_pull(Efr32GpioState *s, unsigned line, bool *level)
{
    unsigned mode = gpio_mode(s, line);

    switch (mode) {
    case GPIO_MODE_DISABLED:
    case GPIO_MODE_INPUTPULL:
    case GPIO_MODE_INPUTPULLFILTER:
        /* DOUT selects pull-up (1) or, for the input modes, pull-down. */
        *level = gpio_dout(s, line);
        return mode != GPIO_MODE_DISABLED || *level;
    case GPIO_MODE_WIREDORPULLDOWN:
        *level = false;
        return true;
    case GPIO_MODE_WIREDANDPULLUP:
    case GPIO_MODE_WIREDANDPULLUPFILTER:
    case GPIO_MODE_WIREDANDALTPULLUPFILTER:
    case GPIO_MODE_WIREDANDALTPULLUP:
        *level = true;
        return true;
    default:
        return false;
    }
}

static bool gpio_input_enabled(Efr32GpioState *s, unsigned line)
{
    unsigned mode = gpio_mode(s, line);
    uint32_t ctrl = gpio_reg(s, line / EFR32_GPIO_PINS_PER_PORT *
                                    GPIO_PORT_STRIDE + GPIO_P_CTRL);
    bool alt = mode == GPIO_MODE_PUSHPULLALT || mode >= GPIO_MODE_WIREDANDALT;

    return mode != GPIO_MODE_DISABLED &&
           !(ctrl & (alt ? GPIO_P_CTRL_DINDISALT : GPIO_P_CTRL_DINDIS));
}

static bool gpio_pin_level(Efr32GpioState *s, unsigned line)
{
    bool level;

    if (gpio_drives(s, line, &level)) {
        /* A wired-and pin pulled low outside reads low, and so on. */
        if (s->ext_driven[line] && s->ext_level[line] != level &&
            gpio_mode(s, line) >= GPIO_MODE_WIREDOR) {
            return s->ext_level[line];
        }
        return level;
    }
    if (s->ext_driven[line]) {
        return s->ext_level[line];
    }
    return gpio_pull(s, line, &level) ? level : false;
}

static unsigned gpio_exti_line(Efr32GpioState *s, unsigned n)
{
    uint32_t psel = gpio_reg(s, n < GPIO_EXTI_PER_SELECT_REG
                                    ? GPIO_EXTIPSELL : GPIO_EXTIPSELH);
    uint32_t pinsel = gpio_reg(s, n < GPIO_EXTI_PER_SELECT_REG
                                      ? GPIO_EXTIPINSELL : GPIO_EXTIPINSELH);
    unsigned shift = (n % GPIO_EXTI_PER_SELECT_REG) * GPIO_EXTI_FIELD_STRIDE;
    unsigned port = (psel >> shift) & GPIO_EXTI_FIELD_MASK;
    unsigned pin = n / GPIO_EXTI_PINS_PER_GROUP * GPIO_EXTI_PINS_PER_GROUP +
                   ((pinsel >> shift) & GPIO_EXTI_FIELD_MASK);

    return EFR32_GPIO_LINE(port, pin);
}

static void gpio_update_irq(Efr32GpioState *s)
{
    uint32_t pending = gpio_reg(s, GPIO_IF) & gpio_reg(s, GPIO_IEN);
    uint32_t odd = 0;

    for (unsigned bit = 1; bit < 32; bit += 2) {
        odd |= 1u << bit;
    }
    qemu_set_irq(s->odd_irq, !!(pending & odd));
    qemu_set_irq(s->even_irq, !!(pending & ~odd));
}

/*
 * Recompute every pin: send changed drive states to the pin lines, then
 * raise the external interrupts whose pin changed level.
 */
static void gpio_update(Efr32GpioState *s)
{
    bool old_level[EFR32_GPIO_LINES];
    uint32_t flags = 0;

    memcpy(old_level, s->level, sizeof(old_level));
    for (unsigned line = 0; line < EFR32_GPIO_LINES; line++) {
        bool out;
        bool oe = gpio_drives(s, line, &out);

        if (oe != s->oe_sent[line] || (oe && out != s->out_sent[line])) {
            trace_efr32_gpio_pin(line / EFR32_GPIO_PINS_PER_PORT,
                                 line % EFR32_GPIO_PINS_PER_PORT, oe, out);
        }
        if (out != s->out_sent[line]) {
            s->out_sent[line] = out;
            qemu_set_irq(s->pin_out[line], out);
        }
        if (oe != s->oe_sent[line]) {
            s->oe_sent[line] = oe;
            qemu_set_irq(s->pin_oe[line], oe);
        }
        s->level[line] = gpio_pin_level(s, line);
    }
    for (unsigned n = 0; n < GPIO_EXTI_COUNT; n++) {
        unsigned line = gpio_exti_line(s, n);
        bool now = s->level[line] && gpio_input_enabled(s, line);
        bool before = old_level[line] && gpio_input_enabled(s, line);

        if (now && !before && (gpio_reg(s, GPIO_EXTIRISE) & (1u << n))) {
            flags |= 1u << n;
        }
        if (!now && before && (gpio_reg(s, GPIO_EXTIFALL) & (1u << n))) {
            flags |= 1u << n;
        }
    }
    if (flags) {
        s->regs[GPIO_IF / 4] |= flags;
    }
    gpio_update_irq(s);
    /*
     * PRS producers GPIO PIN0-7 (RM SS13.3.3): "PIN0 uses settings for the
     * EXTI0 interrupt, PIN1 uses settings for EXTI1": the level of the pin
     * EXTIn selects.  [I] The edge selects (EXTIRISE/EXTIFALL) do not
     * gate it.
     */
    for (unsigned n = 0; n < EFR32_GPIO_PRS_PINS; n++) {
        unsigned line = gpio_exti_line(s, n);
        bool level = s->level[line] && gpio_input_enabled(s, line);

        if (level != s->prs_pin_sent[n]) {
            s->prs_pin_sent[n] = level;
            qemu_set_irq(s->prs_pin[n], level);
        }
    }
}

static uint32_t gpio_din(Efr32GpioState *s, unsigned port)
{
    uint32_t din = 0;

    for (unsigned pin = 0; pin < EFR32_GPIO_PINS_PER_PORT; pin++) {
        unsigned line = EFR32_GPIO_LINE(port, pin);

        if (gpio_input_enabled(s, line) && s->level[line]) {
            din |= 1u << pin;
        }
    }
    return din;
}

static bool gpio_lockable(uint32_t offset)
{
    if (offset < GPIO_PORTS_END) {
        unsigned reg = offset % GPIO_PORT_STRIDE;

        return reg == GPIO_P_CTRL || reg == GPIO_P_MODEL ||
               reg == GPIO_P_MODEH;
    }
    return (offset >= GPIO_ABUSALLOC && offset <= GPIO_CDBUSALLOC) ||
           (offset >= GPIO_EXTIPSELL && offset <= GPIO_EXTIPINSELH) ||
           (offset >= GPIO_ROUTES_BEGIN && offset < GPIO_ROUTES_END);
}

static void gpio_access(Efr32GpioState *s, hwaddr offset, unsigned size,
                        bool is_write)
{
    if (s->bus_model) {
        icount_bus_access(s->bus_model, current_cpu, s->base + offset, size,
                          is_write);
    }
}

static uint64_t gpio_read(void *opaque, hwaddr addr, unsigned size)
{
    Efr32GpioState *s = opaque;
    uint32_t offset = addr & ~(hwaddr)GPIO_ALIAS_MASK;

    gpio_access(s, addr, size, false);
    if (offset < GPIO_PORTS_END && offset % GPIO_PORT_STRIDE == GPIO_P_DIN) {
        return gpio_din(s, offset / GPIO_PORT_STRIDE);
    }
    if (offset == GPIO_GPIOLOCKSTATUS) {
        return s->locked;
    }
    if (offset == GPIO_LOCK) {
        return 0;
    }
    return gpio_reg(s, offset);
}

static void gpio_write(void *opaque, hwaddr addr, uint64_t data, unsigned size)
{
    Efr32GpioState *s = opaque;
    uint32_t offset = addr & ~(hwaddr)GPIO_ALIAS_MASK;
    unsigned alias = (addr & GPIO_ALIAS_MASK) >> GPIO_ALIAS_SHIFT;
    uint32_t old = gpio_reg(s, offset);
    uint32_t value = data;

    gpio_access(s, addr, size, true);
    switch (alias) {
    case GPIO_ALIAS_SET:
        value = old | value;
        break;
    case GPIO_ALIAS_CLR:
        value = old & ~value;
        break;
    case GPIO_ALIAS_TGL:
        value = old ^ value;
        break;
    }
    if (offset == GPIO_LOCK) {
        s->locked = (value & 0xffffu) != GPIO_LOCK_UNLOCK_KEY;
        return;
    }
    if (offset == GPIO_GPIOLOCKSTATUS ||
        (offset < GPIO_PORTS_END && offset % GPIO_PORT_STRIDE == GPIO_P_DIN)) {
        return; /* read-only */
    }
    if (s->locked && gpio_lockable(offset)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "efr32-gpio: write to locked offset 0x%03x\n", offset);
        return;
    }
    if (offset < GPIO_PORTS_END && offset % GPIO_PORT_STRIDE == GPIO_P_DOUT) {
        value &= GPIO_P_DOUT_MASK;
    } else if (offset == GPIO_IF || offset == GPIO_IEN) {
        value &= GPIO_IF_MASK;
    }
    s->regs[offset / 4] = value;
    gpio_update(s);
}

static const MemoryRegionOps gpio_ops = {
    .read = gpio_read,
    .write = gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {.min_access_size = 4, .max_access_size = 4},
};

static void gpio_pin_in(void *opaque, int line, int level)
{
    Efr32GpioState *s = opaque;

    s->ext_driven[line] = true;
    s->ext_level[line] = level != 0;
    gpio_update(s);
}

static void gpio_timer_cc(void *opaque, int n, int level)
{
    Efr32GpioState *s = opaque;

    s->timer_cc[n] = level != 0;
    gpio_update(s);
}

static void gpio_prs_ch(void *opaque, int n, int level)
{
    Efr32GpioState *s = opaque;

    s->prs_ch[n] = level != 0;
    gpio_update(s);
}

static void gpio_letimer_out(void *opaque, int n, int level)
{
    Efr32GpioState *s = opaque;

    s->letimer_out[n] = level != 0;
    gpio_update(s);
}

static void gpio_reset_hold(Object *obj, ResetType type)
{
    Efr32GpioState *s = EFR32_GPIO(obj);

    memset(s->regs, 0, sizeof(s->regs));
    for (unsigned port = 0; port < EFR32_GPIO_PORT_COUNT; port++) {
        s->regs[(port * GPIO_PORT_STRIDE + GPIO_P_CTRL) / 4] =
            GPIO_P_CTRL_RESET;
    }
    s->locked = false;
}

/* Outputs go to the reset state (all pins high impedance) after reset. */
static void gpio_reset_exit(Object *obj, ResetType type)
{
    gpio_update(EFR32_GPIO(obj));
}

static void gpio_init(Object *obj)
{
    Efr32GpioState *s = EFR32_GPIO(obj);
    DeviceState *dev = DEVICE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->mmio, obj, &gpio_ops, s, TYPE_EFR32_GPIO,
                          EFR32_GPIO_BLOCK_SIZE);
    sysbus_init_mmio(sbd, &s->mmio);
    sysbus_init_irq(sbd, &s->odd_irq);
    sysbus_init_irq(sbd, &s->even_irq);
    qdev_init_gpio_out_named(dev, s->pin_out, EFR32_GPIO_PIN_OUT,
                             EFR32_GPIO_LINES);
    qdev_init_gpio_out_named(dev, s->pin_oe, EFR32_GPIO_PIN_OE,
                             EFR32_GPIO_LINES);
    qdev_init_gpio_in_named(dev, gpio_pin_in, EFR32_GPIO_PIN_IN,
                            EFR32_GPIO_LINES);
    qdev_init_gpio_in_named(dev, gpio_timer_cc, EFR32_GPIO_TIMER_CC,
                            EFR32_GPIO_TIMERS * EFR32_GPIO_TIMER_CCS);
    qdev_init_gpio_in_named(dev, gpio_prs_ch, EFR32_GPIO_PRS_CH,
                            EFR32_GPIO_PRS_CHANNELS);
    qdev_init_gpio_in_named(dev, gpio_letimer_out, EFR32_GPIO_LETIMER_OUT,
                            EFR32_GPIO_LETIMER_OUTS);
    qdev_init_gpio_out_named(dev, s->prs_pin, EFR32_GPIO_PRS_PIN,
                             EFR32_GPIO_PRS_PINS);
}

static void gpio_class_init(ObjectClass *klass, void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = gpio_reset_hold;
    rc->phases.exit = gpio_reset_exit;
}

static const TypeInfo gpio_info = {
    .name = TYPE_EFR32_GPIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Efr32GpioState),
    .instance_init = gpio_init,
    .class_init = gpio_class_init,
};

static void gpio_register_types(void)
{
    type_register_static(&gpio_info);
}
type_init(gpio_register_types)

DeviceState *efr32_gpio_create(Object *soc, MemoryRegion *memory,
                               DeviceState *nvic, IcountBusModel *bus_model,
                               hwaddr base, hwaddr ns_offset,
                               unsigned odd_irq, unsigned even_irq)
{
    DeviceState *dev = qdev_new(TYPE_EFR32_GPIO);
    Efr32GpioState *s = EFR32_GPIO(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    object_property_add_child(soc, "gpio", OBJECT(dev));
    s->bus_model = bus_model;
    s->base = base;
    sysbus_realize_and_unref(sbd, &error_fatal);
    memory_region_add_subregion_overlap(memory, base,
                                        sysbus_mmio_get_region(sbd, 0), 1);
    memory_region_init_alias(&s->mmio_ns, OBJECT(dev), "efr32-gpio-ns",
                             &s->mmio, 0, EFR32_GPIO_BLOCK_SIZE);
    memory_region_add_subregion_overlap(memory, base + ns_offset,
                                        &s->mmio_ns, 1);
    sysbus_connect_irq(sbd, 0, qdev_get_gpio_in(nvic, odd_irq));
    sysbus_connect_irq(sbd, 1, qdev_get_gpio_in(nvic, even_irq));
    return dev;
}
