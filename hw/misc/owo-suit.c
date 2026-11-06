/*
 * The OWO suit's control unit as a board around the efr32bg22 SoC model:
 * the parts on its buses (owo-suit-parts.c) and an electrical model of the
 * output stage they drive, with a waveform recorder.
 *
 * Wiring, from the stock firmware 7.9.07 (owo-open-fw docs/stock-stim-path.md
 * and docs/suit-board.md; [F] read from code, [I] inferred, [U] unknown and
 * modelled by a named parameter with a nominal value):
 *  - H-bridge gates on PB0 (high side A), PB1 (high side B), PB2 (low side
 *    A), PB3 (low side B), active high [F code, I polarity].
 *  - DAC (LTC2634 class) on USART1's SPI bus, CS = PA5 [F]; /CLR = expander
 *    P5 [F].  DAC channel "dac-channel" sets the output current [I].
 *  - Switch chain on USART0's SPI bus, latch = PC3 [F].  Pad k is in
 *    circuit when bit 16 + k (side A, first two bytes shifted) and bit k
 *    (side B) of the latched chain are both set [I: the stock table repeats
 *    one bit in both halves].
 *  - Expander (PCF8575 class) at 7-bit 0x20 on I2C0 [F]; its /INT is PC4
 *    [F]; P2 enables the boost converter's supply ("boost-enable-mask")
 *    [I]; P0 and P4 are inputs (stock: P0 low = power-off request, P4 low
 *    0.5 s = self-test) [F]; P6/P7 a hardware variant [F].  Unidentified
 *    part at 7-bit 0x69 [F probe only].
 *  - Boost PWM from TIMER1 CC0 on PC7 [F].
 *  - Feedback (sense) on PD2, battery divider on PD3 (x 33/133) [F: the
 *    stock IADC inputs 0xB2/0xB3 and the 133/33 scaling].
 *
 * Electrical model (all values are parameters; none was measured on a
 * suit):
 *  - The output is a current sink regulated to I = Vdac / "sense-ohm"
 *    [I: the stock firmware compares its sense ADC level with the commanded
 *    level, 19.15 mV per level, against 19.0 mV per level from the DAC, so
 *    the sense voltage follows the DAC], drawn from the boost rail.  The
 *    current is limited by compliance: the rail must supply the load
 *    voltage plus the sense resistor drop.
 *  - Bridge: high side A + low side B drives the selected pads A->B
 *    ("positive"), high side B + low side A B->A; both low sides on shorts
 *    the pads (the stock "off" state); a high and a low side of the same
 *    leg together is a shoot-through, which crowbars the rail (counted).
 *    Any other combination leaves the pads floating.
 *  - Each pad is Rs in series with Rp || Cp ("electrode-rs-ohm",
 *    "electrode-rp-ohm", "electrode-cp-nf"), the textbook electrode/skin
 *    model, integrated with a step below a fifth of its time constants.
 *  - Sense voltage = I x sense-ohm, read by the IADC on PD2 at each
 *    conversion.
 *  - Boost rail: rises to "boost-target-mv" with "boost-tau-us" while the
 *    boost supply is enabled (expander P2, "boost-enable-mask") [I: the
 *    stock firmware toggles PC7 only for a 0.2 s kick after Start and
 *    stimulates with PC7 low]; with "boost-needs-pwm" it also needs PC7 to
 *    toggle (an edge in the last "pwm-timeout-us").  It bleeds with
 *    "bleed-tau-us", and the output current drains "rail-cap-nf".
 *  - Battery: "battery-mv" through the 33/133 divider on PD3 [I: a two-cell
 *    pack, from the stock firmware's 2 % level of about 6840 mV].
 *  - A pin the MCU does not drive leaves its net floating: chip selects
 *    and latches read high ([U] pull-ups), a floating gate is off unless
 *    "gate-float-on" ([U]: decides what the bridge does while the MCU is in
 *    reset).
 *
 * Fault injection: the board's fault properties and the parts' are
 * settable at run time (qom-set), and the "script" property applies
 * "<ms>,<qom path>,<property>,<value>" entries (separated by ';') at those
 * virtual times, e.g. "1500,/machine/suit,fault-open,0x100".  The paths
 * "pause" (stop the VM; an attached gdb sees the stop and resumes it) and
 * "quit" (end the run) take no property: "2000,pause,-,-".
 *
 * Recorder: "vcd" names a Value Change Dump file (virtual nanoseconds) of
 * the nets, the bridge state, the switch, DAC and expander outputs, the
 * rail, the load current and voltage, every sense and battery conversion,
 * MCU resets and fault changes; "vcd-max-bytes" bounds it.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "qemu/error-report.h"
#include "qemu/timer.h"
#include "qemu/units.h"
#include <math.h>
#include "qom/object.h"
#include "hw/qdev-core.h"
#include "hw/qdev-properties.h"
#include "hw/irq.h"
#include "hw/arm/efr32-gpio.h"
#include "hw/arm/efr32bg22.h"
#include "hw/misc/analog-source.h"
#include "hw/misc/owo-suit.h"
#include "sysemu/reset.h"
#include "sysemu/runstate.h"
#include "sysemu/sysemu.h"

OBJECT_DECLARE_SIMPLE_TYPE(OwoSuitState, OWO_SUIT)

#define SUIT_PADS 16
#define SUIT_SIDE_A_SHIFT 16
#define SUIT_PAD_NONE 255
#define SUIT_PAD_MULTI 254
#define SUIT_EXPANDER_ADDR 0x20
#define SUIT_UNKNOWN_ADDR 0x69
#define SUIT_EXPANDER_CLR_BIT 5
#define PIN(port, pin) EFR32_GPIO_LINE((port) - 'A', (pin))
#define SUIT_PIN_HSA PIN('B', 0)
#define SUIT_PIN_HSB PIN('B', 1)
#define SUIT_PIN_LSA PIN('B', 2)
#define SUIT_PIN_LSB PIN('B', 3)
#define SUIT_PIN_DAC_CS PIN('A', 5)
#define SUIT_PIN_SEL2 PIN('A', 6)
#define SUIT_PIN_SWITCH_LATCH PIN('C', 3)
#define SUIT_PIN_EXP_INT PIN('C', 4)
#define SUIT_PIN_BOOST PIN('C', 7)
#define NS_PER_US 1000
#define US_PER_S 1e6
#define MV_PER_V 1e3
#define UV_PER_V 1e6
#define NF_PER_F 1e9
#define MA_PER_A 1e3
/* Integration steps: active (driving or rail changing) and idle bounds. */
#define SUIT_STEP_ACTIVE_NS 1000
#define SUIT_STEP_IDLE_NS 20000
#define SUIT_TAU_FRACTION 0.2
/* Below these everything is at rest and time can jump. */
#define SUIT_REST_V 1e-6
/* VCD: a real value is rewritten after a relative change of this much. */
#define SUIT_VCD_REL_TOL 0.01
#define SUIT_VCD_ABS_TOL_V 1e-3
#define SUIT_VCD_ABS_TOL_A 1e-6

enum { NET_LOW = 0, NET_HIGH = 1, NET_FLOAT = 2 };
enum {
    BRIDGE_SHORTED = 0,  /* both low sides on: the stock "off" */
    BRIDGE_POSITIVE = 1, /* high A + low B */
    BRIDGE_NEGATIVE = 2, /* high B + low A */
    BRIDGE_FLOAT = 3,    /* no complete path */
    BRIDGE_SHOOT = 4,    /* a leg's high and low side together */
};

/* VCD signals. */
enum {
    SIG_PB0, SIG_PB1, SIG_PB2, SIG_PB3, SIG_PA5, SIG_PA6, SIG_PC3, SIG_PC4,
    SIG_PC7, SIG_BRIDGE, SIG_SWITCH, SIG_PAD, SIG_DAC, SIG_DAC_MV,
    SIG_EXPANDER, SIG_BOOST_ON, SIG_RAIL_V, SIG_I_MA, SIG_LOAD_V,
    SIG_SENSE_MV, SIG_ADC_SENSE_MV, SIG_ADC_BATT_MV, SIG_BATTERY_MV,
    SIG_MCU_RESET, SIG_SHOOT, SIG_FAULTS, SIG_SCRIPT, SIG_PAD_VMAX,
    SIG_COUNT
};

typedef struct {
    const char *name;
    const char *type; /* "wire", "integer", "real" */
    unsigned width;
} SuitSignal;

static const SuitSignal suit_signals[SIG_COUNT] = {
    [SIG_PB0] = { "PB0_hs_a", "wire", 1 },
    [SIG_PB1] = { "PB1_hs_b", "wire", 1 },
    [SIG_PB2] = { "PB2_ls_a", "wire", 1 },
    [SIG_PB3] = { "PB3_ls_b", "wire", 1 },
    [SIG_PA5] = { "PA5_dac_cs", "wire", 1 },
    [SIG_PA6] = { "PA6_sel2", "wire", 1 },
    [SIG_PC3] = { "PC3_switch_latch", "wire", 1 },
    [SIG_PC4] = { "PC4_exp_int", "wire", 1 },
    [SIG_PC7] = { "PC7_boost_pwm", "wire", 1 },
    [SIG_BRIDGE] = { "bridge", "integer", 8 },
    [SIG_SWITCH] = { "switch_latched", "integer", 32 },
    [SIG_PAD] = { "pad", "integer", 8 },
    [SIG_DAC] = { "dac_code", "integer", 12 },
    [SIG_DAC_MV] = { "dac_mv", "real", 64 },
    [SIG_EXPANDER] = { "expander_pins", "integer", 16 },
    [SIG_BOOST_ON] = { "boost_active", "wire", 1 },
    [SIG_RAIL_V] = { "rail_v", "real", 64 },
    [SIG_I_MA] = { "i_out_ma", "real", 64 },
    [SIG_LOAD_V] = { "load_v", "real", 64 },
    [SIG_SENSE_MV] = { "sense_mv", "real", 64 },
    [SIG_ADC_SENSE_MV] = { "adc_sense_mv", "real", 64 },
    [SIG_ADC_BATT_MV] = { "adc_battery_mv", "real", 64 },
    [SIG_BATTERY_MV] = { "battery_mv", "real", 64 },
    [SIG_MCU_RESET] = { "mcu_reset", "integer", 16 },
    [SIG_SHOOT] = { "shoot_through_count", "integer", 16 },
    [SIG_FAULTS] = { "faults", "integer", 32 },
    [SIG_SCRIPT] = { "script_step", "integer", 16 },
    [SIG_PAD_VMAX] = { "pad_vc_max_v", "real", 64 },
};

/* Bits of the "faults" signal. */
enum {
    FAULT_BIT_OPEN = 1u << 0,
    FAULT_BIT_SHORT = 1u << 1,
    FAULT_BIT_OVERCURRENT = 1u << 2,
    FAULT_BIT_SENSE_STUCK = 1u << 3,
    FAULT_BIT_DAC_IGNORE = 1u << 4,
    FAULT_BIT_READBACK = 1u << 5,
    FAULT_BIT_EXP_NACK = 1u << 6,
};

typedef struct {
    int64_t at_ns;
    char *path;
    char *prop;
    char *value;
} SuitScriptStep;

struct OwoSuitState {
    DeviceState parent_obj;

    Object *soc;
    LTC2634State *dac;
    OwoSwitchChainState *sw;
    PCF8575State *exp;

    /* Pin state from the SoC GPIO. */
    bool pin_out[EFR32_GPIO_LINES];
    bool pin_oe[EFR32_GPIO_LINES];
    qemu_irq dac_cs;
    qemu_irq dac_clr;
    qemu_irq switch_latch;

    /* Electrical state. */
    int64_t t_ns;
    double rail_v;
    double vc[SUIT_PADS];
    double i_a;      /* signed, A->B positive */
    double load_v;
    double sense_v;
    int bridge;
    int64_t pwm_last_edge_ns;
    bool pwm_level;
    uint32_t shoot_count;
    uint32_t reset_count;

    /* Parameters (see the header comment). */
    uint32_t dac_channel;
    uint32_t sense_milliohm;
    uint32_t rs_ohm;
    uint32_t rp_ohm;
    uint32_t cp_nf;
    uint32_t short_milliohm;
    uint32_t boost_target_mv;
    uint32_t boost_tau_us;
    uint32_t bleed_tau_us;
    uint32_t rail_cap_nf;
    uint32_t pwm_timeout_us;
    uint32_t boost_enable_mask;
    bool boost_needs_pwm;
    uint32_t battery_div_num;
    uint32_t battery_div_den;
    bool gate_float_on;
    char *sense_net;
    char *battery_net;
    bool unknown_part_present;
    /* Run-time tunables and faults (qom-set or "script"). */
    uint32_t battery_mv;
    uint32_t fault_open;
    uint32_t fault_short;
    uint32_t fault_overcurrent_pct;
    uint32_t fault_sense_stuck_mv; /* UINT32_MAX = none */
    uint32_t fault_dac_ignore;
    uint32_t fault_readback_xor;
    uint32_t fault_expander_nack;
    uint32_t expander_ext_low;
    uint32_t applied_ext_low;

    /* Script. */
    char *script;
    GArray *steps;
    unsigned next_step;
    QEMUTimer *script_timer;

    /* Recorder. */
    char *vcd_path;
    uint64_t vcd_max_bytes;
    FILE *vcd;
    uint64_t vcd_bytes;
    int64_t vcd_time;
    bool vcd_full;
    Notifier exit_notifier;
    Notifier mcu_reset_notifier;
    double vcd_real[SIG_COUNT];
    int64_t vcd_int[SIG_COUNT];
    bool vcd_known[SIG_COUNT];
};

/* ------------------------------------------------------------ recorder */

static void suit_vcd_id(char *buf, unsigned sig)
{
    /* Printable identifiers, two characters. */
    buf[0] = '!' + sig / 90;
    buf[1] = '!' + sig % 90;
    buf[2] = 0;
}

static void suit_vcd_printf(OwoSuitState *s, const char *fmt, ...)
    G_GNUC_PRINTF(2, 3);

static void suit_vcd_printf(OwoSuitState *s, const char *fmt, ...)
{
    va_list ap;
    int n;

    if (!s->vcd || s->vcd_full) {
        return;
    }
    va_start(ap, fmt);
    n = vfprintf(s->vcd, fmt, ap);
    va_end(ap);
    if (n > 0) {
        s->vcd_bytes += n;
    }
    if (s->vcd_bytes >= s->vcd_max_bytes) {
        fprintf(s->vcd, "$comment vcd-max-bytes reached $end\n");
        fflush(s->vcd);
        s->vcd_full = true;
        warn_report("owo-suit: %s reached vcd-max-bytes, recording stopped",
                    s->vcd_path);
    }
}

static void suit_vcd_time(OwoSuitState *s)
{
    if (s->t_ns != s->vcd_time) {
        s->vcd_time = s->t_ns;
        suit_vcd_printf(s, "#%" PRId64 "\n", s->t_ns);
    }
}

static void suit_rec_int(OwoSuitState *s, unsigned sig, int64_t value)
{
    char id[3];

    if (s->vcd_known[sig] && s->vcd_int[sig] == value) {
        return;
    }
    s->vcd_known[sig] = true;
    s->vcd_int[sig] = value;
    suit_vcd_time(s);
    suit_vcd_id(id, sig);
    if (!strcmp(suit_signals[sig].type, "wire")) {
        suit_vcd_printf(s, "%c%s\n",
                        value == NET_FLOAT ? 'z' : value ? '1' : '0', id);
    } else {
        char bits[65];
        unsigned w = suit_signals[sig].width, i;

        for (i = 0; i < w; i++) {
            bits[i] = (value >> (w - 1 - i)) & 1 ? '1' : '0';
        }
        bits[w] = 0;
        suit_vcd_printf(s, "b%s %s\n", bits, id);
    }
}

static void suit_rec_real(OwoSuitState *s, unsigned sig, double value,
                          double abs_tol, bool force)
{
    char id[3];
    double last = s->vcd_real[sig];

    if (!force && s->vcd_known[sig] &&
        fabs(value - last) <= MAX(abs_tol, fabs(last) * SUIT_VCD_REL_TOL)) {
        return;
    }
    s->vcd_known[sig] = true;
    s->vcd_real[sig] = value;
    suit_vcd_time(s);
    suit_vcd_id(id, sig);
    suit_vcd_printf(s, "r%.6g %s\n", value, id);
}

static void suit_vcd_open(OwoSuitState *s, Error **errp)
{
    char id[3];

    if (!s->vcd_path) {
        return;
    }
    s->vcd = fopen(s->vcd_path, "w");
    if (!s->vcd) {
        error_setg_errno(errp, errno, "owo-suit: cannot open %s",
                         s->vcd_path);
        return;
    }
    suit_vcd_printf(s, "$comment OWO suit board model (QEMU efr32bg22 "
                       "owo-suit); time is QEMU virtual time $end\n"
                       "$timescale 1ns $end\n$scope module suit $end\n");
    for (unsigned sig = 0; sig < SIG_COUNT; sig++) {
        suit_vcd_id(id, sig);
        suit_vcd_printf(s, "$var %s %u %s %s $end\n", suit_signals[sig].type,
                        suit_signals[sig].width, id, suit_signals[sig].name);
    }
    suit_vcd_printf(s, "$upscope $end\n$enddefinitions $end\n");
    s->vcd_time = -1;
}

/* ------------------------------------------------------- electrical model */

static int suit_net(OwoSuitState *s, unsigned line)
{
    return s->pin_oe[line] ? s->pin_out[line] : NET_FLOAT;
}

static bool suit_gate(OwoSuitState *s, unsigned line)
{
    int net = suit_net(s, line);

    return net == NET_FLOAT ? s->gate_float_on : net;
}

/* A chip select or latch net: floating reads high ([U] pull-up). */
static bool suit_select_level(OwoSuitState *s, unsigned line)
{
    return suit_net(s, line) != NET_LOW;
}

static int suit_bridge(OwoSuitState *s)
{
    bool ha = suit_gate(s, SUIT_PIN_HSA), hb = suit_gate(s, SUIT_PIN_HSB);
    bool la = suit_gate(s, SUIT_PIN_LSA), lb = suit_gate(s, SUIT_PIN_LSB);

    if ((ha && la) || (hb && lb)) {
        return BRIDGE_SHOOT;
    }
    if (ha && lb) {
        return BRIDGE_POSITIVE;
    }
    if (hb && la) {
        return BRIDGE_NEGATIVE;
    }
    if (la && lb) {
        return BRIDGE_SHORTED;
    }
    return BRIDGE_FLOAT;
}

/* Pads in circuit: both sides of the switch chain select them. */
static uint32_t suit_selected(OwoSuitState *s)
{
    uint32_t latched = s->sw->latched;

    return (latched >> SUIT_SIDE_A_SHIFT) & latched &
           MAKE_64BIT_MASK(0, SUIT_PADS) & ~s->fault_open;
}

static int suit_pad_code(OwoSuitState *s)
{
    uint32_t sel = suit_selected(s);

    if (!sel) {
        return SUIT_PAD_NONE;
    }
    return is_power_of_2(sel) ? ctz32(sel) : SUIT_PAD_MULTI;
}

static bool suit_boost_enabled(OwoSuitState *s)
{
    return (pcf8575_pins(s->exp) & s->boost_enable_mask) ==
           s->boost_enable_mask;
}

/*
 * Whether the boost converter is raising the rail.  [I] The stock firmware
 * toggles PC7 only for a 0.2 s kick after Start and then stimulates for the
 * whole frame with PC7 low, so on the product the rail cannot need PC7 to
 * run: by default the converter runs whenever its supply enable (expander
 * P2) is on.  "boost-needs-pwm" models the other reading, a converter
 * driven by PC7 (the rail rises only while PC7 toggles).
 */
static bool suit_boost_active(OwoSuitState *s, int64_t now)
{
    if (!suit_boost_enabled(s)) {
        return false;
    }
    if (!s->boost_needs_pwm) {
        return true;
    }
    return s->pwm_last_edge_ns >= 0 &&
           now - s->pwm_last_edge_ns <=
               (int64_t)s->pwm_timeout_us * NS_PER_US;
}

static double suit_rs(OwoSuitState *s, unsigned k)
{
    return s->fault_short & (1u << k) ? s->short_milliohm / MV_PER_V
                                      : (double)s->rs_ohm;
}

static double suit_target_current(OwoSuitState *s)
{
    double dac_v = ltc2634_output_uv(s->dac, s->dac_channel) / UV_PER_V;

    return dac_v / (s->sense_milliohm / MV_PER_V) *
           s->fault_overcurrent_pct / 100.0;
}

/* One integration step of dt seconds with the current topology. */
static void suit_step(OwoSuitState *s, double dt, bool boost_on)
{
    uint32_t sel = suit_selected(s);
    double rsense = s->sense_milliohm / MV_PER_V;
    double cp = s->cp_nf / NF_PER_F, rp = s->rp_ohm;
    double g = 0, j = 0, v = 0, i_mag = 0, branch[SUIT_PADS] = { 0 };
    int pol = s->bridge == BRIDGE_POSITIVE ? 1
              : s->bridge == BRIDGE_NEGATIVE ? -1 : 0;

    for (unsigned k = 0; k < SUIT_PADS; k++) {
        if (sel & (1u << k)) {
            double rs = suit_rs(s, k);
            double vc = s->fault_short & (1u << k) ? 0 : s->vc[k];

            g += 1 / rs;
            j += vc / rs;
        }
    }
    if (g > 0 && pol) {
        i_mag = suit_target_current(s);
        v = (pol * i_mag + j) / g;
        if (pol * v + i_mag * rsense > s->rail_v) {
            /* Compliance: what the rail can push through the load. */
            i_mag = MAX(0.0, (s->rail_v - pol * j / g) / (1 / g + rsense));
            v = (pol * i_mag + j) / g;
        }
    } else if (g > 0 && s->bridge == BRIDGE_SHORTED) {
        v = 0;
    } else if (g > 0) {
        v = j / g; /* floating: pads share their charge */
    }
    for (unsigned k = 0; k < SUIT_PADS; k++) {
        bool shorted = s->fault_short & (1u << k);

        if (sel & (1u << k)) {
            branch[k] = (v - (shorted ? 0 : s->vc[k])) / suit_rs(s, k);
        }
        if (shorted) {
            s->vc[k] = 0;
        } else {
            s->vc[k] += dt * (branch[k] - s->vc[k] / rp) / cp;
        }
    }
    s->i_a = g > 0 ? pol * i_mag : 0;
    s->load_v = g > 0 ? v : 0;
    s->sense_v = i_mag * rsense;
    if (boost_on) {
        s->rail_v += dt * (s->boost_target_mv / MV_PER_V - s->rail_v) /
                     (s->boost_tau_us / US_PER_S);
    }
    s->rail_v -= dt * s->rail_v / (s->bleed_tau_us / US_PER_S);
    s->rail_v -= dt * i_mag / (s->rail_cap_nf / NF_PER_F);
    s->rail_v = MAX(s->rail_v, 0.0);
}

/*
 * Nothing changes: no pad holds charge and the rail is either discharged
 * (converter off) or settled at its target (converter on; the bleed the
 * converter replaces is then not integrated).
 */
static bool suit_at_rest(OwoSuitState *s, bool boost_on)
{
    double target = s->boost_target_mv / MV_PER_V;

    if (boost_on ? fabs(s->rail_v - target) > SUIT_VCD_ABS_TOL_V
                 : s->rail_v > SUIT_REST_V) {
        return false;
    }
    if (boost_on) {
        s->rail_v = target;
    }
    for (unsigned k = 0; k < SUIT_PADS; k++) {
        if (fabs(s->vc[k]) > SUIT_REST_V) {
            return false;
        }
    }
    return true;
}

static void suit_record_analog(OwoSuitState *s, bool force)
{
    double vmax = 0;

    for (unsigned k = 0; k < SUIT_PADS; k++) {
        vmax = MAX(vmax, fabs(s->vc[k]));
    }
    suit_rec_real(s, SIG_RAIL_V, s->rail_v, SUIT_VCD_ABS_TOL_V, force);
    suit_rec_real(s, SIG_I_MA, s->i_a * MA_PER_A,
                  SUIT_VCD_ABS_TOL_A * MA_PER_A, force);
    suit_rec_real(s, SIG_LOAD_V, s->load_v, SUIT_VCD_ABS_TOL_V, force);
    suit_rec_real(s, SIG_SENSE_MV, s->sense_v * MV_PER_V,
                  SUIT_VCD_ABS_TOL_V * MV_PER_V, force);
    suit_rec_real(s, SIG_PAD_VMAX, vmax, SUIT_VCD_ABS_TOL_V, force);
}

/* Integrate up to now with the topology that held since t_ns. */
static void suit_advance(OwoSuitState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    double tau = MIN((double)s->rs_ohm, (double)s->rp_ohm) * s->cp_nf /
                 NF_PER_F;
    int64_t tau_ns = MAX((int64_t)1, (int64_t)(tau * SUIT_TAU_FRACTION *
                                               NANOSECONDS_PER_SECOND));

    while (s->t_ns < now) {
        bool boost_on = suit_boost_active(s, s->t_ns);
        bool driving = s->bridge == BRIDGE_POSITIVE ||
                       s->bridge == BRIDGE_NEGATIVE;
        int64_t step;

        if (!driving && suit_at_rest(s, boost_on)) {
            s->t_ns = now;
            break;
        }
        step = driving ? SUIT_STEP_ACTIVE_NS : SUIT_STEP_IDLE_NS;
        step = MIN(MIN(step, tau_ns), now - s->t_ns);
        /* The boost stops one timeout after the last PWM edge. */
        if (boost_on && s->pwm_last_edge_ns >= 0) {
            int64_t stop = s->pwm_last_edge_ns +
                           (int64_t)s->pwm_timeout_us * NS_PER_US;

            if (stop > s->t_ns) {
                step = MIN(step, stop - s->t_ns + 1);
            }
        }
        suit_step(s, step / (double)NANOSECONDS_PER_SECOND, boost_on);
        s->t_ns += step;
        suit_record_analog(s, false);
    }
    s->t_ns = now;
    suit_rec_int(s, SIG_BOOST_ON, suit_boost_active(s, now));
}

static uint32_t suit_fault_bits(OwoSuitState *s)
{
    return (s->fault_open ? FAULT_BIT_OPEN : 0) |
           (s->fault_short ? FAULT_BIT_SHORT : 0) |
           (s->fault_overcurrent_pct != 100 ? FAULT_BIT_OVERCURRENT : 0) |
           (s->fault_sense_stuck_mv != UINT32_MAX ? FAULT_BIT_SENSE_STUCK
                                                   : 0) |
           (s->fault_dac_ignore ? FAULT_BIT_DAC_IGNORE : 0) |
           (s->fault_readback_xor ? FAULT_BIT_READBACK : 0) |
           (s->fault_expander_nack ? FAULT_BIT_EXP_NACK : 0);
}

/* Push run-time faults to the parts and record the digital state. */
static void suit_refresh(OwoSuitState *s)
{
    int bridge;

    s->dac->fault_ignore = s->fault_dac_ignore;
    s->sw->fault_readback_xor = s->fault_readback_xor;
    s->exp->fault_nack = s->fault_expander_nack;
    if (s->expander_ext_low != s->applied_ext_low) {
        s->applied_ext_low = s->expander_ext_low;
        pcf8575_set_ext_low(s->exp, s->expander_ext_low);
    }
    bridge = suit_bridge(s);
    if (bridge == BRIDGE_SHOOT && s->bridge != BRIDGE_SHOOT) {
        s->shoot_count++;
        s->rail_v = 0; /* the leg crowbars the rail */
    }
    s->bridge = bridge;
    suit_rec_int(s, SIG_PB0, suit_net(s, SUIT_PIN_HSA));
    suit_rec_int(s, SIG_PB1, suit_net(s, SUIT_PIN_HSB));
    suit_rec_int(s, SIG_PB2, suit_net(s, SUIT_PIN_LSA));
    suit_rec_int(s, SIG_PB3, suit_net(s, SUIT_PIN_LSB));
    suit_rec_int(s, SIG_PA5, suit_net(s, SUIT_PIN_DAC_CS));
    suit_rec_int(s, SIG_PA6, suit_net(s, SUIT_PIN_SEL2));
    suit_rec_int(s, SIG_PC3, suit_net(s, SUIT_PIN_SWITCH_LATCH));
    suit_rec_int(s, SIG_PC7, suit_net(s, SUIT_PIN_BOOST));
    suit_rec_int(s, SIG_PC4, pcf8575_int_released(s->exp));
    suit_rec_int(s, SIG_BRIDGE, bridge);
    suit_rec_int(s, SIG_SWITCH, s->sw->latched);
    suit_rec_int(s, SIG_PAD, suit_pad_code(s));
    suit_rec_int(s, SIG_DAC, s->dac->powered[s->dac_channel]
                                 ? s->dac->dac_reg[s->dac_channel] : 0);
    suit_rec_real(s, SIG_DAC_MV,
                  ltc2634_output_uv(s->dac, s->dac_channel) / MV_PER_V, 0,
                  false);
    suit_rec_int(s, SIG_EXPANDER, pcf8575_pins(s->exp));
    suit_rec_int(s, SIG_SHOOT, s->shoot_count);
    suit_rec_int(s, SIG_FAULTS, suit_fault_bits(s));
    suit_rec_real(s, SIG_BATTERY_MV, s->battery_mv, 0, false);
    suit_record_analog(s, false);
}

static void suit_changed(void *opaque)
{
    OwoSuitState *s = opaque;

    suit_advance(s);
    suit_refresh(s);
}

static void suit_pin_out(void *opaque, int line, int level)
{
    OwoSuitState *s = opaque;

    suit_advance(s);
    s->pin_out[line] = level;
    if (line == SUIT_PIN_BOOST && s->pin_oe[line] && level != s->pwm_level) {
        s->pwm_level = level;
        s->pwm_last_edge_ns = s->t_ns;
    }
    if (line == SUIT_PIN_DAC_CS) {
        qemu_set_irq(s->dac_cs, suit_select_level(s, line));
    } else if (line == SUIT_PIN_SWITCH_LATCH) {
        qemu_set_irq(s->switch_latch, suit_select_level(s, line));
    }
    suit_refresh(s);
}

static void suit_pin_oe(void *opaque, int line, int level)
{
    OwoSuitState *s = opaque;

    suit_advance(s);
    s->pin_oe[line] = level;
    if (line == SUIT_PIN_DAC_CS) {
        qemu_set_irq(s->dac_cs, suit_select_level(s, line));
    } else if (line == SUIT_PIN_SWITCH_LATCH) {
        qemu_set_irq(s->switch_latch, suit_select_level(s, line));
    }
    suit_refresh(s);
}

static void suit_expander_changed(void *opaque)
{
    OwoSuitState *s = opaque;

    suit_changed(s);
    /* /CLR of the DAC is expander P5, active low. */
    qemu_set_irq(s->dac_clr,
                 extract32(pcf8575_pins(s->exp), SUIT_EXPANDER_CLR_BIT, 1));
}

/* --------------------------------------------------------- analog source */

static int64_t suit_read_uv(AnalogSource *src, const char *net)
{
    OwoSuitState *s = OWO_SUIT(src);

    suit_advance(s);
    suit_refresh(s);
    if (!strcmp(net, s->sense_net)) {
        double mv = s->fault_sense_stuck_mv != UINT32_MAX
                        ? s->fault_sense_stuck_mv
                        : s->sense_v * MV_PER_V;

        suit_rec_real(s, SIG_ADC_SENSE_MV, mv, 0, true);
        return (int64_t)(mv * MV_PER_V);
    }
    if (!strcmp(net, s->battery_net)) {
        double mv = (double)s->battery_mv * s->battery_div_num /
                    s->battery_div_den;

        suit_rec_real(s, SIG_ADC_BATT_MV, mv, 0, true);
        return (int64_t)(mv * MV_PER_V);
    }
    return -1;
}

/* ------------------------------------------------------------- script */

static void suit_script_arm(OwoSuitState *s)
{
    if (s->next_step < s->steps->len) {
        timer_mod(s->script_timer,
                  g_array_index(s->steps, SuitScriptStep, s->next_step).at_ns);
    }
}

static void suit_script_fire(void *opaque)
{
    OwoSuitState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    while (s->next_step < s->steps->len) {
        SuitScriptStep *st = &g_array_index(s->steps, SuitScriptStep,
                                            s->next_step);
        Object *obj;
        Error *err = NULL;

        if (st->at_ns > now) {
            break;
        }
        suit_advance(s);
        if (!strcmp(st->path, "pause") || !strcmp(st->path, "quit")) {
            /* Hand control to a debugger at this virtual time, or end. */
            info_report("owo-suit: t=%" PRId64 " ns: %s", now, st->path);
            s->next_step++;
            suit_rec_int(s, SIG_SCRIPT, s->next_step);
            if (!strcmp(st->path, "pause")) {
                vm_stop(RUN_STATE_PAUSED);
            } else {
                qemu_system_shutdown_request(SHUTDOWN_CAUSE_HOST_QMP_QUIT);
            }
            break;
        }
        obj = object_resolve_path(st->path, NULL);
        if (!obj || !object_property_parse(obj, st->prop, st->value, &err)) {
            error_report("owo-suit: script step %u (%s %s=%s) failed: %s",
                         s->next_step, st->path, st->prop, st->value,
                         err ? error_get_pretty(err) : "no such object");
            error_free(err);
        } else {
            info_report("owo-suit: t=%" PRId64 " ns: %s %s=%s", now,
                        st->path, st->prop, st->value);
        }
        s->next_step++;
        suit_rec_int(s, SIG_SCRIPT, s->next_step);
        suit_refresh(s);
    }
    suit_script_arm(s);
}

static void suit_script_parse(OwoSuitState *s, Error **errp)
{
    g_auto(GStrv) entries = NULL;

    s->steps = g_array_new(false, true, sizeof(SuitScriptStep));
    if (!s->script || !*s->script) {
        return;
    }
    entries = g_strsplit(s->script, ";", -1);
    for (unsigned i = 0; entries[i]; i++) {
        g_auto(GStrv) f = g_strsplit(g_strstrip(entries[i]), ",", 4);
        SuitScriptStep st;

        if (!*entries[i]) {
            continue;
        }
        if (g_strv_length(f) != 4) {
            error_setg(errp, "owo-suit: script entry '%s' is not "
                       "<ms>,<path>,<property>,<value>", entries[i]);
            return;
        }
        st.at_ns = (int64_t)(g_ascii_strtod(f[0], NULL) * SCALE_MS);
        st.path = g_strdup(f[1]);
        st.prop = g_strdup(f[2]);
        st.value = g_strdup(f[3]);
        g_array_append_val(s->steps, st);
    }
}

/* ------------------------------------------------------------- device */

static void suit_mcu_reset(Notifier *n, void *data)
{
    OwoSuitState *s = container_of(n, OwoSuitState, mcu_reset_notifier);

    suit_advance(s);
    s->reset_count++;
    suit_rec_int(s, SIG_MCU_RESET, s->reset_count);
}

static void suit_exit_notify(Notifier *n, void *data);

static DeviceState *suit_child(OwoSuitState *s, const char *name,
                               Error **errp)
{
    Object *o = object_resolve_path_component(s->soc, name);

    if (!o) {
        error_setg(errp, "owo-suit: the SoC has no '%s'", name);
        return NULL;
    }
    return DEVICE(o);
}

static void suit_realize(DeviceState *dev, Error **errp)
{
    OwoSuitState *s = OWO_SUIT(dev);
    DeviceState *gpio, *usart0, *usart1, *i2c0, *part;
    I2CBus *i2c;

    if (!s->soc) {
        error_setg(errp, "owo-suit: soc link not set");
        return;
    }
    if (!s->sense_milliohm || !s->cp_nf || !s->rs_ohm || !s->rp_ohm ||
        !s->boost_tau_us || !s->bleed_tau_us || !s->rail_cap_nf ||
        !s->battery_div_den) {
        error_setg(errp, "owo-suit: zero electrical parameter");
        return;
    }
    gpio = suit_child(s, "gpio", errp);
    usart0 = gpio ? suit_child(s, "usart0", errp) : NULL;
    usart1 = usart0 ? suit_child(s, "usart1", errp) : NULL;
    i2c0 = usart1 ? suit_child(s, "i2c0", errp) : NULL;
    if (!i2c0) {
        return;
    }
    suit_script_parse(s, errp);
    if (*errp) {
        return;
    }
    suit_vcd_open(s, errp);
    if (*errp) {
        return;
    }

    qdev_init_gpio_in_named(dev, suit_pin_out, EFR32_GPIO_PIN_OUT,
                            EFR32_GPIO_LINES);
    qdev_init_gpio_in_named(dev, suit_pin_oe, EFR32_GPIO_PIN_OE,
                            EFR32_GPIO_LINES);
    for (unsigned line = 0; line < EFR32_GPIO_LINES; line++) {
        qdev_connect_gpio_out_named(
            gpio, EFR32_GPIO_PIN_OUT, line,
            qdev_get_gpio_in_named(dev, EFR32_GPIO_PIN_OUT, line));
        qdev_connect_gpio_out_named(
            gpio, EFR32_GPIO_PIN_OE, line,
            qdev_get_gpio_in_named(dev, EFR32_GPIO_PIN_OE, line));
    }

    part = qdev_new(TYPE_LTC2634);
    object_property_add_child(OBJECT(s), "dac", OBJECT(part));
    ssi_realize_and_unref(part,
                          (SSIBus *)(qdev_get_child_bus(usart1, "usart1.ssi")),
                          &error_fatal);
    s->dac = LTC2634(part);
    s->dac->notify = suit_changed;
    s->dac->opaque = s;
    s->dac_cs = qdev_get_gpio_in_named(part, "cs", 0);
    s->dac_clr = qdev_get_gpio_in_named(part, "clr", 0);

    part = qdev_new(TYPE_OWO_SWITCH_CHAIN);
    object_property_add_child(OBJECT(s), "switch", OBJECT(part));
    ssi_realize_and_unref(part,
                          (SSIBus *)(qdev_get_child_bus(usart0, "usart0.ssi")),
                          &error_fatal);
    s->sw = OWO_SWITCH_CHAIN(part);
    s->sw->notify = suit_changed;
    s->sw->opaque = s;
    s->switch_latch = qdev_get_gpio_in_named(part, "latch", 0);

    i2c = I2C_BUS(qdev_get_child_bus(i2c0, "i2c0"));
    part = DEVICE(i2c_slave_new(TYPE_PCF8575, SUIT_EXPANDER_ADDR));
    object_property_add_child(OBJECT(s), "expander", OBJECT(part));
    i2c_slave_realize_and_unref(I2C_SLAVE(part), i2c, &error_fatal);
    s->exp = PCF8575(part);
    s->exp->notify = suit_expander_changed;
    s->exp->opaque = s;
    qdev_connect_gpio_out_named(
        part, "int", 0,
        qdev_get_gpio_in_named(gpio, EFR32_GPIO_PIN_IN, SUIT_PIN_EXP_INT));
    qemu_set_irq(s->exp->int_out, 1);

    part = DEVICE(i2c_slave_new(TYPE_I2C_PLACEHOLDER, SUIT_UNKNOWN_ADDR));
    object_property_add_child(OBJECT(s), "unknown-0x69", OBJECT(part));
    qdev_prop_set_bit(part, "present", s->unknown_part_present);
    i2c_slave_realize_and_unref(I2C_SLAVE(part), i2c, &error_fatal);

    s->t_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->pwm_last_edge_ns = -1;
    s->bridge = BRIDGE_FLOAT;
    s->applied_ext_low = UINT32_MAX;
    qemu_set_irq(s->dac_cs, 1);
    qemu_set_irq(s->switch_latch, 1);
    qemu_set_irq(s->dac_clr, 1);
    s->script_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, suit_script_fire, s);
    suit_script_arm(s);
    s->mcu_reset_notifier.notify = suit_mcu_reset;
    efr32bg22_soc_add_reset_notifier(s->soc, &s->mcu_reset_notifier);
    s->exit_notifier.notify = suit_exit_notify;
    qemu_add_exit_notifier(&s->exit_notifier);
    suit_refresh(s);
}

static void suit_vcd_close(OwoSuitState *s)
{
    if (s->vcd) {
        suit_advance(s);
        suit_refresh(s);
        suit_record_analog(s, true);
        suit_vcd_time(s);
        fclose(s->vcd);
        s->vcd = NULL;
    }
}

/* A bus-less device is not unrealized at exit: close the recording here. */
static void suit_exit_notify(Notifier *n, void *data)
{
    suit_vcd_close(container_of(n, OwoSuitState, exit_notifier));
}

static void suit_unrealize(DeviceState *dev)
{
    suit_vcd_close(OWO_SUIT(dev));
}

static Property suit_properties[] = {
    DEFINE_PROP_LINK("soc", OwoSuitState, soc, TYPE_OBJECT, Object *),
    DEFINE_PROP_UINT32("dac-channel", OwoSuitState, dac_channel, 0),
    /* [U] nominal 47 ohm: 2.5 V full scale => 53 mA. */
    DEFINE_PROP_UINT32("sense-milliohm", OwoSuitState, sense_milliohm,
                       47000),
    /* [U] textbook electrode/skin values. */
    DEFINE_PROP_UINT32("electrode-rs-ohm", OwoSuitState, rs_ohm, 500),
    DEFINE_PROP_UINT32("electrode-rp-ohm", OwoSuitState, rp_ohm, 2000),
    DEFINE_PROP_UINT32("electrode-cp-nf", OwoSuitState, cp_nf, 47),
    DEFINE_PROP_UINT32("short-milliohm", OwoSuitState, short_milliohm, 1000),
    /* [U] boost converter. */
    DEFINE_PROP_UINT32("boost-target-mv", OwoSuitState, boost_target_mv,
                       60000),
    DEFINE_PROP_UINT32("boost-tau-us", OwoSuitState, boost_tau_us, 2000),
    DEFINE_PROP_UINT32("bleed-tau-us", OwoSuitState, bleed_tau_us, 50000),
    DEFINE_PROP_UINT32("rail-cap-nf", OwoSuitState, rail_cap_nf, 10000),
    DEFINE_PROP_UINT32("pwm-timeout-us", OwoSuitState, pwm_timeout_us, 1000),
    DEFINE_PROP_BOOL("boost-needs-pwm", OwoSuitState, boost_needs_pwm, false),
    DEFINE_PROP_UINT32("boost-enable-mask", OwoSuitState, boost_enable_mask,
                       1u << 2),
    /* [F] the stock firmware's battery scaling. */
    DEFINE_PROP_UINT32("battery-div-num", OwoSuitState, battery_div_num, 33),
    DEFINE_PROP_UINT32("battery-div-den", OwoSuitState, battery_div_den, 133),
    DEFINE_PROP_BOOL("gate-float-on", OwoSuitState, gate_float_on, false),
    DEFINE_PROP_STRING("sense-net", OwoSuitState, sense_net),
    DEFINE_PROP_STRING("battery-net", OwoSuitState, battery_net),
    DEFINE_PROP_BOOL("unknown-0x69-present", OwoSuitState,
                     unknown_part_present, true),
    DEFINE_PROP_STRING("script", OwoSuitState, script),
    DEFINE_PROP_STRING("vcd", OwoSuitState, vcd_path),
    DEFINE_PROP_UINT64("vcd-max-bytes", OwoSuitState, vcd_max_bytes,
                       256 * MiB),
    DEFINE_PROP_END_OF_LIST(),
};

static void suit_get_ext_low(Object *obj, Visitor *v, const char *name,
                             void *opaque, Error **errp)
{
    visit_type_uint32(v, name, &OWO_SUIT(obj)->expander_ext_low, errp);
}

/* Inputs pulled low outside the expander (buttons): applied at once. */
static void suit_set_ext_low(Object *obj, Visitor *v, const char *name,
                             void *opaque, Error **errp)
{
    OwoSuitState *s = OWO_SUIT(obj);
    uint32_t value;

    if (!visit_type_uint32(v, name, &value, errp)) {
        return;
    }
    s->expander_ext_low = value;
    if (s->exp) {
        suit_changed(s);
    }
}

static void suit_init(Object *obj)
{
    OwoSuitState *s = OWO_SUIT(obj);

    s->sense_net = g_strdup("PD02");
    s->battery_net = g_strdup("PD03");
    /*
     * [I] A two-cell lithium pack: the stock firmware calls about 6840 mV
     * its 2 % level and the 133/33 divider keeps 8.4 V within the IADC's
     * 3.3 V reference.  7600 mV is a nominal mid-charge value (unmeasured).
     */
    s->battery_mv = 7600;
    s->fault_overcurrent_pct = 100;
    s->fault_sense_stuck_mv = UINT32_MAX;
    /* Run-time tunables: settable after realize (qom-set, script). */
    object_property_add_uint32_ptr(obj, "battery-mv", &s->battery_mv,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint32_ptr(obj, "fault-open", &s->fault_open,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint32_ptr(obj, "fault-short", &s->fault_short,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint32_ptr(obj, "fault-overcurrent-pct",
                                   &s->fault_overcurrent_pct,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint32_ptr(obj, "fault-sense-stuck-mv",
                                   &s->fault_sense_stuck_mv,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint32_ptr(obj, "fault-dac-ignore",
                                 &s->fault_dac_ignore,
                                 OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint32_ptr(obj, "fault-readback-xor",
                                   &s->fault_readback_xor,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint32_ptr(obj, "fault-expander-nack",
                                 &s->fault_expander_nack,
                                 OBJ_PROP_FLAG_READWRITE);
    object_property_add(obj, "expander-ext-low", "uint32",
                        suit_get_ext_low, suit_set_ext_low, NULL, NULL);
}

static void suit_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    AnalogSourceClass *ac = ANALOG_SOURCE_CLASS(klass);

    dc->realize = suit_realize;
    dc->unrealize = suit_unrealize;
    dc->desc = "OWO suit control-unit board (output stage model)";
    device_class_set_props(dc, suit_properties);
    ac->read_uv = suit_read_uv;
}

static const TypeInfo suit_info = {
    .name = TYPE_OWO_SUIT,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(OwoSuitState),
    .instance_init = suit_init,
    .class_init = suit_class_init,
    .interfaces = (InterfaceInfo[]) {
        { TYPE_ANALOG_SOURCE },
        { }
    },
};

static void suit_register_types(void)
{
    type_register_static(&suit_info);
}
type_init(suit_register_types)
