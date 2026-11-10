/*
 * BCM4387 d11 PSM (programmable state machine): the MAC's microcode CPU, one
 * per d11 slice, run as a second core next to the ARM. ISA and register
 * evidence: notes/33-psm-model.md in the bcm4387 project, from stock's
 * rev-85 ucode as stock downloads it, against seemoo-lab d11-emu's rev 15-54
 * interpreter (reimplemented here from its documented semantics, no code
 * taken) and Linux brcmsmac/b43 for the host interface.
 *
 * Evidence: [B] Broadcom/Linux source, [E] d11-emu (rev <= 54), [F] silicon,
 * [I] inferred from the ucode, [U] unknown.
 *
 * Host interface:
 *  - objaddr +0x160: [15:0] address, [22:16] select, bit 24 WINC, bit 25
 *    RINC [B d11.h OBJADDR_*]. objdata +0x164 (32-bit, or its low half)
 *    and +0x166 (high half) [B brcmsmac]. Select 0 = microcode memory,
 *    addressed in 32-bit words, an instruction being two words, low word
 *    first [I: stock's image decodes only that way]. Select 1 = SHM,
 *    addressed in 32-bit words (SHM word 2a in the low half, 2a+1 in the
 *    high half) [B brcmsmac brcms_b_write_objmem]. Auto-increment after a
 *    32-bit +0x164 access or a +0x166 access [I]. Other selects are not
 *    modelled (logged; plain storage in the machine).
 *  - maccontrol +0x120: MCTL_PSM_RUN (bit 1) runs the PSM, MCTL_PSM_JMP_0
 *    (bit 2) holds its PC at 0 [B d11.h; b43 uploads ucode with JMP_0 set
 *    and starts it with RUN and JMP_0 clear]. The PSM executes only while
 *    the machine reports its slice clocked (out of reset, clock enabled,
 *    MAC fast clock) [I]. Core reset halts it and clears PC, flags and
 *    stack; microcode memory, SHM and SCRs survive [U].
 *
 * Machine: 64-bit instructions, 12-bit opcode [52:42] and three 14-bit
 * operands [41:28] [27:14] [13:0] [I]. Operands: 0x0000-0x1fff SHM word,
 * 0x2000-0x27ff IHR, 0x2800-0x2eff SHM[base_reg(b) + off] with b = [10:7],
 * off = [6:0], 0x2f00-0x2f7f SCR, 0x3000-0x3fff 12-bit signed immediate [I].
 * Opcode bit 5 makes add/sub (and probably the compare-jumps) 32-bit, on
 * operand pairs (x, x+1), low word first [I]. Semantics of the shared
 * opcodes as d11-emu [E]; see notes/33 for the table and evidence.
 *
 * IHR i is the host register d11+0x400+2i [E], confirmed for rev 85 by the
 * ucode's own SampleCollect use [I]. The registers the PSM knows itself:
 *  - indirect bases b0..b6 = IHR 0x221 0x222 0x225 0x226 0x229 0x22a 0x22d [I]
 *  - mul low result 0x1c7 [I]
 *  - PHY port: data 0x091, address/status 0x092 (bit 15 busy); radio port:
 *    data 0x094, address 0x095. Opcode 0x102 issues the access: second
 *    operand 0 = read into the data register, 0x20 = write it [I]. The
 *    access completes at once (busy and the bit-14 status read 0) [I].
 *    What a port the machine refuses (not live) does is unknown: a strict
 *    abort, or with the machine's psm-dead-port=busy the command is
 *    dropped and the port's busy bit stays set until core reset [U], to
 *    exercise microcode that bounds its busy-waits.
 * The machine supplies the others it models (SampleCollect). Every other
 * IHR, external condition other than 0x5f (always true [I]), an opcode
 * with unknown semantics, and nap (its wake sources) are gaps: in strict
 * mode the PSM stops there with a "PSMn strict abort" guest error; with
 * strict=off unknown IHRs are plain storage, conditions read false and
 * unknown opcodes stop the PSM.
 *
 * Time: one instruction per PSM clock, PSM_HZ [U], executed in slices from
 * a virtual-clock timer.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/bitmap.h"
#include "hw/arm/bcm4387.h"
#include "trace.h"

#define UCM_INSNS       0x4000      /* 14-bit branch target field [I] */
#define SHM_WORDS       0x2000      /* 13-bit direct operand field [I] */
#define SCR_N           128
#define IHR_N           0x800
#define STACK_N         16          /* [U]; rev <= 54 had 15 [E] */

#define PSM_HZ          160000000   /* [U] */
#define TICK_NS         100000

#define MCTL_PSM_RUN    (1u << 1)
#define MCTL_PSM_JMP_0  (1u << 2)

#define IHR_PHY_DATA    0x091
#define IHR_PHY_ADDR    0x092
#define IHR_RADIO_DATA  0x094
#define IHR_RADIO_ADDR  0x095
#define IHR_MUL_LO      0x1c7

#define COND_TRUE       0x5f

typedef enum {
    PSM_HALTED,         /* PSM_RUN clear or JMP_0 held */
    PSM_RUNNING,
    PSM_NAP,            /* waiting for an event the model does not produce */
    PSM_STOPPED,        /* strict abort or unknown opcode */
} PsmState;

static const char *const state_name[] = {
    "halted", "running", "nap", "stopped",
};

static const uint16_t base_ihr[7] = {
    0x221, 0x222, 0x225, 0x226, 0x229, 0x22a, 0x22d,
};

struct Bcm4387Psm {
    int slice;
    Bcm4387PsmHooks hooks;

    uint64_t *ucm;
    uint16_t shm[SHM_WORDS];
    uint16_t scr[SCR_N];
    uint16_t ihr[IHR_N];        /* PSM-owned IHRs, and storage with strict=off */
    unsigned long *ihr_logged;
    unsigned long *cond_logged;

    uint32_t objaddr;
    uint32_t mctl;

    PsmState state;
    uint16_t pc;
    uint16_t stack[STACK_N];
    int sp;
    bool carry, borrow;
    bool port_stuck[2];         /* psm-dead-port=busy: PHY, radio */

    QEMUTimer *timer;
    int64_t last_ns;
    uint64_t insns;
};

/* ---- faults ------------------------------------------------------------- */

static void psm_set_state(Bcm4387Psm *p, PsmState st)
{
    if (p->state != st) {
        trace_bcm4387_psm_state(p->slice, state_name[p->state],
                                state_name[st], p->pc, p->insns);
        p->state = st;
    }
}

static void G_GNUC_PRINTF(2, 3) psm_abort(Bcm4387Psm *p, const char *fmt, ...)
{
    g_autofree char *msg = NULL;
    va_list ap;

    va_start(ap, fmt);
    msg = g_strdup_vprintf(fmt, ap);
    va_end(ap);
    qemu_log_mask(LOG_GUEST_ERROR, "bcm4387: PSM%d strict abort: %s at "
                  "pc 0x%04x (insn 0x%016" PRIx64 ", instruction #%" PRIu64 ")\n",
                  p->slice, msg, p->pc, p->ucm[p->pc], p->insns);
    psm_set_state(p, PSM_STOPPED);
}

/* ---- IHR ---------------------------------------------------------------- */

static int base_index(uint16_t idx)
{
    for (int b = 0; b < ARRAY_SIZE(base_ihr); b++) {
        if (base_ihr[b] == idx) {
            return b;
        }
    }
    return -1;
}

static bool ihr_own(uint16_t idx)
{
    return base_index(idx) >= 0 || idx == IHR_MUL_LO ||
           idx == IHR_PHY_DATA || idx == IHR_PHY_ADDR ||
           idx == IHR_RADIO_DATA || idx == IHR_RADIO_ADDR;
}

static bool ihr_unknown(Bcm4387Psm *p, uint16_t idx, bool wr, uint16_t v)
{
    if (bcm4387_strict) {
        psm_abort(p, "unmodelled IHR 0x%03x (host +0x%03x) %s 0x%04x", idx,
                  0x400 + 2 * idx, wr ? "write" : "read", wr ? v : 0);
        return false;
    }
    if (!test_and_set_bit(idx * 2 + wr, p->ihr_logged)) {
        qemu_log_mask(LOG_UNIMP, "bcm4387: PSM%d unmodelled IHR 0x%03x %s "
                      "(plain storage) pc 0x%04x\n", p->slice, idx,
                      wr ? "write" : "read", p->pc);
    }
    return true;
}

static bool ihr_read(Bcm4387Psm *p, uint16_t idx, uint16_t *v)
{
    if (idx == IHR_PHY_ADDR || idx == IHR_RADIO_ADDR) {
        /* status (14) clear; busy (15) only on a refused port */
        *v = (p->ihr[idx] & 0x3fff) |
             (p->port_stuck[idx == IHR_RADIO_ADDR] ? 0x8000 : 0);
    } else if (ihr_own(idx)) {
        *v = p->ihr[idx];
    } else if (!p->hooks.ihr(p->hooks.opaque, p->slice, idx, false, v)) {
        if (!ihr_unknown(p, idx, false, 0)) {
            return false;
        }
        *v = p->ihr[idx];
    }
    trace_bcm4387_psm_ihr(p->slice, p->pc, "R", idx, *v);
    return true;
}

static bool ihr_write(Bcm4387Psm *p, uint16_t idx, uint16_t v)
{
    trace_bcm4387_psm_ihr(p->slice, p->pc, "W", idx, v);
    if (idx == IHR_PHY_ADDR || idx == IHR_RADIO_ADDR) {
        /* the ucode only addresses the ports through opcode 0x102 */
        if (bcm4387_strict) {
            psm_abort(p, "plain write 0x%04x to port address IHR 0x%03x "
                      "(effect unknown)", v, idx);
            return false;
        }
        p->ihr[idx] = v;
    } else if (ihr_own(idx)) {
        p->ihr[idx] = v;
    } else if (!p->hooks.ihr(p->hooks.opaque, p->slice, idx, true, &v)) {
        if (!ihr_unknown(p, idx, true, v)) {
            return false;
        }
        p->ihr[idx] = v;
    }
    return true;
}

/* opcode 0x102: PHY or radio port access through its address register */
static bool port_cmd(Bcm4387Psm *p, uint16_t addr, uint16_t cmd, uint16_t dst)
{
    int port = dst == IHR_PHY_ADDR ? 0 : dst == IHR_RADIO_ADDR ? 1 : -1;
    uint16_t data = port ? IHR_RADIO_DATA : IHR_PHY_DATA;
    bool wr = cmd == 0x20;

    if (port < 0 || (cmd != 0 && cmd != 0x20)) {
        psm_abort(p, "opcode 0x102 with operand 0x%04x -> IHR 0x%03x "
                  "(only PHY/radio read 0 / write 0x20 known)", cmd, dst);
        return false;
    }
    addr &= 0x7fff;
    p->ihr[dst] = addr;
    if (!p->hooks.port(p->hooks.opaque, p->slice, port, wr, addr,
                       &p->ihr[data])) {
        if (p->hooks.dead_port_busy) {
            qemu_log_mask(LOG_GUEST_ERROR, "bcm4387: PSM%d %s %s 0x%04x "
                          "through the PSM port while the port is not "
                          "live: dropped, busy until core reset "
                          "(psm-dead-port=busy) pc 0x%04x\n", p->slice,
                          port ? "radio" : "PHY", wr ? "write" : "read",
                          addr, p->pc);
            p->port_stuck[port] = true;
            return true;
        }
        psm_abort(p, "%s %s 0x%04x through the PSM port while the port is "
                  "not live (result unknown)", port ? "radio" : "PHY",
                  wr ? "write" : "read", addr);
        return false;
    }
    trace_bcm4387_psm_port(p->slice, p->pc, port ? "RAD" : "PHY",
                           wr ? "W" : "R", addr, p->ihr[data]);
    return true;
}

/* ---- operands ----------------------------------------------------------- */

static bool shm_index(Bcm4387Psm *p, uint32_t a, uint16_t *i)
{
    if (a >= SHM_WORDS) {
        psm_abort(p, "SHM word 0x%x beyond the modelled 0x%x words", a,
                  SHM_WORDS);
        return false;
    }
    *i = a;
    return true;
}

/* base register of an indirect operand; fields 7..13 are not used by stock */
static bool base_read(Bcm4387Psm *p, uint16_t o, uint16_t *base)
{
    unsigned b = (o >> 7) & 0xf;

    if (b >= ARRAY_SIZE(base_ihr)) {
        psm_abort(p, "indirect operand 0x%04x: base field %u unknown", o, b);
        return false;
    }
    return ihr_read(p, base_ihr[b], base);
}

/* one 16-bit operand location; k = 0/1 picks the word of a 32-bit pair */
static bool opnd_rd16(Bcm4387Psm *p, uint16_t o, int k, uint16_t *v)
{
    uint16_t i, base;

    if (o < 0x2000) {
        if (!shm_index(p, o + k, &i)) {
            return false;
        }
        *v = p->shm[i];
    } else if (o < 0x2800) {
        return ihr_read(p, (o & 0x7ff) + k, v);
    } else if (o < 0x2f00) {
        if (!base_read(p, o, &base) ||
            !shm_index(p, (uint16_t)(base + (o & 0x7f) + k), &i)) {
            return false;
        }
        *v = p->shm[i];
    } else if (o < 0x2f80) {
        *v = p->scr[((o & 0x7f) + k) % SCR_N];
    } else if (o < 0x3000) {
        psm_abort(p, "operand 0x%04x (base field 15) unknown", o);
        return false;
    } else {
        *v = k ? ((o & 0x800) ? 0xffff : 0) : (uint16_t)sextract32(o, 0, 12);
    }
    return true;
}

static bool opnd_wr16(Bcm4387Psm *p, uint16_t o, int k, uint16_t v)
{
    uint16_t i, base;

    if (o < 0x2000) {
        if (!shm_index(p, o + k, &i)) {
            return false;
        }
        p->shm[i] = v;
    } else if (o < 0x2800) {
        return ihr_write(p, (o & 0x7ff) + k, v);
    } else if (o < 0x2f00) {
        if (!base_read(p, o, &base) ||
            !shm_index(p, (uint16_t)(base + (o & 0x7f) + k), &i)) {
            return false;
        }
        p->shm[i] = v;
    } else if (o < 0x2f80) {
        p->scr[((o & 0x7f) + k) % SCR_N] = v;
    } else if (o < 0x3000) {
        psm_abort(p, "operand 0x%04x (base field 15) unknown", o);
        return false;
    }
    /* immediate destination: result discarded [E] */
    return true;
}

static bool opnd_rd(Bcm4387Psm *p, uint16_t o, bool w32, uint32_t *v)
{
    uint16_t lo, hi = 0;

    if (!opnd_rd16(p, o, 0, &lo) || (w32 && !opnd_rd16(p, o, 1, &hi))) {
        return false;
    }
    *v = (uint32_t)hi << 16 | lo;
    return true;
}

static bool opnd_wr(Bcm4387Psm *p, uint16_t o, bool w32, uint32_t v)
{
    return opnd_wr16(p, o, 0, v) && (!w32 || opnd_wr16(p, o, 1, v >> 16));
}

/* ---- execution ---------------------------------------------------------- */

static bool cond_test(Bcm4387Psm *p, uint8_t c, bool *t)
{
    if (c == COND_TRUE) {
        *t = true;
        return true;
    }
    if (bcm4387_strict) {
        psm_abort(p, "external condition 0x%02x (source unknown)", c);
        return false;
    }
    if (!test_and_set_bit(c, p->cond_logged)) {
        qemu_log_mask(LOG_UNIMP, "bcm4387: PSM%d external condition 0x%02x "
                      "not modelled (false) pc 0x%04x\n", p->slice, c, p->pc);
    }
    *t = false;
    return true;
}

static bool jump_cond(uint16_t op, uint32_t a, uint32_t b, bool w32, bool *t)
{
    int32_t d = w32 ? (int32_t)(a - b) : (int16_t)(a - b);
    int32_t sa = w32 ? (int32_t)a : (int16_t)a;
    int32_t sb = w32 ? (int32_t)b : (int16_t)b;

    switch (op & ~0x60) {
    case 0x040: *t = !(a & b); break;               /* jnand */
    case 0x041: *t = (a & b) != 0; break;           /* jand */
    case 0x050: *t = (a & b) == a; break;           /* js */
    case 0x051: *t = (a & b) != a; break;           /* jns */
    case 0x090: *t = a == b; break;                 /* je */
    case 0x091: *t = a != b; break;                 /* jne */
    case 0x092: *t = sa < sb; break;                /* jls */
    case 0x093: *t = sa >= sb; break;               /* jges */
    case 0x094: *t = sa > sb; break;                /* jgs */
    case 0x095: *t = sa <= sb; break;               /* jles */
    case 0x096: *t = d < 0; break;                  /* jdn */
    case 0x097: *t = d >= 0; break;                 /* jdpz */
    case 0x098: *t = d > 0; break;                  /* jdp */
    case 0x099: *t = d <= 0; break;                 /* jdnz */
    case 0x09a: *t = a < b; break;                  /* jl */
    case 0x09b: *t = a >= b; break;                 /* jge */
    case 0x09c: *t = a > b; break;                  /* jg */
    case 0x09d: *t = a <= b; break;                 /* jle */
    default:
        return false;
    }
    return true;
}

static void unknown_op(Bcm4387Psm *p, uint16_t op)
{
    psm_abort(p, "opcode 0x%03x (semantics unknown)", op);
}

/* execute the instruction at pc; false once the PSM can no longer run */
static bool psm_step(Bcm4387Psm *p)
{
    uint64_t in = p->ucm[p->pc];
    uint16_t op = extract64(in, 42, 12);
    uint16_t o0 = extract64(in, 28, 14), o1 = extract64(in, 14, 14);
    uint16_t o2 = extract64(in, 0, 14);
    uint16_t next = (p->pc + 1) % UCM_INSNS;
    uint32_t a, b, r;
    bool t, w32 = op & 0x20;
    unsigned m = (op >> 4) & 0xf, sh = op & 0xf;

    trace_bcm4387_psm_insn(p->slice, p->pc, in);
    p->insns++;
    switch (op >> 8) {
    case 0x0:
        if (op == 0x004) {                          /* calls */
            if (p->sp >= STACK_N) {
                psm_abort(p, "call stack overflow (depth %d assumed)",
                          STACK_N);
                return false;
            }
            p->stack[p->sp++] = next;
            p->pc = o2;
            return true;
        }
        if (op == 0x005) {                          /* rets */
            if (!p->sp) {
                psm_abort(p, "return with an empty call stack");
                return false;
            }
            p->pc = p->stack[--p->sp];
            return true;
        }
        if (op == 0x001 || op == 0x002) {           /* nap, nap2 */
            p->pc = next;
            qemu_log_mask(LOG_GUEST_ERROR, "bcm4387: PSM%d strict abort: "
                          "nap%s at pc 0x%04x: wake sources not modelled "
                          "(instruction #%" PRIu64 ")\n", p->slice,
                          op == 2 ? "2" : "", p->pc, p->insns);
            psm_set_state(p, PSM_NAP);
            return false;
        }
        if (op == 0x070 || op == 0x071 || op == 0x086 ||
            (w32 && (op & ~0x60) < 0x090)) {
            unknown_op(p, op);
            return false;
        }
        if (!opnd_rd(p, o0, w32, &a) || !opnd_rd(p, o1, w32, &b)) {
            return false;
        }
        if (!jump_cond(op, a, b, w32, &t)) {
            unknown_op(p, op);
            return false;
        }
        p->pc = t ? o2 : next;
        return true;
    case 0x1:
        if ((op & 0xf80) == 0x180) {                /* add/sub family */
            bool sub = op & 0x10;
            uint64_t full, mask = w32 ? 0xffffffffull : 0xffff;

            if (op & 0xc) {
                unknown_op(p, op);
                return false;
            }
            if (!opnd_rd(p, o0, w32, &a) || !opnd_rd(p, o1, w32, &b)) {
                return false;
            }
            if (sub) {
                full = (uint64_t)a - b - ((op & 1) && p->borrow);
                if (op & 2) {
                    p->borrow = full > mask;
                }
            } else {
                full = (uint64_t)a + b + ((op & 1) && p->carry);
                if (op & 2) {
                    p->carry = full > mask;
                }
            }
            if (!opnd_wr(p, o2, w32, full & mask)) {
                return false;
            }
            p->pc = next;
            return true;
        }
        if (op == 0x102) {
            if (!opnd_rd(p, o0, false, &a) || !opnd_rd(p, o1, false, &b)) {
                return false;
            }
            if (o2 < 0x2000 || o2 >= 0x2800) {
                psm_abort(p, "opcode 0x102 to operand 0x%04x", o2);
                return false;
            }
            if (!port_cmd(p, a, b, o2 & 0x7ff)) {
                return false;
            }
            p->pc = next;
            return true;
        }
        if (!opnd_rd(p, o0, false, &a) || !opnd_rd(p, o1, false, &b)) {
            return false;
        }
        switch (op) {
        case 0x101:                                 /* mul */
            r = (uint16_t)a * (uint16_t)b;
            p->ihr[IHR_MUL_LO] = r;
            r >>= 16;
            break;
        case 0x110: r = b >= 16 ? 0 : a << b; break;                /* sl */
        case 0x120: r = b >= 16 ? 0 : a >> b; break;                /* sr */
        case 0x130: r = (int16_t)a >> MIN(b, 15); break;            /* sra */
        case 0x140: r = a & b; break;
        case 0x150: r = a & ~b; break;              /* nand */
        case 0x160: r = a | b; break;
        case 0x170: r = a ^ b; break;
        default:
            unknown_op(p, op);
            return false;
        }
        if (!opnd_wr(p, o2, false, r & 0xffff)) {
            return false;
        }
        p->pc = next;
        return true;
    case 0x2:                                       /* srx */
    case 0x3:                                       /* orx */
    case 0x4:                                       /* jzx */
    case 0x5:                                       /* jnzx */
        if (!opnd_rd(p, o0, false, &a) || !opnd_rd(p, o1, false, &b)) {
            return false;
        }
        if ((op >> 8) == 3) {
            uint16_t mk = rol16((1u << (m + 1)) - 1, sh);

            r = (rol16(a, sh) & mk) | (b & ~mk);
            if (!opnd_wr(p, o2, false, r & 0xffff)) {
                return false;
            }
            p->pc = next;
            return true;
        }
        r = ((b << 16 | a) >> sh) & ((1u << (m + 1)) - 1);
        if ((op >> 8) == 2) {
            if (!opnd_wr(p, o2, false, r)) {
                return false;
            }
            p->pc = next;
            return true;
        }
        p->pc = ((op >> 8) == 4) == !r ? o2 : next;
        return true;
    case 0x6:                                       /* jnext */
    case 0x7:                                       /* jext */
        if (!cond_test(p, op & 0xff, &t)) {
            return false;
        }
        p->pc = ((op >> 8) == 7) == t ? o2 : next;
        return true;
    }
    unknown_op(p, op);
    return false;
}

static bool psm_clocked(Bcm4387Psm *p)
{
    return p->hooks.clocked(p->hooks.opaque, p->slice);
}

static void psm_tick(void *opaque)
{
    Bcm4387Psm *p = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int64_t budget = MIN(now - p->last_ns, 10 * TICK_NS) *
                     (PSM_HZ / 1000000) / 1000;

    p->last_ns = now;
    if (p->state != PSM_RUNNING) {
        return;
    }
    if (psm_clocked(p)) {
        while (budget-- > 0 && psm_step(p)) {
        }
    }
    if (p->state == PSM_RUNNING) {
        timer_mod(p->timer, now + TICK_NS);
    }
}

static void psm_update(Bcm4387Psm *p)
{
    bool run = (p->mctl & MCTL_PSM_RUN) && !(p->mctl & MCTL_PSM_JMP_0);

    if (p->mctl & MCTL_PSM_JMP_0) {
        p->pc = 0;
    }
    if (!run) {
        psm_set_state(p, PSM_HALTED);
        timer_del(p->timer);
        return;
    }
    if (p->state == PSM_HALTED) {
        qemu_log_mask(LOG_UNIMP, "bcm4387: PSM%d runs from pc 0x%04x\n",
                      p->slice, p->pc);
        psm_set_state(p, PSM_RUNNING);
        p->last_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        timer_mod(p->timer, p->last_ns + TICK_NS);
    }
}

void bcm4387_psm_control(Bcm4387Psm *p, uint32_t maccontrol)
{
    p->mctl = maccontrol;
    psm_update(p);
}

void bcm4387_psm_core_reset(Bcm4387Psm *p)
{
    p->pc = 0;
    p->sp = 0;
    p->carry = p->borrow = false;
    p->port_stuck[0] = p->port_stuck[1] = false;
    p->mctl = 0;
    psm_update(p);
}

/* ---- object memory ------------------------------------------------------ */

static bool obj_ucm(Bcm4387Psm *p, uint32_t w, bool wr, uint32_t *v)
{
    uint32_t i = w >> 1, sh = (w & 1) * 32;

    if (i >= UCM_INSNS) {
        qemu_log_mask(LOG_GUEST_ERROR, "bcm4387: d11.%d microcode word 0x%x "
                      "beyond the modelled 0x%x instructions\n", p->slice, w,
                      UCM_INSNS);
        return false;
    }
    if (wr) {
        p->ucm[i] = deposit64(p->ucm[i], sh, 32, *v);
    } else {
        *v = p->ucm[i] >> sh;
    }
    return true;
}

bool bcm4387_psm_objmem(Bcm4387Psm *p, hwaddr off, unsigned size, bool wr,
                        uint64_t *val)
{
    uint32_t sel = extract32(p->objaddr, 16, 7);
    uint32_t a = p->objaddr & 0xffff;
    bool hi = off == 0x166, full = off == 0x164 && size == 4;
    uint32_t v = *val;

    if (off == 0x160) {
        if (wr) {
            p->objaddr = *val;
        } else {
            *val = p->objaddr;
        }
        return true;
    }
    if ((off != 0x164 && off != 0x166) || (sel != 0 && sel != 1)) {
        return false;
    }
    if (sel == 0) {
        if (!full) {
            qemu_log_mask(LOG_GUEST_ERROR, "bcm4387: d11.%d %u-bit microcode "
                          "access at +0x%03" HWADDR_PRIx " not modelled\n",
                          p->slice, size * 8, off);
            return false;
        }
        if (!obj_ucm(p, a, wr, &v)) {
            return false;
        }
    } else if (2 * a + 1 >= SHM_WORDS) {
        qemu_log_mask(LOG_GUEST_ERROR, "bcm4387: d11.%d SHM word 0x%x beyond "
                      "the modelled 0x%x words\n", p->slice, 2 * a,
                      SHM_WORDS);
        return false;
    } else if (wr) {
        p->shm[2 * a + hi] = v;
        if (full) {
            p->shm[2 * a + 1] = v >> 16;
        }
    } else {
        v = p->shm[2 * a + hi];
        if (full) {
            v |= (uint32_t)p->shm[2 * a + 1] << 16;
        }
    }
    if (!wr) {
        *val = v;
    }
    if ((full || hi) && (p->objaddr & (wr ? 0x01000000 : 0x02000000))) {
        p->objaddr = (p->objaddr & ~0xffffu) | ((a + 1) & 0xffff);
    }
    return true;
}

Bcm4387Psm *bcm4387_psm_new(int slice, const Bcm4387PsmHooks *hooks)
{
    Bcm4387Psm *p = g_new0(Bcm4387Psm, 1);

    p->slice = slice;
    p->hooks = *hooks;
    p->ucm = g_new0(uint64_t, UCM_INSNS);
    p->ihr_logged = bitmap_new(IHR_N * 2);
    p->cond_logged = bitmap_new(256);
    p->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, psm_tick, p);
    return p;
}

/* power-on: memories cleared as well */
void bcm4387_psm_reset(Bcm4387Psm *p)
{
    memset(p->ucm, 0, UCM_INSNS * sizeof(*p->ucm));
    memset(p->shm, 0, sizeof(p->shm));
    memset(p->scr, 0, sizeof(p->scr));
    memset(p->ihr, 0, sizeof(p->ihr));
    p->objaddr = 0;
    p->insns = 0;
    bcm4387_psm_core_reset(p);
}
