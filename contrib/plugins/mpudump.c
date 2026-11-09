/*
 * Capture ARMv7-R PMSA MPU region programming by decoding the Thumb
 * `mcr p15,0,Rt,c6,c1/c2,{opc2}` writes and reading Rt at execution time.
 * Tracks the current region (RGNR = c6,c2,0) and logs DRBAR/DRSR/DRACR
 * (c6,c1,{0/2/4}) per region. Prints a decoded region table at exit.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include <glib.h>
#include <inttypes.h>

#include <qemu-plugin.h>

QEMU_PLUGIN_EXPORT int qemu_plugin_version = QEMU_PLUGIN_VERSION;

enum { DRBAR, DRSR, DRACR };
static uint32_t drbar[32], drsr[32], dracr[32];
static int rgnr;
static struct qemu_plugin_register *reg[16];

static uint32_t rd(int n)
{
    g_autoptr(GByteArray) b = g_byte_array_new();
    uint32_t v = 0;

    if (n >= 0 && n < 16 && reg[n] &&
        qemu_plugin_read_register(reg[n], b) && b->len >= 4) {
        memcpy(&v, b->data, 4);
    }
    return v;
}

static void vcpu_init(unsigned int vcpu_index, void *ud)
{
    g_autoptr(GArray) regs = qemu_plugin_get_registers();

    for (guint i = 0; i < regs->len; i++) {
        qemu_plugin_reg_descriptor *d =
            &g_array_index(regs, qemu_plugin_reg_descriptor, i);
        if (d->name[0] == 'r' && g_ascii_isdigit(d->name[1])) {
            int n = atoi(d->name + 1);
            if (n >= 0 && n < 16) {
                reg[n] = d->handle;
            }
        }
    }
}

/* userdata encodes: (kind<<4)|rt, kind 0=RGNR 1=DRBAR 2=DRSR 3=DRACR */
static void mcr_exec(unsigned int vcpu_index, void *ud)
{
    uintptr_t u = (uintptr_t)ud;
    int kind = u >> 4, rt = u & 0xf;
    uint32_t v = rd(rt);

    switch (kind) {
    case 0:
        rgnr = v & 0x1f;
        break;
    case 1:
        drbar[rgnr] = v;
        break;
    case 2:
        drsr[rgnr] = v;
        break;
    case 3:
        dracr[rgnr] = v;
        break;
    }
}

static void tb_trans(struct qemu_plugin_tb *tb, void *ud)
{
    size_t n = qemu_plugin_tb_n_insns(tb);

    for (size_t i = 0; i < n; i++) {
        struct qemu_plugin_insn *insn = qemu_plugin_tb_get_insn(tb, i);
        uint8_t b[4];
        uint16_t hw0, hw1;
        int kind = -1;

        if (qemu_plugin_insn_data(insn, b, 4) < 4) {
            continue;
        }
        hw0 = b[0] | (b[1] << 8);
        hw1 = b[2] | (b[3] << 8);
        if (hw0 != 0xee06) {            /* mcr p15,0,Rt,c6,... */
            continue;
        }
        switch (hw1 & 0x0fff) {
        case 0xf12: kind = 0; break;    /* c6,c2,0  RGNR  */
        case 0xf11: kind = 1; break;    /* c6,c1,0  DRBAR */
        case 0xf51: kind = 2; break;    /* c6,c1,2  DRSR  */
        case 0xf91: kind = 3; break;    /* c6,c1,4  DRACR */
        default: continue;
        }
        qemu_plugin_register_vcpu_insn_exec_cb(
            insn, mcr_exec, QEMU_PLUGIN_CB_R_REGS,
            (void *)(uintptr_t)((kind << 4) | ((hw1 >> 12) & 0xf)));
    }
}

static void plugin_exit(void *ud)
{
    qemu_plugin_outs("mpudump: region  base        size      XN AP   enabled\n");
    for (int i = 0; i < 32; i++) {
        if (!drsr[i] && !drbar[i] && !dracr[i]) {
            continue;
        }
        uint32_t en = drsr[i] & 1;
        uint32_t sz = (drsr[i] >> 1) & 0x1f;
        uint64_t bytes = 1ull << (sz + 1);
        uint32_t xn = (dracr[i] >> 12) & 1;
        uint32_t ap = (dracr[i] >> 8) & 7;
        g_autofree char *s = g_strdup_printf(
            "mpudump: %2d      0x%08x  0x%09" PRIx64 " %u  %u    %u  "
            "(drbar=%08x drsr=%08x dracr=%08x)\n",
            i, drbar[i] & ~0x1f, bytes, xn, ap, en,
            drbar[i], drsr[i], dracr[i]);
        qemu_plugin_outs(s);
    }
}

QEMU_PLUGIN_EXPORT int qemu_plugin_install(qemu_plugin_id_t id,
                                           const qemu_info_t *info,
                                           int argc, char **argv)
{
    qemu_plugin_register_vcpu_init_cb(id, vcpu_init, NULL);
    qemu_plugin_register_vcpu_tb_trans_cb(id, tb_trans, NULL);
    qemu_plugin_register_atexit_cb(id, plugin_exit, NULL);
    return 0;
}
