/*
 * Log every call into a ROM window from code outside it: caller (lr),
 * target, r0-r3 and the first four stack words. Consecutive repeats of the
 * same (caller, target) collapse into one line with a count.
 *
 * Arguments:
 *   lo=ADDR,hi=ADDR  ROM window [lo, hi) (default 0x400000-0x59b000)
 *   out=FILE         output TSV (default romcall.tsv)
 *   start=ADDR       begin logging when a TB at ADDR first runs (default 0)
 *
 * A call is a ROM TB entered from a TB outside the window whose lr does not
 * point back at it (that would be a RAM callee returning into ROM).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include <glib.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include <qemu-plugin.h>

QEMU_PLUGIN_EXPORT int qemu_plugin_version = QEMU_PLUGIN_VERSION;

static uint64_t lo = 0x400000, hi = 0x59b000, start;
static bool logging;
static FILE *out;
static uint64_t prev_tb, seq;
static struct qemu_plugin_register *reg[16];

/* the pending (collapsed) record */
static bool have;
static uint32_t rec[11];        /* lr, prev, target, r0-r3, s0-s3 */
static uint64_t rec_seq, rec_count;

static bool in_rom(uint64_t a)
{
    return a >= lo && a < hi;
}

static uint32_t rd(int n)
{
    g_autoptr(GByteArray) buf = g_byte_array_new();
    uint32_t v = 0;

    if (reg[n] && qemu_plugin_read_register(reg[n], buf) && buf->len >= 4) {
        memcpy(&v, buf->data, 4);
    }
    return v;
}

static void flush(void)
{
    if (!have) {
        return;
    }
    fprintf(out, "%" PRIu64 "\t%08x\t%08x\t%08x\t%08x\t%08x\t%08x\t%08x"
            "\t%08x\t%08x\t%08x\t%08x\t%" PRIu64 "\n", rec_seq,
            rec[0], rec[1], rec[2], rec[3], rec[4], rec[5], rec[6],
            rec[7], rec[8], rec[9], rec[10], rec_count);
    have = false;
}

static void vcpu_init(unsigned int vcpu_index, void *userdata)
{
    g_autoptr(GArray) regs = qemu_plugin_get_registers();

    for (guint i = 0; i < regs->len; i++) {
        qemu_plugin_reg_descriptor *d =
            &g_array_index(regs, qemu_plugin_reg_descriptor, i);
        int n = -1;

        if (d->name[0] == 'r' && g_ascii_isdigit(d->name[1])) {
            n = atoi(d->name + 1);
        } else if (!strcmp(d->name, "sp")) {
            n = 13;
        } else if (!strcmp(d->name, "lr")) {
            n = 14;
        }
        if (n >= 0 && n < 16) {
            reg[n] = d->handle;
        }
    }
}

static void tb_outside(unsigned int vcpu_index, void *userdata)
{
    prev_tb = (uint64_t)userdata;
    if (!logging && prev_tb == start) {
        logging = true;
    }
}

static void tb_rom(unsigned int vcpu_index, void *userdata)
{
    uint64_t tb = (uint64_t)userdata, from = prev_tb;
    uint32_t lr, sp, st[4] = { 0 };
    g_autoptr(GByteArray) buf = NULL;

    prev_tb = tb;
    if (!logging && tb == start) {
        logging = true;
    }
    if (!logging || in_rom(from)) {
        return;
    }
    lr = rd(14);
    if ((lr & ~1u) == tb) {
        return;                         /* RAM callee returning into ROM */
    }
    seq++;
    if (have && rec[0] == lr && rec[2] == tb) {
        rec_count++;
        return;
    }
    flush();
    sp = rd(13);
    buf = g_byte_array_new();
    if (qemu_plugin_read_memory_vaddr(sp, buf, 16) && buf->len >= 16) {
        memcpy(st, buf->data, 16);
    }
    rec[0] = lr;
    rec[1] = from;
    rec[2] = tb;
    for (int i = 0; i < 4; i++) {
        rec[3 + i] = rd(i);
        rec[7 + i] = st[i];
    }
    rec_seq = seq;
    rec_count = 1;
    have = true;
}

static void vcpu_tb_trans(struct qemu_plugin_tb *tb, void *userdata)
{
    uint64_t va = qemu_plugin_tb_vaddr(tb);

    if (in_rom(va)) {
        qemu_plugin_register_vcpu_tb_exec_cb(tb, tb_rom, QEMU_PLUGIN_CB_R_REGS,
                                             (void *)va);
    } else {
        qemu_plugin_register_vcpu_tb_exec_cb(tb, tb_outside,
                                             QEMU_PLUGIN_CB_NO_REGS,
                                             (void *)va);
    }
}

static void plugin_exit(void *userdata)
{
    flush();
    fclose(out);
}

QEMU_PLUGIN_EXPORT int qemu_plugin_install(qemu_plugin_id_t id,
                                           const qemu_info_t *info,
                                           int argc, char **argv)
{
    const char *path = "romcall.tsv";

    for (int i = 0; i < argc; i++) {
        g_auto(GStrv) kv = g_strsplit(argv[i], "=", 2);

        if (!kv[1]) {
            fprintf(stderr, "romcall: bad option %s\n", argv[i]);
            return -1;
        }
        if (!strcmp(kv[0], "lo")) {
            lo = g_ascii_strtoull(kv[1], NULL, 0);
        } else if (!strcmp(kv[0], "hi")) {
            hi = g_ascii_strtoull(kv[1], NULL, 0);
        } else if (!strcmp(kv[0], "start")) {
            start = g_ascii_strtoull(kv[1], NULL, 0);
        } else if (!strcmp(kv[0], "out")) {
            path = argv[i] + 4;
        } else {
            fprintf(stderr, "romcall: unknown option %s\n", kv[0]);
            return -1;
        }
    }
    out = fopen(path, "w");
    if (!out) {
        perror(path);
        return -1;
    }
    setvbuf(out, NULL, _IOFBF, 1 << 20);
    fprintf(out, "seq\tlr\tprev_tb\ttarget\tr0\tr1\tr2\tr3\tsp0\tsp4\tsp8\tsp12"
            "\tcount\n");
    logging = start == 0;
    qemu_plugin_register_vcpu_init_cb(id, vcpu_init, NULL);
    qemu_plugin_register_vcpu_tb_trans_cb(id, vcpu_tb_trans, NULL);
    qemu_plugin_register_atexit_cb(id, plugin_exit, NULL);
    return 0;
}
