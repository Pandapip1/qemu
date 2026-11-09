/*
 * Log the PC of every guest store whose target address is in [lo,hi), plus the
 * value if it can be recovered. Used to find who writes a relocated code range.
 *
 *   lo=ADDR,hi=ADDR   watched range (default 0x82c990-0x82c9a0)
 *   limit=N           stop after N hits (default 200)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include <glib.h>
#include <inttypes.h>

#include <qemu-plugin.h>

QEMU_PLUGIN_EXPORT int qemu_plugin_version = QEMU_PLUGIN_VERSION;

static uint64_t lo = 0x82c990, hi = 0x82c9a0, limit = 200, hits;
static uint64_t probe;   /* if set, dump r0/r1 + *[r0] at this PC */
static struct qemu_plugin_register *pc_reg, *r0_reg, *r1_reg;

static uint32_t rdreg(struct qemu_plugin_register *r)
{
    g_autoptr(GByteArray) b = g_byte_array_new();
    uint32_t v = 0;

    if (r && qemu_plugin_read_register(r, b) && b->len >= 4) {
        memcpy(&v, b->data, 4);
    }
    return v;
}

static uint32_t rdmem(uint64_t a)
{
    g_autoptr(GByteArray) b = g_byte_array_new();
    uint32_t v = 0;

    if (qemu_plugin_read_memory_vaddr(a, b, 4) && b->len >= 4) {
        memcpy(&v, b->data, 4);
    }
    return v;
}

static void vcpu_init(unsigned int vcpu_index, void *userdata)
{
    g_autoptr(GArray) regs = qemu_plugin_get_registers();

    for (guint i = 0; i < regs->len; i++) {
        qemu_plugin_reg_descriptor *d =
            &g_array_index(regs, qemu_plugin_reg_descriptor, i);
        if (!strcmp(d->name, "pc")) {
            pc_reg = d->handle;
        } else if (!strcmp(d->name, "r0")) {
            r0_reg = d->handle;
        } else if (!strcmp(d->name, "r1")) {
            r1_reg = d->handle;
        }
    }
}

static void probe_cb(unsigned int vcpu_index, void *userdata)
{
    uint32_t r0 = rdreg(r0_reg), r1 = rdreg(r1_reg);
    g_autofree char *s = g_strdup_printf(
        "memwatch: PROBE pc=0x%08" PRIx64 " r0=0x%08x r1=0x%08x "
        "[r0+0]=0x%08x [+4]=0x%08x [+8]=0x%08x | [+0x20]=0x%08x [+0x24]=0x%08x "
        "[+0x28]=0x%08x [+0x2c]=0x%08x\n",
        probe, r0, r1, rdmem(r0), rdmem(r0 + 4), rdmem(r0 + 8),
        rdmem(r0 + 0x20), rdmem(r0 + 0x24), rdmem(r0 + 0x28), rdmem(r0 + 0x2c));
    g_autofree char *s2 = g_strdup_printf("memwatch: MEMCHK [0x76ca40]=0x%08x [0x82c998]=0x%08x [0x76ca38]=0x%08x\n", rdmem(0x76ca40), rdmem(0x82c998), rdmem(0x76ca38));
    qemu_plugin_outs(s2);
    qemu_plugin_outs(s);
}

static uint32_t cur_pc(void)
{
    g_autoptr(GByteArray) b = g_byte_array_new();
    uint32_t v = 0;

    if (pc_reg && qemu_plugin_read_register(pc_reg, b) && b->len >= 4) {
        memcpy(&v, b->data, 4);
    }
    return v;
}

static void mem_cb(unsigned int vcpu_index, qemu_plugin_meminfo_t info,
                   uint64_t vaddr, void *udata)
{
    if (vaddr < lo || vaddr >= hi || !qemu_plugin_mem_is_store(info)) {
        return;
    }
    if (++hits > limit) {
        return;
    }
    g_autofree char *s = g_strdup_printf(
        "memwatch: store to 0x%08" PRIx64 " by pc~0x%08x (hit %" PRIu64 ")\n",
        vaddr, cur_pc(), hits);
    qemu_plugin_outs(s);
}

static void tb_trans(struct qemu_plugin_tb *tb, void *udata)
{
    size_t n = qemu_plugin_tb_n_insns(tb);

    for (size_t i = 0; i < n; i++) {
        struct qemu_plugin_insn *insn = qemu_plugin_tb_get_insn(tb, i);

        qemu_plugin_register_vcpu_mem_cb(insn, mem_cb, QEMU_PLUGIN_CB_R_REGS,
                                         QEMU_PLUGIN_MEM_W, NULL);
        if (probe && qemu_plugin_insn_vaddr(insn) == probe) {
            qemu_plugin_register_vcpu_insn_exec_cb(insn, probe_cb,
                                                   QEMU_PLUGIN_CB_R_REGS, NULL);
        }
    }
}

QEMU_PLUGIN_EXPORT int qemu_plugin_install(qemu_plugin_id_t id,
                                           const qemu_info_t *info,
                                           int argc, char **argv)
{
    for (int i = 0; i < argc; i++) {
        g_auto(GStrv) kv = g_strsplit(argv[i], "=", 2);

        if (!kv[1]) {
            continue;
        }
        if (!strcmp(kv[0], "lo")) {
            lo = g_ascii_strtoull(kv[1], NULL, 0);
        } else if (!strcmp(kv[0], "hi")) {
            hi = g_ascii_strtoull(kv[1], NULL, 0);
        } else if (!strcmp(kv[0], "limit")) {
            limit = g_ascii_strtoull(kv[1], NULL, 0);
        } else if (!strcmp(kv[0], "probe")) {
            probe = g_ascii_strtoull(kv[1], NULL, 0);
        }
    }
    qemu_plugin_register_vcpu_init_cb(id, vcpu_init, NULL);
    qemu_plugin_register_vcpu_tb_trans_cb(id, tb_trans, NULL);
    return 0;
}
