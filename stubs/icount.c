#include "qemu/osdep.h"
#include "qapi/error.h"
#include "sysemu/cpu-timers.h"

/* icount - Instruction Counter API */

ICountMode use_icount = ICOUNT_DISABLED;

bool icount_configure(QemuOpts *opts, Error **errp)
{
    /* signal error */
    error_setg(errp, "cannot configure icount, TCG support not available");

    return false;
}
int64_t icount_get_raw(void)
{
    abort();
    return 0;
}
void icount_start_warp_timer(void)
{
    abort();
}
void icount_account_warp_timer(void)
{
    abort();
}
void icount_notify_exit(void)
{
    abort();
}
bool icount_cpu_clocks_enable(int64_t quantum_ns)
{
    return false;
}
bool icount_cpu_clocks_enabled(void)
{
    return false;
}
void icount_set_cpu_clock(CPUState *cpu, uint64_t hz,
                          uint32_t cycles_per_insn_milli)
{
}
void icount_charge_cycles(CPUState *cpu, uint64_t millicycles)
{
}
void icount_cpu_clocks_wake(CPUState *cpu)
{
}
void icount_set_cpu_exception_cycles(CPUState *cpu, uint64_t millicycles)
{
}
void icount_charge_exception(CPUState *cpu)
{
}
void icount_register_bus_model(IcountBusModel *model)
{
}
void icount_bus_access(IcountBusModel *model, CPUState *master,
                       uint64_t addr, unsigned size, bool is_write)
{
}
