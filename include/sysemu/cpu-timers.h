/*
 * CPU timers state API
 *
 * Copyright 2020 SUSE LLC
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 *
 */
#ifndef SYSEMU_CPU_TIMERS_H
#define SYSEMU_CPU_TIMERS_H

#include "qemu/timer.h"

/* init the whole cpu timers API, including icount, ticks, and cpu_throttle */
void cpu_timers_init(void);

/* icount - Instruction Counter API */

/**
 * ICountMode: icount enablement state:
 *
 * @ICOUNT_DISABLED: Disabled - Do not count executed instructions.
 * @ICOUNT_PRECISE: Enabled - Fixed conversion of insn to ns via "shift" option
 * @ICOUNT_ADAPTATIVE: Enabled - Runtime adaptive algorithm to compute shift
 */
typedef enum {
    ICOUNT_DISABLED = 0,
    ICOUNT_PRECISE,
    ICOUNT_ADAPTATIVE,
} ICountMode;

#if defined(CONFIG_TCG) && !defined(CONFIG_USER_ONLY)
extern ICountMode use_icount;
#define icount_enabled() (use_icount)
#else
#define icount_enabled() ICOUNT_DISABLED
#endif

/*
 * Update the icount with the executed instructions. Called by
 * cpus-tcg vCPU thread so the main-loop can see time has moved forward.
 */
void icount_update(CPUState *cpu);

/* get raw icount value */
int64_t icount_get_raw(void);

/* return the virtual CPU time in ns, based on the instruction counter. */
int64_t icount_get(void);
/*
 * convert an instruction counter value to ns, based on the icount shift.
 * This shift is set as a fixed value with the icount "shift" option
 * (precise mode), or it is constantly approximated and corrected at
 * runtime in adaptive mode.
 */
int64_t icount_to_ns(int64_t icount);

/**
 * icount_configure: configure the icount options, including "shift"
 * @opts: Options to parse
 * @errp: pointer to a NULL-initialized error object
 *
 * Return: true on success, else false setting @errp with error
 */
bool icount_configure(QemuOpts *opts, Error **errp);

/* used by tcg vcpu thread to calc icount budget */
int64_t icount_round(int64_t count);

/* if the CPUs are idle, start accounting real time to virtual clock. */
void icount_start_warp_timer(void);
void icount_account_warp_timer(void);
void icount_notify_exit(void);

/*
 * Per-CPU clocks under icount (opt-in, precise icount only).
 *
 * By default every instruction of every vCPU advances one global virtual
 * clock by 2^shift ns, so vCPUs that share the round-robin thread share one
 * instruction rate.  With per-CPU clocks each vCPU has its own clock rate
 * and keeps its own virtual time.  Virtual time advances in rounds of at
 * most @quantum_ns, never past the next QEMU_CLOCK_VIRTUAL deadline; in a
 * round every runnable vCPU executes until its own time reaches the round
 * end, and QEMU_CLOCK_VIRTUAL read from inside a vCPU returns that vCPU's
 * own time.  A vCPU that overshoots the round end (an instruction or an
 * access charge cannot be split) carries the excess into the next round, so
 * no time is lost or invented.  A halted vCPU woken by another vCPU resumes
 * at the waker's time.
 *
 * icount_cpu_clocks_enable: switch the mode on, before the vCPUs start.
 * Returns false (and changes nothing) unless icount is in precise mode.
 */
bool icount_cpu_clocks_enable(int64_t quantum_ns);
bool icount_cpu_clocks_enabled(void);
/*
 * icount_set_cpu_clock: @cpu runs at @hz and an instruction takes
 * @cycles_per_insn_milli / 1000 cycles on average.  @hz = 0 stops the
 * clock: the vCPU keeps its state, does not execute and its time follows
 * the rounds.  May be called from the vCPU itself (e.g. on a clock-tree
 * register write): the instructions executed so far are accounted at the
 * old rate first.
 */
void icount_set_cpu_clock(CPUState *cpu, uint64_t hz,
                          uint32_t cycles_per_insn_milli);
/*
 * icount_set_cpu_cycle_counting: time  by cycles counted per instruction
 * class by its translator (one cycle per instruction plus the class's extra
 * cycles) instead of by an average per instruction; the average set by
 * icount_set_cpu_clock() then only sizes the slices.  Call before the vCPU
 * runs.  Only the Armv6-M Thumb translator counts cycles.
 */
void icount_set_cpu_cycle_counting(CPUState *cpu, bool on);
/* icount_set_cpu_exception_cycles: cycles each exception entry costs. */
void icount_set_cpu_exception_cycles(CPUState *cpu, uint64_t millicycles);
void icount_charge_exception(CPUState *cpu);
/*
 * icount_charge_cycles: add @millicycles / 1000 of @cpu's own cycles to its
 * time.  @cpu may be the executing vCPU (call before the access takes
 * effect) or another one (cycles it loses to the executing vCPU, e.g. bus
 * contention).  A no-op unless per-CPU clocks are on.
 */
void icount_charge_cycles(CPUState *cpu, uint64_t millicycles);

/*
 * Bus model of the per-CPU clocks.  A device calls icount_bus_access() for
 * every access a vCPU makes to it; the model's @access hook prices it as
 * (master, address, size, direction) -> extra cycles of that master and a
 * target class < ICOUNT_BUS_TARGETS.  The core charges the cycles and
 * counts the access in the master's per-round occupancy
 * (CPUState.icount_bus_accesses).  @access may also charge other vCPUs
 * through icount_charge_cycles() for contention.  @round_end, if set, runs
 * when a round completes, before the occupancy is cleared, so a model can
 * resolve contention from what every master did in the round.
 */
typedef struct IcountBusModel {
    uint64_t (*access)(void *opaque, CPUState *master, uint64_t addr,
                       unsigned size, bool is_write, unsigned *target);
    void (*round_end)(void *opaque, int64_t start_ns, int64_t end_ns);
    void *opaque;
} IcountBusModel;
void icount_register_bus_model(IcountBusModel *model);
void icount_bus_access(IcountBusModel *model, CPUState *master,
                       uint64_t addr, unsigned size, bool is_write);
/*
 * icount_cpu_clocks_wake: @cpu, if halted, is being woken; it resumes no
 * earlier than the caller's time (the running vCPU's, else the global
 * time).  Called on every interrupt raise; a device that un-halts a vCPU by
 * other means calls it first.
 */
void icount_cpu_clocks_wake(CPUState *cpu);

/*
 * CPU Ticks and Clock
 */

/* Caller must hold BQL */
void cpu_enable_ticks(void);
/* Caller must hold BQL */
void cpu_disable_ticks(void);

/*
 * return the time elapsed in VM between vm_start and vm_stop.
 * cpu_get_ticks() uses units of the host CPU cycle counter.
 */
int64_t cpu_get_ticks(void);

/*
 * Returns the monotonic time elapsed in VM, i.e.,
 * the time between vm_start and vm_stop
 */
int64_t cpu_get_clock(void);

void qemu_timer_notify_cb(void *opaque, QEMUClockType type);

/* get/set VIRTUAL clock and VM elapsed ticks via the cpus accel interface */
int64_t cpus_get_virtual_clock(void);
void cpus_set_virtual_clock(int64_t new_time);
int64_t cpus_get_elapsed_ticks(void);

#endif /* SYSEMU_CPU_TIMERS_H */
