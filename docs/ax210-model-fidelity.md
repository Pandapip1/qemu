# AX210 board model fidelity

Scope: this document tracks the accuracy of `hw/arc/ax210.c`, the QEMU
machine model for the Intel Wi-Fi 6E AX210's on-die ARCompact (ARC700) MAC
processor. It exists purely to make and keep an accurate simulator; it is
not a vulnerability-hunting document (see the separate, user-held audit
report for that work, which this model was originally built to support).

Evidence tags, as used throughout: **[F]** fact — observed directly in a
named file, trace, or driver source line. **[I]** inferred — a model
built to explain observed firmware control flow, not independently
confirmed. **[?]** unknown — not modelled, and not safe to guess.

## The evidence gap vs. a debuggable SoC

A project reverse-engineering a debuggable microcontroller (SWD/JTAG, a
reference manual, silicon measurement via a probe) can tag most findings
[F]. The AX210's on-die ARC core has no such access path available here:
it is not a standalone debuggable chip, there is no public register
reference for its internal MMIO, and this session has no hardware probe
into a physical unit. The only oracles available are:

1. **The host driver source** (`linux-iwlwifi`, upstream `drivers/net/
   wireless/intel/iwlwifi`): authoritative [F] for the *host-side* PCI CSR
   offsets and the structures the driver DMAs to the device, because the
   kernel source is unambiguous and cited by file:line below.
2. **Static/dynamic disassembly of the signed firmware** (IML and
   LMAC/UMAC images from `linux-firmware`), run on the ARC700 QEMU core in
   this fork, cross-checked once against a ground truth (the IML's
   RSA-operand-load loop, §15.1 of the audit report: 64/64/64 iterations,
   r1=0x40 — this is the one point where the emulator's instruction
   semantics were checked against a known-correct behavioral signature,
   not just "it disassembles").
3. **Firmware control-flow behavior under this model itself**: what
   address a routine reads/writes next, in what order, is observed, but
   the *meaning* QEMU assigns to an unmodelled register is still ours to
   invent — so every register this model didn't get from (1) is [I] at
   best, and must be labelled as such rather than presented as measured.

Nothing below should be read as a claim about real silicon strength,
timing, or any property beyond "this is consistent with what the signed
firmware images do when run against this model."

## Findings

| # | Register / mechanism | Status | Evidence |
|---|---|---|---|
| R0 | ARC700 (ARCompact/ARCv1) is the core ISA | **yes** | [F] ground-truth match: IML reset stub decodes and executes correctly only as ARCv1/arc700; garbage under arcem/archs (ARCv2). Audit report §15/§15.1. |
| R1 | Unmodelled MMIO reads the last value written (sparse write-readback store) | modelled, [I] | Inferred from firmware set-bit/confirm sequences (e.g. `0xc0e20800` loop at IML `0x80445f68`); not derived from any driver or RM source — there is none. Tested: `tests/qtest/ax210-test.c` `/ax210/write-readback`. |
| T1 | `0xc0d02c68` is a free-running counter/timebase | modelled, [I] | IML delay loop at `0x80445ecc-0x80445ef2` waits for this address to advance by `0x14`; a constant value hangs it. No independent confirmation of the real tick rate — the model increments once per read, which is not a claim about real time. Tested: `/ax210/default-counter-increments`. |
| M1 | Host-CSR proxy at `0xc0d0001c`(cmd)/`0x20`(index)/`0x24`(data)/`0x28`(result) | modelled, [I] | Firmware at IML `0x8044470e-0x8044471e` writes an index then polls a result; the index values observed (`0x118`, `0x11c`) match `CSR_CTXT_INFO_ADDR` split lo/hi, a real [F] host-side offset: `drivers/net/wireless/intel/iwlwifi/fw/api/ctxt-info-v2.h:14` (`#define CSR_CTXT_INFO_ADDR 0x118`). The command-byte encoding (`0x69`=write, `0x6a`=read) and the exact field widths are **not** independently confirmed — only self-consistent with this one code path. Tested: `/ax210/host-csr-proxy-seed-and-read`. |
| D1 | DMA engine at `0xc0a0a500` (cmd/dev-addr/host-addr-lo/hi/len at `+0x34/+0x38/+0x3c/+0x40/+0x44`), completion = bit 29 of `0xc0d02e24` | modelled, [I] | Inferred purely from the IML routine at `0x804458a4` (push/pop pattern around four register args) and the poll loop at `0x804458c8`. [?] whether this is really a generic DMA engine, a doorbell, or something else; the field layout is a guess that happens to let the IML proceed. Tested: `/ax210/dma-copies-host-to-device`, `/ax210/override-wins-over-store` (precedence rule only). |
| C1 | `CSR_CTXT_INFO_ADDR=0x118`, `CSR_IML_DATA_ADDR=0x120`, `CSR_IML_SIZE_ADDR=0x128`, `CSR_CTXT_INFO_BOOT_CTRL=0x0`, `CSR_AUTO_FUNC_BOOT_ENA=BIT(1)` | **yes**, host-side only | [F] `fw/api/ctxt-info-v2.h:13-22`. These are real, cited constants; what is [I] is only the *proxy mechanism* (M1) by which the on-die core reaches them, since the core has no direct PCI BAR access of its own. |
| E1 | LMAC has an undocumented ~16-bit instruction-set extension (opcodes in the `0x40xx`-`0x47xx` 16-bit-major-0x08 range, extension registers r32+) | **no**, blocking | [F]: faults at LMAC reset+6 (`ECR=0x20000`, illegal instruction) in every binutils ARC class (arc700/arc600/nps400/arcem/archs) and in this QEMU fork; ~6% of decoded LMAC lines are opaque `.word`s vs ~0% in UMAC. [?] semantics: not in SSTIC 2021 (Iooss) or SSTIC 2022 (Campana/Iooss), the only two public sources checked (see audit report §18 for citations). **Not guessed at; LMAC cannot be emulated past its reset vector until this is resolved.** |
| B1 | IML runs to completion (sets its done flag, idles) given R1+T1+M1 overrides on `c0d02c7c`, `c0e27cec`, `c0d02ecc`, `c0d02c00`, plus a host CSR seed at `0x118`/`0x11c` | **partial** | [F] behavioral observation under this model, not a claim the real device would do the same with the same stimulus — the model is permissive (write-readback, unauthenticated CSR values) in ways real silicon need not be. See audit report §17/§18. |
| U1 | UMAC vector/entry point | **no** | [?]: not recoverable from the firmware file; the real device presumably derives it from the host-DMA'd context info (D1/M1), which this model has not driven far enough to test. |

## Harness vs. tests

Two tiers exist, and they should not be confused:

- **`tests/qtest/ax210-test.c`** — real, build-system-integrated qtests
  (`qtests_arc` in `tests/qtest/meson.build`). These assert only the
  model's own defined mechanics (R1, T1, M1's proxy plumbing, D1's copy
  mechanic, override precedence) via direct MMIO access, with no CPU
  execution involved. They will catch regressions in the model; they
  prove nothing about silicon.
- **`scratchpad/ax210/scripts/*.py`** (GDB-RSP driven, in the separate
  audit workspace) — ad hoc, firmware-execution-driven probes used to
  *discover* R1/T1/M1/D1 in the first place, and to observe B1. These are
  evidence-gathering tools, not regression tests, and are cited above by
  address/PC rather than promoted into this table as if qtest-verified.

## Not modelled / open items

- Real timing of T1, or of anything else — this model has no timing
  fidelity goal (unlike the EFR32 project this methodology is modelled
  on, nothing here has silicon timing access to fit to).
- E1 (LMAC extension ISA): blocking, see above.
- U1 (UMAC entry/vector table derivation from host context info).
- The crypto coprocessor path (`0xa248xx`/`0xc0e2xxxx`): traced as a
  no-op MMIO window; never exercised by the IML in any run captured so
  far, because no run has supplied a DMA'd firmware image for it to
  verify (D1/M1 would need to be driven further for that).
- Whether `0xc0d00000`-window addresses other than the ones listed above
  are read/write, RO, or unmapped on real silicon: **[?]**, all currently
  fall through to the catch-all tracer (R1) for lack of any contrary
  evidence.

## Rule going forward

Same rule as the project this format is drawn from: the model follows
evidence, not what makes the firmware happiest. A register whose only
support is "this value lets execution continue" stays tagged [I] with its
exact provenance (PC, address, what was observed) — it is never written
up as [F], and nothing is added to make firmware boot further unless the
new finding is logged here with its evidence tag first.
