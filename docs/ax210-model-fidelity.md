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
| E1 | ~~LMAC has an undocumented instruction-set extension~~ | **contradicts → resolved by L1** | Wrong conclusion, kept for the record. The "extension opcodes" are ordinary ARCv2 16-bit encodings; every earlier `cpu=arcem/archs` comparison used an ELF tagged ARC700, which constrains objdump to ARCompact. See L1. |
| L1 | **LMAC is an ARCv2 (ARC EM-class) core; UMAC and IML are ARCompact (ARC700)** — two different ISAs on one die | **yes**, run as `-cpu arcem` | [I], quantitatively strong: with an ARCv2-tagged ELF, LMAC decodes 0.0% undecodable (12-15% as arc700) while UMAC/IML do the reverse (ARCv2 rejects their `mov_s b,h` and garbles `mov_s r,limm`). LMAC reset stub (`0x60`) reads as ARCv2: `mov_s r1..r25,r0` (`0x4100..0x4118`), then KSTACK_TOP/KSTACK_BASE (aux `0x264/0x265`, EM stack checking) bracket the sp it set; code uses ARCv2-only `enter_s/leave_s/dbnz/wevt`. |
| L2 | LMAC boots from its reset vector through init to its main event loop (`seti; sleep; clri; check [0x4f50e8]` at `0x4d08fa`) | **partial** | [F] behavioural, under X1-X2, M2, P1, W1, V1, F1, S1, A1, Q1 below; loaded sections `0x0-0x18000`, `0x4b5000-0x4f8000`, DCCM `0x800000`. Interrupt sources that would give it work are [?]. |
| X1 | LMAC vendor aux registers (e.g. `0x20a` at reset, `0x5d`) | modelled, [I] | Read back last write, traced `AUX.R/W` (board hook `arc_unknown_aux_*` in `target/arc/op_helper.c`; default stays an instruction error). Meanings [?]. |
| X2 | LMAC extension core register `r32` is a read-only hardware status word | modelled, [I] | Only read (`btst/bbit/tst/xbfu`); code waits for bit 29 set after kicking an engine (`0x4bc4b0`, `0xc202`) and for bits 12/18/25 clear (`0x4bc48a`, `0xc5be`). Model: constant `1<<29` (`AX210_R32` overrides). Real bit meanings, and whether bit 29 ever drops, are [?]. |
| M2 | LMAC memory map = its MPU programming (13 regions, MPU enabled, no background access) | **yes** | [F] for the firmware's view: read live at the main loop (gdbstub `mpurdb*/mpurdp*`), decoded with ARCv2 RDB/RDP encoding from Zephyr `arch/arc/core/mpu/arc_mpu_v2_internal.h` and `include/zephyr/arch/arc/v2/mpu/arc_mpu.h` (commit f3fc2490). ICCM `0x0-0x17fff` (X), read-only `0x440000-0x63ffff`, code/data `0x4b4000-0x4fffff` (RWX), SRAM `0x600000-0x63ffff`, buffer `0x660800-0x660fff`, DCCM `0x800000-0x807fff`, MMIO `0xa00000-0xbfffff` and `0xd00000-0xefffff`. The board maps exactly these under `-cpu arcem`; boot to L2 still completes and no LMAC MMIO access falls outside the two MMIO regions. The paged image at `0x1000000+` is outside every LMAC region, so it is UMAC's ([I]). Tested `/ax210/lmac/mpu-map-ram`. |
| P1 | LMAC MMIO is `0xa00000-0xbfffff` and `0xd00000-0xefffff` | **yes** (extent) | Superseded by M2: those are the two RW regions that only ever see MMIO-style accesses. Earlier coarse window `0xa00000-0xffffff` retired. |
| W1 | LMAC indirect register window: `0xa2ac04` CTRL (addr[16:0], bit17 write, bit18 GO/BUSY), `0xa2ac00` DATA | modelled, [I] | LMAC `0x4be6f8` (read), `0x4be714` (write), `0x4be818` (wait while BUSY). BUSY self-clears at once; backing contents [?] (zero, traced `IND.R/W`). Tested `/ax210/lmac/indirect-window`. |
| V1 | LMAC timer/event unit at `0xa02000`: per-channel count at `+0xac+4n`, enable bit n `+0x0c`, start bit n `+0x08`; `+0x20` is a free-running timestamp | modelled, [I] | `0x166e4` (arm), `0x16668` (`wevt`, return `[+0x20]`), `0x4d5222/0x4d5280` (`deadline = [+0x20] + N`). Expiry wakes the core from `wevt/sleep` as an event (IRQ line [?]); 1 tick = 1 us is a guess (makes ch10 `200000` = 200 ms). Relative vs absolute compare [?]. Tested `/ax210/lmac/timestamp`. |
| F1 | Power/clock request FSM at `0xd03380`, SET `+0x8` / CLR `+0x4`; state `[3:0]` = 5 when bit 4 set, 0 when clear | modelled, [I] | `0x4d525c` (SET `0x04000010`, wait state 5, error `0xfbc`), `0x4d520c` (CLR `0x10`, wait state 0, error `0xfbd`). Transition latency [?] (model: immediate). Tested `/ax210/lmac/power-fsm`. |
| S1 | Hardware semaphore bank at `0xd03800`: per-core request/granted/release, LMAC `+0x00/+0x04/+0x08`, UMAC `+0x20/+0x24/+0x28` | modelled, [I] | LMAC `0x4ba960`, `0x4d1c88`; UMAC `0xc008d258/0xc008d278` uses the `+0x20` set at boot. Each bit owned by at most one core. Tested `/ax210/lmac/semaphore`, `/ax210/semaphore-two-cores`. |
| A1 | **AES-128 engine at `0xa01800`** | **yes** (function), [I] (control bits) | [F]: LMAC's boot self-test (`0x4d9ba0`, table `0x4e1828`) is FIPS-197 Appendix C.1 (key `000102..0f`, pt `00112233..ff`, ct `69c4e0d8 6a7b0430 d8cdb780 70b4c55a`, recomputed independently). Packing pinned by that vector: key FIFO `+0x08` low word first, byte-swapped; data/result `+0x98+4j` = BE word `3-j`. [I]: ctrl `+0x94` bit0 start, bit1 encrypt, status bit2 done; decrypt path unverified. Tested `/ax210/lmac/aes-fips197-c1`. |
| Q1 | LMAC outgoing message FIFO at `0xa00400`: capacity `+0x30`, fill `+0x38`, push `+0x40` | modelled, [I] | Send routine `0x13140(ptr,len,id)`: free = `max(cap - fill - 8, 0)`; asserts with the message id when full (the `0x3601` assert seen when both read 0). Model: consumer drains at once, capacity `0x400` (guess); pushes traced `MSG`. Tested `/ax210/lmac/msg-fifo-space`. |
| B1 | IML runs to completion (sets its done flag, idles) given R1+T1+M1 overrides on `c0d02c7c`, `c0e27cec`, `c0d02ecc`, `c0d02c00`, plus a host CSR seed at `0x118`/`0x11c` | **partial** | [F] behavioral observation under this model, not a claim the real device would do the same with the same stimulus — the model is permissive (write-readback, unauthenticated CSR values) in ways real silicon need not be. See audit report §17/§18. |
| U1 | UMAC entry point | **yes** | [F] from IML code: `mov r16,0xc0080000` (`0x80444814`) then `sr r16,[int_vector_base]; jl [r16]` (`0x804448b8`). `0xc0080000` is UMAC's ARC700 vector table, reset slot jumps to `0xc0080050`. |
| U2 | UMAC boots on `-cpu arc700` through early init, the context-info gate (H1), the register self-test (K2), its first message send (Q2), the `0xd03f00` start (G1) and its thread scheduler to its idle `sleep` (`0x804a6126`) | **partial** | [F] behavioural, with `AX210_HOSTFW` and `AX210_IND=940c=1` (I1), under A2-A7. Harness enters through the IML handoff (U1), so `int_vector_base` = `0xc0080000`. Woken every 200 ms by IRQ6 (G2), which it handles and returns to idle; IRQ5 and IRQ8 sources are [?]. |
| U3 | UMAC fatal path signals the host with `CSR_INT_BIT_SW_ERR` | **yes** (meaning) | [F]: `0x80483c8e-0x80483cc2` writes bit 25 to host CSR `0x008` (`CSR_INT`, `iwl-csr.h:31`; `CSR_INT_BIT_SW_ERR` = "uCode error", line 201) or, when on-die `0xc0d05c00` bit 25 is set, to `0x2808` (`CSR_MSIX_HW_INT_CAUSES_AD`, line 602), then halts at `0x804a5be4` (`[0xc0d03864] = 1; b .`). Also corroborates M1's offset space. |
| C2 | Host CSR defaults: `CSR_HW_REV` (`0x028`) type `0x420`, `CSR_HW_IF_CONFIG_REG[3:0]` = step/dash | modelled; [F] driver behaviour, [?] value | Type `CSR_HW_REV_TYPE_TY = 0x420` (`iwl-csr.h:358`); `iwl_mvm_nic_config` copies HW_REV[3:0] into HW_IF_CONFIG (`mvm/ops.c:100`, from `pcie/trans-gen2.c:273`). IML `0x80445a2c` / UMAC `0x8044a702` require dash 2 when on-die `[0xd03400] & 7 != 2`, else dash 1 (error `0x2fc`). Model: step A, dash 2, on-die 0 — consistent, but which pairing real silicon has is [?]. Replaces the earlier `c0d00028=2` override. Tested `/ax210/host-csr-driver-defaults`. |
| K1 | Cross-core address view: LMAC `0x00axxxxx` == UMAC `0xc0exxxxx`; LMAC `0x00d0xxxx` == UMAC `0xc0d0xxxx` | modelled | First half [F]: the IML converts addresses with `(a & 0xff0fffff) \| 0xc0e20000` (`0x804441a6`, `0x80447082`). Second half [I]: UMAC's semaphore set (`0xc0d03820/24/28`) is the LMAC bank at `0xd03800` + `0x20`. Shared models are keyed by the LMAC-view address. Tested `/ax210/semaphore-two-cores`. |
| H1 | Host driver model: context info, prph scratch, DRAM map, IML and CSRs as iwlwifi leaves them (`AX210_HOSTFW=<ucode>`) | **yes** (layout), modelled | [F] driver: `pcie/ctxt-info-v2.c` `iwl_pcie_ctxt_info_v2_alloc/_kick`, `pcie/ctxt-info.c` `iwl_pcie_init_fw_sec`, `fw/api/ctxt-info-v2.h` (torvalds/linux `eedaa7d6`). [F] firmware agrees: UMAC `0x804485e4` DMAs `0x68` bytes (`sizeof(iwl_context_info_v2)`), follows `prph_scratch_base_addr` (`+0x58`, error `0x1000aa` if 0), DMAs `1660` = `offsetofend(iwl_prph_scratch, dram.common)`, checks `version.mac_id` against `CSR_HW_REV` (error `0x1000ac`), stores `prph_info_base_addr`; `0x80448710` decodes `control_flags` bits 4/8-11 and `hwm_cfg` at their driver offsets. Values: `rx_buf_size` 4K default (`iwl-modparams.h`) so `control_flags = 0x000f0000`; `mtr_size = 4`, `mcr_size = 12`. Simplifications: page-granular bump allocator at `0x20000000`; rings allocated but empty; no debug destination. Tested `/ax210/host-driver-ctxt-info`. |
| A2 | ARC700 `AUX_IRQ_LEV` (`0x200`) | **yes** (storage) | Architectural ARCv1 register, declared but without get/set in `target/arc`, so UMAC's `sr 0x100,[aux_irq_lev]` (`0x80447836`) raised an instruction error. Now read/write storage; level semantics are unused because this fork has no ARCv1 interrupt delivery. |
| A3 | ARC700 `extw`/`sexw` (32-bit and `_s`) | **yes** | Architectural; decoded but unmapped in `semfunc-mapping.def` (ARCv2 names them `exth`/`sexh`), so UMAC `0x8044911a` raised an instruction error. Mapped to the `EXTH`/`SEXH` semantics. |
| A4 | ARCompact exception entry: vector numbers `0x20-0x26` above Instruction Error, 8-byte slots, PC = slot | **yes** | Architectural (Linux `arch/arc` `arcregs.h` `ECR_V_*`, `entry-compact.S` vector table of `j` instructions). The fork used ARCv2 entry (load handler address from the table), jumping into instruction words. UMAC's table at `0xc0080000` is `j` slots (U1). |
| A5 | Actionpoint registers `AP_AMV/AMM/AC 0-7` (aux `0x220-0x237`) | **yes** (storage) | Declared without get/set for every family. UMAC's context switch (`0xc008159e`) writes per-thread stack bounds to them and to `0x260/0x261` (stack check, board hook X1). Triggering is not modelled. |
| A6 | ARC700 XMAC `AUX_MACMODE`, `AUX_XMAC0-2` (aux `0x41`, `0x44-0x46`) | **yes** (storage) | Saved/restored per thread by UMAC (`0xc00816f6`); no MAC instruction executed so far. |
| A7 | ARCompact `RTIE`: encoding `0x242F003F`; AE → `ERET`, else A2 → `ILINK2/STATUS32_L2/BTA_L2`, else `ILINK1/STATUS32_L1/BTA_L1` | **yes**, [F] firmware | The table knew only `0x246F003F` (same zero-operand instruction, `u6` form), and `RTIE` followed ARCv2 rules. UMAC starts threads from non-interrupt context (`STATUS32` = 0) by loading `ILINK1`, `STATUS32_L1` (aux `0xb`, now defined for ARCv1 with `0xc` `STATUS32_L2`) and `BTA_L1` and executing `rtie` (`0xc008171a`), which only works with the level-1 fallback. |
| A8 | ARCompact interrupt delivery: lines 3-31, `AUX_IENABLE`, `AUX_IRQ_LEV` levels, `E1/E2`/`A1/A2`, `ILINKn`/`STATUS32_Ln`/`BTA_Ln`, `ICAUSE1/2`, sticky `AUX_IRQ_LV12`, vector slot `8n` | **yes**, architectural | Register numbers and bits from Linux `arch/arc` (`irqflags-compact.h`, `entry-compact.S`: `AUX_IRQ_LV12` write-1-to-clear). `AUX_IENABLE` resets all-enabled: nothing in the `.ucode` writes it, yet UMAC handles IRQ5/6/8. UMAC's table at `0xc0080000` confirms the layout: IRQ3/4 (timers) are `flag 1` halts, IRQ5/6/8 have handlers, IRQ7/9 share an `ICAUSE`-reporting default; `0x100-0x120` are the A4 exception slots. Interrupt priority within a level: lowest line first. |
| K2 | Register aliases: `0xa200b0` reads `0xa3408c`; indirect `0xb800` reads `0xa34090` | modelled, [I] | UMAC `0x80448ca4` writes `0x400410` to each source and requires it back (first masked `0xff0fffff`), then writes the read-back value to the source; a mismatch sets status bit 1, fatal `0x1013f1`. No firmware names the read-side addresses otherwise. One register per pair is the simplest model consistent with that. Tested `/ax210/lmac/register-aliases`. |
| Q2 | UMAC outgoing message FIFO at `0xc0a00500` | modelled, [I] | Q1's layout in UMAC's own `0xc0a0xxxx` MMIO block (now traced, `0xc0a00000-0xc0a0ffff`): UMAC `0xc008d0a0` selects channel `0xe` at `+0x3c` as LMAC does at `0xa0043c`, free = `max(cap - fill - 8, 0)`, push via `0x804a8d48` to `+0x40`; error `0x100efa` when it does not fit. Modelled as Q1. Tested `/ax210/lmac/msg-fifo-space`. |
| I1 | Indirect `0x940c` bit 0 | [?], harness-supplied | UMAC `0x8044a49a` requires it after init call `0x8049f934` (else fatal `0x10133c`); no firmware writes it, so it is hardware status. Not modelled: supplied by the harness via `AX210_IND=940c=1` (seeds the W1 store). |
| G1 | Configure-and-start unit at `0xd03f00`: config `+0x0/+0x4/+0x8`, start `+0x10` bit 0, done `+0x14` bit 0 | modelled, [I] | UMAC `0x8044acd0` loads three words from `0xc088400c` (the last two are addresses it then writes through), `0x804a6c94` starts and waits ~20 timebase ticks for done, else fatal `0x101035`. No firmware on either core writes `+0x14`. Model: done follows start at once. Tested `/ax210/cfg-unit-start-done`. |
| G2 | UMAC timeout timer: write N to `0xd02c78` arms; expiry sets `0xd02e2c` bit 4 and raises IRQ6; writing bit 4 clears | modelled, [I]; tick rate [?] | UMAC's idle loop re-arms `0x30d40` at `0x804a66a0` before each `sleep`; its IRQ6 handler `0xc0080b30` writes `0x10` to `0xd02e2c` and re-arms. Model: one-shot, 1 tick = 1 us of virtual time. Under it UMAC takes IRQ6 every 200 ms from its idle `sleep` and returns. Tested `/ax210/umac-timeout-timer`. |

## Harness vs. tests

Two tiers exist, and they should not be confused:

- **`tests/qtest/ax210-test.c`** — real, build-system-integrated qtests
  (`qtests_arc` in `tests/qtest/meson.build`). These assert only the
  model's own defined mechanics (R1, T1, M1's proxy plumbing, D1's copy
  mechanic, override precedence, and for LMAC A1 against FIPS-197 C.1, F1,
  S1, W1, V1's timestamp, Q1) via direct MMIO access, with no CPU
  execution involved. They will catch regressions in the model; they
  prove nothing about silicon.
- **`~/Projects/ax210/scripts/*.py`** (GDB-RSP driven, in the separate
  audit workspace) — ad hoc, firmware-execution-driven probes used to
  *discover* R1/T1/M1/D1 in the first place, and to observe B1. These are
  evidence-gathering tools, not regression tests, and are cited above by
  address/PC rather than promoted into this table as if qtest-verified.

## Not modelled / open items

- Real timing of T1, or of anything else — this model has no timing
  fidelity goal (unlike the EFR32 project this methodology is modelled
  on, nothing here has silicon timing access to fit to).
- The board runs one core at a time (`-cpu arc700` for UMAC/IML, `-cpu arcem`
  for LMAC). The real die has both; S1 and Q1 are where they would meet.
- LMAC interrupt sources (V1 expiry, Q1 inbound, anything giving the main
  loop at `0x4d08fa` work): [?].
- UMAC IRQ5 and IRQ8 (level 2) sources: [?]. IRQ8's handler (`0xc0080658`) reads `0xd02fa0`, `0xd00ee0`, `0xd05498`, `0xd03944`, `0xc0a01c64`.
- Signatures UMAC `0x80448ca4` checks: `0xa340f0` == `0x4552414e` is written by UMAC itself later (`0x804490fc`), so it is a warm-boot marker; the writer of `0xa34048` (`|0xe == 0x5442440f`) is unknown. Both read 0 on this cold boot.
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
