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
it is not a standalone debuggable chip, there is no public reference
manual for its internal MMIO, and there is no AX210 on this machine to
probe. The oracles available are:

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

4. **Intel's own debug metadata in the `.ucode`**: the region TLVs list
   device memories by address and size and the periphery registers that
   exist (finding M3).
5. **Register names in `iwl-prph.h`**: a partial public register model
   (TXF/RXF FIFOs, FSEQ block, chip IDs, ...).
6. **Silicon snapshots from public AX210 error dumps**: iwlwifi prints the
   LMAC/UMAC error tables and the FSEQ/ID registers when firmware crashes.
   One dump is from a card running the same firmware API version, 89, but
   a different build (89.6b44fa0b; the model runs 89.b20dcd1f, sha256
   `792478ef...`): register values carry over, code addresses do not.
   Only registers the firmware never writes give hardware values.

**Where the evidence is:** `~/Projects/ax210/ref-silicon/README.md` indexes
every source (original URL, fetch date, local copy, which findings use it):
the saved dumps (`ref-silicon/*.txt`, raw pages beside them), the decoded
region TLVs (`ini_regions.txt`, `scripts/ini_regions.py`), and the Linux
headers (`ref-linux-iwl/`, `ref-linux-arc/`).

Rows marked **silicon** rest on (6); rows marked [F] vendor on (1), (4) or
(5). Everything else is "consistent with what the signed firmware images do
when run against this model", not a claim about real silicon.

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
| L2 | LMAC boots from its reset vector through init to its main event loop (`seti; sleep; clri; check [0x4f50e8]` at `0x4d08fa`) | **partial** | [F] behavioural, under X1-X2, M2, P1, W1, V1, F1, S1, A1, Q1 below; loaded sections `0x0-0x18000`, `0x4b5000-0x4f8000`, DCCM `0x800000`. Interrupt sources that would give it work are [?]. Silicon cross-check (`ref-silicon/fw89-framework16-dmesg.txt`, firmware 89.6b44fa0b vs the model's 89.b20dcd1f, after a UMAC fatal error): `LMAC1 CURRENT PC 0xd0` is the instruction after `flag 1` in the halt helper `0xc8` that the model also reaches when LMAC halts; LMAC `interruptlink1` `0x4d091a` falls inside `0x4d0914` (the routine after the main loop), near but not at the model's idle point, and is not an instruction boundary in this decoding; code addresses differ between builds, so it neither confirms nor contradicts L2. LMAC interrupt map [F]: its ARCv2 vector table (`0x0`, 24 entries) wires only IRQ19 (`0x1bbc`: timer/event unit, reads the `0xa02020` timestamp and `0xa01e60`) and IRQ22 (`0xdac`: NIC interrupt, cause = `[0xa01e80] & [0xa01e84]`, then RX FIFO `0xa00c34`); every other vector is `0x4de07c`. These are the IRQs it programs at boot (`irq_select` 19, 22). What raises them (and the R2 status bits) is [?]. |
| X1 | LMAC vendor aux registers (e.g. `0x20a` at reset, `0x5d`) | modelled, [I] | Read back last write, traced `AUX.R/W` (board hook `arc_unknown_aux_*` in `target/arc/op_helper.c`; default stays an instruction error). Meanings [?]. |
| X2 | LMAC extension core register `r32` is a read-only hardware status word | modelled, [I] | Only read (`btst/bbit/tst/xbfu`); code waits for bit 29 set after kicking an engine (`0x4bc4b0`, `0xc202`) and for bits 12/18/25 clear (`0x4bc48a`, `0xc5be`). Model: constant `1<<29` (`AX210_R32` overrides). Real bit meanings, and whether bit 29 ever drops, are [?]. |
| M2 | LMAC memory map = its MPU programming (13 regions, MPU enabled, no background access) | **yes** | [F] for the firmware's view: read live at the main loop (gdbstub `mpurdb*/mpurdp*`), decoded with ARCv2 RDB/RDP encoding from Zephyr `arch/arc/core/mpu/arc_mpu_v2_internal.h` and `include/zephyr/arch/arc/v2/mpu/arc_mpu.h` (commit f3fc2490). ICCM `0x0-0x17fff` (X), read-only `0x440000-0x63ffff`, code/data `0x4b4000-0x4fffff` (RWX), SRAM `0x600000-0x63ffff`, buffer `0x660800-0x660fff`, DCCM `0x800000-0x807fff`, MMIO `0xa00000-0xbfffff` and `0xd00000-0xefffff`. The board maps exactly these under `-cpu arcem`; boot to L2 still completes and no LMAC MMIO access falls outside the two MMIO regions. The paged image at `0x1000000+` is outside every LMAC region, so it is UMAC's ([I]). Tested `/ax210/lmac/mpu-map-ram`. |
| M3 | One device address space: LMAC addresses it directly, UMAC through `0x80000000` (cached) and `0xc0000000` (uncached); `hw_smem` `0x400000` (`0x38000`), `sw_smem` `0x440000` (`0xc0000`), `chmem` `0x620000` (`0x16000`), `dccm_lmac1` `0x800000`, `dccm_umac` `0x880000` (`0x8000` each) | **yes** (map), [F] vendor | Intel's own debug region TLVs in the `.ucode` (`IWL_UCODE_TLV_TYPE_REGIONS`, decoded by `scripts/ini_regions.py`; layout `fw/api/dbg-tlv.h`). Cross-checked: UMAC names its error table `0x80471488` (`0x80497904`), the TLV says `0x471488`; LMAC names `0x4f2358`, the TLV says `0x4f2358`; UMAC bss is `0xc0880000` = `dccm_umac`. Both images carry a `0x678`-byte signed header for `0x440000` (UMAC's as `0x80440000`), one area verified in turn. Replaces independent RAM per UMAC window. The TLVs also list 1341 MAC and 3911 PHY periphery register addresses (not yet used by the model). Tested `/ax210/umac-windows-alias-device`. |
| P1 | LMAC MMIO is `0xa00000-0xbfffff` and `0xd00000-0xefffff` | **yes** (extent) | Superseded by M2: those are the two RW regions that only ever see MMIO-style accesses. Earlier coarse window `0xa00000-0xffffff` retired. |
| W1 | LMAC indirect register window: `0xa2ac04` CTRL (addr[16:0], bit17 write, bit18 GO/BUSY), `0xa2ac00` DATA | modelled; target **yes**, silicon | LMAC `0x4be6f8` (read), `0x4be714` (write), `0x4be818` (wait while BUSY). BUSY self-clears at once; backing contents [?] (zero, traced `IND.R/W`). Tested `/ax210/lmac/indirect-window`. Target is device `0xa20000 + addr`, confirmed on silicon: the firmware writes `0x9061` to `0x9890` and `0x61` to `0x9938` through the window, and the fw89 dump reads exactly those values from `CNVR_SCU_SD_REGS_SD_REG_DIG_DCDC_VTRIM` `0xA29890` and `..._ACTIVE_VDIG_MIRROR` `0xA29938`; also UMAC reads `0xb800` as the CNVR ID, which `iwl-prph.h` places at `0xa2b800` (`CNVR_AUX_MISC_CHIP`); the `0xa29xxx` targets match the header's CNVR SCU registers. |
| V1 | LMAC timer/event unit at `0xa02000`: per-channel count at `+0xac+4n`, enable bit n `+0x0c`, start bit n `+0x08`; `+0x20` is a free-running timestamp | modelled, [I] | `0x166e4` (arm), `0x16668` (`wevt`, return `[+0x20]`), `0x4d5222/0x4d5280` (`deadline = [+0x20] + N`). Expiry wakes the core from `wevt/sleep` as an event (IRQ line [?]); 1 tick = 1 us is a guess (makes ch10 `200000` = 200 ms). Relative vs absolute compare [?]. Tested `/ax210/lmac/timestamp`. |
| F1 | Power/clock request FSM at `0xd03380`, SET `+0x8` / CLR `+0x4`; state `[3:0]` = 5 when bit 4 set, 0 when clear | modelled, [I] | `0x4d525c` (SET `0x04000010`, wait state 5, error `0xfbc`), `0x4d520c` (CLR `0x10`, wait state 0, error `0xfbd`). Transition latency [?] (model: immediate). Tested `/ax210/lmac/power-fsm`. |
| S1 | Hardware semaphore bank at `0xd03800`: per-core request/granted/release, LMAC `+0x00/+0x04/+0x08`, UMAC `+0x20/+0x24/+0x28` | modelled, [I] | LMAC `0x4ba960`, `0x4d1c88`; UMAC `0xc008d258/0xc008d278` uses the `+0x20` set at boot. Each bit owned by at most one core. Tested `/ax210/lmac/semaphore`, `/ax210/semaphore-two-cores`. |
| A1 | **AES-128 engine at `0xa01800`** | **yes** (function), [I] (control bits) | [F]: LMAC's boot self-test (`0x4d9ba0`, table `0x4e1828`) is FIPS-197 Appendix C.1 (key `000102..0f`, pt `00112233..ff`, ct `69c4e0d8 6a7b0430 d8cdb780 70b4c55a`, recomputed independently). Packing pinned by that vector: key FIFO `+0x08` low word first, byte-swapped; data/result `+0x98+4j` = BE word `3-j`. [I]: ctrl `+0x94` bit0 start, bit1 encrypt, status bit2 done; decrypt path unverified. Tested `/ax210/lmac/aes-fips197-c1`. |
| Q1 | LMAC TX FIFOs (TXF) at `0xa00400`: `TXF_RD_PTR` `+0x10`, `TXF_WR_PTR` `+0x14`, `TXF_FENCE_PTR` `+0x18`, `TXF_LOCK_FENCE` `+0x24`, size `+0x30`, `TXF_FIFO_ITEM_CNT` `+0x38`, `TXF_LARC_NUM` (FIFO select) `+0x3c`, push `+0x40` | modelled, [I] | Send routine `0x13140(ptr,len,id)`: free = `max(cap - fill - 8, 0)`; asserts with the message id when full (the `0x3601` assert seen when both read 0). Model: consumer drains at once, capacity `0x400` (guess); pushes traced `MSG`. Tested `/ax210/lmac/msg-fifo-space`. Names: `iwl-prph.h` (the driver dumps these FIFOs on error); the consumer is the MAC TX path, not the other core. |
| B1 | IML runs to completion (sets its done flag, idles) given R1+T1+M1 overrides on `c0d02c7c`, `c0e27cec`, `c0d02ecc`, `c0d02c00`, plus a host CSR seed at `0x118`/`0x11c` | **partial** | [F] behavioral observation under this model, not a claim the real device would do the same with the same stimulus — the model is permissive (write-readback, unauthenticated CSR values) in ways real silicon need not be. See audit report §17/§18. |
| U1 | UMAC entry point | **yes** | [F] from IML code: `mov r16,0xc0080000` (`0x80444814`) then `sr r16,[int_vector_base]; jl [r16]` (`0x804448b8`). `0xc0080000` is UMAC's ARC700 vector table, reset slot jumps to `0xc0080050`. |
| U2 | UMAC boots on `-cpu arc700` through early init, the context-info gate (H1), the register self-test (K2), its first message send (Q2), the `0xd03f00` start (G1) and its thread scheduler to its idle `sleep` (`0x804a6126`), having raised the ALIVE interrupt to the host (H1) | **partial** | [F] behavioural, with `AX210_HOSTFW` and `AX210_IND=940c=1` (I1), under A2-A7. Harness enters through the IML handoff (U1), so `int_vector_base` = `0xc0080000`. Woken every 200 ms by IRQ6 (G2), which it handles and returns to idle; IRQ5 and IRQ8 sources are [?]. Silicon cross-check (same dump): UMAC writes `FSEQ_TOP_INIT_VERSION` `0x80440007` and `FSEQ_TOP_CONTENT_VERSION` `2` in the model, the values the card reports. |
| U3 | UMAC fatal path signals the host with `CSR_INT_BIT_SW_ERR` | **yes** (meaning) | [F]: `0x80483c8e-0x80483cc2` writes bit 25 to host CSR `0x008` (`CSR_INT`, `iwl-csr.h:31`; `CSR_INT_BIT_SW_ERR` = "uCode error", line 201) or, when on-die `0xc0d05c00` bit 25 is set, to `0x2808` (`CSR_MSIX_HW_INT_CAUSES_AD`, line 602), then halts at `0x804a5be4` (`[0xc0d03864] = 1; b .`). Also corroborates M1's offset space. |
| C2 | Host CSR defaults: `CSR_HW_REV` (`0x028`) = `0x420`; mailbox index 0 (`CSR_HW_IF_CONFIG_REG` if M1 holds) bits `[1:0]` = 2 | `CSR_HW_REV` **yes**, silicon; index 0 **partial**, [I], conflicts with the driver | `0x420` from the saved dumps ("rev=0x420", "hw version 0x00000420", `ref-silicon/`); this replaces the earlier guess `0x422`. IML `0x80445a2c` / UMAC `0x8044a702` read mailbox index 0 (`0x80492fac`) and require bits `[1:0]` == 2, or == 1 when on-die `[0xd03400] & 7` == 2, else error `0x2fc`. `iwl_mvm_nic_config` (`mvm/ops.c`) writes `HW_REV[3:0]` = 0 into `HW_IF_CONFIG[3:0]`, which would fail that check, yet the real firmware boots on these cards: so either M1's index = CSR offset mapping is wrong for index 0 or those bits are hardware-driven. 2 is kept as the [I] value that the real boot implies. A CSR snapshot (the `csr` region of an iwlwifi devcoredump) would settle it. Tested `/ax210/host-csr-driver-defaults`. |
| K1 | Cross-core address view: LMAC `0x00axxxxx` == UMAC `0xc0exxxxx`; LMAC `0x00d0xxxx` == UMAC `0xc0d0xxxx` | modelled | First half [F]: the IML converts addresses with `(a & 0xff0fffff) \| 0xc0e20000` (`0x804441a6`, `0x80447082`). Second half [I]: UMAC's semaphore set (`0xc0d03820/24/28`) is the LMAC bank at `0xd03800` + `0x20`. Shared models are keyed by the LMAC-view address. Tested `/ax210/semaphore-two-cores`. |
| H1 | Host driver model: context info, prph scratch, DRAM map, IML and CSRs as iwlwifi leaves them (`AX210_HOSTFW=<ucode>`) | **yes** (layout), modelled | [F] driver: `pcie/ctxt-info-v2.c` `iwl_pcie_ctxt_info_v2_alloc/_kick`, `pcie/ctxt-info.c` `iwl_pcie_init_fw_sec`, `fw/api/ctxt-info-v2.h` (torvalds/linux `eedaa7d6`). [F] firmware agrees: UMAC `0x804485e4` DMAs `0x68` bytes (`sizeof(iwl_context_info_v2)`), follows `prph_scratch_base_addr` (`+0x58`, error `0x1000aa` if 0), DMAs `1660` = `offsetofend(iwl_prph_scratch, dram.common)`, checks `version.mac_id` against `CSR_HW_REV` (error `0x1000ac`), stores `prph_info_base_addr`; `0x80448710` decodes `control_flags` bits 4/8-11 and `hwm_cfg` at their driver offsets. Values: `rx_buf_size` 4K default (`iwl-modparams.h`) so `control_flags = 0x000f0000`; `mtr_size = 4`, `mcr_size = 12`. Simplifications: page-granular bump allocator at `0x20000000`; rings allocated but empty; no debug destination. Tested `/ax210/host-driver-ctxt-info`. ALIVE handshake [F + V]: UMAC raises `CSR_INT_BIT_ALIVE` (host CSR `0x008` = 1 through the M1 mailbox, `0x80493008`), and the model answers as the gen2 driver does (`pcie/rx.c`: no restock before the kick; on ALIVE `iwl_pcie_rxmq_restock` posts 4095 4 KB buffers as `{rbid, addr}` transfer descriptors and writes `RFH_Q0_FRBDCB_WIDX_TRG` `0x1c80` = 4088; mirrored to `RFH_Q0_FRBDCB_WIDX` `0xa08080` [I]). The firmware has not yet sent the ALIVE notification after the restock (no push to the `0xd00e00` RX FIFO, no doorbell read). |
| A2 | ARC700 `AUX_IRQ_LEV` (`0x200`) | **yes** (storage) | Architectural ARCv1 register, declared but without get/set in `target/arc`, so UMAC's `sr 0x100,[aux_irq_lev]` (`0x80447836`) raised an instruction error. Now read/write storage; level semantics are unused because this fork has no ARCv1 interrupt delivery. |
| A3 | ARC700 `extw`/`sexw` (32-bit and `_s`) | **yes** | Architectural; decoded but unmapped in `semfunc-mapping.def` (ARCv2 names them `exth`/`sexh`), so UMAC `0x8044911a` raised an instruction error. Mapped to the `EXTH`/`SEXH` semantics. |
| A4 | ARCompact exception entry: vector numbers `0x20-0x26` above Instruction Error, 8-byte slots, PC = slot | **yes** | Architectural (Linux `arch/arc` `arcregs.h` `ECR_V_*`, `entry-compact.S` vector table of `j` instructions). The fork used ARCv2 entry (load handler address from the table), jumping into instruction words. UMAC's table at `0xc0080000` is `j` slots (U1). |
| A5 | Actionpoint registers `AP_AMV/AMM/AC 0-7` (aux `0x220-0x237`) | **yes** (storage) | Declared without get/set for every family. UMAC's context switch (`0xc008159e`) writes per-thread stack bounds to them and to `0x260/0x261` (stack check, board hook X1). Triggering is not modelled. |
| A6 | ARC700 XMAC `AUX_MACMODE`, `AUX_XMAC0-2` (aux `0x41`, `0x44-0x46`) | **yes** (storage) | Saved/restored per thread by UMAC (`0xc00816f6`); no MAC instruction executed so far. |
| A7 | ARCompact `RTIE`: encoding `0x242F003F`; AE → `ERET`, else A2 → `ILINK2/STATUS32_L2/BTA_L2`, else `ILINK1/STATUS32_L1/BTA_L1` | **yes**, [F] firmware | The table knew only `0x246F003F` (same zero-operand instruction, `u6` form), and `RTIE` followed ARCv2 rules. UMAC starts threads from non-interrupt context (`STATUS32` = 0) by loading `ILINK1`, `STATUS32_L1` (aux `0xb`, now defined for ARCv1 with `0xc` `STATUS32_L2`) and `BTA_L1` and executing `rtie` (`0xc008171a`), which only works with the level-1 fallback. |
| A8 | ARCompact interrupt delivery: lines 3-31, `AUX_IENABLE`, `AUX_IRQ_LEV` levels, `E1/E2`/`A1/A2`, `ILINKn`/`STATUS32_Ln`/`BTA_Ln`, `ICAUSE1/2`, sticky `AUX_IRQ_LV12`, vector slot `8n` | **yes**, architectural | Register numbers and bits from Linux `arch/arc` (`irqflags-compact.h`, `entry-compact.S`: `AUX_IRQ_LV12` write-1-to-clear). `AUX_IENABLE` resets all-enabled: nothing in the `.ucode` writes it, yet UMAC handles IRQ5/6/8. UMAC's table at `0xc0080000` confirms the layout: IRQ3/4 (timers) are `flag 1` halts, IRQ5/6/8 have handlers, IRQ7/9 share an `ICAUSE`-reporting default; `0x100-0x120` are the A4 exception slots. Interrupt priority within a level: lowest line first. |
| A9 | ARC700 `BPU_FLUSH` (aux `0xf`) write | **yes** | Architectural; declared without a setter, so UMAC's `sr 0xf,[bpu_flush]` in its ALIVE builder (`0x80493954`) raised an instruction error. QEMU has no branch predictor, so the write is a no-op. Found by calling the message-30 handler directly (Q3 experiment). |
| A10 | ARC700 `mpyh`/`mpyhu` | **yes** | Architectural; same encodings as ARCv2 `mpym`/`mpymu` (`0x201B..`/`0x201C..`) but unmapped for ARC700, so UMAC's 64-bit multiply helper (`0xc0081d36`, on the ALIVE path) raised an instruction error. Mapped to the `MPYM`/`MPYMU` semantics. |
| A11 | ARCv2 interrupt vector fetch | **yes** (F) | LMAC's MPU region 0 (ICCM, base 0) is execute-only (`--x`), and its vector table lives there; IRQ22 still works on silicon, so the vector entry is fetched through the instruction path. The fork read it as data (`cpu_ldl_data`), raising Protection Violation `ecr 0x00060104` on the first LMAC interrupt; now `cpu_ldl_code`. (Exception entry already read the vector physically.) |
| Q4 | NIC doorbells between the cores | **partial** (F) | LMAC: `0xa01e80` mask (built bit by bit at boot, `0x09109b31`), `0xa01e84` status, `0xa01e88` set; IRQ22 (`0xdac`) maps cause bits to events (`0x20` RX, `0x10` -> `0x71`, `0x100`/`0x200` -> `0x90`/`0x91`, `0x800` -> `0x46`, `0x1000` -> `0x4c`, `0x8000` -> `0x68`, `0x100000` -> `0x65`, `0x8000000` -> `0x72`, others `0x84`). UMAC writes `0x10` to `0xa01e88` in its fatal path after the `0xfd11` timeout (`0x804839b2`), then waits for `0xd02e84` bit 1 (`0x80483b4c`); LMAC answers event `0x71` by dumping its ISRs and writing 2 to `0xd02e88` before halting (`0x1560`), so UMAC's `0xd02e80/84/88` is the mirror block. Modelled: LMAC's block (status write-1-to-clear assumed, qtest `lmac-nic-doorbell`); UMAC's mirror is not. Both cross-core signals (this and Q3) end the sender's round-robin timeslice, since the fork runs both cores on one TCG thread (100 ms kick) and UMAC's busy-waits count reads of the `0xd02c68` counter, not time. |
| Q5 | UMAC -> LMAC commands | **partial** (F) | UMAC selects FIFO 14 at `TXF_CPU2_NUM` (`0xa0053c`) and pushes `{cmd header, byte length, payload}` to `0xa00540` (`0xf700` at boot, `0xfd11` from the ALIVE builder). LMAC reads the same FIFO from its own TXF block: IRQ19 (`0x1bbc`, cause `[0xa01e60] & 0x58902167`, highest bit indexes handlers at `0x806844`) bit 28 -> `0x4b78d0` selects `TXF_LARC_NUM` 14, waits for `TXF_FIFO_ITEM_CNT` (`0xa00438`) >= 8 bytes, reads words from `0xa00408` and dispatches by group/ID (`0x4cd49c`: groups < `0x11` via `0x8060d4`, >= `0xf4` via `0x80678c[0xff-g]`). UMAC configured FIFO 14 as one SMEM ring (`0x23000`, size `0x800`), so both blocks see the same queue. Modelled as one queue taking whole frames; IRQ19 and cause bit 28 follow its level. LMAC enables IRQ19 only late in boot, so `0xf700` is consumed together with `0xfd11`. LMAC answers `0xfd11` with `{0x8001fd11, 0}` on FIFO 13 (Q3), and UMAC consumes it. |
| P1 | PHY sync read and PHY engines | **partial** (F; addresses V) | LMAC reads PHY registers through `0xadfc0c` (address `\| bit 28`) / `0xadfc1c` (`0x1697c`); UMAC through `0xadfc14` / ready `0xadfc24` bit 15 / data `0xadfc20` (`0x804a9d70`). Engines start with a pulse on control bit 0 or 1 and report bit 30 or 23 on a status register: `0xad131c` -> `0xad13c0` (asserts `0x13c1`, `0x28a9`; `0xad1324` bit 29 clears), `0xad32d0+4i` -> `0xad3680+4i` (`0x13c0`, `0x28aa`). A capture block reports progress at `0xac0458` up to the length written at `0xac0450` (`0x4d39a4`; `0x4c9988` requires `0x1000`, assert `0x14b5`). Model: completion at once; result registers (`0xad1350-0xad137c`, ...) read 0, which the firmware accepts. All addresses are in the ucode's PHY periphery list; their semantics are read off the polls. |
| W2 | Request/grant pair behind the W1 window | **partial** (F) | Both cores write 1 to `0x8c38` and spin on `0x8c50` bit 0 (LMAC `0x4bdf90`, no timeout; UMAC `0x804865d0`). Model: the grant follows the request bit. No vendor or silicon record. |
| W3 | SMEM range watcher and the `0xa2b0xx` entropy engine | **partial** (F) | After the `0xfd11` reply the ALIVE builder (`0x8049ea88`) moves engine samples in 512-byte chunks to SMEM `0x437000` (`[0x8047173c]`): `0x8049da44` arms watcher channel 0 (`0xd0c154` range, `0xd0c150` bit 31), `0xc0084a0c` starts the engine through W1 (`0xb044` start, `0xb00c` end, in thirds of a chunk; `0xb008` control `0x61e000`), and `0x80486638` waits without timeout for `0xd02e24` bit 7. The samples are entropy: `0x8047f564` packs bit 0 of each into a `0x840`-bit map at `0x660c00`, or, when `([0xd038c8] & 0xf2) == 0x32` (the value that also skips `BPU_FLUSH` at `0x80493954`, so a simulation/FPGA platform), bit 0 of the software PRNG `0x804a0bc0` (Park-Miller, seed x `0x41a7` mod 2^31) instead. `0x8049da98` loads the first 2048 bits into the paging engine (W4) and raises fatal `0x10023f` if they XOR to 0 or start with `0xbadc0fee`; `0x80493974` seeds the PRNG from word `0x660d04` just before the ALIVE send. Model: engine start with the watcher armed fills the watched range with `qemu_guest_getrandom` words and sets bit 7. |
| W4 | Paging engine `0xd04800` | **partial** (F) | Takes the entropy at `+0x200`, page length at `+0xc`, per page `+8`/`+4` and a command at `+0`: 1 in the initial page loop (`0xc0084dac`, waits status bit 2), 2 when paging in from host DRAM (`0x8047f0fc`, waits bit 0); `0x8047efd4` polls `[+0x20]` (`0x32` tries, bit 3 error, fatal `0x100230`). Fresh per-boot entropy suggests it protects pages held in host memory. Model: transparent, done bit at once, never an error. |
| W5 | SHA-2 and RSA engines (`0xa25000`, `0xa24800`) | **no** | Next stop: fatal `0x30c` in `0x804a2228`. `0xa25000` is a SHA-256/512 compression engine (mode at `0xa26410`; big-endian block words to `0xa25000+4i`, SHA-512 upper half at `0xa25080`; `[0xa25060]` bit 0 per block (fatal `0x308`), `& 3 == 2` at the end; digest at `0xa25040` (+`0xa250c0`); software padding `0x804a2054`). `0x804a1ffc`/`0x804a2028` load 64-word operands into `0xa24900`/`0xa24800` (2048-bit, an RSA engine) for the paged-image signature check (`0x804a1978`, fatal `0x100240`). Both are standard algorithms and can be modelled exactly; a correct model would verify Intel's real signature. |
| K2 | Chip IDs, read-only: `CNVI_AUX_MISC_CHIP` `0xa200b0` = `0x400410`, `SD_REG_VER_GEN2`/`CNVR_AUX_MISC_CHIP` `0xa2b800` = `0x400410`, `WFPM_OTP_CFG1_ADDR` `0xd03098` = `0x80000000` | **yes**, silicon | Three AX210 cards' dmesg: `ref-silicon/fw89-framework16-dmesg.txt` (firmware 89.6b44fa0b, same API version as the model's 89.b20dcd1f: "Detected crf-id 0x400410, cnv-id 0x400410 wfpm id 0x80000000" and `0x00400410 | CNVI_AUX_MISC_CHIP` / `CNVR_AUX_MISC_CHIP`), `ref-silicon/fw67-freebsd262150-dmesg.txt` (same IDs, fw 67), an Intel community log (URL in `ref-silicon/README.md`). `FSEQ_OTP_VERSION` `0xa340fc` = `0xa652` on all three saved cards; the firmware only reads it (UMAC `0x80448cae` branches on `>= 0xa440`); per-card OTP value; the driver reads exactly these registers (`pcie/trans.c`, names and addresses `iwl-prph.h`; WFPM at UMAC periphery offset `0x300000`). UMAC `0x80448ca4` writes the expected IDs to `FSEQ_CNVI_ID` `0xa3408c` / `FSEQ_CNVR_ID` `0xa34090` and compares (`0xa200b0` masked `0xff0fffff`; CNVR through the W1 window as `0xb800`); a mismatch is fatal `0x1013f1`. Now passes on the silicon values (status `0x20000061`, bit 1 clear). Replaces an earlier [I] model of these as write/readback aliases, which the register names contradicted. Tested `/ax210/lmac/indirect-target-chip-ids`. |
| K3 | Cross-core view of LMAC's `0xa0` page: UMAC `0xc0a0xxxx` == LMAC `0x00a0xxxx` | **yes**, [F] | UMAC `0x804a8b80` builds the LMAC-view address `0xa0a404 + 16*ch` and converts it with `(a & 0xff0fffff) \| 0xc0a00000`, the counterpart of K1's `\| 0xc0e20000` for `0xa2/0xa3`. So D1's DMA engine and Q2's FIFO are LMAC `0xa0a500` / `0xa00500`, and Q1 (`0xa00400`) is the same unit UMAC configures at `0xc0a00400` (`0x804a8b18`: per channel, select `+0x3c`, buffer `+0x34`, size `+0x30`). Models are keyed accordingly. |
| Q2 | CPU2 (UMAC) TX FIFOs at `0xa00500` (UMAC `0xc0a00500`): `TXF_CPU2_*` (`iwl-prph.h`) | modelled, [I] | Q1's layout in UMAC's own `0xc0a0xxxx` MMIO block (now traced, `0xc0a00000-0xc0a0ffff`): UMAC `0xc008d0a0` selects channel `0xe` at `+0x3c` as LMAC does at `0xa0043c`, free = `max(cap - fill - 8, 0)`, push via `0x804a8d48` to `+0x40`; error `0x100efa` when it does not fit. Modelled as Q1. Tested `/ax210/lmac/msg-fifo-space`. |
| I1 | Indirect `0x940c` (device `0xa2940c`, W1) bit 0 | [?], harness-supplied | UMAC `0x8044a49a` requires it after init call `0x8049f934` (else fatal `0x10133c`); no firmware writes it, so it is hardware status. Not modelled: supplied by the harness via `AX210_IND=940c=1` (seeds the W1 store). |
| G1 | Configure-and-start unit at `0xd03f00`: config `+0x0/+0x4/+0x8`, start `+0x10` bit 0, done `+0x14` bit 0 | modelled, [I] | UMAC `0x8044acd0` loads three words from `0xc088400c` (the last two are addresses it then writes through), `0x804a6c94` starts and waits ~20 timebase ticks for done, else fatal `0x101035`. No firmware on either core writes `+0x14`. Model: done follows start at once. Tested `/ax210/cfg-unit-start-done`. |
| G2 | UMAC timeout timer: write N to `0xd02c78` arms; expiry sets `0xd02e2c` bit 4 and raises IRQ6; writing bit 4 clears | modelled, [I]; tick rate [?] | UMAC's idle loop re-arms `0x30d40` at `0x804a66a0` before each `sleep`; its IRQ6 handler `0xc0080b30` writes `0x10` to `0xd02e2c` and re-arms. Model: one-shot, 1 tick = 1 us of virtual time. Under it UMAC takes IRQ6 every 200 ms from its idle `sleep` and returns. Tested `/ax210/umac-timeout-timer`. |
| R2 | `LMPM_NIC_ISR0-4` at `0xa01e20-0xa01e30` (LMAC interrupt status) read an idle AX210's values: `0x00020000`, `0`, `0x48F00002`, `0x00C0001C`, `0` | **partial**; values silicon, semantics [I] | LMAC's fatal handler `0x4d1eaa` copies these five registers into its error table as isr0-isr4 (field names `fw/dump.c`; table `0x4f2358`). Values from `ref-silicon/fw89-lkml-2407-00526-dmesg.txt` (firmware booted, waiting on the host); the Framework card (after a fault) agrees on ISR0/ISR2. The model used to read back LMAC's init writes (`0xffffffff`, `0xf5ffff7f`, ...), values silicon never shows (graded **contradicts** until now); they are now read-only with the idle values. LMAC does not read them during boot, so the boot is unchanged. Not modelled: the events that set or clear bits (they belong with LMAC's interrupt sources, N3), and `lmpm_pmg_sel` `0xa01c0c` (silicon 9, model reads back LMAC's `0xb`; it may change over time, unverified). Tested `/ax210/lmac/indirect-target-chip-ids`. |
| Q3 | LMAC reports ALIVE to UMAC through TX FIFO 13 (`TXF_LARC_NUM` `0xa0043c` = `0xd`, the selection written just before the push): header `0x0000ffff`, byte length `0x9c`, then `struct iwl_lmac_alive` and a memory-region list | **partial** (contents [F] + V; delivery [F]) | Delivery, found 2026-11-09 and modelled: the frame reaches UMAC's RX FIFO `0xd00e00` as command `0xffff`. `0x804903f4` falls back, when a command's record index is 0, to a second table counted from the top of each group (count byte `0x8047320c[0xff-g]`, halfwords `word[0x80478814+4*(0xff-g)][0xff-id]`; `scripts/fallback_routes.py`), and that table maps `0xffff` to message 30 (also `0xfffe` to 89 and `0xfffd` to 35). Message 30's entry expects payload `0x9c`, LMAC's exact body length. UMAC's parser (`0x80497898`/`0x80497938`) needs first word bit 27, payload (`(w & 0xfffc) - 4`) bytes, `0xd05498` bit 31, then the command header (the `iwl_rx_packet` layout `len_n_flags`, `iwl_cmd_header`); the model reframes LMAC's `{hdr, len, body}` into `{0x08000000 | len+4, hdr, body}` and holds IRQ5 summary bit 14 while words remain (qtest `lmac-to-umac-frame`). The wording below up to "Experiment" predates this: its claims that no command maps to index 30 and that the RX FIFO path is ruled out were wrong (only the first table had been scanned). With delivery modelled, UMAC runs the message-30 handler and then sends `0xfd11` (`{0x8001fd11, 8, 3, 0xc0000001}`) on its own TX FIFO 14 (`TXF_CPU2_NUM` `0xa0053c`) to LMAC; LMAC never receives it (the reverse path is not modelled), UMAC times out and raises `CSR_INT_BIT_SW_ERR`. Earlier record: The 41 words LMAC pushes at boot (send routine `0x13140`, push `0xa00440`) decode exactly as `iwl_lmac_alive` (`fw/api/alive.h`, `UCODE_ALIVE_NTFY_API_S_VER_3`): major `0x59`, minor `0xb20dcd1f` (this build), ver_type 1, `error_event_table_ptr` `0x4f2358` and `scd_base_ptr` `0x620000`, both matching the ucode region TLVs (`error_table_lmac1`, `chmem`); then (addr, size, flag) triples including `hw_smem` `0x400000`/`0x38000`. UMAC must receive this to build the host ALIVE notification; the model drains it (Q1), which is the likely reason ALIVE is never sent (H1). UMAC reads RX FIFO data by popping `0xd00e68` after writing 0 to `0xd00e10` (`0xc008e49c`, 18-word header; the driver's RXF read protocol names these `RXF_FIFO_RD_FENCE_INC`/`RXF_LD_FENCE_OFFSET_ADDR`). Which UMAC FIFO or event receives TXF 14, and how, is [?]. UMAC's RX FIFO `0xd00e00` is its host-command intake, not this path [F]: IRQ5 cause bit 14 (`0xc00823d0`) pops two header words from `0xd00e68` (after writing 0 to `0xd00e10`; first word: bit 27 required, else fatal `0x100124`, byte length in bits 15:2; second word: command header, ID in bits 15:0) and `0x80497938` dispatches by command ID (special cases `0x31`, `0xc1`; handler lookup `0x804903f4`). Where LMAC's TXF 14 report is consumed is [?]. Tested and rejected: that UMAC has queued the ALIVE notification and only waits for output space. With `0xd00e34` reporting `0xb400` and IRQ5 cause 13 raised after the restock (experiment, not kept), UMAC's sender `0x8049305c` ran but found its queue (`0x80467870`) empty. So UMAC has not composed ALIVE, consistent with it waiting for LMAC's report. Framing: LMAC sends an 8-byte header from its stack (`0x807f28`, ID `0x3601`) and then the body (ID `0x3604`) with `0x13140(buf, len, id)`, which pushes words as they are. Ruled out as the receive side: UMAC's RX FIFO `0xd00e00` (host commands) and the third vendor-listed RX FIFO `0xd05a00`, which both cores use as a send port (space `+0x34` > 1024, UMAC data `+0x04/+0x08`, go `+0x1c` = 3, with `0xfea8...` trace markers; likely debug logging). UMAC side [F]: the host ALIVE notification is sent by `0x80486a4c` (status `0xcafe` = `IWL_ALIVE_STATUS_OK` into buffer `0x8047872c`, then `0x80493238(1, 0x90, buf, 0x3f0001)`: notification 1, 144 bytes), reached only through `0x804938f4` from `0x80486a0c` when byte `[0x80471bdd]` (received) equals `[0x80471bdc]` (expected; 1 at idle, received 0). `0x80486a0c` is the handler for internal message 30 (entry `{0x9c0001, 0, 0x80486a0c}` at `0x8047374c + 12*30`, looked up by `0xc008b2b0` / `0x804903f4`; length `0x9c` = LMAC's report) and calls `0x80447194(msg)`, which counts the report and copies `0x30` bytes (`sizeof(iwl_lmac_alive)`) into `lmac_data[msg[0xa]]` of the ALIVE buffer. Missing: how LMAC's TXF 14 push becomes a message-30 dispatch on UMAC. The ALIVE sender also reads 12 bytes from the paged image (`0x102b020`); the harness does not load paged sections (`0x1000000`-`0x10ba000`). Message indices come from per-command info records (`0x8049038c`: group g >= `0xf4`, count byte `0x804731fc[0xff-g]`, records `word[0x804787e4 + 4*(0xff-g)] + 8*id`, index `(rec[4] >> 4) & 0xfff`; out-of-range IDs use `0x8047a370`, index 0). Read at idle (`scripts/find_msg.py`), no command in groups `0xf4-0xff` maps to index 30 (used: 1-4, 25-27, 33, 34, 46-105, ...), and the static image references entry 30 only through that lookup. So the route to message 30 is installed at runtime by an event the model does not produce, or goes through an indirect path not yet found. Dynamic check (`scripts/dual_wp.py`, GDB watchpoints work on this fork: validated on `0x804741f8`): nothing accesses entry 30 (`0x804738b4-0x804738bf`) from reset to idle. Replaying the report into UMAC's RX FIFO is ruled out by the routing tables (no command reaches index 30). Also ruled out: the `0xa03800` block, whose only boot traffic is UMAC writing debug-trace records to `+0x40` (`{magic 0xeec0xxxx, marker 0xfea8xxxx, timebase, data}`, `0x80491e6c`). Also ruled out: command `0x31` (`0x80492198`: pops one word, then `0x804a5b94`) and `0xc1` (`0xc0080360`: RX frame descriptor path), the two IDs UMAC handles before the table lookup; and TX FIFO data read from `hw_smem` (no code names the channel 14 ring at `0x423000` in any view). The routing tables are static (only their pointer arrays and the lookup reference them; no code writes routes), and no record anywhere in `0x80479900-0x8047a400` maps to index 30 (`scripts/scan_idx.py`). Also ruled out: UMAC IRQ8, whose every cause bit ends in the fatal reporter `0xc008477c` with an error code (`0xc008076e`: `0x45`, `0x47`, `0x5d`, `0x65-0x67`, `0x84`, ...); it is a hardware-error interrupt. No candidate path is left from static or boot-time evidence; the next method is to watch writes to UMAC's routing tables (`0x804787e4`, the `0x80479xxx` records) while LMAC runs its post-boot paths. Experiment (`scripts/dual_inject.py`, not a model claim): calling the message-30 handler at idle with LMAC's actual report, in a worker-task context, exercises UMAC's ALIVE builder. It found two CPU gaps (A9 `BPU_FLUSH`, A10 `mpyh/mpyhu`, both fixed) and then stops at fatal `0x100ef8`: UMAC sends internal command `0xfd11` and waits on an RTOS event object (magic `0x4456444e`, `0xc0083c4c`) for the reply, which times out (code 7). The send (`0x80497b3c`) posts `0xfd11` to another UMAC task's queue (`0x8049ce0c`), not to hardware, so the timeout may be an artifact of the injection (the call runs from the idle context with a borrowed task index, and the receiving task may never be scheduled); it does not by itself show that LMAC must answer. |

## Acceptance criteria: "accurate enough" (timing out of scope), 2026-11-09

1. Both cores boot from their entry points to their idle states with no
   exception and no fatal error. **Met** up to LMAC's ALIVE report (U2, L2;
   `dual_boot.py`). With Q3 delivery UMAC goes on to request `0xfd11`
   from LMAC; with Q5, P1, W2-W4 LMAC answers and UMAC stops at fatal
   `0x30c` (W5: SHA engine not modelled).
2. The firmware completes the documented host boot protocol: ALIVE
   interrupt, the driver's RX restock, then the ALIVE notification in a
   host RX buffer with fields matching `fw/api/alive.h` and the ucode's
   region TLVs. **Partly met**: interrupt and restock done (H1); LMAC's report reaches
   UMAC (Q3), LMAC answers `0xfd11` (Q5, P1, W2), and the ALIVE builder
   then seeds the paging engine from the entropy engine (W3, W4) and
   verifies the paged image with SHA-2/RSA engines (W5, not modelled);
   the notification is not yet sent.
3. No row graded **contradicts**. **Met** (R2 resolved to silicon values).
4. Every value with silicon evidence matches it (chip IDs, FSEQ, CSR_HW_REV,
   CNVR SCU registers, ISR registers). **Met** (lmpm_pmg_sel unverified).
5. Every modelled behaviour is graded with its evidence class in the audit
   table, and no harness-supplied value hides a model gap without a row.
   **Met** (I1 is the one harness-supplied value, graded **no**).
6. qtests pin every modelled register behaviour that has a row. **Met** for
   the rows marked tested.

## Audit (EFR32 method), 2026-11-09

Method of `~/Projects/owo-open-fw/docs/model-fidelity.md`: every finding
graded **yes** (modelled as evidenced), **partial** (the gap says what is
missing), **no** (not modelled), **contradicts** (model disagrees with
evidence), **n/a**. Evidence: **S** silicon (saved AX210 dumps,
`~/Projects/ax210/ref-silicon/`), **V** vendor (the `.ucode`'s debug TLVs,
iwlwifi and Linux arch/arc sources), **F** firmware behaviour, **I**
inferred, **?** unknown. N-rows list hardware not modelled at all.

| | yes | partial | no | contradicts | n/a | total |
|---|---|---|---|---|---|---|
| 2026-11-09 | 18 | 31 | 9 | 0 | 1 | 59 |

Contradictions resolved in this pass (now **yes**): K2 (write/readback
aliases vs. read-only chip IDs, by iwl-prph.h names and silicon values),
CSR_HW_REV `0x422` vs silicon `0x420` (C2), Q1/Q2 "message FIFOs" vs TX
FIFOs (iwl-prph.h), UMAC windows as separate RAM vs one device address
space (M3), FSEQ_OTP_VERSION 0 vs `0xa652`.

| ID | Status | Evidence | Gap |
|---|---|---|---|
| R0 | yes | F | - |
| R1 | partial | I | Write-readback is the model's default for unmodelled MMIO, not a hardware claim; real registers have their own semantics. |
| T1 | partial | I | Counts per read; no real rate (timing). |
| M1 | partial | I; V for indices 0x118/0x120/0x128/0x028 | Index-to-CSR mapping conflicts for index 0 (C2); command encoding unconfirmed. |
| D1 | partial | I; registers in vendor list (V) | Host-to-device copy only; completes at once (no timing). |
| C1 | yes | V (driver) | - |
| E1 | n/a | - | Withdrawn (L1). |
| L1 | yes | F | - |
| L2 | partial | F; S (halt helper, fw89 dump) | LMAC interrupt sources [?]. |
| X1 | partial | I | Meanings [?]. |
| X2 | partial | I | Bit meanings beyond bit 29 [?]. |
| M2 | yes | F | - |
| M3 | yes | V (ucode region TLVs) + F | - |
| P1 | yes | F; V (every listed register lies in these ranges) | - |
| W1 | partial | F (protocol); V + S (target: firmware writes via 0x9890/0x9938 match the dump at 0xA29890/0xA29938) | BUSY completes at once (no timing). |
| V1 | partial | F | Tick = 1 us assumed; interrupt wiring [?]. |
| F1 | partial | F/I | State follows the request at once (no latency). |
| S1 | partial | I; 0xd03800 in vendor list | Arbitration and fairness [?]. |
| A1 | partial | F (FIPS-197 C.1) | Control bits [I]; no timing. |
| Q1 | partial | V (iwl-prph.h TXF_*) | FIFO contents, the TX engine that drains them and its timing not modelled. |
| B1 | partial | F | Relies on R1/T1/M1 overrides. |
| U1 | yes | F | - |
| U2 | partial | F; S cross-checks | Needs I1 from the harness; IRQ5/IRQ8 sources [?]. |
| U3 | yes | V + F | - |
| C2 | partial | S (CSR_HW_REV); I (index 0) | Index 0 value conflicts with driver code. |
| K1 | partial | F (0xa2/0xa3); I (0xd0) | - |
| H1 | partial | V (driver) + F (ALIVE interrupt) | ALIVE notification not yet sent by the firmware (likely waits on LMAC via N4); no debug destination; page bump allocator. |
| A2 | yes | V (Linux arch/arc) | - |
| A3 | yes | Architectural | - |
| A4 | yes | V (Linux entry-compact.S) + F (UMAC table) | - |
| A5 | partial | Architectural | Storage only; actionpoint triggers not modelled. |
| A6 | partial | Architectural | Storage only; MAC instructions not modelled. |
| A7 | yes | F + architectural | - |
| A8 | yes | V (Linux arch/arc) + F | Priority within a level assumed lowest line first. |
| A9 | yes | Architectural | - |
| A10 | yes | Architectural | - |
| A11 | yes | F (LMAC MPU config) | - |
| Q4 | partial | F | UMAC's mirror block `0xd02e80/84/88` (LMAC's halt ack) not modelled. |
| Q5 | partial | F | Frame queue only; FIFO ring pointers and capacity not modelled. |
| P1 | partial | F, V | Engines complete at once; results read 0. |
| W2 | partial | F | Grant semantics inferred. |
| W3 | partial | F | Samples are random, not the silicon noise source's statistics. |
| W4 | partial | F | Pages not transformed or authenticated. |
| W5 | no | F | SHA-2/RSA engines not modelled; boot stops at fatal `0x30c`. |
| K2 | yes | S (three cards) + V | - |
| K3 | yes | F | - |
| Q2 | partial | V (iwl-prph.h TXF_CPU2_*) | As Q1. |
| I1 | no | ? | Hardware status bit supplied by the harness, not modelled. |
| G1 | partial | I | Done follows start at once; register meaning [?]. |
| G2 | partial | I | Tick rate [?] (1 us assumed); one-shot assumed. |
| R2 | partial | S (two dumps) + V (field names) | Event sources and set/clear semantics not modelled; lmpm_pmg_sel unverified. |
| Q3 | partial | F + V (alive.h, region TLVs) | LMAC TXF 13 -> UMAC RXF `0xd00e00` modelled (frame layout and IRQ5 cause 14 from the firmware's parser); the reverse UMAC TXF_CPU2 14 -> LMAC path is not, so UMAC's `0xfd11` request times out. |
| N1 | no | V (iwl-prph.h, rxf_config) | MAC TX/RX engines: TXF drain, RX FIFOs (0xa00c00, 0xd00e00, 0xd05a00) and the RX DMA to the host (ALIVE and notifications). |
| N2 | no | - | PCI device model: the real iwlwifi driver cannot bind to the simulation; the host side is the scripted H1/M1 model. |
| N3 | no | F (structure) | Per-CPU interrupt blocks, handlers known, sources not modelled. UMAC `0xd02e00`: IRQ5 summary `0xd02e64` over groups (mask `+4n`, status `+0x20+4n`); IRQ8 (level 2) cause `[0xd02e84] & [0xd02e80]`, highest bit dispatched (`0xc0080658`: bits 1, 2, 5, 15, 21, 23-26). LMAC `0xa01e00`: IRQ22 cause `[0xa01e80] & [0xa01e84]` (`0xdac`), ISR0-4 at `+0x20-0x30` (R2); IRQ19 timer/event. The two blocks mirror each other. |
| N4 | no | F | Inter-core message path: LMAC to UMAC is TXF 14 (Q3); the receiving side and the UMAC to LMAC direction (UMAC TXF_CPU2 14, 8 words from `0x804a8d48`) are [?]. |
| N5 | no | V (TLV PHY register list) | PHY/radio registers (0xa44400-0xad3dd8): 3911 listed, none modelled. |
| N6 | no | - | Timing: core clocks, bus and peripheral latencies (no silicon timing evidence available). |
| N7 | no | - | IML/ROM boot path from reset (ROM image not available); the harness starts UMAC at the IML handoff. |

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
- Both cores run in one machine (GDB thread 1 / monitor cpu 0 = UMAC ARC700,
  thread 2 / cpu 1 = LMAC ARCv2EM; `-cpu` is ignored) over one flat address
  space, so they share the K1 register store, S1 and RAM. Started together
  (`dual_boot.py`), each reaches its idle state (U2 `sleep`, L2 main loop).
  The message FIFOs (Q1, Q2) still drain on push; nothing delivers one core's
  messages to the other, because the receive side is [?].
- LMAC interrupt sources (V1 expiry, Q1 inbound, anything giving the main
  loop at `0x4d08fa` work): [?].
- UMAC IRQ5 is a cause aggregator [F structure, not modelled]: `0xc0083f60` reads the summary word `0xd02e64` (fatal `0x100116` if 0), takes the highest set bit k and calls `table[k]` at `0xc0880a64` (filled at runtime; at idle ~18 bits have handlers, `0xc0082100` is the empty entry). The `0xd02e00` block holds groups n: mask `+4n`, status `+0x20+4n`, acknowledged by writing the bit (handlers `0xc008214c` n=1, `0xc00822e8` n=2, `0xc0082334` n=4, `0xc008208c` n=6); D1's DMA-done bit (`+0x24` bit 29) and G2's bit (`+0x2c` bit 4) sit in groups 1 and 3. Which group bit an LMAC message raises, and where UMAC reads it, are [?]; `0xd00e00` (UMAC `0xc00824b8` compares `0xd00e4c`/`0xd00e50`; LMAC `0x1544` polls `0xd00e40`) is the next lead.
- UMAC IRQ5 and IRQ8 (level 2) sources: [?].
- TX FIFOs (Q1, Q2) are the MAC TX path ([iwl-prph.h] `TXF_*`, `TXF_CPU2_*`): in a dual run neither core reads them back, as expected of a TX FIFO. UMAC `0x804a8b18` programs 15 FIFOs (`TXF_LARC_NUM` 0-14) from static data (`+0x34` buffer offset `0x9000-0x23800`, `+0x30` size) and clears `TXF_LOCK_FENCE`; LMAC `0x1e64` manages `TXF_LOCK_FENCE`/`TXF_RD_PTR`. FIFO contents, the TX engine that drains them and its timing are not modelled (drain at once). How the cores exchange messages is still [?] (not through these FIFOs).
- RX FIFOs (vendor `rxf_config` region lists three: `0xa00c00`, `0xd00e00`, `0xd05a00`, each with `+0x84` base, `+0x88` size, `+0x50/+0x54` pointers, `+0x4c` fence). `0xa00c00` (`iwl-prph.h`: RX FIFO `RXF_*`, e.g. `RXF_SIZE_ADDR` `+0x88`, `RXF_RD_WR_PTR` `+0x50`, `RXF_RD_RD_PTR` `+0x54`, `RXF_RD_FENCE_PTR` `+0x4c`) and `0xd00e00` (UMAC) are a pair of identical ports (LMAC data table `0x804194` lists both): free `+0x34`, push header (bits 27-29 \| length) then data to `+0xc0`, buffer `+0x84`/`+0x88` (UMAC: `0x2a000`/`0xb400`). UMAC's sender is `0x8049305c`. Not used during boot to idle. IRQ8's handler (`0xc0080658`) reads `0xd02fa0`, `0xd00ee0`, `0xd05498`, `0xd03944`, `0xc0a01c64`.
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
