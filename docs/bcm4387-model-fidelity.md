# BCM4387 board model fidelity

Scope: tracks the accuracy of `hw/arm/bcm4387.c`, the QEMU machine model for
the Broadcom BCM4387 (rev C2) Wi-Fi dongle's on-chip ARM core, as used in
Apple T6000-class Macs (board `apple,madagascar`). It exists to build an
accurate-enough simulator to trace what the signed-but-unauthenticated RAM
firmware image needs from the on-chip ROM; it is not a silicon model.

Evidence tags: **[F]** fact, observed in a named file/trace/driver line.
**[I]** inferred, a model built to explain observed firmware control flow.
**[?]** unknown, not modelled.

## Strict model (2026-11-09) -- current state

The sections after this one describe the early scaffold and are kept for
history. The model is now split into devices: `bcm4387-pmu.c` (resource
manager, indirect files, always-on write synchroniser), `bcm4387-gci.c`,
`bcm4387-cc.c`, and `bcm4387.c` (machine, d11, CR4, wrappers). Each file
header states what is modelled and why. Evidence classes: **[F]** silicon,
**[B]** Broadcom/Linux source (bcmdhd 101.10.x: sbchipc.h, hndsoc.h,
sbgci.h, d11.h, hndpmu.c, siutils.c), **[I]** inferred, **[U]** unknown
(stored and traced only).

Under `strict=on` the stock firmware boots to "Notify host through MSI -
FW boot complete": RTE banner, PCIe MSGBUF attach, wl0/wl1/wl2 attach on
all three slices, TCAM, both reclaims, ThreadX. At its first PHY write the
model's res_state (0x5ecefbff) and all 94 PMU/GCI indirect entries dumped
on silicon (run 14) equal silicon (`scripts/stock_state_check.py`). Runs 9,
15 and 16 reproduce exactly (`scripts/model_regress.py`); runs 10/11 differ
only in the raw PHY dump pattern (first read 0x0084, then 0xdb32), which is
not explained.

The per-register gap list is generated from the source tables:
`scripts/strict_gaps.py > fw/trace/strict_gaps.tsv` (sibling repo). Values
stock reads that the model lacks are in `fw/trace/strict_unknown_reads.tsv`.

### Enable chain (what the model says makes the PHY usable)

In stock's order, each a single write followed by a read-back of the same
register (stock's ai_corereg pattern):

1. PMU mac_res_req_mask (+0x68c) = 0x40000000 (MACPHY_CLK_MAIN; the PMU
   closes the dependencies), mac_res_req_timer (+0x688) = 0x15000000. [F]
   values from stock, [B] si_pmu_set_mac_rsrc_req.
2. srpwr request domain 3 (MAC main) or 0x1e; poll status. [F]
3. d11 core reset with PHY clock+reset held, ioctl to 0x145. [F]
   With step 1 done, the PHY clock (MACPHY_CLK_MAIN) is up as soon as the
   core is out of reset with its clock on. [I]
4. d11 maccontrol (+0x120) = 0x04000400 (MCTL_WAKE | MCTL_IHR_EN). [B]
   After this the clk_ctl_st PHY PLL request no longer reaches the PMU, so
   step 1 must come first. [F] run 16. Gives SISF_FCLKA (iostatus 0x11006)
   and live PHY/radio ports. [I]
5. Optional, stock does them, no effect modelled: d11 +0x18c = 0, 16-bit
   +0x3e6 = 0. [U]

ChipCommon ERSRC0 (clk_ctl_st bit 24, CCS_SECI_AVAIL [B]) needs, in the
model, the FLL LPO: pmucontrol_ext (PMU +0x6d8) |= 0x80, poll GCI
chipstatus[15] bit 1 (index write 15 to GCI +0x40, read back, read +0x204),
|= 0x200, then the CC request 0x110. [B] names, [I] that this gates bit 24.
Before touching any PMU/GCI index register, either read every write back or
set pmucontrol_ext bit 0 (FAST_TRANS_ENAB) as stock does (0x281). [F] run 15.

## Evidence sources

1. **brcmfmac driver source** (`drivers/net/wireless/broadcom/brcm80211`):
   authoritative [F] for the host's pre-boot writes and the backplane layout.
2. **Static disassembly** of `brcmfmac4387c2-pcie.apple,madagascar.bin`
   (`notes/madagascar.thumb.asm` in the sibling repo), Thumb-2.
3. **A live RAM dump from real hardware** (`fw/dump/ram_live.bin`, taken via a
   debug brcmfmac coredump) and its firmware console (`fw/dump/kernel.log`):
   ground truth for memory extents and the boot banner. The ROM itself is
   **not** dumpable — host-side attempts to read it over PCIe hard-crash the
   Mac's PCIe controller — so this QEMU model is the primary path for ROM RE.

## Findings

| # | Item | Status | Evidence |
|---|---|---|---|
| C0 | Core is ARM **Cortex-R4** (ARMv7-R, PMSAv7 MPU, Thumb-2, no MMU) | modelled as `cortex-r5` | [F] brcmfmac `chip.c` treats 4387's CPU as `BCMA_CORE_ARM_CR4` (`brcmf_chip_get_core(..., BCMA_CORE_ARM_CR4)`, 4387 case rambase 0x740000). [F] firmware uses PMSAv7 MPU regs (`mcr p15,0,Rd,c6,c1/c2` region/enable) and TCM regs (`c9,c1`), no TTBR/MMU, 524 `sdiv/udiv`, zero VFP. QEMU has no R4; R5 is the nearest ARMv7-R PMSA core. [I] the R4/R5 delta (no LPAE, minor id regs) is not exercised by early boot. |
| C1 | Core resets into **Thumb** at the vector in TCM | **yes**, via new `cfgte` CPU prop | [F] reset handler `0x761c24` and all vectors are Thumb `b.w`. QEMU's AArch32 R-profile reset clears T; added a `cfgte` property that sets `SCTLR.TE` at reset (real CR4 samples a CFGTHUMBEXCEPTIONS pin). Without it the first instruction fetch runs as ARM and aborts. |
| C2 | CR4 TCM bank registers (`BANKIDX` 0x40 / `BANKINFO` 0x44 / `NAB` at 0x04) | modelled, [I] | [F] startup `0x829d30` reads bank count from `[cr4+4]` and walks banks via 0x40/0x44 to compute RAM size, comparing against `c9,c1` TCM size and `0x1f9000`. Model synthesises banks summing to atcm-size + (0x1f9000-atcm). Exact Apple bank split [?]; any split that sums right passes. |
| M0 | RAM/TCM: ATCM at 0 and aliased at 0x740000, BTCM contiguous to 0x939000 | **yes** | [F] rambase 0x740000, size 0x1f9000 (`ramsize-srsize`) from `chip.c` + confirmed by the live dump extent (0x740000–0x939000). [F] reset handler copies 32 B of vectors from 0x740020 to 0x0, so address 0 must be RAM. atcm/btcm split is a model knob (`atcm-size`, default 0x140000) [?]. |
| H0 | Host pre-boot writes: firmware at rambase; NVRAM+seed+footer at top of RAM; reset word | **yes** | [F] `brcmf_pcie_download_fw_nvram()`: image at rambase, then at top `[256 B random seed][len=0x100,magic=0xfeedc0de][stripped NVRAM][token]`. Generated by `scripts/mkhostblob.py`; loaded via `hostblob=`. Startup `0x829a94`/`0x82add6` reads the NVRAM token and seed (`0xfeedc0de` literal at `0x829c74`). |
| B0 | ChipCommon: chip id 0x4387 rev 7, EROM pointer | modelled, [I] | [F] startup `0x829a98` reads `[0x18000000]` (chipid) and `[0x180000fc]` (eromptr). Model returns id `0x4387|rev7` and a synthesised EROM. "BCM4387 r7" matches the live console banner [F]. |
| B1 | EROM: ChipCommon + ARM-CR4 cores, brcmfmac DMP format | modelled, [I] | [F] DMP descriptor encoding from `chip.c` (`DMP_DESC_COMPONENT/ADDRESS/EOT`, designer 0x4bf, CR4 partnum 0x83e). Enough for `brcmf_chip_dmp_erom_scan`; the firmware's own EROM walk (`erom`/`Reached end of erom` strings) is only lightly exercised pre-hang. |
| R0 | **On-chip ROM 0x400000–0x57ffff** | **stub only**: `bx lr`, or a dump file via `rom=` | [F] the RAM image makes thousands of `bl`/`blx` into this window (histogram in `notes/00-triage.md`); it is not in any file. Model fills it with `bx lr` so stubbed calls return; `contrib/plugins/romtrap` logs/stubs entries. **No real ROM available** (hardware dump abandoned). |
| X0 | RNG at ChipCommon 0x3c0/0x3e0/0x3e4 and 0x18030070 TCAM window | modelled, [I] | Catch-all returns last-written value (like ax210 R1); a deterministic xorshift feeds the RNG FIFO so the early RNG drain (`0x79adfc`) terminates. All [I], meanings mostly [?]. |
| Z1 | **Boot crashes in the firmware's ASLR/reclaim self-relocation**, not in a ROM leaf call | **blocker, root-caused** | [F] see below. The shared-RAM boot word at 0x938ffc never initialises. |
| Z2 | Hand-modelled `malloc` (bump allocator) at ROM 0x47eadc | modelled, [I], **not yet exercised** | Native Thumb stub patched into the ROM (`install_rom_stub`), arena in scratch 0x600000–0x740000, cursor at 0x600000. r0=size→r0=ptr, 8-byte aligned, never frees. [F] a later attach callsite is `malloc(432)` NULL-checked. **Boot dies in relocation (Z1) before this is reached**, so it's currently untested; kept because it's correct and needed once Z1 is passed. |

## Root cause: ASLR code relocation, fed a bad range by the model (Z1)

The firmware **relocates its own code to an ASLR-randomized address** early in
boot (the "Code ASLR RAM offset" path) and then **re-links the moved code's
branch instructions**. The relocator is at `0x8a4380` (BL, ±16MB) and `0x8a42e8`
(B.W, ±32MB), driven by a per-section loop at `0x8a44e4`, called once from
`0x8a4034` (which also reprograms the MPU and toggles the cache around it).

Mechanism (all [F], read out of the running model):
- **Re-link, not memcpy.** For each entry in a per-section relocation table
  (list of halfword offsets at `tbl+20`), it loads the instruction at
  `base + off`, decodes the Thumb BL/B.W signed immediate (`ubfx`/sign-extend),
  **adds a per-section delta**, range-checks (±16/32MB), re-encodes the
  S/J1/J2 bits, and writes it back in place. Delta = `desc[0] - desc[1]`
  (new section address − link address); `base = desc[2]`.
- **ASLR source = the host random seed.** The new address comes from
  `0x8a3eb8`: `new = (rng() % section_range)` (8-byte aligned), where `rng()`
  (`0x8a469c`) **consumes the host-provided 0x100-byte random seed** 4 bytes at
  a time (seed ptr/len at `ctx+0x34`/`ctx+0x38`). That seed is exactly what
  `brcmf_pcie_provide_random_bytes()` writes and what `scripts/mkhostblob.py`
  places. So **whoever controls the seed controls the ASLR layout** — a fixed
  seed makes relocation fully deterministic.

Two earlier claims in this doc were **wrong and are corrected here**: (a) code is
**not** copied "+0x990" — it is patched **in place** at its loaded image address
(the runtime≠image bytes at `0x82c99c` are the in-place branch rewrite, not a
move); (b) the cause is **not** the CR4 bank/RAM-size calc (C2). The ASLR region
fields are **hardcoded constants in the image** (loaded by `0x8a3d74`:
`[ctx+0x20..0x2c] = {0x400000, 0x19b000, 0x400, 0x3ffc00}`, `[ctx+8]=0x740000`),
identical to hardware — so they are not a model input at all.

Why the model still crashes: `0x8a3eb8` computes the placement from those
constants plus the seed `rng()`, and `0x8a453c`/`0x8a4576` refine it; in this
model `desc[0]` comes out `0xffd8f418` (delta `0xffd8f018`), the re-linked
branches point below RAM (runtime `0x82c998 = bl 0x20df9c`), and calling one
nop-slides through the zero-returning catch-all into ROM at `0x400000` (the
"every ROM entry is 0x400000" artifact — explained, not a plugin bug). Since the
region constants and seed are faithful, the remaining suspect is the **`-roml`
ROM-side placement**: the "Code ASLR RAM offset **ROM offset**" string and the
`0x400000`-based region fields show the allocator tracks ROM-resident code too,
and with a `bx lr` stub ROM its ROM-side bookkeeping is garbage. **This points
back to needing the real ROM** (or a faithful model of the ROM code map) for the
allocator, not to a leaf-function or bank-size fix. NOP-ing the relocation driver
call at `0x8a4034` does **not** avoid it — the reclaim/ASLR machinery
(`0x8a453c`, `0x8a469c`) is reached via other callers and boot then stalls inside
it (seed-draw loop at `0x8a469c`). So **the model does not yet boot past
relocation**, and the ordered ROM-entry list is still not obtainable.

**Consequence:** with a `bx lr` ROM, boot can't pass self-relocation, so it never
issues the clean ROM *code* calls we wanted to enumerate — the first ROM "call"
is the nop-slide, not a real entry. Hand-modelling leaf functions (malloc/memcpy)
does **not** help: the wall is the relocator producing an out-of-RAM call target.
The relocated attach code a little further on does make clean ROM calls (e.g.
`0x417ee8`, a register-config helper, called repeatedly with r1=5), so the model
*would* enumerate them once relocation is correct.

Next step (software-only): RE the reclaim/relocation routine (the +0x990 copier;
`str.w r7,[r5,#0x990]` at static `0x765838` is inside it) to find which
base/offset it uses, then either feed that value from the model or neutralise the
relocation so the image runs in place. Only then is the ordered ROM-entry list
meaningful.

## Z3: identity-relocation patch (validated)

Correcting Z1 again: the ASLR **does move code** — it copies each section to
`link + offset` (offset was +0x400 for one seed; runtime `0x76ca38` ← image
`0x76c638`, `0x82c998` ← image `0x82c598`) and the earlier wild `0xffd8f418` was
a degenerate offset. The placement offset is produced by two helpers: `0x8a3eb8`
(page) and `0x8a3e48` (intra-page), both drawing the host seed via `0x8a469c`.

Forcing **both helpers to return 0** makes every section's offset 0 → code is
copied to its own link address (no move) → per-section delta 0 → branches
unchanged. Patch (image offset = addr − 0x740000):

    0x163e48 (0x8a3e48):  08 b5 07 4b  ->  00 20 70 47   ; movs r0,#0 ; bx lr
    0x163eb8 (0x8a3eb8):  13 4b 70 b5  ->  00 20 70 47   ; movs r0,#0 ; bx lr

Verified in the emulator:
- **Byte-identical** after relocation: runtime == image at `0x76ca38` (a
  non-branch `push`), `0x82c998`, and the reloc-table BL at `0x76ca40`
  (`0xfafaf49d`, unchanged). So a hook patched at image offset `0x76ca38` runs
  verbatim in place.
- **Boot proceeds into the real ROM-call phase** at image addresses (callers at
  `0x82c598`, `0x824bf4`, `0x840e1a`…), reaching the ROM-dependent stall (hot
  loop on ROM `0x47eb84`). First distinct real ROM entries: `0x47eb84`←`0x82c598`,
  `0x47cb94`←`0x824bf4`, `0x47eb10`←`0x83e78a`, `0x40a5b4`←`0x840e1a`,
  `0x40a26c`←`0x840e26`, `0x484ccc`←`0x840fd4`.
- **Seed-independent:** the patch bypasses the `0x8a469c` seed draw for
  placement, so no fixed host seed is needed for identity.

## romtrap (fixed)

`contrib/plugins/romtrap` now detects the **edge** into the ROM window (previous
instruction outside, current inside) per executed instruction, logging the true
target reached and the true caller (previous PC) instead of the TB vaddr. This
is what revealed Z1 (the `bl 0x20df9c` nop-slide). Args: `lo`/`hi` (window),
`stub=on|off` (return to lr with r0=0), `limit`. `lo=0x140000,hi=0x740000`
catches wild branches below RAM.

## Harness vs. tests

No qtests yet (the ax210 model has them; this one is younger). The model is
exercised only by `scripts/sim.sh` in the sibling repo. First qtest to add:
assert the machine constructs, loads a firmware+hostblob, and that the catch-all
store reads back what was written (the R1-equivalent).

## Z4: MPU map + trampoline code cave (for the ROM-dump hook)

MPU regions captured from the running model under identity relocation
(`contrib/plugins/mpudump`, XN = DRACR[12]):

| rgn | base | size | XN | AP | note |
|---|---|---|---|---|---|
| 0 | 0x18000000 | 128M | 1 | 3 | backplane/peripherals |
| 1 | 0xe8000000 | 32M | 1 | 3 | peripherals |
| 2 | 0x00000000 | 16M | 0 | 3 | RWX fallback |
| 3 | 0x00000000 | 2M (SRD 0xe0) | 0 | 6 | low RO+X |
| 4 | 0x00000000 | 32B | 0 | 6 | vector table RO+X |

This is the **pre-"FW boot complete"** state (the model stalls at the ROM gate
during attach and never reaches boot-complete). On real silicon a *finer*
text/data split is programmed later: the canary showed `0x7478b0` faulting XN
(ifsr 0xd) — `0x7478b0` is at `0x740000+0x78b0`, **below** the executed-code
range (min PC `0x752d18`), i.e. in the low vectors/.bss/data area, hence XN.

**Code cave = `0x864c98`** (zero run `0x864c96..0x864d34`, 0x9e bytes; 4-aligned
start gives ~0x9c usable). It is zero in the image **and** in the real
post-boot coredump (`fw/dump/ram_live.bin`), so it survives from load through
boot-complete untouched. It lies inside the executable code+rodata span
(~`0x750000..0x8bec9f`): rodata strings (e.g. "Code ASLR RAM offset" at
`0x81a740`) are interleaved with executable code, so that whole span is one RX
MPU region — unlike the separate low data region holding `0x7478b0`.

Emulator confirmation (`fw_tramp.bin` = identity patch + a copy-loop trampoline
at `0x864c98`, invoked from early startup):
- Trampoline **executes** at the cave; copy loop reads `0x400000` (bx-lr stub ->
  garbage, expected) and the 8 stores land at `0x830000..0x83001c` (memwatch).
- `0x830000` is **writable** — no data-abort.
- Caveat: this model does **not** enforce MPU XN (jumping into the XN peripheral
  region raised Undefined-Instruction, not a prefetch abort), and never reaches
  the fine text/data split, so cave *executability* is argued from the RX
  interleaving above + the hardware data point, not emulator-proven. Confirm on
  silicon by re-running the canary with CAVE = 0x864c98.
