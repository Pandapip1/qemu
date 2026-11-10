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
on silicon (run 14) equal silicon (`scripts/stock_state_check.py`), and at
its first PHY read the model's state equals run 31's phylog snapshot.
Our firmware's runs 38/39 (with stock's MPU) reproduce exactly
(`scripts/model_regress.py`). Runs 7-37 of ours ran with the MPU off and
are not modelled (see "Memory attributes" below).

The per-register gap list is generated from the source tables:
`scripts/strict_gaps.py > fw/trace/strict_gaps.tsv` (sibling repo). Values
stock reads that the model lacks are in `fw/trace/strict_unknown_reads.tsv`.

### SDR test bench: SampleCollect + template RAM

`bcm4387-d11sc.c` models each slice's template RAM (port +0x130/+0x134)
and SampleCollect engine (+0xaa0..+0xade). An armed capture (PlayCtrl START,
source 0) stores one I/Q word per 20 MHz sample from StartPtr through
StopPtr while the slice's PHY ports are live and PHY 0x467 bit 13 is set.
The samples come from `-M bcm4387,...,iq-source=<file>,iq-format=s16|cf32`
(interleaved I/Q at 20 MHz, played back by virtual time and looped), or
from deterministic noise. Completion semantics are [I] and are listed in
the gap file for run 17. The d11 core window is strict: offsets with no
model are AXI slave errors. `model_regress.py` feeds a 1.25 MHz tone through
our MPU-enabled capture image (fw/open/open-capmpu.bin) with no model
assumption and checks the recovered frequency. That image crashed the host
on silicon (run 40, no data; cause unknown), so the check is about the
model only.

### PSM: the d11 microcode CPU as a second core

`bcm4387-psm.c` runs each slice's PSM. The ISA and IHR evidence is in
notes/33-psm-model.md (sibling repo), taken from stock's rev-85 ucode as stock
downloads it.

**What the host side does**
- objaddr/objdata (+0x160/+0x164/+0x166) fill microcode memory (select 0, one
  64-bit instruction per two 32-bit words, low word first) and SHM (select 1,
  32-bit word addressing, auto-increment) [B names, I layout].
- maccontrol MCTL_PSM_RUN / MCTL_PSM_JMP_0 run the PSM and reset its PC [B].
- Core reset halts the PSM.
- The PSM executes on a virtual-clock timer while its slice is out of reset
  with its MAC fast clock (FCLKA) [I]. It runs at an assumed 160 MHz [U].

**What the PSM itself models**
- The full rev-85 instruction set as far as its semantics are known: 14-bit
  operands, 32-bit add/sub (opcode bit 5), and jext 0x5f as the
  unconditional jump [I].
- The indirect-SHM base registers b0..b6 at IHR 0x221/0x222/0x225/0x226/
  0x229/0x22a/0x22d [I], and the mul low result at IHR 0x1c7 [I].
- Its own PHY port (IHR 0x91 data / 0x92 address, opcode 0x102: 0 = read,
  0x20 = write) and radio port (0x94/0x95) [I]. Both ports reach the same
  PHY/radio register files as the host ports, gated by `port_live` [I].
  A PSM port command while the port is not live is a strict abort; machine
  option `psm-dead-port=busy` instead drops the command and keeps the port's
  busy bit (address register bit 15) set until core reset [U], to test
  microcode that bounds its busy-waits (bcm4387 `fw/ucode/phyprobe.asm`).
- SampleCollect through IHR i = host +0x400+2i [I].

**Strict aborts.** Every other IHR, every external condition except 0x5f, the
opcodes with unknown semantics (0x070/0x071 jboh, 0x086, 0x1a4/0x1a5/0x1a6/
0x1b4, and 0x102 to anything but the port registers), and `nap` (no modelled
wake source) stop the PSM with "PSMn strict abort". With `strict=off`,
unknown IHRs become plain storage and conditions read false.

**Stock never starts the PSM in this model.** Its attach-only boot keeps
maccontrol at 0x04000400 (= silicon at its first PHY write [F]). The PSM is
started by wlc_up, which needs a host WLC_UP.
`scripts/psm_bench.py` (sibling repo) plays the host-driven start over qtest:
1. Stock's core reset.
2. Stock's own ucode words from the trace.
3. JMP_0, then IHR_EN | INFRA | PSM_RUN | WAKE (brcmsmac coreinit [B]).

Result, slice 0:
- `strict=on`: abort at instruction #2, a read of unmodelled IHR 0x1c6
  (host +0x78c) at pc 0x133f, the init entry.
- `strict=off`: the PSM writes 0x8002 to IHR 0x1e0 (+0x7c0), sets IHR 0x1f4
  (+0x7e8) bit 5, and spins forever at pc 0x1347 waiting for 0x1f4 bit 13.
  That is a request/acknowledge whose other side is unknown [U].
- It never reaches its PHY port.

Next gaps, in the order the ucode needs them: IHR 0x1c6 bit 5 (warm-start
flag?), and the 0x1f4 bit 5 -> bit 13 handshake.

### Memory attributes: the root cause of runs 7-37 (2026-11-10)

Our firmware ran runs 7-37 with the Cortex-R4 MPU off (SCTLR 0x40e50878 at
entry, M = 0). Under the ARMv7-R default memory map, 0x00000000-0x7fffffff
is then Normal (non-cacheable) memory, so every backplane access at
0x18000000 was a Normal access: the core may merge stores in its store
buffer, let a load overtake an earlier buffered store, or forward a load
from a buffered store [B: ARMv7 memory model]. Stock enables its MPU before
its first backplane access (RAM 0x829c94 + 0x75357a, table 0x829cf0: region
0x18000000/128M and 0xe8000000/32M with DRACR 0x1305 = shareable Device,
plus two Normal RAM regions) [F]. With stock's MPU in our firmware (run 38,
`cpu::enable_stock_mpu()`):
- MPUIR reads 0xc00 (12 data regions); the model's CPU now has 12
  (`pmsav7-dregion`). [F]
- the PHY port answers from the attach on: PHY 0x00b 0x9bd2, 0x031 0x03ff,
  0x000 0xdb32, 0x001 0x1510 at IOCTL 0x145 before any PLL request, the same
  as stock's (runs 22/31), and after every enable_phy step (run 39). [F]
- PHYPLL is granted (0x1f0b0342, res_state 0x5ecefbff) for every image,
  including the run-21 code with mask 0x40000000 and the later sources that
  never got it before (one-shot |= 0x302). [F]
- radio 0/1 read 0x000c/0x50dd; ChipCommon ext resource 0 (SECI clock) comes
  up on our request with no SECI init (0x030b0152). [F]
- PHY 0x12e reads back what was written (0x1012), PHY 0x480 0xce00, and the
  raw dump reads PHY 0x000-0x005 = 0xdb32, 0x1510, 0x0041, 0, 0x0080, 0, then
  takes a synchronous external abort (DFSR 0x1008) on PHY 0x006, in every
  image of runs 38/39. [F]

TCG performs every access in program order with no store buffer, which is
Device behaviour, so the model's register semantics are now the Device
ones, and Normal-memory MMIO is a stated fidelity gap: under `strict=on`,
the first backplane access (ChipCommon, PMU, GCI, d11/wrappers) made with
the MPU off or through a non-Device region logs, once per run,
"FIDELITY: Normal-memory MMIO ... (SCTLR, DRACR)" (bcm4387_mmio_attr).
`model_regress.py` reports a diff for such a run as "NOT MODELLED
(Normal-memory MMIO)" and does not count it; probes (which run with the MPU
off, as on silicon) whose checked outcome is a PHY port or PHYPLL one are
listed in `NORMAL_MMIO_OUTCOME` and treated the same way.

Rules fitted to Normal-memory runs and now **withdrawn**: the PHY port is
dead / a latch / echoes (runs 8-37; kept only as `phy-live=off`, a
description of those observations, not of the hardware); the PHY window
aborts without PHYPLL (runs 6, 22, 30-32); the PMU honours a request
mask without closure (runs 23, 26-29); the PHYPLL grant depends on the
source (`mac-pll`, runs 17-33; the option is gone); the port goes live
after ucode (runs 21/31), after a first pass (run 32), or with
PHYPLL + FCLKA + IHR_EN (run 31); the PLL grant needs stock's FORCEHT-first
request form (run 32); the SECI clock needs SECI_config bit 2 (runs 10/11).

### Enable chain (what the model says makes the PHY usable)

1. Stock's si_lhl_timer_config (RAM 0x82b0be): GCI 0xc70 |= 3, PMU
   mac_res_req_mask (+0x68c) = 0x56cac1ff, poll pmustatus bit 10
   (PST_SLOW_WR_PENDING) clear, mac_res_req_timer (+0x688) = 0x15000000,
   GCI 0xc78 = 0. [F] values (run 30 read-backs), [B] sequence. The PMU
   grants a request timer's mask with its dependency closure: 0x40000000
   alone was granted as 0x5ecefbff (run 38). [F]
2. srpwr request domain 3 (MAC main) or 0x1e; poll status. [F]
3. d11 core reset with PHY clock+reset held, ioctl to 0x145. [F] The PHY
   port is live from here (step 6). The MAC does not yet ask the PMU for
   anything (res_state 0x1e4efbdf at stock's attach read, run 31; ours, run
   38). [F]
4. d11 maccontrol (+0x120) = 0x04000400 (MCTL_WAKE | MCTL_IHR_EN) [B], then
   the PLL request clk_ctl_st |= 0x302: the MAC asserts its request to the
   PMU MAC resource request and PHYPLL comes up (0x1f0b0342). [F] run 38.
   Without WAKE the 0x200 request asks for MACPHY_CLK directly (runs 7-11;
   Normal-memory runs, kept as [I]); with WAKE only the MAC request does
   (run 16, Normal memory: [I]). A core reset drops the clk_ctl_st requests
   (0x150b0040 right after the post-PLL PHY reset, runs 38/39). [F]
5. Optional, stock does them, no effect modelled: d11 +0x18c = 0, 16-bit
   +0x3e6 = 0. [U]
6. **PHY/radio ports.** Live whenever the core answers (powered, out of
   reset, clocked), PLL or not [F, Device MMIO]. Values seeded from runs
   31/38/39: PHY 0x000 0xdb32, 0x001 0x1510, 0x002 0x0041, 0x003 0,
   0x004 0x0080, 0x005 0, 0x00b 0x9bd2, 0x031 0x03ff, 0x480 0xce00; radio
   0 0x000c, 1 0x50dd. Reads of PHY 0x006 answer an AXI slave error before
   PHY init [F runs 38/39]; which other registers do so, and which part of
   PHY init clears it, are unknown [U]: the model keeps the fault for the
   whole boot so a later read is flagged, not guessed. Run 40 (our capture
   image with the MPU, raw dump and SampleCollect capture) hard-crashed the
   host with no data; the model does not encode a cause. [F]
   `phy-live=off` reproduces the runs 8-37 Normal-memory observations
   (latches) for reference only.

ChipCommon ERSRC0 (clk_ctl_st bit 24, CCS_SECI_AVAIL [B]) follows the SECI
clock request while the FLL LPO runs, which it does from handoff
(pmucontrol_ext 0x14291). [F] run 38. Before touching any PMU/GCI index
register, either read every write back or set pmucontrol_ext bit 0
(FAST_TRANS_ENAB) as stock does (0x281). [F] run 15 (Normal memory: the
synchroniser rule is [I] until re-measured with the MPU on).

### Reload-persistent PMU/GCI state and the PMU/LHL registers

- **Reload-persistent state** [F]: brcmfmac's reload resets the cores and
  slice domains but not the PMU or GCI/LHL (notes/36 §3.1). The model's
  handoff state is stock's: res_dep_mask, res_updn_timer, chipcontrol,
  regcontrol, pllcontrol and GCI chipcontrol from run 14
  (`fw/trace/indirect_stock_run14.tsv`, 94 entries); pmucontrol 0x380, min
  1, max 0x7fffffff; pmucontrol_ext 0x14291 and GCI 0xc70 = 3, 0xc78 = 0
  (run 30); GCI 0xc44 = 1 (run 31). Inferred to persist too, never read at
  our handoff: GCI 0xc40 = 3 and res_req_mask/timer = 0x160880df/0x15000000
  (stock reads exactly that at its attach read, run 31). [I] The MAC
  request registers do not persist (run 16, Normal memory). [I]
- **REQ_ACTIVE**: a request timer reads bit 25 while its request is asserted
  and its masked resources are up [I]: stock's res_req 0x17000000 (run 31),
  our MAC timers 0x15000000 before the MAC asks. [F]
- **pmustatus PST_SLOW_WR_PENDING**: set while a posted always-on index
  write is pending, and for 3 ILP cycles after a write to an ILP-domain
  register (pmuwatchdog, res_req_timer, mac_res_req_timer 0/1/2) made with
  FAST_TRANS_ENAB clear. [B] register list (siutils.c
  PMUREGS_ILP_SENSITIVE, HND_PMU_SYNC_WR), [I] gating and duration; a
  second ILP write in that window is dropped [I] (bcmdhd calls it a stall
  [B]). With FAST_TRANS set at handoff it never pends for us (run 30). [F]
- **Always-on index synchroniser**: posted until the firmware itself writes
  pmucontrol_ext with FAST_TRANS (run 15's observation; Normal memory). [I]
- **GCI LHL timers** (bcm4387-gci.c), named per sbgci.h. MAC timers
  0xc70-0xc9c: stored; an int_adr write clears the status [B]. ARM timer
  0xc40/0xc44/0xc48: a non-zero count expires that many ILP cycles later,
  sets 0xc44 bit 0 (W1C) and clears the count [I]; reads back as written
  while running (0x39, run 31) and 0 after expiry (stock, run 31) [F]. No
  PMU effect: "the expiry asserts the ARM request with XTAL_HQ" was refuted
  by run 31. [F]
- **d11 extra wrappers' error-log block** (+0x900..+0x93f) is storage
  (run 32: errlogctrl 0x330 reads back on 14/14 wrappers). [F]

Regressions (`scripts/model_regress.py`, sibling repo):
- runs 38/39 (queued-38/1-r21mpu, 2-a1mpu, 3-probempu, queued-39/1-steps):
  SAME, and the decode must show PHYPLL, the attach-time PHY values and the
  PHY 0x006 abort (DFAR 0x180313fe);
- runs 7-37 of ours (queued-2..33): NOT MODELLED (Normal-memory MMIO),
  reported, not counted;
- stock at its first PHY read (queued-31/1-phylog-hq.bin run in the model,
  its attach-time record decoded by scripts/phylog_decode.py and compared
  with silicon's; it depends on stock's LHL ARM timer having expired by
  then in virtual time) and stock_state_check.py: no model assumption.
- SDR tone check: fw/open/open-capmpu.bin, no assumption.

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
