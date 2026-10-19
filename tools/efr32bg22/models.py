"""EFR32BG22 peripheral models (register level, just enough to run firmware)."""
import struct
import resets


class Dev:
    """Register file with reset values; subclasses add read_<off>/write_<off> hooks."""
    name = "dev"

    def __init__(self, name, kind=None):
        self.name = name
        self.m = None
        kind = kind or name.rstrip("0123456789")
        self.regs = {off: v for off, (_, v) in resets.REGS.get(kind, {}).items()}

    def read(self, off, size=4):
        h = getattr(self, "r_%03x" % (off & ~3), None)
        if h:
            return h()
        return self.regs.get(off & ~3, 0)

    def write(self, off, val):
        h = getattr(self, "w_%03x" % off, None)
        if h:
            h(val)
        else:
            self.regs[off] = val & 0xFFFFFFFF


class Fallback(Dev):
    """Anything unmodelled: plain read/write memory, so software sees what it wrote."""
    def __init__(self):
        super().__init__("?")


class SyncFallback(Fallback):
    """Unmodelled block whose register at +0x40 is a self-clearing busy flag.  Inferred
    from the stock firmware (0x199ea..0x199f4 sets bit 0 via the SET alias and spins until
    it reads 0), not from documentation: the radio register headers are not public."""
    def r_040(self):
        return 0


SYNC_BLOCKS = (0xA8028000, 0xA802C000)


def fallback_for(m, base):
    d = SyncFallback() if base in SYNC_BLOCKS else Fallback()
    d.m = m
    return d


class DCDC(Dev):
    """IPVERSION 0, EN 4, CTRL 8 (MODE bit0), IF 0x24, STATUS 0x2C."""
    BYPSW, WARM, RUNNING, REGULATION = 1, 2, 4, 0x20

    def _settle(self):
        mode = self.regs.get(0x008, 0) & 1
        if mode:
            self.regs[0x024] = self.regs.get(0x024, 0) | self.WARM | self.RUNNING | self.REGULATION
            self.regs[0x02C] = self.WARM | self.RUNNING
        else:
            self.regs[0x024] = self.regs.get(0x024, 0) | self.BYPSW
            self.regs[0x02C] = self.BYPSW

    def w_008(self, v):
        self.regs[0x008] = v
        self.m.after(20_000, self._settle)

    def r_02c(self):
        return self.regs.get(0x02C, self.BYPSW)


class Osc(Dev):
    """On-demand oscillator.  ON when CTRL.FORCEEN (bit0 of ctrl_off) or when a
    consumer requests it (request(m) -> bool).  STATUS shows RDY|ENS while on."""
    def __init__(self, name, status_off, ctrl_off=None, request=None, always=False):
        super().__init__(name)
        self.ctrl_off, self.request, self.always = ctrl_off, request, always
        setattr(self, "r_%03x" % status_off, self.status)

    def on(self):
        if self.always:
            return True
        if self.ctrl_off is not None and self.regs.get(self.ctrl_off, 0) & 1:
            return True
        return bool(self.request and self.request(self.m))

    def status(self):
        return (1 | 1 << 16) if self.on() else 0


class HFXO(Dev):
    """Starts when forced (CTRL.FORCEEN) or when something requests it (m.hfxo_requested).
    FSMLOCK is set while the oscillator state machine owns the core bias; a
    MANUALOVERRIDE command (CMD bit1) releases it."""
    ON = 1 | 2 | 1 << 16 | 1 << 17 | 1 << 19

    def __init__(self, name):
        super().__init__(name)
        self.lock = True

    def on(self):
        return bool(self.regs.get(0x028, 0) & 1) or self.m.hfxo_requested()

    def r_058(self):
        if not self.on():
            self.lock = True
            return 0
        return self.ON | (1 << 30 if self.lock else 0)

    def w_050(self, v):
        if v & 2:
            self.lock = False


class RadioAES(Dev):
    """BA411 front-end: idle (WAITING only); crypto operations are not performed."""
    def r_03c(self):
        return 0x20


class TRNG(Dev):
    """CRYPTOACC BA431 TRNG.

    Normal mode: after NDRNG_ENABLE the state machine reaches FIFOFULL and the
    FIFO stays full (data is a fixed-seed xorshift stream, NOT random).
    Conditioning test mode (CTRL.TEST_ENABLE): TESTDATA words are fed through the
    conditioning function, AES-128-CBC with IV 0 keyed by KEY0..3 (little-endian
    words), and the last block is pushed to the FIFO after NB_128BIT_BLOCKS blocks
    (checked against the SDK known-answer test).
    Registers: 0 CTRL, 4 FIFOLEVEL, 0xC FIFODEPTH, 0x10..0x1C KEY, 0x20 TESTDATA,
    0x30 STATUS."""
    DEPTH = 64
    STARTUP, FIFOFULLON = 2, 4

    def __init__(self, name, kind=None):
        super().__init__(name, kind)
        self.state = 0
        self.fifo = []
        self.tbuf = b""
        self.cbc = None
        self.x = 0x2545F491
        self.nb = 4

    def rnd(self):
        x = self.x
        x ^= (x << 13) & 0xFFFFFFFF
        x ^= x >> 17
        x ^= (x << 5) & 0xFFFFFFFF
        self.x = x
        return x

    def pop(self):
        if self.test_mode():
            return self.fifo.pop(0) if self.fifo else 0
        return self.rnd()

    def test_mode(self):
        return bool(self.regs.get(0x000, 0) & 4)

    def level(self):
        if self.test_mode():
            return len(self.fifo)
        return self.DEPTH if self.state >= self.FIFOFULLON else 0

    def w_000(self, v):
        old = self.regs.get(0x000, 0)
        self.regs[0x000] = v
        if (v >> 16) & 0xF:
            self.nb = (v >> 16) & 0xF       # the SDK writes CTRL after NB_128BIT_BLOCKS without it; the
                                            # block count is sticky (needed for the SDK's own self-test to pass)
        if v & 0x100 or (v & 4 and not old & 4):          # SOFTRESET / entering test mode
            self.fifo, self.tbuf, self.cbc, self.state = [], b"", None, 0
        if v & 1 and not old & 1:
            self.state = self.STARTUP
            self.m.after(50_000, lambda: setattr(self, "state", self.FIFOFULLON))
        if not v & 1 and not v & 4:
            self.state = 0

    def r_004(self):
        return self.level()

    def r_00c(self):
        return self.DEPTH

    def w_020(self, v):                                    # TESTDATA
        if not self.test_mode():
            return
        self.tbuf += struct.pack("<I", v)
        if len(self.tbuf) == 16:
            from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
            key = b"".join(struct.pack("<I", self.regs.get(o, 0)) for o in (0x10, 0x14, 0x18, 0x1C))
            if self.cbc is None:
                self.cbc = bytes(16)
                self.nblk = 0
            enc = Cipher(algorithms.AES(key), modes.CBC(self.cbc)).encryptor()
            self.cbc = enc.update(self.tbuf) + enc.finalize()
            self.tbuf = b""
            self.nblk += 1
            if self.nblk == self.nb:
                self.fifo += list(struct.unpack("<4I", self.cbc))
                self.cbc = None

    def r_030(self):                                       # STATUS: STATE in bits 3:1
        return self.state << 1 if not self.test_mode() else 0


class TRNGFifo(Dev):
    def __init__(self, name, trng):
        super().__init__(name, "")
        self.trng = trng

    def read(self, off, size=4):
        return self.trng.pop()


class IrqDev(Dev):
    """Device with IF/IEN registers that drives one NVIC line (IF & IEN)."""
    irq_name = None
    IF_OFF, IEN_OFF = 0, 0

    def irq_update(self):
        n = resets.IRQS.get(self.irq_name)
        if n is not None:
            self.m.set_irq(n, bool(self.regs.get(self.IF_OFF, 0) & self.regs.get(self.IEN_OFF, 0)))

    def raise_if(self, bits):
        self.regs[self.IF_OFF] = self.regs.get(self.IF_OFF, 0) | bits
        self.irq_update()

    def write(self, off, val):
        super().write(off, val)
        if off in (self.IF_OFF, self.IEN_OFF):
            self.irq_update()


class BUFC(IrqDev):
    """xG22 four-channel radio byte-buffer controller.

    xG22's public device headers omit this block.  The layout and bit fields here
    combine the public efr32xg21_bufc.h with offsets observed in the xG22 stock
    trace: channels start at +0x0c with a 0x40 stride; IF/IEN are +0x114/+0x118.
    This model covers the programmed data port, offsets, status, clear command,
    threshold flags and IRQ.  The bridge can now read guest memory, but the
    trace has not established BUFC's DMA trigger or transfer length, so DMA is
    deliberately not pretended here.
    """
    IF_OFF, IEN_OFF = 0x114, 0x118
    irq_name = "BUFC"
    BUF0, STRIDE = 0x00C, 0x040

    def __init__(self, name):
        super().__init__(name, "")
        self.data = [bytearray() for _ in range(4)]

    def decode(self, off):
        rel = off - self.BUF0
        if 0 <= rel < 4 * self.STRIDE:
            return rel // self.STRIDE, rel % self.STRIDE
        return None, None

    def capacity(self, n):
        return 64 << (self.regs.get(self.BUF0 + n * self.STRIDE, 0) & 7)

    def update_threshold(self, n):
        base = self.BUF0 + n * self.STRIDE
        cfg = self.regs.get(base + 0x24, 0)
        threshold = cfg & 0xFFF
        hit = len(self.data[n]) <= threshold if cfg & 0x2000 else len(self.data[n]) > threshold
        # Public xG21 BUFC_IF places each buffer's threshold flag at bit
        # 8*n+2 (BUF0THR=2, BUF1THR=10, BUF2THR=18, BUF3THR=26).
        bit = 1 << (8 * n + 2)
        if hit:
            self.regs[self.IF_OFF] = self.regs.get(self.IF_OFF, 0) | bit
        else:
            self.regs[self.IF_OFF] = self.regs.get(self.IF_OFF, 0) & ~bit
        self.irq_update()

    def push(self, n, value, xor=False):
        base = self.BUF0 + n * self.STRIDE
        pos = self.regs.get(base + 0x08, 0) & 0x1FFF
        buf = self.data[n]
        if pos >= self.capacity(n):
            return
        if pos >= len(buf):
            buf.extend(b"\0" * (pos + 1 - len(buf)))
        buf[pos] = (buf[pos] ^ value) if xor else value
        self.regs[base + 0x08] = (pos + 1) & 0x1FFF
        self.update_threshold(n)

    def read(self, off, size=4):
        n, reg = self.decode(off & ~3)
        if n is None:
            return super().read(off, size)
        base = self.BUF0 + n * self.STRIDE
        if reg == 0x14:                         # READDATA
            pos = self.regs.get(base + 0x0C, 0) & 0x1FFF
            value = self.data[n][pos] if pos < len(self.data[n]) else 0
            self.regs[base + 0x0C] = (pos + 1) & 0x1FFF
            self.update_threshold(n)
            return value
        if reg == 0x20:                         # STATUS
            rd = self.regs.get(base + 0x0C, 0) & 0x1FFF
            count = max(0, len(self.data[n]) - rd) & 0x1FFF
            # STATUS.THRESHOLDFLAG is bit 20 for every buffer.  Its source
            # is that buffer's sparse global IF threshold bit, 8*n+2.
            flag = (1 << 20) if self.regs.get(self.IF_OFF, 0) & (1 << (8 * n + 2)) else 0
            return count | flag
        return super().read(off, size)

    def write(self, off, val):
        n, reg = self.decode(off)
        if n is None:
            return super().write(off, val)
        base = self.BUF0 + n * self.STRIDE
        if reg == 0x18:                         # WRITEDATA
            self.push(n, val & 0xFF)
        elif reg == 0x1C:                       # XWRITE
            self.push(n, val & 0xFF, xor=True)
        elif reg == 0x28:                       # CMD
            self.regs[base + reg] = val & 0xF   # public xG21 CMD mask; bit 2 is unassigned there.
            if val & 1:                         # CLEAR
                self.data[n].clear()
                self.regs[base + 0x08] = self.regs[base + 0x0C] = 0
                self.update_threshold(n)
        else:
            super().write(off, val)
            if reg in (0x00, 0x08, 0x0C, 0x24):
                self.update_threshold(n)

    def contents(self, n):
        """Return bytes currently exposed by a channel (test/pcap hook)."""
        return bytes(self.data[n])


class RAC(Dev):
    """EFR32xG22 RAC command/state and interrupt core.

    The register layout and field names are from Silicon Labs' public generated
    revB2 register model.  Packet modulation remains deliberately abstract: a
    TXEN/FORCETX command enters state 9 (TX), then completes after 150 us.
    RXENSRCEN.SWRXEN enters state 2 (RXSEARCH) until RXDIS clears it.
    """
    RXENSRCEN, STATUS, CMD = 0x008, 0x00C, 0x010
    IF_OFF, IEN_OFF = 0x01C, 0x020
    SEQIF_OFF, SEQIEN_OFF = 0x028, 0x02C

    CMD_TXEN = 1 << 0
    CMD_FORCETX = 1 << 1
    CMD_TXDIS = 1 << 5
    CMD_RXCAL = 1 << 7
    CMD_RXDIS = 1 << 8

    STATEOFF, STATERXSEARCH, STATETX = 0, 2, 9
    IF_STATECHANGE = 1 << 0
    SEQ_STATEOFF = 1 << 16
    SEQ_STATERXSEARCH = 1 << 18
    SEQ_STATETX = 1 << 25

    def __init__(self, name):
        super().__init__(name, "")
        self.state = self.STATEOFF
        self.generation = 0

    def status(self):
        active = self.state != self.STATEOFF
        tx = self.state == self.STATETX
        rx = self.state == self.STATERXSEARCH
        return self.state << 24 | active << 28 | tx << 30 | rx << 31

    def r_00c(self):
        return self.status()

    def irq_update(self):
        rsm = resets.IRQS.get("RAC_RSM")
        seq = resets.IRQS.get("RAC_SEQ")
        if rsm is not None:
            self.m.set_irq(rsm, bool(self.regs.get(self.IF_OFF, 0) &
                                     self.regs.get(self.IEN_OFF, 0)))
        if seq is not None:
            self.m.set_irq(seq, bool(self.regs.get(self.SEQIF_OFF, 0) &
                                     self.regs.get(self.SEQIEN_OFF, 0)))

    def set_state(self, state, seq_flag):
        if state == self.state:
            return
        self.state = state
        self.regs[self.IF_OFF] = self.regs.get(self.IF_OFF, 0) | self.IF_STATECHANGE
        self.regs[self.SEQIF_OFF] = self.regs.get(self.SEQIF_OFF, 0) | seq_flag
        self.irq_update()

    def write(self, off, val):
        if off == self.RXENSRCEN:
            self.regs[off] = val & 0x3FFF
            if val & 0xFF:
                self.generation += 1
                self.set_state(self.STATERXSEARCH, self.SEQ_STATERXSEARCH)
            elif self.state == self.STATERXSEARCH:
                self.generation += 1
                self.set_state(self.STATEOFF, self.SEQ_STATEOFF)
            return
        super().write(off, val)
        if off in (self.IF_OFF, self.IEN_OFF, self.SEQIF_OFF, self.SEQIEN_OFF):
            self.irq_update()

    def w_010(self, value):
        self.generation += 1
        generation = self.generation
        if value & (self.CMD_TXDIS | self.CMD_RXDIS):
            self.set_state(self.STATEOFF, self.SEQ_STATEOFF)
        elif value & (self.CMD_TXEN | self.CMD_FORCETX):
            self.set_state(self.STATETX, self.SEQ_STATETX)
            def complete():
                if generation == self.generation and self.state == self.STATETX:
                    self.set_state(self.STATEOFF, self.SEQ_STATEOFF)
            self.m.after(150_000, complete)


# Compatibility for out-of-tree scripts written against patch 0006.
SimRAC = RAC


class Synth(Dev):
    """EFR32xG22 SYNTH frequency/channel controls from the public revB2 model.

    Frequency synthesis itself remains outside this bridge; retaining the
    architecturally visible field masks lets firmware select a BLE channel and
    makes the configuration observable to higher-level radio simulation.
    """
    FREQ, IFFREQ, DIVCTRL, CHCTRL, CHSP = 0x034, 0x038, 0x03C, 0x040, 0x044
    MASKS = {
        FREQ: 0x0FFFFFFF,
        IFFREQ: 0x001FFFFF,
        DIVCTRL: 0x000001FF,
        CHCTRL: 0x0000003F,
        CHSP: 0x0003FFFF,
    }

    def write(self, off, val):
        if off in self.MASKS:
            self.regs[off] = val & self.MASKS[off]
        else:
            super().write(off, val)


class RFCRC(Dev):
    """EFR32xG22 RFCRC register file from the public revB2 model.

    This records configuration and exposes the documented masks.  Feeding the
    FRC packet pipeline and calculating DATA are sequencer behavior, not yet
    represented by the bridge.
    """
    EN, CTRL, STATUS, CMD = 0x004, 0x008, 0x00C, 0x010
    INPUTDATA, INIT, DATA, POLY = 0x014, 0x018, 0x01C, 0x020
    MASKS = {
        EN: 0x00000001,
        CTRL: 0x00001FEF,
        INIT: 0xFFFFFFFF,
        POLY: 0xFFFFFFFF,
    }

    def write(self, off, val):
        if off in self.MASKS:
            self.regs[off] = val & self.MASKS[off]
        elif off == self.CMD:
            pass                            # INITIALIZE is sequencer-owned here.
        elif off == self.INPUTDATA:
            pass                            # Packet CRC calculation is unmodelled.
        else:
            super().write(off, val)


class FRC(Dev):
    """Public xG22 FRC BLE configuration registers.

    The packet pipeline, including applying the whitening LFSR, belongs to
    FRC/SEQ behavior and is deliberately not modelled here.  Retaining these
    public register masks makes the source-backed BLE setup observable.
    """
    DFLCTRL, MAXLENGTH = 0x00C, 0x010
    WCNTCMP0, WCNTCMP1 = 0x020, 0x024
    WHITECTRL, WHITEPOLY, WHITEINIT = 0x030, 0x034, 0x038
    FECCTRL, BLOCKRAMADDR, CONVRAMADDR = 0x03C, 0x040, 0x044
    CTRL, RXCTRL, TRAILTXDATACTRL = 0x048, 0x04C, 0x050
    CONVGENERATOR, PUNCTCTRL = 0x05C, 0x060
    AUTOCG = 0x0A8
    WCNTCMP3, BOICTRL, DSLCTRL, WCNTCMP4 = 0x0BC, 0x0C0, 0x0C4, 0x0C8
    PKTBUFCTRL = 0x0CC
    FCD0, FCD1, FCD2, FCD3 = 0x104, 0x108, 0x10C, 0x110
    MASKS = {
        DFLCTRL: 0x01FFFF7F,
        MAXLENGTH: 0x0000FFFF,
        WCNTCMP0: 0x00000FFF,
        WCNTCMP1: 0x00000FFF,
        WHITECTRL: 0x00001F7F,
        WHITEPOLY: 0x0000FFFF,
        WHITEINIT: 0x0000FFFF,
        FECCTRL: 0x003FFFF7,
        BLOCKRAMADDR: 0xFFFFFFFC,
        CONVRAMADDR: 0xFFFFFFFC,
        CTRL: 0x001F3FF7,
        RXCTRL: 0x000007FF,
        TRAILTXDATACTRL: 0x007FFFFF,
        CONVGENERATOR: 0x00037F7F,
        PUNCTCTRL: 0x00007F7F,
        AUTOCG: 0x0000FFFF,
        WCNTCMP3: 0x00000FFF,
        BOICTRL: 0x0001FFFF,
        DSLCTRL: 0x7FFFFF7F,
        WCNTCMP4: 0x00000FFF,
        PKTBUFCTRL: 0x0103FFFF,
        FCD0: 0x0001FFFF,
        FCD1: 0x0001FFFF,
        FCD2: 0x0001FFFF,
        FCD3: 0x0001FFFF,
    }

    def write(self, off, val):
        if off in self.MASKS:
            self.regs[off] = val & self.MASKS[off]
        else:
            super().write(off, val)


class Modem(Dev):
    """Public xG22 MODEM BLE configuration registers.

    These retain the public configurator's software-visible values only.
    Sync detection, demodulation, and interpreting SYNC0 as a Bluetooth access
    address remain FRC/SEQ behavior and are deliberately not modelled here.
    """
    MIXCTRL = 0x01C
    CTRL0, CTRL1, CTRL2, CTRL3, CTRL4, CTRL5, CTRL6 = (0x020 + 4 * n for n in range(7))
    TXBR, RXBR, CF, PRE, SYNC0, SYNC1 = (0x058 + 4 * n for n in range(6))
    SHAPING10, SHAPING11 = 0x0BC, 0x0C0
    DCCOMP, DCCOMPFILTINIT = 0x0E0, 0x0E4
    DSATHD0, DSATHD1 = 0x0F4, 0x0F8
    AUTOCG = 0x124
    DSATHD2, DIRECTMODE = 0x130, 0x134
    LRFRC, DSATHD3, DSATHD4 = 0x154, 0x168, 0x16C
    BLEIQDSA, SYNCPROPERTIES = 0x230, 0x238
    DIGIGAINCTRL, PADEBUG = 0x23C, 0x244
    ETSCTRL, ANTSWCTRL, ANTSWSTART, ANTSWEND = 0x254, 0x258, 0x25C, 0x260
    TRECPMPATT, TRECPMDET, CFGANTPATT, ETSTIM = 0x264, 0x268, 0x26C, 0x270
    ANTSWCTRL1, ANTDIVCTRL, BLEIQDSAEXT2 = 0x274, 0x27C, 0x280
    MASKS = {
        MIXCTRL: 0x0000001F,
        CTRL0: 0xFFFFFFFF, CTRL1: 0xFFFFDFFF, CTRL2: 0xFFFFFFFF,
        CTRL3: 0xFFFFFF81, CTRL4: 0xBFFFFFFF, CTRL5: 0x607007FE,
        CTRL6: 0xF6000000, TXBR: 0x00FFFFFF, RXBR: 0x00001FFF,
        CF: 0xCFFFFFFF, PRE: 0xFFFF1FFF, SYNC0: 0xFFFFFFFF,
        SYNC1: 0xFFFFFFFF, SHAPING10: 0xFFFFFFFF, SHAPING11: 0xFFFFFFFF,
        DCCOMP: 0x000001FF, DCCOMPFILTINIT: 0x7FFFFFFF,
        DSATHD0: 0xFFFFFFFF, DSATHD1: 0x7FFFFFFF, AUTOCG: 0x0000FFFF,
        DSATHD2: 0x7FFFFEFF, DIRECTMODE: 0x00001F0F, LRFRC: 0x000001FF,
        DSATHD3: 0xFFFFFFFF, DSATHD4: 0x07FFFFFF, BLEIQDSA: 0xFFFFFFFF,
        SYNCPROPERTIES: 0x000000FF, DIGIGAINCTRL: 0x000001FF,
        PADEBUG: 0x00001E00, ETSCTRL: 0x3FFFF7FF, ANTSWCTRL: 0x01FFFFFF,
        ANTSWSTART: 0x0003FFFF, ANTSWEND: 0x0003FFFF,
        TRECPMPATT: 0xFFFFFFFF, TRECPMDET: 0xBFFFC3FF,
        CFGANTPATT: 0x3FFFFFFF, ETSTIM: 0x0003FFFF,
        ANTSWCTRL1: 0x00FFFFFF, ANTDIVCTRL: 0x000001FF,
        BLEIQDSAEXT2: 0x000007FF,
    }

    def write(self, off, val):
        if off in self.MASKS:
            self.regs[off] = val & self.MASKS[off]
        else:
            super().write(off, val)


class PCF8575:
    """16-bit quasi-bidirectional I/O expander.  Two data bytes per transfer, P0-P7 first."""
    def __init__(self, addr7=0x20):
        self.addr = addr7
        self.latch = 0xFFFF
        self.ext_low = 0                  # pins externally pulled low (inputs held low)
        self.wcount = 0
        self.rcount = 0
        self.on_change = None
        self.history = []

    def start(self, rw):
        self.wcount = self.rcount = 0

    def write(self, b):
        sh = 8 * (self.wcount & 1)
        self.latch = (self.latch & ~(0xFF << sh)) | (b << sh)
        self.wcount += 1
        if self.wcount & 1 == 0:
            self.history.append(self.latch)
            if self.on_change:
                self.on_change(self.latch)
        return True

    def read(self):
        v = self.latch & ~self.ext_low & 0xFFFF
        b = (v >> (8 * (self.rcount & 1))) & 0xFF
        self.rcount += 1
        return b


class I2C(IrqDev):
    """Byte-level I2C master (EFM32 'I2C' IP).  One TXDATA register in front of the
    shift register; ~50 us per byte."""
    BYTE_NS = 50_000
    IF_OFF, IEN_OFF = 0x03C, 0x040
    START, RSTART, ADDR, TXC, TXBL, RXDATAV, ACK, NACK, MSTOP = (1 << i for i in range(9))

    def __init__(self, name, irq_name, kind="I2C"):
        super().__init__(name, kind)
        self.irq_name = irq_name
        self.slaves = {}
        self.reset_state()

    def reset_state(self):
        self.busy = self.master = self.transmitter = self.nacked = False
        self.start_pending = False
        self.txq = []                    # bytes waiting to be shifted out
        self.shifting = False
        self.rx = []
        self.slave = None
        self.addr_phase = False
        self.stop_pending = False
        self.txc = True
        self.pstop = False
        self.regs[self.IF_OFF] = 0
        self.regs[0x040] = self.regs.get(0x040, 0)

    def txbl(self):
        return len(self.txq) < 1 and self.regs.get(0x004, 0) & 1 != 0 or len(self.txq) < 1

    # -- registers ----------------------------------------------------
    def r_010(self):                                # STATE
        return (self.busy << 0 | self.master << 1 | self.transmitter << 2 | self.nacked << 3)

    def r_014(self):                                # STATUS
        return (self.start_pending << 0 | self.pstop << 1 | (self.txc and not self.shifting) << 6
                | (len(self.txq) < 1) << 7 | bool(self.rx) << 8)

    def r_024(self):                                # RXDATA
        v = self.rx.pop(0) if self.rx else 0
        if not self.rx:
            self.regs[self.IF_OFF] &= ~self.RXDATAV
            self.irq_update()
        return v

    r_02c = r_024                                   # RXDATAP (peek not modelled separately)

    def w_004(self, v):                             # EN
        self.regs[0x004] = v
        if not v & 1:
            self.reset_state()
        else:
            self.raise_if(self.TXBL)

    def w_008(self, v):
        self.regs[0x008] = v
        if v & 1:
            self.reset_state()

    def w_034(self, v):                             # TXDATA
        self.txq.append(v & 0xFF)
        self.txc = False
        self.regs[self.IF_OFF] &= ~(self.TXBL | self.TXC)
        self.irq_update()
        self.kick()

    def w_00c(self, v):                             # CMD
        if v & 1:                                   # START
            self.start_pending = True
            self.busy = self.master = True
            self.raise_if(self.START if not self.slave else self.RSTART)
            self.addr_phase = True
            self.kick()
        if v & 4 or v & 8:                          # ACK / NACK of a received byte
            if v & 8:
                self.nacked = True
            else:
                self.m.after(self.BYTE_NS, self.rx_byte)
        if v & 2:                                   # STOP
            self.stop_pending = True
            self.kick()
        if v & 0x20:                                # ABORT
            self.reset_state()
        if v & 0x40:
            self.txq.clear()

    # -- engine -------------------------------------------------------
    def kick(self):
        if self.shifting:
            return
        if self.txq and (self.start_pending or self.slave or not self.addr_phase):
            self.shifting = True
            self.regs[self.IF_OFF] |= self.TXBL if len(self.txq) <= 1 else 0
            self.m.after(self.BYTE_NS, self.byte_done)
            self.raise_if(self.TXBL)
        elif self.stop_pending and not self.txq:
            self.stop_pending = False
            self.m.after(self.BYTE_NS // 5, self.stopped)

    def byte_done(self):
        b = self.txq.pop(0)
        self.shifting = False
        if self.addr_phase:
            self.addr_phase = False
            self.start_pending = False
            a7, rw = b >> 1, b & 1
            self.slave = self.slaves.get(a7)
            if self.slave is None:
                self.nacked, ok = True, False
            else:
                self.slave.start(rw)
                self.transmitter = not rw
                ok = True
            self.raise_if(self.ACK if ok else self.NACK)
            if ok and rw:                            # first byte clocked in right away
                self.m.after(self.BYTE_NS, self.rx_byte)
        else:
            ok = bool(self.slave and self.slave.write(b))
            self.nacked = not ok
            self.raise_if(self.ACK if ok else self.NACK)
        if not self.txq:
            self.txc = True
            self.raise_if(self.TXC | self.TXBL)
        else:
            self.raise_if(self.TXBL)
        self.kick()

    def rx_byte(self):
        if self.slave:
            self.rx.append(self.slave.read())
            self.raise_if(self.RXDATAV)

    def stopped(self):
        self.busy = self.master = self.transmitter = False
        self.slave = None
        self.pstop = True
        self.raise_if(self.MSTOP)
        self.m.after(1000, lambda: setattr(self, "pstop", False))


class GPIO(Dev):
    """Ports A-D.  Tracks the level each pin is driven to (DOUT, or a routed peripheral
    output) and logs every change in m.pinlog as (t_ns, 'PC7', level).  Pins in input
    modes read m.ext_pin (set by the scenario), default: pulled by DOUT for input-pull
    modes, else 0."""
    MODE_DISABLED, MODE_INPUT, MODE_INPUTPULL, MODE_PUSHPULL, MODE_WIREDOR = 0, 1, 2, 4, 5
    NAMES = "ABCD"

    def __init__(self, name):
        super().__init__(name)
        self.drives = {}              # (port, pin) -> level driven by a peripheral
        self.last = {}                # (port, pin) -> last logged effective level
        self.ext = {}                 # (port, pin) -> externally forced input level

    def mode(self, port, pin):
        o = 0x30 * port + (0x4 if pin < 8 else 0xC)
        return (self.regs.get(o, 0) >> (4 * (pin & 7))) & 0xF

    def dout(self, port, pin):
        return (self.regs.get(0x30 * port + 0x10, 0) >> pin) & 1

    def is_output(self, port, pin):
        return self.mode(port, pin) in (4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15) and self.mode(port, pin) != 0

    def level(self, port, pin):
        """Electrical level seen on the pin (outputs: driven value; inputs: external/pull)."""
        mode = self.mode(port, pin)
        if mode >= 4:
            lv = self.drives.get((port, pin), self.dout(port, pin))
            if mode in (4, 5, 6, 7) or True:
                return lv
        if (port, pin) in self.ext:
            return self.ext[(port, pin)]
        if mode == self.MODE_INPUTPULL or mode == 3:
            return self.dout(port, pin)
        return 0

    def drive(self, port, pin, level):
        self.drives[(port, pin)] = level
        self.note(port, pin)

    def release(self, port, pin):
        self.drives.pop((port, pin), None)
        self.note(port, pin)

    def note(self, port, pin):
        if self.mode(port, pin) < 4:
            lv = None                              # not driven
        else:
            lv = self.level(port, pin)
        if self.last.get((port, pin)) != lv:
            self.last[(port, pin)] = lv
            self.m.pinlog.append((self.m.now, "P%s%d" % (self.NAMES[port], pin), lv))
            for cb in self.m.pin_watchers.get((port, pin), ()):
                cb(lv)

    def read(self, off, size=4):
        for port in range(4):
            if off == 0x30 * port + 0x14:                  # DIN
                v = 0
                for pin in range(16):
                    v |= self.level(port, pin) << pin
                return v
        return super().read(off, size)

    def write(self, off, val):
        super().write(off, val)
        for port in range(4):
            if 0x30 * port <= off < 0x30 * port + 0x30:
                for pin in range(16):
                    self.note(port, pin)

    def route(self, v):
        """CCxROUTE-style value -> (port, pin)."""
        return (v >> 16) & 3, v & 0xF


class Timer(IrqDev):
    """Up-counting TIMER (EFR32 Series 2 'TIMER' IP) with output compare / PWM outputs.
    CNT is derived from virtual time, so no events are needed unless an interrupt or a
    routed pin must follow the waveform."""
    IF_OFF, IEN_OFF = 0x014, 0x018
    OF, UF = 1, 2

    def __init__(self, name, irq_name, idx):
        super().__init__(name, "TIMER")
        self.irq_name, self.idx = irq_name, idx
        self.running = False
        self.cnt0 = 0
        self.t0 = 0
        self.out = [0, 0, 0]
        self.gen = 0
        self.pending = []

    # time base ---------------------------------------------------------
    def tick_hz(self):
        presc = ((self.regs.get(0x004, 0) >> 18) & 0x3FF) + 1
        return self.m.cmu.em01grpa_hz() / presc

    def top(self):
        return self.regs.get(0x01C, 0xFFFF)

    def count(self, now=None):
        if not self.running:
            return self.cnt0
        now = self.m.now if now is None else now
        ticks = int((now - self.t0) * self.tick_hz() / 1e9)
        return (self.cnt0 + ticks) % (self.top() + 1)

    def r_024(self):
        return self.count()

    def w_024(self, v):
        self.cnt0, self.t0 = v & 0xFFFF_FFFF, self.m.now
        self.reschedule()

    def r_010(self):                                # STATUS
        return int(self.running)

    # commands ----------------------------------------------------------
    def w_00c(self, v):
        if v & 1 and not self.running and self.regs.get(0x030, 0) & 1:
            self.running, self.t0 = True, self.m.now
        if v & 2 and self.running:
            self.cnt0 = self.count()
            self.running = False
        self.reschedule()

    def w_030(self, v):                             # EN
        self.regs[0x030] = v
        if not v & 1:
            self.running, self.cnt0 = False, 0
        self.reschedule()

    def write(self, off, val):
        super().write(off, val)
        if off in (0x01C, 0x020, 0x004) or 0x60 <= off < 0xC0 or off in (self.IEN_OFF,):
            self.reschedule()

    # channels ----------------------------------------------------------
    def cc(self, n, reg):
        return self.regs.get(0x60 + 0x20 * n + reg, 0)

    def cc_mode(self, n):
        return self.cc(n, 0) & 3

    def apply_action(self, n, act):
        if act == 1:
            self.out[n] ^= 1
        elif act == 2:
            self.out[n] = 0
        elif act == 3:
            self.out[n] = 1
        self.push_pin(n)

    def push_pin(self, n):
        g = self.m.gpio
        base = 0x580 + 0x20 * self.idx
        if not g.regs.get(base - 4, 0) & (1 << n):          # ROUTEEN.CCnPEN
            return
        port, pin = g.route(g.regs.get(base + 4 * n, 0))
        inv = (self.cc(n, 4) >> 2) & 1
        g.drive(port, pin, self.out[n] ^ inv)

    def edge_times(self):
        """(time_ns, kind, channel) of the next waveform events, from now."""
        hz = self.tick_hz()
        if not self.running or hz <= 0:
            return []
        top = self.top()
        cnt = self.count()
        per = (top + 1) / hz * 1e9
        ev = []
        t_of = self.m.now + (top + 1 - cnt) / hz * 1e9
        ev.append((t_of, "of", -1))
        for n in range(3):
            if self.cc_mode(n) in (2, 3):
                cmp = self.cc(n, 8)
                if cmp <= top:
                    dt = (cmp - cnt) if cmp > cnt else (top + 1 - cnt + cmp)
                    ev.append((self.m.now + dt / hz * 1e9, "cc", n))
        return sorted(ev)

    def reschedule(self):
        self.gen += 1
        gen = self.gen
        if not self.running:
            return
        need = bool(self.regs.get(self.IEN_OFF, 0)) or any(
            self.cc_mode(n) in (2, 3) and self.m.gpio.regs.get(0x580 + 0x20 * self.idx - 4, 0) >> n & 1
            for n in range(3))
        if not need:
            return
        ev = self.edge_times()
        if ev:
            t = ev[0][0]
            self.pending = [e for e in ev if e[0] <= t + 1]
            self.m.at(int(t) + 1, lambda: self.fire(gen))

    def fire(self, gen):
        if gen != self.gen or not self.running:
            return
        now = self.m.now
        pend, self.pending = self.pending, []
        # re-anchor so the counter stays exact across the wrap
        self.cnt0, self.t0 = self.count(), now
        for t, kind, n in pend:
            if kind == "of":
                self.raise_if(self.OF)
                for c in range(3):
                    if self.cc_mode(c) in (2, 3):
                        self.apply_action(c, (self.cc(c, 4) >> 10) & 3)
            else:
                self.raise_if(1 << (4 + 2 * n))                    # CCn
                self.apply_action(n, (self.cc(n, 4) >> 8) & 3)
        self.reschedule()


class MSC(IrqDev):
    """Flash controller.  Words written to WDATA are programmed (AND-ed, as flash can
    only clear bits) at ADDRB, which auto-increments; ERASEPAGE erases the 8 KiB page
    at ADDRB.  The server keeps a shadow of the flash (m.flash) and mirrors every change
    into guest memory."""
    IF_OFF, IEN_OFF = 0x020, 0x024
    PAGE = 0x2000
    FLASH_SIZE = 0x80000

    def __init__(self, name):
        super().__init__(name)
        self.addr = 0
        self.busy_until = 0
        self.locked = True
        self.invaddr = False

    def status(self):
        busy = self.m.now < self.busy_until
        wren = self.regs.get(0x00C, 0) & 1
        return (busy << 0 | self.locked << 1 | self.invaddr << 2 | (not busy and wren) << 3
                | self.locked << 16 | 1 << 24 | (not busy) << 27)

    r_01c = status

    def r_03c(self):
        return int(self.locked)

    def w_03c(self, v):
        self.locked = v != 0x1B71

    def w_014(self, v):
        self.regs[0x014] = v
        self.addr = v
        self.invaddr = v >= self.FLASH_SIZE

    def program(self, addr, word):
        if addr + 4 > self.FLASH_SIZE:
            self.invaddr = True
            return
        old = int.from_bytes(self.m.flash[addr:addr + 4], "little")
        new = old & word
        data = new.to_bytes(4, "little")
        self.m.flash[addr:addr + 4] = data
        self.m.guest_write(addr, data)

    def w_018(self, v):                                # WDATA
        if self.locked or not self.regs.get(0x00C, 0) & 1:
            return
        self.program(self.addr, v)
        self.addr += 4
        self.busy_until = self.m.now + 20_000
        self.raise_if(2)

    def w_010(self, v):                                # WRITECMD
        if v & 2 and not self.locked and self.regs.get(0x00C, 0) & 1:      # ERASEPAGE
            base = self.addr & ~(self.PAGE - 1)
            if base + self.PAGE > self.FLASH_SIZE:
                self.invaddr = True
            else:
                self.m.flash[base:base + self.PAGE] = b"\xff" * self.PAGE
                self.m.guest_write(base, b"\xff" * self.PAGE)
                self.busy_until = self.m.now + 2_000_000
                self.m.after(2_000_000, lambda: self.raise_if(1))
        if v & 4:                                      # WRITEEND
            self.busy_until = self.m.now + 5_000
            self.raise_if(2)
        if v & 0x1000:                                 # CLEARWDATA
            pass


class EMU(Dev):
    def r_094(self):                          # RSTCAUSE: why we are running
        return self.regs.get(0x094, 1)

    def w_070(self, v):                       # CMD: RSTCAUSECLR
        if v & (1 << 17):
            self.regs[0x094] = 0


class CMU(Dev):
    def sysclk_sel(self):
        return self.regs.get(0x070, 0) & 7

    def clken(self, reg, bit):
        return bool(self.regs.get(reg, 0) >> bit & 1)

    def lf_selected(self, sel):
        """True if an enabled low-frequency consumer selects oscillator `sel`
        (1 LFRCO, 2 LFXO, 3 ULFRCO)."""
        r = self.regs.get
        eu = self.clken(0x064, 24) and (r(0x220, 0) & 3) == 2        # EUART0 on EM23GRPACLK
        uses = (
            ((r(0x140, 0) & 3) == sel and (self.clken(0x064, 12) or eu)),   # EM23GRPA: LETIMER0/EUART0
            ((r(0x160, 0) & 3) == sel and self.clken(0x064, 29)),           # EM4GRPA: BURTC
            ((r(0x200, 0) & 7) == sel and self.clken(0x064, 13)),           # WDOG0
            ((r(0x240, 0) & 3) == sel and self.clken(0x064, 30)),           # RTCC
            ((r(0x248, 0) & 3) == sel and self.clken(0x068, 10)),           # PRORTC
        )
        return any(uses)

    HFXO_HZ = 38_400_000
    FSRCO_HZ = 20_000_000

    def hfrco_hz(self):
        # DPLL running -> (N+1)/(M+1) * reference; otherwise HFRCO at its reset calibration (~19 MHz)
        d = self.m.dpll
        if d.regs.get(0x004, 0) & 1:
            cfg1 = d.regs.get(0x00C, 0)
            n, mm = (cfg1 >> 8) & 0xFFF, cfg1 & 0xFF
            return self.HFXO_HZ * (n + 1) // (mm + 1)
        return 19_000_000

    def src_hz(self, code, table):
        name = table.get(code)
        return {"FSRCO": self.FSRCO_HZ, "HFXO": self.HFXO_HZ, "HFRCODPLL": self.hfrco_hz()}.get(name, 0)

    def sysclk_hz(self):
        return self.src_hz(self.regs.get(0x070, 1) & 7, {1: "FSRCO", 2: "HFRCODPLL", 3: "HFXO"})

    def em01grpa_hz(self):
        return self.src_hz(self.regs.get(0x120, 1) & 7, {1: "HFRCODPLL", 2: "HFXO", 3: "FSRCO", 4: "HFRCODPLL", 5: "HFXO"})

    def r_008(self):                          # STATUS: clock sources that are enabled+ready
        return 0xFFFFFFFF


class WDOG(IrqDev):
    """Series-2 watchdog.  Clocked at 1 kHz (ULFRCO); period = 2^(3+PERSEL)+1 cycles
    (em_wdog.h wdogPeriod_*).  A CMD.CLEAR write restarts the countdown; expiry requests a
    system reset with RSTCAUSE.WDOG0 (EMU_RSTCAUSE_WDOG0, bit 3).  EM2RUN/EM3RUN are not
    modelled (the model has no sleep states)."""
    IF_OFF, IEN_OFF = 0x18, 0x1C
    irq_name = "WDOG0"
    CAUSE = 1 << 3

    def __init__(self, name):
        super().__init__(name)
        self.gen = 0
        self.feeds = 0
        self.deadline = None

    def period_ns(self):
        per = (self.regs.get(0x08, 0) >> 16) & 0xF
        return ((1 << (3 + per)) + 1) * 1_000_000

    def arm(self):
        self.gen += 1
        gen = self.gen
        if not self.regs.get(0x04, 0) & 1:
            self.deadline = None
            return
        self.deadline = self.m.now + self.period_ns()
        def fire():
            if gen != self.gen:
                return
            self.m.wdog_expired = self.m.now
            import sys
            print('WDOG expired t=%d lastpc=%08x recent=%s' % (self.m.now, self.m.pc, ['%08x:%s' % x for x in list(self.m.recent)]), file=sys.stderr)
            if not self.regs.get(0x08, 0) & 0x100:        # WDOGRSTDIS clear -> reset
                self.m.reset_cause_next = self.CAUSE
                self.m.request_reset()
        self.m.at(self.deadline, fire)

    def w_004(self, v):
        self.regs[0x04] = v & 1
        self.arm()

    def w_00c(self, v):
        if v & 1:
            self.feeds += 1
            self.arm()

    def r_024(self):
        return 0


class RTCC(IrqDev):
    """Real-time counter + 3 compare channels (efr32bg22_rtcc.h).  Counts the 32768 Hz LF clock
    divided by CFG.CNTPRESC once CMD.START has run.  CC[n] in output-compare mode (CTRL.MODE=2)
    sets IF.CCn (bit 4+2n) when CNT reaches OCVALUE; wrapping raises IF.OF (bit 0).  IRQ: RTCC."""
    IF_OFF, IEN_OFF = 0x14, 0x18
    irq_name = "RTCC"

    def __init__(self, name):
        super().__init__(name)
        self.t0 = None
        self.base = 0
        self.gen = 0

    def hz(self):
        return 32768 >> ((self.regs.get(0x08, 0) >> 4) & 0xF)

    def cnt(self):
        if self.t0 is None:
            return self.base
        return (self.base + ((self.m.now - self.t0) * self.hz()) // 1_000_000_000) & 0xFFFFFFFF

    def reschedule(self):
        self.gen += 1
        gen = self.gen
        if self.t0 is None or not self.regs.get(0x04, 0) & 1:
            return
        cur = self.cnt()
        best = None
        for n in range(3):
            if (self.regs.get(0x30 + 12 * n, 0) & 3) == 2:             # CC[n].CTRL.MODE = output compare
                d = (self.regs.get(0x34 + 12 * n, 0) - cur) & 0xFFFFFFFF
                if d == 0:
                    d = 0x100000000                                     # already matched: next lap
                if best is None or d < best[0]:
                    best = (d, n)
        d_ovf = 0x100000000 - cur
        if best is None or d_ovf < best[0]:
            best = (d_ovf, -1)
        d, n = best
        when = self.m.now + (d * 1_000_000_000 + self.hz() - 1) // self.hz() + 1
        def fire():
            if gen != self.gen:
                return
            if n < 0:
                self.raise_if(1)
            else:
                self.raise_if(1 << (4 + 2 * n))
            self.reschedule()
        self.m.at(when, fire)

    def write(self, off, val):
        super().write(off, val)
        if off in (0x04, 0x08) or 0x30 <= off < 0x54:
            self.reschedule()

    def w_00c(self, v):
        if v & 1 and self.t0 is None and self.regs.get(0x04, 0) & 1:
            self.t0 = self.m.now
        if v & 2 and self.t0 is not None:
            self.base, self.t0 = self.cnt(), None
        self.reschedule()

    def r_020(self):
        return self.cnt()

    def r_024(self):
        return self.cnt()

    def r_010(self):                                   # STATUS: RUNNING
        return 1 if self.t0 is not None else 0

    def r_028(self):
        return 0


class USART(IrqDev):
    """Synchronous (SPI) master.  Every transmitted frame is appended to m.spilog as
    (t_ns, name, tx, rx, {cs-ish snapshot not taken; correlate with m.pinlog}).  The slave is
    an optional callable m.spi_slaves[name](tx) -> rx (default: MISO high, 0xFFFF)."""
    IF_OFF, IEN_OFF = 0x48, 0x4C

    def __init__(self, name, irq):
        super().__init__(name)
        self.irq_name = irq
        self.rx = []
        self.txen = self.rxen = self.master = False
        if not hasattr(self, "spilog"):
            pass

    def r_018(self):                                   # STATUS
        v = (1 << 5) | (1 << 6) | (1 << 13)            # TXC, TXBL, TXIDLE
        if self.rxen:
            v |= 1
        if self.txen:
            v |= 2
        if self.master:
            v |= 4
        if self.rx:
            v |= 1 << 7
        return v

    def r_048(self):
        return (self.regs.get(0x48, 0) | 3 | (4 if self.rx else 0))

    def w_048(self, v):                                # IF is write-1-to-set via the SET alias
        self.regs[0x48] = v
    def r_024(self):                                   # RXDATA
        return self.rx.pop(0) & 0xFF if self.rx else 0
    def r_020(self):                                   # RXDATAX
        return self.rx.pop(0) & 0x1FF if self.rx else 0
    def r_02c(self):
        lo = self.rx.pop(0) & 0xFF if self.rx else 0
        hi = self.rx.pop(0) & 0xFF if self.rx else 0
        return lo | (hi << 8)

    def w_014(self, v):                                # CMD
        for bit, attr, val in ((0, "rxen", True), (1, "rxen", False), (2, "txen", True),
                               (3, "txen", False), (4, "master", True), (5, "master", False)):
            if v & (1 << bit):
                setattr(self, attr, val)
        if v & (1 << 10):                              # CLEARTX
            pass
        if v & (1 << 11):                              # CLEARRX
            self.rx.clear()

    def xfer(self, tx, bits):
        slave = self.m.spi_slaves.get(self.name)
        rx = slave(tx) if slave else (1 << bits) - 1
        self.m.spilog.append((self.m.now, self.name, tx, rx))
        if self.rxen:
            self.rx.append(rx)
            self.rx = self.rx[-3:]

    def bits(self):
        return (self.regs.get(0x0C, 0) & 0xF) + 3 if (self.regs.get(0x0C, 0) & 0xF) else 8

    def w_03c(self, v):                                # TXDATA
        if self.txen:
            self.xfer(v & 0xFF, 8)
    def w_038(self, v):                                # TXDATAX
        if self.txen:
            self.xfer(v & 0x1FF, 8)
    def w_044(self, v):                                # TXDOUBLE
        if self.txen:
            self.xfer(v & 0xFF, 8)
            self.xfer((v >> 8) & 0xFF, 8)


class IADC(IrqDev):
    """Single-conversion IADC.  CMD.SINGLESTART (or any TRIGGER) pushes one result into
    the single FIFO ~10 us later and raises IF.SINGLEDONE.  The sampled value comes from
    m.adc_inputs[(portpos, pinpos)] (int, or callable(m)->int), default 0."""
    IF_OFF, IEN_OFF = 0x24, 0x28
    irq_name = "IADC"

    def __init__(self, name):
        super().__init__(name)
        self.fifo = []
        self.last = 0
        self.busy = False

    def key(self):
        s = self.regs.get(0x98, 0)
        return ((s >> 12) & 0xF, (s >> 8) & 0xF)

    def sample(self):
        v = self.m.adc_inputs.get(self.key(), 0)
        v = v(self.m) if callable(v) else v
        self.m.adc_log.append((self.m.now, self.key(), v))
        return v & 0xFFFFFFFF

    def start(self):
        if self.busy or not self.regs.get(0x04, 0) & 1:
            return
        self.busy = True
        def done():
            self.busy = False
            v = self.sample()
            self.last = v
            self.fifo.append(v)
            self.fifo = self.fifo[-8:]
            self.raise_if((1 << 9) | 1)
        self.m.after(10_000, done)

    def w_00c(self, v):
        if v & 1:
            self.start()
        if v & (1 << 24):
            self.fifo.clear()

    def w_02c(self, v):
        self.start()

    def r_014(self):
        v = (1 << 30) if self.regs.get(0x04, 0) & 1 else 0
        if self.busy:
            v |= 1 << 6
        if self.fifo:
            v |= 1 << 8
        return v

    def r_074(self):
        return self.fifo.pop(0) if self.fifo else 0

    def r_078(self):
        return len(self.fifo)

    def r_07c(self):
        return self.last

def build(m, reset=False, cause=1):
    # USERDATA/CHIPCONFIG read as erased flash; DEVINFO: only what is known from the
    # datasheet (sizes) plus a made-up, obviously fake EUI-64. Calibration words are zero.
    import struct
    m.guest_write(0x0FE00000, b"\xff" * 0x8000)
    dev = bytearray(0x2000)
    struct.pack_into("<I", dev, 0x0C, 512 | (32 << 16))           # MSIZE: flash KiB, SRAM KiB
    struct.pack_into("<II", dev, 0x48, 0x0000BEEF, 0x00000A00)    # EUI64L/H (fake)
    m.guest_write(0x0FE08000, bytes(dev))
    m.reset_cause = cause
    m.devs = []
    m.fallback = Fallback()
    m.fallback.m = m
    emu = m.add(EMU("EMU"), 0x40004000)
    emu.regs[0x094] = cause
    m.add(MSC("MSC"), 0x40030000)
    m.cmu = m.add(CMU("CMU"), 0x40008000)
    m.hfxo_requested = lambda: m.cmu.sysclk_sel() == 3
    m.add(HFXO("HFXO0"), 0x4000C000)
    m.add(Osc("HFRCO0", 0x00C, always=True), 0x40010000)
    m.add(Osc("FSRCO", 0x004, always=True), 0x40018000)
    m.dpll = m.add(Osc("DPLL0", 0x018, ctrl_off=0x004), 0x4001C000)
    m.add(Osc("LFXO", 0x010, ctrl_off=0x004, request=lambda m: m.cmu.lf_selected(2)), 0x40020000)
    m.add(Osc("LFRCO", 0x008, ctrl_off=0x004, request=lambda m: m.cmu.lf_selected(1)), 0x40024000)
    m.add(Osc("ULFRCO", 0x008, always=True), 0x40028000)
    m.add(DCDC("DCDC"), 0x40094000)
    m.gpio = m.add(GPIO("GPIO"), 0x4003C000)
    for n, (base, irq) in enumerate(((0x40048000, "TIMER0"), (0x4004C000, "TIMER1"), (0x40050000, "TIMER2"),
                                     (0x40054000, "TIMER3"), (0x40058000, "TIMER4"))):
        m.add(Timer("TIMER%d" % n, irq, n), base)
    m.i2c0 = m.add(I2C("I2C0", "I2C0"), 0x4A010000)
    m.expander = PCF8575()
    m.i2c0.slaves[0x20] = m.expander
    m.i2c1 = m.add(I2C("I2C1", "I2C1"), 0x40068000)
    m.add(RadioAES("RADIOAES", "AES"), 0x44000000)
    m.add(Dev("SMU"), 0x44008000)
    m.add(Dev("SYSCFG"), 0x4007C000)
    m.add(RTCC("RTCC"), 0x48000000)
    m.add(WDOG("WDOG0"), 0x4A018000)
    m.add(IADC("IADC0"), 0x4A004000)
    m.add(USART("USART0", "USART0_RX"), 0x4005C000)
    m.add(USART("USART1", "USART1_RX"), 0x40060000)
    m.rac = m.add(RAC("RAC"), 0xA8020000)
    m.synth = m.add(Synth("SYNTH"), 0xA8018000)
    m.rfcrc = m.add(RFCRC("RFCRC"), 0xA8010000)
    m.frc = m.add(FRC("FRC"), 0xA8004000)
    m.modem = m.add(Modem("MODEM"), 0xA8014000)
    m.bufc = m.add(BUFC("BUFC"), 0xAA000000)
    trng = m.add(TRNG("TRNG", ""), 0x4C021000, 0x1000)
    m.add(TRNGFifo("TRNGFIFO", trng), 0x4C024000, 0x1000)
