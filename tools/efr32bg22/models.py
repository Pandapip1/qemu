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
            self.m.at(int(t) + 1, lambda: self.fire(gen))

    def fire(self, gen):
        if gen != self.gen or not self.running:
            return
        now = self.m.now
        # re-anchor at the event so cnt wrapping stays exact
        for t, kind, n in self.edge_times():
            if t > now + 1:
                break
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


def build(m, reset=False, cause=1):
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
    trng = m.add(TRNG("TRNG", ""), 0x4C021000, 0x1000)
    m.add(TRNGFifo("TRNGFIFO", trng), 0x4C024000, 0x1000)
