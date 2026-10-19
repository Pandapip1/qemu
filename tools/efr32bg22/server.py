#!/usr/bin/env python3
"""Peripheral server for the efr32bg22 QEMU machine (see hw/arm/efr32bg22.c).

Peripheral models live in models.py; this file is the transport, the address
decoder, the Series-2 SET/CLR/TGL alias handling and the event queue.
"""
import argparse, heapq, os, socket, struct, sys, collections

REQ = struct.Struct("<BBHIIIQ")
REP = struct.Struct("<IIQQ")
RESET_REQ = 0x80000000


class Machine:
    def __init__(self, trace=False):
        self.devs = []            # (base, size, dev)
        self.irq = 0              # NVIC line levels, bit n = IRQ n
        self.now = 0
        self.events = []          # heap of (when_ns, seq, callback)
        self.seq = 0
        self.trace = trace
        self.seen = collections.Counter()
        self.unknown = collections.Counter()
        self.pinlog = []
        self.mem_writes = []      # (addr, bytes) to apply to guest memory with the next reply
        self.reset_requested = False
        self.resets = 0
        self.pin_watchers = {}
        self.spilog, self.spi_slaves = [], {}
        self.adc_inputs, self.adc_log = {}, []
        self.reset_cause_next = 0x40
        self.wdog_expired = None
        self.stuck = (None, 0)
        self.stuck_limit = int(os.environ.get("STUCK", 20000))
        self.exit_on_stuck = True

    # -- wiring ----------------------------------------------------------
    def add(self, dev, base, size=0x4000):
        dev.m = self
        self.devs.append((base, size, dev))
        return dev

    def set_irq(self, n, level):
        if level:
            self.irq |= 1 << n
        else:
            self.irq &= ~(1 << n)

    def at(self, when_ns, cb):
        self.seq += 1
        heapq.heappush(self.events, (when_ns, self.seq, cb))

    def after(self, delay_ns, cb):
        self.at(self.now + delay_ns, cb)

    def next_ns(self):
        return self.events[0][0] if self.events else 0

    def run_events(self):
        while self.events and self.events[0][0] <= self.now:
            _, _, cb = heapq.heappop(self.events)
            cb()

    def reset(self):
        """System reset (also the very first reset at power-on)."""
        import models
        first = not getattr(self, "built", False)
        self.irq = 0
        self.events.clear()
        self.stuck = (None, 0)
        keep_flash, keep_pins = self.flash, self.pinlog
        models.build(self, reset=not first, cause=1 if first else self.reset_cause_next)
        self.reset_cause_next = 0x40
        self.built = True
        self.resets += 1

    def guest_write(self, addr, data):
        self.mem_writes.append((addr, bytes(data)))

    def request_reset(self):
        self.reset_requested = True

    # -- bus -------------------------------------------------------------
    def find(self, addr):
        a = (addr & ~0x10000000) if 0x50000000 <= addr < 0x60000000 else addr
        for base, size, dev in self.devs:
            if base <= a < base + size:
                return dev, a - base
        return None, a

    def access(self, op, addr, val, size):
        dev, off = self.find(addr)
        name = dev.name if dev else "?%08x" % (addr & ~0x3FFF)
        alias = (off >> 12) & 3
        off &= 0xFFF
        if dev is None:
            self.unknown[name] += 1
            dev = self.fallback
        if op == 0:
            r = dev.read(off, size)
            key = (addr, r)
            n = self.stuck[1] + 1 if self.stuck[0] == key else 1
            self.stuck = (key, n)
            if n == self.stuck_limit:
                print(f"STUCK polling {name}+{off:03x} (addr {addr:#010x}) = {r:#010x} at {self.now} ns",
                      file=sys.stderr, flush=True)
                if self.exit_on_stuck:
                    os._exit(3)
            if self.trace:
                print(f"{self.now:>12} R {name}+{off:03x} = {r:08x}", file=sys.stderr)
            return r
        if alias:
            cur = dev.read(off, 4)
            val = {1: cur | val, 2: cur & ~val, 3: cur ^ val}[alias]
        self.stuck = (None, 0)
        if size < 4:
            sh = (addr & 3) * 8
            msk = ((1 << (size * 8)) - 1) << sh
            val = (dev.read(off & ~3, 4) & ~msk) | ((val << sh) & msk)
        if self.trace:
            print(f"{self.now:>12} W {name}+{off:03x} <- {val:08x}", file=sys.stderr)
        dev.write(off & ~3, val)
        return 0


def serve(m, path):
    if os.path.exists(path):
        os.unlink(path)
    srv = socket.socket(socket.AF_UNIX)
    srv.bind(path)
    srv.listen(1)
    print("listening", path, file=sys.stderr, flush=True)
    c, _ = srv.accept()
    c.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1) if c.family == socket.AF_INET else None
    buf = b""
    while True:
        while len(buf) < REQ.size:
            d = c.recv(65536)
            if not d:
                return
            buf += d
        while len(buf) >= REQ.size:
            op, size, _, addr, val, _, now = REQ.unpack(buf[:REQ.size])
            buf = buf[REQ.size:]
            m.now = now
            m.run_events()
            if op == 3:
                m.reset()
                r = 0
            else:
                r = m.access(op, addr, val, size) if op < 2 else 0
            m.run_events()
            out = b"".join(struct.pack("<II", a, len(d)) + d + b"\0" * (-len(d) % 4) for a, d in m.mem_writes)
            flags = len(m.mem_writes) | (RESET_REQ if m.reset_requested else 0)
            m.mem_writes, m.reset_requested = [], False
            c.sendall(REP.pack(r & 0xFFFFFFFF, flags, m.irq, m.next_ns()) + out)


def main():
    import models
    ap = argparse.ArgumentParser()
    ap.add_argument("--sock", default="/tmp/claude/efr32.sock")
    ap.add_argument("--trace", action="store_true")
    ap.add_argument("--no-stuck-exit", action="store_true")
    ap.add_argument("--flash", help="flash image given to QEMU (-M efr32bg22,flash=...); the server keeps a shadow")
    a = ap.parse_args()
    m = Machine(a.trace)
    m.exit_on_stuck = not a.no_stuck_exit
    m.flash = bytearray(open(a.flash, 'rb').read()) if a.flash else bytearray(b'\xff' * 0x80000)
    import signal
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    try:
        serve(m, a.sock)
    finally:
        for t_ns, pin, lv in m.pinlog:
            print("PIN %12d %s %s" % (t_ns, pin, lv), file=sys.stderr)
        for t_ns, dev, tx, rx in m.spilog:
            print("SPI %12d %s tx=%02x rx=%02x" % (t_ns, dev, tx, rx), file=sys.stderr)
        for t_ns, key, v in m.adc_log:
            print("ADC %12d %s %d" % (t_ns, key, v), file=sys.stderr)
        print("unmodelled:", dict(m.unknown.most_common(20)), file=sys.stderr)


if __name__ == "__main__":
    main()
