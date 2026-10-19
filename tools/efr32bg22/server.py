#!/usr/bin/env python3
"""Peripheral server for the efr32bg22 QEMU machine (see hw/arm/efr32bg22.c)."""
import argparse, os, socket, struct, sys, collections

REQ = struct.Struct("<BBHIIIQ")
REP = struct.Struct("<IIQQ")


class Generic:
    """Plain register file with Series-2 SET/CLR/TGL aliases."""
    def __init__(self):
        self.regs = {}

    def read(self, off, size, now):
        return self.regs.get(off & ~3, 0)

    def write(self, off, val, size, now):
        self.regs[off & ~3] = val


class Machine:
    def __init__(self, trace):
        self.devs = []          # (base, size, dev, name)
        self.default = Generic()
        self.irq = 0
        self.next_ns = 0
        self.trace = trace
        self.seen = collections.Counter()

    def add(self, base, size, dev, name):
        self.devs.append((base, size, dev, name))

    def find(self, addr):
        a = addr & ~0x50000000 | 0x40000000 if addr >= 0x50000000 else addr
        for base, size, dev, name in self.devs:
            if base <= a < base + size:
                return dev, a - base, name
        return self.default, a & 0x3FFF, "?%08x" % (a & ~0x3FFF)

    def access(self, op, addr, val, size, now):
        dev, off, name = self.find(addr)
        alias = (off >> 12) & 3
        off &= 0xFFF
        if op == 0:
            r = dev.read(off, size, now)
            if self.trace:
                print(f"{now:>12} R {name}+{off:03x} = {r:08x}", file=sys.stderr)
            self.seen[name] += 1
            return r
        if alias:
            cur = dev.read(off, 4, now)
            val = {1: cur | val, 2: cur & ~val, 3: cur ^ val}[alias]
        if self.trace:
            print(f"{now:>12} W {name}+{off:03x} <- {val:08x}", file=sys.stderr)
        self.seen[name] += 1
        dev.write(off, val, size, now)
        return 0


def serve(m, path):
    if os.path.exists(path):
        os.unlink(path)
    srv = socket.socket(socket.AF_UNIX)
    srv.bind(path)
    srv.listen(1)
    print("listening", path, file=sys.stderr, flush=True)
    c, _ = srv.accept()
    buf = b""
    while True:
        while len(buf) < REQ.size:
            d = c.recv(4096)
            if not d:
                return
            buf += d
        op, size, _, addr, val, _, now = REQ.unpack(buf[:REQ.size])
        buf = buf[REQ.size:]
        r = m.access(op, addr, val, size, now) if op < 2 else m.tick(now)
        c.sendall(REP.pack(r & 0xFFFFFFFF, 0, m.irq, m.next_ns))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--sock", default="/tmp/claude/efr32.sock")
    ap.add_argument("--trace", action="store_true")
    a = ap.parse_args()
    m = Machine(a.trace)
    m.tick = lambda now: 0
    try:
        serve(m, a.sock)
    finally:
        print(dict(m.seen), file=sys.stderr)
