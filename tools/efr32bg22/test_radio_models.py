#!/usr/bin/env python3
import heapq
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).parent))
import models


class Machine:
    def __init__(self):
        self.now = 0
        self.irq = 0
        self.events = []
        self.seq = 0

    def set_irq(self, number, level):
        if level:
            self.irq |= 1 << number
        else:
            self.irq &= ~(1 << number)

    def after(self, delay, callback):
        self.seq += 1
        heapq.heappush(self.events, (self.now + delay, self.seq, callback))

    def advance(self, delay):
        self.now += delay
        while self.events and self.events[0][0] <= self.now:
            heapq.heappop(self.events)[2]()


def attach(device):
    machine = Machine()
    device.m = machine
    return machine, device


class BufcTests(unittest.TestCase):
    def test_xg22_trace_offsets(self):
        dev = models.BUFC("BUFC")
        self.assertEqual((dev.BUF0, dev.STRIDE), (0x00C, 0x040))
        self.assertEqual((dev.IF_OFF, dev.IEN_OFF), (0x114, 0x118))

    def test_programmed_io_offsets_status_and_contents(self):
        _, dev = attach(models.BUFC("BUFC"))
        base = dev.BUF0
        dev.write(base + 0x18, 0x41)
        dev.write(base + 0x18, 0x42)
        self.assertEqual(dev.contents(0), b"AB")
        self.assertEqual(dev.read(base + 0x20) & 0x1FFF, 2)
        self.assertEqual(dev.read(base + 0x14), ord("A"))
        self.assertEqual(dev.read(base + 0x20) & 0x1FFF, 1)

    def test_xwrite_clear_threshold_and_irq(self):
        machine, dev = attach(models.BUFC("BUFC"))
        base = dev.BUF0 + dev.STRIDE
        dev.write(dev.IEN_OFF, 1 << 1)
        dev.write(base + 0x24, 0)             # larger than zero
        dev.write(base + 0x1C, 0x55)
        self.assertEqual(dev.contents(1), b"U")
        self.assertTrue(machine.irq & (1 << models.resets.IRQS["BUFC"]))
        dev.write(base + 0x28, 1)
        self.assertEqual(dev.contents(1), b"")
        self.assertFalse(machine.irq & (1 << models.resets.IRQS["BUFC"]))


class SimRacTests(unittest.TestCase):
    def test_tx_completes_and_raises_rac_seq(self):
        machine, dev = attach(models.SimRAC("RAC_SIM"))
        dev.write(dev.IEN_OFF, 1)
        dev.write(0x10, 3)
        self.assertEqual(dev.read(0x14), dev.TX)
        machine.advance(149_999)
        self.assertEqual(dev.read(0x14), dev.TX)
        machine.advance(1)
        self.assertEqual(dev.read(0x14), dev.IDLE)
        self.assertTrue(machine.irq & (1 << models.resets.IRQS["RAC_SEQ"]))

    def test_disable_cancels_pending_tx_completion(self):
        machine, dev = attach(models.SimRAC("RAC_SIM"))
        dev.write(0x10, 3)
        dev.write(0x10, 1)
        machine.advance(150_000)
        self.assertEqual(dev.read(0x14), dev.IDLE)
        self.assertEqual(dev.read(dev.IF_OFF), 0)


if __name__ == "__main__":
    unittest.main()
