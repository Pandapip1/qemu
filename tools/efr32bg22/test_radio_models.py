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


class RacTests(unittest.TestCase):
    def test_tx_completes_and_raises_real_rac_seq(self):
        machine, dev = attach(models.RAC("RAC"))
        dev.write(dev.SEQIEN_OFF, dev.SEQ_STATEOFF)
        dev.write(dev.CMD, dev.CMD_TXEN)
        self.assertEqual((dev.read(dev.STATUS) >> 24) & 0xF, dev.STATETX)
        self.assertTrue(dev.read(dev.STATUS) & (1 << 30))
        machine.advance(149_999)
        self.assertEqual((dev.read(dev.STATUS) >> 24) & 0xF, dev.STATETX)
        machine.advance(1)
        self.assertEqual((dev.read(dev.STATUS) >> 24) & 0xF, dev.STATEOFF)
        self.assertTrue(machine.irq & (1 << models.resets.IRQS["RAC_SEQ"]))

    def test_disable_cancels_pending_tx_completion(self):
        machine, dev = attach(models.RAC("RAC"))
        dev.write(dev.CMD, dev.CMD_FORCETX)
        dev.write(dev.CMD, dev.CMD_TXDIS)
        machine.advance(150_000)
        self.assertEqual((dev.read(dev.STATUS) >> 24) & 0xF, dev.STATEOFF)

    def test_software_rx_enable_and_rx_disable(self):
        _, dev = attach(models.RAC("RAC"))
        dev.write(dev.RXENSRCEN, 1)
        self.assertEqual((dev.read(dev.STATUS) >> 24) & 0xF,
                         dev.STATERXSEARCH)
        self.assertTrue(dev.read(dev.STATUS) & (1 << 31))
        dev.write(dev.CMD, dev.CMD_RXDIS)
        self.assertEqual((dev.read(dev.STATUS) >> 24) & 0xF, dev.STATEOFF)

    def test_rsm_and_seq_interrupts_are_distinct(self):
        machine, dev = attach(models.RAC("RAC"))
        dev.write(dev.IEN_OFF, dev.IF_STATECHANGE)
        dev.write(dev.RXENSRCEN, 1)
        self.assertTrue(machine.irq & (1 << models.resets.IRQS["RAC_RSM"]))
        self.assertFalse(machine.irq & (1 << models.resets.IRQS["RAC_SEQ"]))
        dev.write(dev.SEQIEN_OFF, dev.SEQ_STATERXSEARCH)
        self.assertTrue(machine.irq & (1 << models.resets.IRQS["RAC_SEQ"]))


class SynthTests(unittest.TestCase):
    def test_public_xg22_channel_and_frequency_field_masks(self):
        _, dev = attach(models.Synth("SYNTH"))
        dev.write(dev.CHCTRL, 0xFF)
        dev.write(dev.FREQ, 0xF1234567)
        dev.write(dev.IFFREQ, 0xFF123456)
        dev.write(dev.CHSP, 0xFFFABCDE)
        self.assertEqual(dev.read(dev.CHCTRL), 0x3F)
        self.assertEqual(dev.read(dev.FREQ), 0x01234567)
        self.assertEqual(dev.read(dev.IFFREQ), 0x00123456)
        self.assertEqual(dev.read(dev.CHSP), 0x0002BCDE)


class RfcrcTests(unittest.TestCase):
    def test_public_configuration_masks_and_command_side_effects(self):
        _, dev = attach(models.RFCRC("RFCRC"))
        dev.write(dev.EN, 3)
        dev.write(dev.CTRL, 0xFFFFF7FF)
        dev.write(dev.INIT, 0x00AAAAAA)
        dev.write(dev.POLY, 0x00DA6000)
        dev.write(dev.CMD, 1)
        self.assertEqual(dev.read(dev.EN), 1)
        self.assertEqual(dev.read(dev.CTRL), 0x000017EF)
        self.assertEqual(dev.read(dev.INIT), 0x00AAAAAA)
        self.assertEqual(dev.read(dev.POLY), 0x00DA6000)
        self.assertEqual(dev.read(dev.STATUS), 0)


class FrcTests(unittest.TestCase):
    def test_public_whitening_configuration_masks(self):
        _, dev = attach(models.FRC("FRC"))
        dev.write(dev.WHITECTRL, 0xFFFFFFFF)
        dev.write(dev.WHITEPOLY, 0x1234ABCD)
        dev.write(dev.WHITEINIT, 0xABCD1234)
        self.assertEqual(dev.read(dev.WHITECTRL), 0x00001F7F)
        self.assertEqual(dev.read(dev.WHITEPOLY), 0x0000ABCD)
        self.assertEqual(dev.read(dev.WHITEINIT), 0x00001234)


class ModemTests(unittest.TestCase):
    def test_public_sync0_is_a_32_bit_configuration_register(self):
        _, dev = attach(models.Modem("MODEM"))
        dev.write(dev.SYNC0, 0x89ABCDEF)
        self.assertEqual(dev.read(dev.SYNC0), 0x89ABCDEF)


if __name__ == "__main__":
    unittest.main()
