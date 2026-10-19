#!/usr/bin/env python3
import heapq
import pathlib
import socket
import sys
import threading
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).parent))
import models
import server


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
        dev.write(dev.IEN_OFF, 1 << (8 * 1 + 2))
        dev.write(base + 0x24, 0)             # larger than zero
        dev.write(base + 0x1C, 0x55)
        self.assertEqual(dev.contents(1), b"U")
        self.assertEqual(dev.read(dev.IF_OFF), 1 << 10)
        self.assertEqual(dev.read(base + 0x20) & (1 << 20), 1 << 20)
        self.assertTrue(machine.irq & (1 << models.resets.IRQS["BUFC"]))
        dev.write(base + 0x28, 1)
        self.assertEqual(dev.contents(1), b"")
        self.assertFalse(machine.irq & (1 << models.resets.IRQS["BUFC"]))

    def test_public_command_mask_retains_unassigned_trace_bit(self):
        _, dev = attach(models.BUFC("BUFC"))
        base = dev.BUF0 + 3 * dev.STRIDE
        dev.write(base + 0x28, 0xF4)
        self.assertEqual(dev.read(base + 0x28), 0x4)


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

    def test_public_ble_frame_configuration_masks(self):
        _, dev = attach(models.FRC("FRC"))
        dev.write(dev.DFLCTRL, 0xFE018101)
        dev.write(dev.MAXLENGTH, 0xFFFF4101)
        dev.write(dev.CTRL, 0xFFFF07A0)
        for off in (dev.FCD0, dev.FCD1, dev.FCD2, dev.FCD3):
            dev.write(off, 0xFFFF4CFF)
        self.assertEqual(dev.read(dev.DFLCTRL), 0x00018101)
        self.assertEqual(dev.read(dev.MAXLENGTH), 0x00004101)
        self.assertEqual(dev.read(dev.CTRL), 0x001F07A0)
        for off in (dev.FCD0, dev.FCD1, dev.FCD2, dev.FCD3):
            self.assertEqual(dev.read(off), 0x00014CFF)

    def test_public_ble_baseline_configuration_masks(self):
        _, dev = attach(models.FRC("FRC"))
        expectations = {
            dev.WCNTCMP0: 0x00000FFF,
            dev.WCNTCMP1: 0x00000FFF,
            dev.FECCTRL: 0x003FFFF7,
            dev.BLOCKRAMADDR: 0xFFFFFFFC,
            dev.CONVRAMADDR: 0xFFFFFFFC,
            dev.TRAILTXDATACTRL: 0x007FFFFF,
            dev.CONVGENERATOR: 0x00037F7F,
            dev.PUNCTCTRL: 0x00007F7F,
            dev.AUTOCG: 0x0000FFFF,
            dev.WCNTCMP3: 0x00000FFF,
            dev.BOICTRL: 0x0001FFFF,
            dev.DSLCTRL: 0x7FFFFF7F,
            dev.WCNTCMP4: 0x00000FFF,
        }
        for off, expected in expectations.items():
            dev.write(off, 0xFFFFFFFF)
            self.assertEqual(dev.read(off), expected)

    def test_public_rx_and_packet_buffer_configuration_masks(self):
        _, dev = attach(models.FRC("FRC"))
        dev.write(dev.RXCTRL, 0xFFFF07FF)
        dev.write(dev.PKTBUFCTRL, 0xFFFFFFFF)
        self.assertEqual(dev.read(dev.RXCTRL), 0x000007FF)
        self.assertEqual(dev.read(dev.PKTBUFCTRL), 0x0103FFFF)


class GuestMemoryTests(unittest.TestCase):
    def test_model_guest_read_delegates_to_bridge_transport(self):
        m = server.Machine()
        m.mem_read = lambda addr, length: bytes((addr + n) & 0xFF for n in range(length))
        self.assertEqual(m.guest_read(0x20000010, 4), b"\x10\x11\x12\x13")

    def test_bridge_memory_read_frame_and_payload(self):
        server_sock, qemu_sock = socket.socketpair()
        result = []
        reader = server.guest_memory_reader(server_sock)
        worker = threading.Thread(target=lambda: result.append(reader(0x20000010, 4)))
        worker.start()
        request = qemu_sock.recv(server.REP.size)
        self.assertEqual(server.REP.unpack(request),
                         (0x20000010, server.MEM_READ_REQ, 0, 4))
        qemu_sock.sendall(b"\x10\x11\x12\x13")
        worker.join()
        self.assertEqual(result, [b"\x10\x11\x12\x13"])
        server_sock.close()
        qemu_sock.close()


class ModemTests(unittest.TestCase):
    def test_public_ble_baseline_configuration_masks(self):
        _, dev = attach(models.Modem("MODEM"))
        expectations = {
            dev.MIXCTRL: 0x0000001F,
            dev.CTRL1: 0xFFFFDFFF,
            dev.CTRL3: 0xFFFFFF81,
            dev.CTRL4: 0xBFFFFFFF,
            dev.CTRL5: 0x607007FE,
            dev.CTRL6: 0xF6000000,
            dev.TXBR: 0x00FFFFFF,
            dev.RXBR: 0x00001FFF,
            dev.CF: 0xCFFFFFFF,
            dev.PRE: 0xFFFF1FFF,
            dev.DCCOMP: 0x000001FF,
            dev.DCCOMPFILTINIT: 0x7FFFFFFF,
            dev.DSATHD1: 0x7FFFFFFF,
            dev.AUTOCG: 0x0000FFFF,
            dev.DSATHD2: 0x7FFFFEFF,
            dev.DIRECTMODE: 0x00001F0F,
            dev.LRFRC: 0x000001FF,
            dev.DSATHD4: 0x07FFFFFF,
            dev.SYNCPROPERTIES: 0x000000FF,
            dev.DIGIGAINCTRL: 0x000001FF,
            dev.PADEBUG: 0x00001E00,
            dev.ETSCTRL: 0x3FFFF7FF,
            dev.ANTSWCTRL: 0x01FFFFFF,
            dev.ANTSWSTART: 0x0003FFFF,
            dev.ANTSWEND: 0x0003FFFF,
            dev.TRECPMDET: 0xBFFFC3FF,
            dev.CFGANTPATT: 0x3FFFFFFF,
            dev.ETSTIM: 0x0003FFFF,
            dev.ANTSWCTRL1: 0x00FFFFFF,
            dev.ANTDIVCTRL: 0x000001FF,
            dev.BLEIQDSAEXT2: 0x000007FF,
        }
        for off, expected in expectations.items():
            dev.write(off, 0xFFFFFFFF)
            self.assertEqual(dev.read(off), expected)
        dev.write(dev.SYNC0, 0x89ABCDEF)
        self.assertEqual(dev.read(dev.SYNC0), 0x89ABCDEF)


if __name__ == "__main__":
    unittest.main()
