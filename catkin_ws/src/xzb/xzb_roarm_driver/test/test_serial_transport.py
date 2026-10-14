import threading
import time
import unittest

from xzb_roarm_driver.serial_transport import SerialTransport, TransportDisconnected


class FakeSerial:
    def __init__(self, lines=None):
        self.lines = list(lines or [])
        self.writes = []
        self.closed = False
        self.flush_count = 0

    @property
    def is_open(self):
        return not self.closed

    def readline(self):
        if self.closed:
            raise OSError('device disconnected')
        return self.lines.pop(0) if self.lines else b''

    def write(self, data):
        if self.closed:
            raise OSError('device disconnected')
        self.writes.append(data)
        return len(data)

    def flush(self):
        self.flush_count += 1

    def close(self):
        self.closed = True


class ConcurrentFakeSerial(FakeSerial):
    def __init__(self):
        super().__init__()
        self._guard = threading.Lock()
        self.active_writes = 0
        self.max_active_writes = 0

    def write(self, data):
        with self._guard:
            self.active_writes += 1
            self.max_active_writes = max(self.max_active_writes, self.active_writes)
        try:
            time.sleep(0.02)
            return super().write(data)
        finally:
            with self._guard:
                self.active_writes -= 1


class SerialTransportTests(unittest.TestCase):
    def test_connect_uses_115200_8n1_without_dtr_rts_calls(self):
        calls = []
        fake = FakeSerial()

        def factory(**kwargs):
            calls.append(kwargs)
            return fake

        transport = SerialTransport('/dev/ttyUSB0', serial_factory=factory)
        transport.connect()

        self.assertTrue(transport.connected)
        self.assertEqual(calls, [{
            'port': '/dev/ttyUSB0',
            'baudrate': 115200,
            'bytesize': 8,
            'parity': 'N',
            'stopbits': 1,
            'timeout': 0.2,
        }])

    def test_read_line_decodes_utf8_and_strips_line_endings(self):
        fake = FakeSerial([b'{"T":1051}\r\n'])
        transport = SerialTransport('/dev/ttyUSB0', serial_factory=lambda **_: fake)
        transport.connect()
        self.assertEqual(transport.read_line(), '{"T":1051}')

    def test_write_requires_connection(self):
        transport = SerialTransport('/dev/ttyUSB0', serial_factory=lambda **_: FakeSerial())
        with self.assertRaises(TransportDisconnected):
            transport.write(b'{"T":105}\n')

    def test_write_flushes_exact_payload(self):
        fake = FakeSerial()
        transport = SerialTransport('/dev/ttyUSB0', serial_factory=lambda **_: fake)
        transport.connect()
        transport.write(b'{"T":105}\n')
        self.assertEqual(fake.writes, [b'{"T":105}\n'])
        self.assertEqual(fake.flush_count, 1)

    def test_write_callback_observes_exact_payload_after_successful_flush(self):
        fake = FakeSerial()
        observed = []
        transport = SerialTransport(
            '/dev/ttyUSB0',
            serial_factory=lambda **_: fake,
            on_write=observed.append,
        )
        transport.connect()
        payload = b'{"T":101,"joint":2,"rad":0.1,"spd":0,"acc":10}\n'
        transport.write(payload)
        self.assertEqual(observed, [payload])
        self.assertEqual(fake.flush_count, 1)

    def test_concurrent_writes_are_serialized(self):
        fake = ConcurrentFakeSerial()
        transport = SerialTransport('/dev/ttyUSB0', serial_factory=lambda **_: fake)
        transport.connect()
        barrier = threading.Barrier(3)
        errors = []

        def writer(payload):
            try:
                barrier.wait(timeout=1.0)
                transport.write(payload)
            except Exception as exc:
                errors.append(exc)

        first = threading.Thread(target=writer, args=(b'{"T":105}\n',))
        second = threading.Thread(target=writer, args=(b'{"T":101,"joint":1,"rad":0.1,"spd":0,"acc":10}\n',))
        first.start(); second.start(); barrier.wait(timeout=1.0)
        first.join(timeout=1.0); second.join(timeout=1.0)

        self.assertEqual(errors, [])
        self.assertFalse(first.is_alive())
        self.assertFalse(second.is_alive())
        self.assertEqual(fake.max_active_writes, 1)
        self.assertEqual(len(fake.writes), 2)

    def test_read_failure_disconnects_transport(self):
        fake = FakeSerial()
        transport = SerialTransport('/dev/ttyUSB0', serial_factory=lambda **_: fake)
        transport.connect()
        fake.close()
        with self.assertRaises(TransportDisconnected):
            transport.read_line()
        self.assertFalse(transport.connected)

    def test_close_is_idempotent(self):
        fake = FakeSerial()
        transport = SerialTransport('/dev/ttyUSB0', serial_factory=lambda **_: fake)
        transport.connect()
        transport.close()
        transport.close()
        self.assertFalse(transport.connected)

    def test_background_worker_delivers_lines(self):
        got = []
        delivered = threading.Event()
        fake = FakeSerial([b'{"T":105}\n'])

        def on_line(line):
            got.append(line)
            delivered.set()

        transport = SerialTransport(
            '/dev/ttyUSB0',
            serial_factory=lambda **_: fake,
            on_line=on_line,
            reconnect_interval=0.01,
        )
        transport.start()
        self.assertTrue(delivered.wait(0.5))
        transport.stop()
        self.assertEqual(got[0], '{"T":105}')

    def test_background_worker_reconnects_after_open_failure(self):
        attempts = []
        connected = threading.Event()
        fake = FakeSerial()

        def factory(**kwargs):
            attempts.append(kwargs)
            if len(attempts) == 1:
                raise OSError('not present')
            connected.set()
            return fake

        transport = SerialTransport(
            '/dev/ttyUSB0',
            serial_factory=factory,
            reconnect_interval=0.01,
        )
        transport.start()
        self.assertTrue(connected.wait(0.5))
        deadline = time.time() + 0.5
        while time.time() < deadline and not transport.connected:
            time.sleep(0.005)
        transport.stop()
        self.assertGreaterEqual(len(attempts), 2)
        self.assertFalse(transport.connected)


if __name__ == '__main__':
    unittest.main()
