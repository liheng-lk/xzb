#!/usr/bin/env python3
"""Offline regression tests. No ROS installation, serial device, or motion needed.

Run: python3 -m unittest discover -s linear_actuator_control/tests -v
The mocks verify host behavior and protocol framing, NOT hardware performance.
"""
import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch


class String:
    def __init__(self, data=""):
        self.data = data


class Publisher:
    def __init__(self, topic, *args, **kwargs):
        self.topic = topic
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg.data)


class RosStub(types.ModuleType):
    def __init__(self, params=None):
        super().__init__("rospy")
        self.params = params or {}
        self.subscriptions = []
        self.Publisher = Publisher
        self.Duration = lambda n: n
        self.Timer = lambda duration, callback: types.SimpleNamespace(shutdown=lambda: None)
        self.on_shutdown = lambda callback: None
        for name in ("loginfo", "logwarn", "logwarn_throttle", "logfatal"):
            setattr(self, name, lambda *a, **k: None)

    def get_param(self, key, default):
        return self.params.get(key, default)

    def Subscriber(self, topic, msg_type, callback, **kwargs):
        sub = types.SimpleNamespace(topic=topic, callback=callback)
        self.subscriptions.append(sub)
        return sub


BOOT_HEADER = b"Dual ST3215 (ID1=PAN, ID2=TILT)\r\n"
HELP_END = b"NOTE: SENT means transmitted, not verified movement.\r\n"


class Clock:
    def __init__(self):
        self.now = 100.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class SerialStub:
    def __init__(self, clock, auto_handshake=True, **kwargs):
        self.clock = clock
        self.auto_handshake = auto_handshake
        self.kwargs = kwargs
        self.is_open = True
        self.rx = bytearray()
        self.writes = []
        self.fail_write = False
        self.fail_read = False
        self.short_write = False

    @property
    def in_waiting(self):
        if self.fail_read:
            raise OSError("disconnected while reading")
        return len(self.rx)

    def reset_input_buffer(self):
        self.rx.clear()

    def read(self, count):
        out = bytes(self.rx[:count])
        del self.rx[:count]
        return out

    def write(self, payload):
        if self.fail_write:
            raise OSError("disconnected while writing")
        if self.short_write:
            return len(payload) - 1
        self.writes.append((self.clock.now, payload))
        for frame in payload.splitlines():
            if frame == b"S":
                self.rx.extend(b"ACT STOP\r\n")
            elif frame == b"F":
                self.rx.extend(b"ACT COAST\r\n")
            elif frame == b"HELP" and self.auto_handshake:
                self.rx.extend(b"\r\n" + BOOT_HEADER + HELP_END)
            elif frame.startswith(b"BAUD ") and self.auto_handshake:
                self.rx.extend(b"SERVO UART BAUD -> " + frame[5:] + b"\r\n")
        return len(payload)

    def close(self):
        self.is_open = False


class SerialFactory:
    def __init__(self, clock, auto_handshake=True):
        self.clock = clock
        self.auto_handshake = auto_handshake
        self.ports = []
        self.fail_open = False

    def Serial(self, **kwargs):
        if self.fail_open:
            raise OSError("device not present")
        port = SerialStub(self.clock, self.auto_handshake, **kwargs)
        self.ports.append(port)
        return port


PACKAGE = Path(__file__).resolve().parents[1]
module_spec = importlib.util.spec_from_file_location(
    "linear_actuator_test_module", PACKAGE / "scripts/linear_actuator_node.py")
bridge = importlib.util.module_from_spec(module_spec)
std_msgs = types.ModuleType("std_msgs")
std_msgs_msg = types.ModuleType("std_msgs.msg")
std_msgs_msg.String = String
with patch.dict(sys.modules, {"rospy": RosStub(), "serial": types.ModuleType("serial"),
                              "std_msgs": std_msgs, "std_msgs.msg": std_msgs_msg}):
    module_spec.loader.exec_module(bridge)


class BridgeTest(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.ros = RosStub()
        self.factory = SerialFactory(self.clock)
        self.patches = [patch.object(bridge, "rospy", self.ros),
                        patch.object(bridge, "serial", self.factory),
                        patch.object(bridge, "time", self.clock)]
        for p in self.patches:
            p.start()
            self.addCleanup(p.stop)
        self.node = bridge.LinearActuatorNode()

    @property
    def port(self):
        return self.factory.ports[-1]

    def run_for(self, seconds):
        target = self.clock.now + seconds
        while self.clock.now < target - 1e-8:
            self.clock.now = min(self.clock.now + 0.02, target)
            self.node._timer_callback(None)

    def ready(self):
        self.run_for(0.5)
        self.assertTrue(self.node._ready)
        self.assertEqual(self.node._active_wire_cmd, "S")

    def command(self, data):
        self.node._command_callback(String(data=data))

    def frames(self, port=None):
        return [frame.decode("ascii") for _, payload in (port or self.port).writes
                for frame in payload.splitlines() if frame]

    def test_legacy_aliases(self):
        cases = {"u": "U", " UP ": "U", "extend": "U", "\u4f38\u51fa": "U",
                 "d": "D", "down": "D", "retract": "D", "\u7f29\u56de": "D",
                 "s": "S", "STOP": "S", "brake": "S", "\u505c\u6b62": "S",
                 "f": "F", "coast": "F", "free": "F", "\u81ea\u7531": "F",
                 "?": "STATUS", "state": "STATUS", "status": "STATUS"}
        for text, expected in cases.items():
            with self.subTest(text=text):
                self.assertEqual(self.node._normalize_command(text), expected)

    def test_servo_command_normalization(self):
        for text, expected in [("PAN 0", "PAN 0"), ("pan 0360", "PAN 360"),
                               ("tilt\t180", "TILT 180"), ("pt 12 345", "PT 12 345"),
                               ("ping", "PING"), ("read", "READ"), ("help", "HELP")]:
            self.assertEqual(self.node._normalize_command(text), expected)
        for baud in self.node.SERVO_BAUDRATES:
            self.assertEqual(self.node._normalize_command("BAUD " + str(baud)),
                             "BAUD " + str(baud))

    def test_unsafe_or_unsupported_commands_rejected(self):
        for text in ("", "PAN -1", "TILT 361", "PT 1", "PT 1 2 3", "PAN 2.5", "ID2",
                     "CENTER", "TORQUE OFF", "BAUD 9600", "READ 1", "PAN 5\nD",
                     "PAN 3\rTILT 9", "PAN 2\x00", "PAN NaN", "PAN inf", "PAN +1",
                     "PAN 999999999999999999", "PAN 5;D", "PAND 10", "PT 1 361",
                     "PAN \uff11\uff18\uff10"):
            with self.subTest(text=text):
                self.assertIsNone(self.node._normalize_command(text))

    def test_startup_handshake_no_servo_motion(self):
        self.ready()
        self.assertEqual(self.frames(), ["S", "HELP", "BAUD 1000000"])
        self.assertEqual(self.port.kwargs["baudrate"], 115200)
        self.assertTrue(any(x.startswith("CONTROLLER_READY:")
                            for x in self.node.status_pub.messages))

    def test_motion_before_handshake_rejected_not_replayed(self):
        self.command("extend")
        self.command("PAN 180")
        self.ready()
        self.assertNotIn("U", self.frames())
        self.assertNotIn("PAN 180", self.frames())
        self.assertIn("REJECTED_NOT_READY:U", self.node.status_pub.messages)

    def test_legacy_firmware_does_not_receive_baud_or_motion(self):
        self.port.auto_handshake = False
        self.factory.auto_handshake = False
        self.command("READ")
        self.command("extend")
        self.run_for(3.1)
        self.assertIn("FIRMWARE_HANDSHAKE_TIMEOUT:REQUIRES_ST3215_V2",
                      self.node.status_pub.messages)
        for port in self.factory.ports:
            self.assertTrue(set(self.frames(port)).issubset({"S", "HELP"}))
        self.assertFalse(self.node._ready)

    def test_baud_ack_required_for_ready(self):
        self.run_for(0.14)
        # Advance until BAUD has been sent, before consuming its reply.
        while not self.node._baud_sent:
            self.run_for(0.02)
        self.port.rx.clear()
        self.assertFalse(self.node._ready)
        self.command("TILT 180")
        self.assertIn("REJECTED_NOT_READY:TILT 180", self.node.status_pub.messages)
        self.port.rx.extend(b"SERVO UART BAUD -> 115200\r\n")
        self.run_for(0.02)
        self.assertFalse(self.node._ready)
        self.port.rx.extend(b"SERVO UART BAUD -> 1000000\r\n")
        self.run_for(0.02)
        self.assertTrue(self.node._ready)

    def test_question_mark_translated_not_written(self):
        self.ready()
        self.command("?")
        self.run_for(0.2)
        self.assertIn("READ", self.frames())
        self.assertFalse(any(b"?" in payload for _, payload in self.port.writes))
        self.assertTrue(any("HEIGHT_FEEDBACK=NONE" in s for s in self.node.status_pub.messages))

    def test_no_periodic_question_mark_or_servo_motion(self):
        self.ready()
        initial = list(self.frames())
        self.run_for(5.0)
        self.assertEqual(self.frames(), initial)

    def test_pan_tilt_pt_frames(self):
        self.ready()
        for cmd in ("PAN 170", "TILT 180", "PT 175 185", "PING", "READ"):
            self.command(cmd)
        self.run_for(1.0)
        self.assertEqual(self.frames()[3:], ["PAN 170", "TILT 180", "PT 175 185", "PING", "READ"])
        self.assertTrue(all(payload.endswith(b"\n") for _, payload in self.port.writes))

    def test_configured_angle_limits_reject_entire_pt(self):
        self.ready()
        self.node.angle_limits = {"pan": (100, 200), "tilt": (160, 190)}
        self.command("PT 180 220")
        self.command("PAN 201")
        self.command("TILT 159")
        self.command("PT 180 180")
        self.run_for(0.5)
        self.assertEqual(self.frames()[3:], ["PT 180 180"])

    def test_actuator_and_servo_interleaving_with_watchdog_margin(self):
        self.ready()
        self.command("extend")
        for i in range(8):
            self.command("READ" if i % 2 else "PT 170 180")
        self.run_for(1.7)
        times = [t for t, payload in self.port.writes if payload == b"U\n"]
        self.assertGreater(len(times), 4)
        self.assertLess(max(b - a for a, b in zip(times, times[1:])), 0.5)
        self.assertEqual(self.frames().count("PT 170 180"), 4)
        self.assertEqual(self.frames().count("READ"), 4)

    def test_high_heartbeat_rate_does_not_starve_servos(self):
        self.ready()
        self.node.resend_hz = 20.0
        self.command("extend")
        self.command("PT 180 180")
        self.run_for(0.6)
        self.assertIn("PT 180 180", self.frames())

    def test_servo_transactions_have_processing_gap(self):
        self.ready()
        for _ in range(5):
            self.command("READ")
        self.command("extend")
        self.run_for(1.4)
        for (time_a, payload), (time_b, _) in zip(self.port.writes, self.port.writes[1:]):
            if payload.strip() in {b"HELP", b"READ"} or payload.startswith(b"BAUD "):
                self.assertGreaterEqual(time_b - time_a + 1e-8, 0.10)

    def test_direction_reversal_has_processing_gap(self):
        self.ready()
        self.command("extend")
        self.run_for(0.3)
        self.command("retract")
        self.command("PT 180 180")
        self.run_for(0.6)
        writes = self.port.writes
        idx = next(i for i, (_, data) in enumerate(writes) if data == b"D\n")
        self.assertGreaterEqual(writes[idx + 1][0] - writes[idx][0] + 1e-8, 0.13)

    def test_stop_cancels_unsent_servo_and_actuator_requests(self):
        self.ready()
        self.command("extend")
        self.run_for(0.2)
        self.command("PAN 170")
        self.command("TILT 170")
        self.command("stop")
        self.run_for(0.5)
        self.assertEqual(self.frames()[-1], "S")
        self.assertNotIn("PAN 170", self.frames())
        self.assertNotIn("TILT 170", self.frames())
        self.assertIsNone(self.node._motion_started_at)

    def test_motion_cannot_override_pending_stop(self):
        self.ready()
        self.command("stop")
        self.command("extend")
        self.run_for(0.3)
        self.assertNotIn("U", self.frames())
        self.assertIn("REJECTED_STOP_PENDING:U", self.node.status_pub.messages)

    def test_repeated_requests_and_reversal_do_not_reset_timeout(self):
        self.ready()
        self.node.max_motion_sec = 0.5
        self.command("extend")
        started = self.node._motion_started_at
        self.run_for(0.18)
        self.command("extend")
        self.run_for(0.18)
        self.command("retract")
        self.assertEqual(self.node._motion_started_at, started)
        self.run_for(0.4)
        self.assertIn("HOST_TIMEOUT_STOP", self.node.status_pub.messages)
        self.assertEqual(self.node._active_wire_cmd, "S")
        self.assertEqual(self.frames()[-1], "S")

    def test_stale_servo_request_dropped(self):
        self.ready()
        self.command("PAN 180")
        self.clock.now += 2.1
        self.node._timer_callback(None)
        self.assertNotIn("PAN 180", self.frames())
        self.assertIn("DROPPED_STALE_COMMAND:PAN 180", self.node.status_pub.messages)

    def test_bounded_command_queue(self):
        self.ready()
        for _ in range(17):
            self.command("READ")
        self.assertEqual(len(self.node._pending), 16)
        self.assertIn("REJECTED_QUEUE_FULL:READ", self.node.status_pub.messages)

    def test_fragmented_status_line_retained(self):
        self.ready()
        self.port.rx.extend(b"ID1: POS=2")
        self.run_for(0.02)
        self.assertNotIn("ID1: POS=2", self.node.status_pub.messages)
        self.port.rx.extend(b"048 DEG~180\r\n")
        self.run_for(0.02)
        self.assertIn("ID1: POS=2048 DEG~180", self.node.status_pub.messages)

    def test_watchdog_stops_heartbeat_without_automatic_resume(self):
        self.ready()
        self.command("extend")
        self.run_for(0.3)
        self.command("PAN 180")
        count = len(self.frames())
        self.port.rx.extend(b"WATCHDOG STOP\r\n")
        self.run_for(0.4)
        self.assertNotIn("U", self.frames()[count:])
        self.assertNotIn("PAN 180", self.frames()[count:])
        self.assertEqual(self.node._active_wire_cmd, "S")

    def test_write_disconnect_drops_motion_and_queue(self):
        self.ready()
        self.command("extend")
        self.run_for(0.2)
        self.command("PAN 180")
        self.port.fail_write = True
        self.run_for(0.02)
        self.assertIsNone(self.node._serial)
        self.assertEqual(self.node._active_wire_cmd, "S")
        self.assertEqual(len(self.node._pending), 0)
        self.command("TILT 180")
        self.run_for(2.8)
        self.assertTrue(self.node._ready)
        self.assertTrue(set(self.frames()).issubset({"S", "HELP", "BAUD 1000000"}))

    def test_read_disconnect_drops_motion(self):
        self.ready()
        self.command("extend")
        self.run_for(0.2)
        self.port.fail_read = True
        self.run_for(0.02)
        self.assertIsNone(self.node._serial)
        self.assertEqual(self.node._active_wire_cmd, "S")

    def test_short_write_is_a_disconnect(self):
        self.ready()
        self.port.short_write = True
        self.command("PAN 180")
        self.run_for(0.02)
        self.assertIsNone(self.node._serial)
        self.assertIn("SERIAL_WRITE_ERROR", self.node.status_pub.messages)

    def test_runtime_baud_is_reapplied_after_controller_reset(self):
        self.ready()
        self.command("BAUD 115200")
        self.run_for(0.3)
        self.assertEqual(self.node.servo_baudrate, 115200)
        self.command("extend")
        self.run_for(0.3)
        self.command("PAN 180")
        count = len(self.frames())
        self.port.rx.extend(b"STM32 DUAL ST3215 + ACTUATOR v2 READY\r\n" + BOOT_HEADER + HELP_END)
        self.run_for(0.8)
        self.assertTrue(self.node._ready)
        later = self.frames()[count:]
        self.assertIn("S", later)
        self.assertIn("BAUD 115200", later)
        self.assertNotIn("U", later)
        self.assertNotIn("PAN 180", later)
        self.assertEqual(self.port.kwargs["baudrate"], 115200)

    def test_servo_sent_is_not_reported_as_arrival(self):
        self.ready()
        self.port.rx.extend(b"PT 180 180 SENT (not confirmed)\r\n")
        self.run_for(0.02)
        self.assertIn("PT 180 180 SENT (not confirmed)", self.node.status_pub.messages)
        self.assertFalse(any(s in {"ARRIVED", "GOAL_REACHED"} for s in self.node.status_pub.messages))

    def test_shutdown_sends_only_actuator_stops_and_closes(self):
        self.ready()
        port = self.port
        self.command("extend")
        self.command("PAN 180")
        count = len(self.frames())
        self.node._on_shutdown()
        self.assertEqual(self.frames(port)[count:], ["S", "S", "S"])
        self.assertFalse(port.is_open)
        self.assertEqual(len(self.node._pending), 0)
        self.node._timer_callback(None)
        self.assertIsNone(self.node._serial)

    def test_original_and_new_topics_are_available(self):
        self.assertEqual([s.topic for s in self.ros.subscriptions],
                         ["/linear_actuator/cmd", "/linear_actuator/servo_cmd"])
        self.assertEqual(self.node.status_pub.topic, "/linear_actuator/status")

    def test_invalid_parameters_fail_before_opening_serial(self):
        for params in ({"~baudrate": 9600}, {"~servo_baudrate": 9600},
                       {"~resend_hz": 0}, {"~resend_hz": float("nan")},
                       {"~max_motion_sec": -1}, {"~max_motion_sec": float("inf")},
                       {"~reconnect_sec": 0}, {"~pan_min_deg": -1},
                       {"~pan_max_deg": 361}, {"~tilt_min_deg": 2.5},
                       {"~tilt_min_deg": 200, "~tilt_max_deg": 180}):
            with self.subTest(params=params):
                before = len(self.factory.ports)
                with patch.object(bridge, "rospy", RosStub(params)):
                    with self.assertRaises(ValueError):
                        bridge.LinearActuatorNode()
                self.assertEqual(len(self.factory.ports), before)


if __name__ == "__main__":
    unittest.main()
