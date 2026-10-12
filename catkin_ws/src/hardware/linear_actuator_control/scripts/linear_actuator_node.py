#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS1 bridge for the supplied STM32 DUAL ST3215 + ACTUATOR v2 firmware.

The original actuator topics/aliases remain valid. Both cmd_topic and the new
servo_cmd_topic also accept PAN, TILT, PT, PING, READ, HELP and BAUD. One node
owns the serial port. Every frame is newline-terminated and transmission is
paced because the supplied firmware polls UART1 and blocks during servo reads.

A requested actuator state is NOT a measured height. SENT is NOT confirmation
that a servo reached its target. S stops the actuator only, not either servo.
"""

import math
import re
import threading
import time
from collections import deque

import rospy
import serial
from std_msgs.msg import String


class LinearActuatorNode:
    MOTION_COMMANDS = {"U", "D"}
    SERVO_BAUDRATES = {1000000, 500000, 250000, 128000, 115200, 76800, 57600, 38400}
    QUEUE_LIMIT = 16
    QUEUE_TTL_SEC = 2.0
    HANDSHAKE_TIMEOUT_SEC = 3.0
    SERVO_GUARD_SEC = 0.10

    def __init__(self):
        self.port = rospy.get_param("~port", "/dev/linear_actuator")
        self.baudrate = int(rospy.get_param("~baudrate", 115200))
        self.cmd_topic = rospy.get_param("~cmd_topic", "/linear_actuator/cmd")
        self.status_topic = rospy.get_param("~status_topic", "/linear_actuator/status")
        self.servo_cmd_topic = rospy.get_param("~servo_cmd_topic", "/linear_actuator/servo_cmd")
        self.servo_baudrate = int(rospy.get_param("~servo_baudrate", 1000000))
        self.resend_hz = float(rospy.get_param("~resend_hz", 10.0))
        self.reconnect_sec = float(rospy.get_param("~reconnect_sec", 2.0))
        self.max_motion_sec = float(rospy.get_param("~max_motion_sec", 20.0))
        self.angle_limits = {}
        for axis in ("pan", "tilt"):
            low = float(rospy.get_param("~" + axis + "_min_deg", 0))
            high = float(rospy.get_param("~" + axis + "_max_deg", 360))
            if not (math.isfinite(low) and math.isfinite(high) and
                    low.is_integer() and high.is_integer() and 0 <= low <= high <= 360):
                raise ValueError(axis + " limits must be integer angles within 0..360")
            self.angle_limits[axis] = (int(low), int(high))
        if self.baudrate != 115200:
            raise ValueError("The supplied firmware fixes the PC UART at 115200 baud")
        if self.servo_baudrate not in self.SERVO_BAUDRATES:
            raise ValueError("Unsupported servo_baudrate")
        if not math.isfinite(self.resend_hz) or not 5.0 <= self.resend_hz <= 20.0:
            raise ValueError("resend_hz must be 5..20 for the 500 ms firmware watchdog")
        for name in ("reconnect_sec", "max_motion_sec"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(name + " must be positive and finite")

        self._lock = threading.RLock()
        self._serial = None
        self._rx_buffer = bytearray()
        self._pending = deque()
        self._active_wire_cmd = "S"
        self._motion_started_at = None
        self._last_actuator_tx = float("-inf")
        self._last_actuator_wire = "S"
        self._pending_stop = None
        self._next_tx_at = 0.0
        self._last_reconnect_attempt = float("-inf")
        self._last_host_status = float("-inf")
        self._ready = False
        self._saw_v2_header = False
        self._firmware_confirmed = False
        self._help_sent = False
        self._baud_sent = False
        self._handshake_started_at = 0.0
        self._shutting_down = False

        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=50, latch=True)
        self.cmd_sub = rospy.Subscriber(self.cmd_topic, String, self._command_callback, queue_size=1)
        self.servo_sub = None
        if self.servo_cmd_topic != self.cmd_topic:
            self.servo_sub = rospy.Subscriber(
                self.servo_cmd_topic, String, self._command_callback, queue_size=1)
        rospy.on_shutdown(self._on_shutdown)
        self._publish_status("NODE_STARTING")
        self._connect_serial(force=True)
        # A fixed 50 Hz scheduler leaves room for servo commands between heartbeats.
        self.timer = rospy.Timer(rospy.Duration(0.02), self._timer_callback)
        rospy.loginfo("Actuator + ST3215 bridge: %s @ %d; cmd=%s; servo_cmd=%s",
                      self.port, self.baudrate, self.cmd_topic, self.servo_cmd_topic)

    def _publish_status(self, text):
        self.status_pub.publish(String(data=str(text)))

    def _clear_motion(self):
        self._active_wire_cmd = "S"
        self._motion_started_at = None
        self._pending.clear()

    def _disconnect(self, reason):
        """Called with the lock held; never replay motion after reconnect."""
        self._clear_motion()
        self._pending_stop = None
        self._ready = False
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
        self._serial = None
        self._rx_buffer.clear()
        self._publish_status(reason)

    def _connect_serial(self, force=False):
        with self._lock:
            if self._shutting_down:
                return False
            if self._serial is not None and self._serial.is_open:
                return True
            now = time.monotonic()
            if not force and now - self._last_reconnect_attempt < self.reconnect_sec:
                return False
            self._last_reconnect_attempt = now
            self._disconnect("SERIAL_CONNECTING")
            try:
                self._serial = serial.Serial(port=self.port, baudrate=self.baudrate,
                                             timeout=0, write_timeout=0.2)
                self._serial.reset_input_buffer()
            except Exception as exc:
                self._disconnect("SERIAL_DISCONNECTED")
                rospy.logwarn_throttle(5.0, "Cannot open %s: %s", self.port, exc)
                return False
            self._saw_v2_header = False
            self._firmware_confirmed = False
            self._help_sent = False
            self._baud_sent = False
            self._last_actuator_wire = "S"
            self._last_actuator_tx = float("-inf")
            self._handshake_started_at = time.monotonic()
            self._publish_status("SERIAL_CONNECTED:VERIFYING_FIRMWARE")
            # Clear a leftover line and stop first. Never send the old '?' byte.
            return self._write_frame("S", resync=True)

    def _write_frame(self, command, resync=False):
        """Write one indivisible frame under the lock, then leave a receive gap."""
        ser = self._serial
        if ser is None or not ser.is_open:
            self._disconnect("SERIAL_DISCONNECTED")
            return False
        payload = (("\n" if resync else "") + command + "\n").encode("ascii")
        try:
            if ser.write(payload) != len(payload):
                raise IOError("short serial write")
        except Exception as exc:
            rospy.logwarn("Controller serial write failed: %s", exc)
            self._disconnect("SERIAL_WRITE_ERROR")
            return False
        now = time.monotonic()
        if command in self.MOTION_COMMANDS:
            # Firmware blocks 100 ms when reversing actuator direction.
            reversing = (self._last_actuator_wire in self.MOTION_COMMANDS and
                         command != self._last_actuator_wire)
            self._next_tx_at = now + (0.13 if reversing else 0.02)
            self._last_actuator_wire = command
            self._last_actuator_tx = now
        elif command in {"S", "F"}:
            self._next_tx_at = now + 0.02
            self._last_actuator_wire = command
        else:
            # PING/READ can block for 2 x 30 ms; HELP also transmits many bytes.
            self._next_tx_at = now + self.SERVO_GUARD_SEC
        return True

    @staticmethod
    def _normalize_command(text):
        if not isinstance(text, str) or len(text) > 80:
            return None
        stripped = text.strip()
        # Reject embedded frames, NULs and other control characters.
        if any(ord(c) < 32 and c != "\t" for c in stripped):
            return None
        aliases = {
            "u": "U", "up": "U", "extend": "U", "\u4f38\u51fa": "U",
            "d": "D", "down": "D", "retract": "D", "\u7f29\u56de": "D",
            "s": "S", "stop": "S", "brake": "S", "\u505c\u6b62": "S",
            "f": "F", "coast": "F", "free": "F", "\u81ea\u7531": "F",
            "?": "STATUS", "status": "STATUS", "state": "STATUS",
        }
        normalized = aliases.get(stripped.lower())
        if normalized:
            return normalized
        parts = stripped.upper().split()
        if not parts:
            return None
        name = parts[0]
        if name in {"PING", "READ", "HELP"} and len(parts) == 1:
            return name
        sizes = {"PAN": 2, "TILT": 2, "PT": 3, "BAUD": 2}
        if name not in sizes or len(parts) != sizes[name]:
            return None
        if not all(re.fullmatch(r"[0-9]{1,7}", p) for p in parts[1:]):
            return None
        values = [int(p) for p in parts[1:]]
        if name == "BAUD":
            if values[0] not in LinearActuatorNode.SERVO_BAUDRATES:
                return None
        elif any(v > 360 for v in values):
            return None
        return " ".join([name] + [str(v) for v in values])

    def _within_angle_limits(self, command):
        parts = command.split()
        if parts[0] == "PAN":
            values = [("pan", int(parts[1]))]
        elif parts[0] == "TILT":
            values = [("tilt", int(parts[1]))]
        elif parts[0] == "PT":
            values = [("pan", int(parts[1])), ("tilt", int(parts[2]))]
        else:
            return True
        return all(self.angle_limits[axis][0] <= val <= self.angle_limits[axis][1]
                   for axis, val in values)

    def _host_status(self):
        self._publish_status("HOST:READY=%s;ACTUATOR_COMMAND=%s;HEIGHT_FEEDBACK=NONE" %
                             (str(self._ready).lower(), self._active_wire_cmd))

    def _command_callback(self, msg):
        command = self._normalize_command(msg.data)
        with self._lock:
            if self._shutting_down:
                return
            if command is None:
                self._publish_status("INVALID_COMMAND:" + str(msg.data))
                return
            if not self._within_angle_limits(command):
                self._publish_status("REJECTED_ANGLE_LIMIT:" + command)
                return
            if command == "STATUS":
                self._host_status()
                # New firmware has no actuator position/state query. READ queries
                # servo positions only, and is queued only after firmware verification.
                if not self._ready:
                    return
                command = "READ"
            if command in {"S", "F"}:
                self._clear_motion()
                self._active_wire_cmd = command
                self._pending_stop = command
                self._publish_status("REQUEST:" + ("STOP" if command == "S" else "COAST"))
                return
            if not self._ready or self._serial is None or not self._serial.is_open:
                self._publish_status("REJECTED_NOT_READY:" + command)
                return
            if command in self.MOTION_COMMANDS:
                if self._pending_stop is not None:
                    self._publish_status("REJECTED_STOP_PENDING:" + command)
                    return
                # Repeated requests and direction changes must not reset the
                # continuous-motion timeout. An explicit stop ends the interval.
                if self._active_wire_cmd not in self.MOTION_COMMANDS:
                    self._motion_started_at = time.monotonic()
                if self._active_wire_cmd != command:
                    self._last_actuator_tx = float("-inf")
                self._active_wire_cmd = command
                self._publish_status("CMD:" + ("EXTEND" if command == "U" else "RETRACT"))
                return
            if len(self._pending) >= self.QUEUE_LIMIT:
                self._publish_status("REJECTED_QUEUE_FULL:" + command)
                return
            self._pending.append((time.monotonic(), command))
            self._publish_status("QUEUED:" + command)

    def _handle_line(self, line):
        self._publish_status(line)
        if line == "STM32 DUAL ST3215 + ACTUATOR v2 READY":
            # A reboot cancels pending work and re-applies the selected UART2 baud.
            self._clear_motion()
            self._pending_stop = "S"
            self._ready = False
            self._firmware_confirmed = False
            self._saw_v2_header = False
            self._help_sent = False
            self._baud_sent = False
            self._handshake_started_at = time.monotonic()
            self._next_tx_at = max(self._next_tx_at, time.monotonic() + 0.15)
            self._publish_status("CONTROLLER_RESET:REQUESTS_CLEARED")
        elif line == "Dual ST3215 (ID1=PAN, ID2=TILT)":
            self._saw_v2_header = True
        elif (self._saw_v2_header and
              line == "NOTE: SENT means transmitted, not verified movement."):
            self._firmware_confirmed = True
        elif line == "SERVO UART BAUD -> " + str(self.servo_baudrate):
            if self._firmware_confirmed and self._baud_sent and not self._ready:
                self._ready = True
                self._publish_status("CONTROLLER_READY:SERVOS_NOT_HOMED")
        elif line == "WATCHDOG STOP":
            self._clear_motion()
            self._pending_stop = "S"
            self._publish_status("WATCHDOG_STOP:NO_AUTOMATIC_RESUME")

    def _drain_serial(self):
        ser = self._serial
        if ser is None:
            return
        try:
            count = min(4096, ser.in_waiting)
            if count:
                self._rx_buffer.extend(ser.read(count))
        except Exception as exc:
            rospy.logwarn("Controller serial read failed: %s", exc)
            self._disconnect("SERIAL_READ_ERROR")
            return
        # Keep partial lines between timer ticks rather than publishing fragments.
        while b"\n" in self._rx_buffer:
            raw, _, rest = self._rx_buffer.partition(b"\n")
            self._rx_buffer = bytearray(rest)
            line = raw.decode("utf-8", errors="replace").strip()
            if line:
                self._handle_line(line)
        if len(self._rx_buffer) > 8192:
            self._disconnect("SERIAL_RX_OVERFLOW")

    def _timer_callback(self, _event):
        with self._lock:
            if self._shutting_down or not self._connect_serial():
                return
            self._drain_serial()
            if self._serial is None:
                return
            now = time.monotonic()
            if not self._ready and now - self._handshake_started_at >= self.HANDSHAKE_TIMEOUT_SEC:
                self._disconnect("FIRMWARE_HANDSHAKE_TIMEOUT:REQUIRES_ST3215_V2")
                return
            if (self._motion_started_at is not None and
                    now - self._motion_started_at >= self.max_motion_sec):
                self._clear_motion()
                self._pending_stop = "S"
                self._publish_status("HOST_TIMEOUT_STOP")
            if now - self._last_host_status >= 1.0:
                self._host_status()
                self._last_host_status = now
            if now < self._next_tx_at:
                return
            if self._pending_stop is not None:
                command = self._pending_stop
                if self._write_frame(command):
                    self._pending_stop = None
                return
            if not self._ready:
                if not self._help_sent:
                    # HELP has no U/D/F bytes: unlike BAUD/READ it cannot
                    # accidentally move an actuator running the old firmware.
                    if self._write_frame("HELP"):
                        self._help_sent = True
                elif self._firmware_confirmed and not self._baud_sent:
                    if self._write_frame("BAUD " + str(self.servo_baudrate)):
                        self._baud_sent = True
                return
            if (self._active_wire_cmd in self.MOTION_COMMANDS and
                    now - self._last_actuator_tx >= 1.0 / self.resend_hz):
                self._write_frame(self._active_wire_cmd)
                return
            while self._pending:
                queued_at, command = self._pending.popleft()
                if now - queued_at > self.QUEUE_TTL_SEC:
                    self._publish_status("DROPPED_STALE_COMMAND:" + command)
                    continue
                if self._write_frame(command):
                    if command.startswith("BAUD "):
                        # This is the host's selected UART2 baud, not a write to
                        # the servos' EEPROM. Re-apply it after an STM32 reset.
                        self.servo_baudrate = int(command.split()[1])
                    self._publish_status("TX:" + command + ";NOT_MOTION_CONFIRMATION")
                return

    def _on_shutdown(self):
        with self._lock:
            self._shutting_down = True
            self._clear_motion()
            # Allow a blocking firmware servo transaction to finish before S.
            delay = max(0.0, self._next_tx_at - time.monotonic())
            time.sleep(min(delay, 0.2))
            for _ in range(3):
                if self._serial is None:
                    break
                self._write_frame("S")
                time.sleep(0.03)
            self._disconnect("NODE_STOPPED:SERVO_STOP_NOT_SUPPORTED_BY_FIRMWARE")


def main():
    rospy.init_node("linear_actuator_node", anonymous=False)
    try:
        LinearActuatorNode()
        rospy.spin()
    except (ValueError, TypeError) as exc:
        rospy.logfatal("Invalid actuator/servo configuration: %s", exc)
        raise SystemExit(2)


if __name__ == "__main__":
    main()
