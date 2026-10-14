from typing import Callable, Dict, List, Optional, Tuple

from .protocol import classify_line, make_boot_mission_query, make_joint_command, make_single_joint_command, make_state_query
from .serial_transport import TransportDisconnected


class ReadOnlyDriverCore:
    """Serial-facing core; historical name retained for compatibility."""
    def __init__(
        self,
        transport,
        on_raw: Optional[Callable[[str], None]] = None,
        on_state: Optional[Callable[[Dict], None]] = None,
        on_boot_mission: Optional[Callable[[str], None]] = None,
    ):
        self.transport = transport
        self.on_raw = on_raw
        self.on_state = on_state
        self.on_boot_mission = on_boot_mission
        self._boot_query_pending = False
        self._boot_capture_active = False
        self._boot_capture_lines = []

    def handle_line(self, line: str):
        if self.on_raw is not None:
            self.on_raw(line)
        event = classify_line(line)
        self._handle_boot_mission_line(event)
        if event.kind == "state" and self.on_state is not None:
            self.on_state(event.data)
        return event

    def _emit_boot_mission(self, text: str):
        if self.on_boot_mission is not None:
            self.on_boot_mission(text)

    def _handle_boot_mission_line(self, event):
        raw = event.raw
        if self._boot_query_pending and raw == "file not found.":
            self._boot_query_pending = False
            self._boot_capture_active = False
            self._boot_capture_lines = []
            self._emit_boot_mission(raw)
            return

        if self._boot_query_pending and raw == "---=== File Content ===---":
            self._boot_query_pending = False
            self._boot_capture_active = True
            self._boot_capture_lines = [raw]
            return

        if not self._boot_capture_active:
            return

        # Periodic T=105 telemetry can interleave with the firmware's textual
        # mission dump. Preserve mission text while excluding telemetry traffic.
        if event.message_type in (105, 1051):
            return

        self._boot_capture_lines.append(raw)
        if "reading file: boot.mission ends." in raw:
            text = "\n".join(self._boot_capture_lines)
            self._boot_capture_active = False
            self._boot_capture_lines = []
            self._emit_boot_mission(text)

    def query_state(self) -> bool:
        if not self.transport.connected:
            return False
        try:
            self.transport.write(make_state_query())
        except TransportDisconnected:
            return False
        return True

    def query_boot_mission(self) -> bool:
        """Request the firmware's read-only boot.mission dump (T=602)."""
        if not self.transport.connected:
            return False
        self._boot_query_pending = True
        self._boot_capture_active = False
        self._boot_capture_lines = []
        try:
            self.transport.write(make_boot_mission_query())
        except TransportDisconnected:
            self._boot_query_pending = False
            return False
        return True

    def send_single_joint_target(self, joint, rad, speed=0, acceleration=10) -> bool:
        if not self.transport.connected:
            return False
        try:
            self.transport.write(make_single_joint_command(joint, rad, speed=speed, acceleration=acceleration))
        except TransportDisconnected:
            return False
        return True

    def send_joint_target(self, base, shoulder, elbow, hand, speed=0, acceleration=10) -> bool:
        if not self.transport.connected:
            return False
        try:
            self.transport.write(make_joint_command(base, shoulder, elbow, hand, speed=speed, acceleration=acceleration))
        except TransportDisconnected:
            return False
        return True


def joint_state_values(state: Dict) -> Tuple[List[str], List[float]]:
    return (["base", "shoulder", "elbow", "hand"], [float(state["base_rad"]), float(state["shoulder_rad"]), float(state["elbow_rad"]), float(state["hand_rad"])])


def position_values(state: Dict) -> Tuple[float, float, float]:
    return float(state["x_m"]), float(state["y_m"]), float(state["z_m"])


def pose_values(state: Dict):
    x, y, z = position_values(state)
    return x, y, z, 0.0, 0.0, 0.0, 1.0
