from typing import Callable, Dict, List, Optional, Tuple

from .protocol import classify_line, make_joint_command, make_state_query
from .serial_transport import TransportDisconnected


class ReadOnlyDriverCore:
    """Serial-facing core; historical name retained for compatibility."""
    def __init__(self, transport, on_raw: Optional[Callable[[str], None]] = None, on_state: Optional[Callable[[Dict], None]] = None):
        self.transport = transport
        self.on_raw = on_raw
        self.on_state = on_state

    def handle_line(self, line: str):
        if self.on_raw is not None:
            self.on_raw(line)
        event = classify_line(line)
        if event.kind == "state" and self.on_state is not None:
            self.on_state(event.data)
        return event

    def query_state(self) -> bool:
        if not self.transport.connected:
            return False
        try:
            self.transport.write(make_state_query())
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
