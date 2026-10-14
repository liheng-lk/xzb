import json
import math
from dataclasses import dataclass
from typing import Any, Dict, Optional

STATE_TYPE = 1051
STATE_QUERY_TYPE = 105
BOOT_MISSION_QUERY_TYPE = 602
_STATE_FIELDS = ("x", "y", "z", "b", "s", "e", "t", "torB", "torS", "torE", "torH")


@dataclass(frozen=True)
class ProtocolEvent:
    kind: str
    raw: str
    message_type: Optional[int] = None
    data: Optional[Dict[str, Any]] = None
    error: Optional[str] = None


def make_state_query() -> bytes:
    return b'{"T":105}\n'


def make_boot_mission_query() -> bytes:
    """Return the only allowed boot-mission diagnostic request.

    T=602 is read-only in the Waveshare firmware: it prints boot.mission
    content and does not alter PID, torque, calibration, or motion targets.
    """
    return b'{"T":602}\n'


def _finite_number(value, name: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise ValueError(f"{name} must be numeric")
    if not math.isfinite(number):
        raise ValueError(f"{name} must be finite")
    return number


def make_single_joint_command(joint, rad, speed=0, acceleration=10) -> bytes:
    if isinstance(joint, bool):
        raise ValueError("joint must be one of 1,2,3,4")
    try:
        joint_number = int(joint)
        joint_value = float(joint)
    except (TypeError, ValueError):
        raise ValueError("joint must be one of 1,2,3,4")
    if joint_value != joint_number or joint_number not in (1, 2, 3, 4):
        raise ValueError("joint must be one of 1,2,3,4")
    payload = {
        "T": 101,
        "joint": joint_number,
        "rad": _finite_number(rad, "rad"),
        "spd": _finite_number(speed, "speed"),
        "acc": _finite_number(acceleration, "acceleration"),
    }
    return (json.dumps(payload, separators=(",", ":")) + "\n").encode("utf-8")


def make_joint_command(base, shoulder, elbow, hand, speed=0, acceleration=10) -> bytes:
    payload = {
        "T": 102,
        "base": _finite_number(base, "base"),
        "shoulder": _finite_number(shoulder, "shoulder"),
        "elbow": _finite_number(elbow, "elbow"),
        "hand": _finite_number(hand, "hand"),
        "spd": _finite_number(speed, "speed"),
        "acc": _finite_number(acceleration, "acceleration"),
    }
    return (json.dumps(payload, separators=(",", ":")) + "\n").encode("utf-8")


def classify_line(line: str) -> ProtocolEvent:
    raw = line.strip()
    if not raw:
        return ProtocolEvent(kind="empty", raw=raw)

    try:
        payload = json.loads(raw)
    except (TypeError, ValueError):
        return ProtocolEvent(kind="firmware_log", raw=raw)

    if not isinstance(payload, dict):
        return ProtocolEvent(kind="protocol", raw=raw, data={"value": payload})

    message_type = payload.get("T")
    if message_type != STATE_TYPE:
        return ProtocolEvent(kind="protocol", raw=raw, message_type=message_type, data=payload)

    missing = [field for field in _STATE_FIELDS if field not in payload]
    if missing:
        return ProtocolEvent(kind="invalid_state", raw=raw, message_type=STATE_TYPE, data=payload, error="missing fields: " + ", ".join(missing))

    try:
        state = {
            "x_m": float(payload["x"]) / 1000.0,
            "y_m": float(payload["y"]) / 1000.0,
            "z_m": float(payload["z"]) / 1000.0,
            "base_rad": float(payload["b"]),
            "shoulder_rad": float(payload["s"]),
            "elbow_rad": float(payload["e"]),
            "hand_rad": float(payload["t"]),
            "torB": payload["torB"],
            "torS": payload["torS"],
            "torE": payload["torE"],
            "torH": payload["torH"],
        }
    except (TypeError, ValueError) as exc:
        return ProtocolEvent(kind="invalid_state", raw=raw, message_type=STATE_TYPE, data=payload, error="state position/joint fields must be numeric: " + str(exc))

    return ProtocolEvent(kind="state", raw=raw, message_type=STATE_TYPE, data=state)
