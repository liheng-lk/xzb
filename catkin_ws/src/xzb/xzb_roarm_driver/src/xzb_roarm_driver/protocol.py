import json
from dataclasses import dataclass
from typing import Any, Dict, Optional


STATE_TYPE = 1051
STATE_QUERY_TYPE = 105
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
        return ProtocolEvent(
            kind="protocol",
            raw=raw,
            message_type=message_type,
            data=payload,
        )

    missing = [field for field in _STATE_FIELDS if field not in payload]
    if missing:
        return ProtocolEvent(
            kind="invalid_state",
            raw=raw,
            message_type=STATE_TYPE,
            data=payload,
            error="missing fields: " + ", ".join(missing),
        )

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
    return ProtocolEvent(
        kind="state",
        raw=raw,
        message_type=STATE_TYPE,
        data=state,
    )
