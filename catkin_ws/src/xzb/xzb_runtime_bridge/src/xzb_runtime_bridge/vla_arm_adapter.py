from dataclasses import dataclass
import math

CARTESIAN_DELTA = 0
JOINT = 1


class VLAActionRejected(ValueError):
    pass


@dataclass(frozen=True)
class ArmAction:
    mode: int
    values: tuple
    gripper: float = 0.0
    source: str = "vla"

    @classmethod
    def cartesian_delta(cls, dx, dy, dz, gripper, source="vla"):
        return cls(CARTESIAN_DELTA, (float(dx), float(dy), float(dz)), float(gripper), source)

    @classmethod
    def joint(cls, base, shoulder, elbow, gripper_joint, source="engineering"):
        return cls(JOINT, (float(base), float(shoulder), float(elbow), float(gripper_joint)), 0.0, source)


class VLAArmAdapter:
    def __init__(self, dispatcher=None, dry_run=True, hardware_enable=False, allow_joint_mode=False,
                 max_axis_delta=0.01, max_delta_norm=0.015):
        self.dispatcher = dispatcher
        self.dry_run = bool(dry_run)
        self.hardware_enable = bool(hardware_enable)
        self.allow_joint_mode = bool(allow_joint_mode)
        self.max_axis_delta = float(max_axis_delta)
        self.max_delta_norm = float(max_delta_norm)

    @staticmethod
    def _finite(values):
        vals = tuple(float(v) for v in values)
        if not all(math.isfinite(v) for v in vals):
            raise VLAActionRejected("action contains NaN or Inf")
        return vals

    def validate(self, action):
        if action.mode == CARTESIAN_DELTA:
            vals = self._finite(action.values)
            if len(vals) != 3:
                raise VLAActionRejected("Cartesian action must contain dx,dy,dz")
            if any(abs(v) > self.max_axis_delta + 1e-12 for v in vals):
                raise VLAActionRejected("Cartesian axis delta too large")
            if math.sqrt(sum(v*v for v in vals)) > self.max_delta_norm + 1e-12:
                raise VLAActionRejected("Cartesian delta norm too large")
            gripper = float(action.gripper)
            if not math.isfinite(gripper) or not 0.0 <= gripper <= 1.0:
                raise VLAActionRejected("gripper must be normalized to [0,1]")
            return ArmAction.cartesian_delta(*vals, gripper, source=action.source)
        if action.mode == JOINT:
            if not self.allow_joint_mode:
                raise VLAActionRejected("joint VLA mode disabled")
            vals = self._finite(action.values)
            if len(vals) != 4:
                raise VLAActionRejected("joint action must contain 4 values")
            limits = ((-3.1416,3.1416),(-1.5708,1.5708),(-1.0,3.1416),(0.0,1.5))
            for i, (value, (low, high)) in enumerate(zip(vals, limits)):
                if not low <= value <= high:
                    raise VLAActionRejected(f"joint {i} outside limits")
            return ArmAction(JOINT, vals, 0.0, action.source)
        raise VLAActionRejected("unknown VLA action mode")

    def process(self, action):
        checked = self.validate(action)
        if self.dry_run or not self.hardware_enable:
            return checked, False
        if self.dispatcher is None:
            raise VLAActionRejected("hardware dispatcher unavailable")
        if not bool(self.dispatcher(checked)):
            raise VLAActionRejected("arm gateway rejected action")
        return checked, True
