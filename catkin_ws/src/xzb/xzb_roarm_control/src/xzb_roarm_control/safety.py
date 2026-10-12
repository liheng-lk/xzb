import math


class SafetyError(ValueError):
    pass


class ArmSafety:
    HARD_LIMITS = (
        (-3.1416, 3.1416),
        (-1.5708, 1.5708),
        (-1.0, 3.1416),
        (0.0, 1.5),
    )

    def __init__(self, max_joint_step=0.05, max_cartesian_axis_step=0.01,
                 max_cartesian_norm=0.015, state_limit_margin=0.1):
        self.max_joint_step = float(max_joint_step)
        self.max_cartesian_axis_step = float(max_cartesian_axis_step)
        self.max_cartesian_norm = float(max_cartesian_norm)
        self.state_limit_margin = float(state_limit_margin)

    @staticmethod
    def _finite(values, label):
        vals = tuple(float(v) for v in values)
        if not all(math.isfinite(v) for v in vals):
            raise SafetyError(f"{label} contains NaN or Inf")
        return vals

    def validate_joint_state(self, values):
        vals = self._finite(values, "joint state")
        if len(vals) != 4:
            raise SafetyError("joint state must contain 4 values")
        for i, (value, (low, high)) in enumerate(zip(vals, self.HARD_LIMITS)):
            if value < low - self.state_limit_margin or value > high + self.state_limit_margin:
                raise SafetyError(f"joint state {i} outside plausible range")
        return vals

    def validate_joint_target(self, values):
        vals = self._finite(values, "joint target")
        if len(vals) != 4:
            raise SafetyError("joint target must contain 4 values")
        for i, (value, (low, high)) in enumerate(zip(vals, self.HARD_LIMITS)):
            if not low <= value <= high:
                raise SafetyError(f"joint target {i} outside [{low}, {high}]")
        return vals

    def validate_joint_delta(self, values):
        vals = self._finite(values, "joint delta")
        if len(vals) != 4:
            raise SafetyError("joint delta must contain 4 values")
        for i, value in enumerate(vals):
            if abs(value) > self.max_joint_step + 1e-12:
                raise SafetyError(f"joint delta {i} exceeds {self.max_joint_step} rad")
        return vals

    def validate_cartesian_delta(self, values):
        vals = self._finite(values, "Cartesian delta")
        if len(vals) != 3:
            raise SafetyError("Cartesian delta must contain 3 values")
        if any(abs(v) > self.max_cartesian_axis_step + 1e-12 for v in vals):
            raise SafetyError(f"Cartesian axis step exceeds {self.max_cartesian_axis_step} m")
        norm = math.sqrt(sum(v * v for v in vals))
        if norm > self.max_cartesian_norm + 1e-12:
            raise SafetyError(f"Cartesian step norm exceeds {self.max_cartesian_norm} m")
        return vals
