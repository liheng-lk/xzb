from dataclasses import dataclass
import math
import time

from .kinematics import RoArmKinematics
from .safety import ArmSafety


class ControlRejected(RuntimeError):
    pass


@dataclass(frozen=True)
class CommandResult:
    accepted: bool
    target: tuple
    message: str


class ArmControlCore:
    def __init__(self, dispatcher, single_dispatcher=None, safety=None, kinematics=None, state_timeout=1.0, clock=None):
        self.dispatcher = dispatcher
        self.single_dispatcher = single_dispatcher
        self.safety = safety or ArmSafety()
        self.kinematics = kinematics or RoArmKinematics()
        self.state_timeout = float(state_timeout)
        self.clock = clock or time.monotonic
        self.connected = False
        self.motion_enabled = False
        self.current_joints = None
        self.last_state_time = None

    def set_connected(self, connected):
        self.connected = bool(connected)
        if not self.connected:
            self.motion_enabled = False

    def set_motion_enabled(self, enabled):
        if enabled and (not self.connected or self.current_joints is None or self.is_stale()):
            raise ControlRejected("cannot enable motion without fresh connected state")
        self.motion_enabled = bool(enabled)
        return self.motion_enabled

    def update_joint_state(self, values):
        self.current_joints = self.safety.validate_joint_state(values)
        self.last_state_time = self.clock()

    def is_stale(self):
        return self.last_state_time is None or (self.clock() - self.last_state_time) > self.state_timeout

    def _require_motion(self):
        if not self.connected:
            raise ControlRejected("arm disconnected")
        if self.current_joints is None:
            raise ControlRejected("no arm state")
        if self.is_stale():
            raise ControlRejected("arm state stale")
        if not self.motion_enabled:
            raise ControlRejected("motion disabled")

    @staticmethod
    def _validate_speed_scale(speed_scale):
        speed_scale = float(speed_scale)
        if not math.isfinite(speed_scale) or not (0.0 < speed_scale <= 1.0):
            raise ControlRejected("speed_scale must be in (0,1]")
        return speed_scale

    def _validate_target_from_current(self, target):
        target = self.safety.validate_joint_target(target)
        self.safety.validate_joint_delta(tuple(t - c for t, c in zip(target, self.current_joints)))
        return tuple(target)

    def _dispatch_single_target(self, joint_index, target, speed_scale=0.1):
        self._require_motion()
        target = self._validate_target_from_current(target)
        speed_scale = self._validate_speed_scale(speed_scale)
        if self.single_dispatcher is None:
            return self._dispatch_target(target, speed_scale)
        if not bool(self.single_dispatcher(int(joint_index), target[int(joint_index)], speed_scale)):
            raise ControlRejected("driver rejected single-joint command")
        return CommandResult(True, target, "accepted")

    def _dispatch_target(self, target, speed_scale=0.1):
        self._require_motion()
        target = self._validate_target_from_current(target)
        speed_scale = self._validate_speed_scale(speed_scale)
        if not bool(self.dispatcher(target, speed_scale)):
            raise ControlRejected("driver rejected command")
        return CommandResult(True, target, "accepted")

    def move_joints(self, values, relative=True, speed_scale=0.1):
        self._require_motion()
        vals = tuple(float(v) for v in values)
        if len(vals) != 4:
            raise ControlRejected("joint request must contain 4 values")
        if relative:
            self.safety.validate_joint_delta(vals)
            target = tuple(c + d for c, d in zip(self.current_joints, vals))
            changed = [index for index, delta in enumerate(vals) if abs(delta) > 1e-12]
            if len(changed) == 1 and self.single_dispatcher is not None:
                return self._dispatch_single_target(changed[0], target, speed_scale)
        else:
            target = vals
        return self._dispatch_target(target, speed_scale)

    def move_cartesian(self, target_xyz, relative=True, speed_scale=0.1):
        self._require_motion()
        if relative:
            delta = self.safety.validate_cartesian_delta(target_xyz)
            current_xyz = self.kinematics.forward(self.current_joints[:3]).position
            goal = tuple(c + d for c, d in zip(current_xyz, delta))
        else:
            goal = tuple(float(v) for v in target_xyz)
            if len(goal) != 3 or not all(math.isfinite(v) for v in goal):
                raise ControlRejected("Cartesian target must contain 3 finite values")
        ik = self.kinematics.inverse(goal, seed=self.current_joints[:3])
        if not ik.success:
            raise ControlRejected(f"IK failed residual={ik.residual:.6f} m")
        target = tuple(ik.joints) + (self.current_joints[3],)
        return self._dispatch_target(target, speed_scale)

    def set_gripper(self, opening, speed_scale=0.1):
        self._require_motion()
        opening = float(opening)
        if not math.isfinite(opening) or not 0.0 <= opening <= 1.0:
            raise ControlRejected("gripper opening must be in [0,1]")
        target = list(self.current_joints)
        target[3] = opening * 1.5
        return self._dispatch_target(tuple(target), speed_scale)

    def compute_fk(self, joints3):
        return self.kinematics.forward(joints3)

    def solve_ik(self, target_xyz, seed=None):
        if seed is None:
            seed = self.current_joints[:3] if self.current_joints else (0.0, 0.0, 1.5708)
        return self.kinematics.inverse(target_xyz, seed=seed)

    def soft_stop(self):
        self.motion_enabled = False
        return "motion gate disabled; this is not an electrical emergency stop"
