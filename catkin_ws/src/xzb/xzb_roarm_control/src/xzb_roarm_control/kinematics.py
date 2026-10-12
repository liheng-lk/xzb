from dataclasses import dataclass
import math
import numpy as np

JOINT_LIMITS_3 = np.array([
    [-3.1416, 3.1416],
    [-1.5708, 1.5708],
    [-1.0, 3.1416],
], dtype=float)


@dataclass(frozen=True)
class FKResult:
    position: tuple
    quaternion: tuple
    matrix: np.ndarray


@dataclass(frozen=True)
class IKResult:
    success: bool
    joints: tuple
    residual: float
    iterations: int


def _translation(x, y, z):
    t = np.eye(4, dtype=float)
    t[:3, 3] = (x, y, z)
    return t


def _rot_x(a):
    c, s = math.cos(a), math.sin(a)
    r = np.eye(4, dtype=float)
    r[:3, :3] = ((1, 0, 0), (0, c, -s), (0, s, c))
    return r


def _rot_y(a):
    c, s = math.cos(a), math.sin(a)
    r = np.eye(4, dtype=float)
    r[:3, :3] = ((c, 0, s), (0, 1, 0), (-s, 0, c))
    return r


def _rot_z(a):
    c, s = math.cos(a), math.sin(a)
    r = np.eye(4, dtype=float)
    r[:3, :3] = ((c, -s, 0), (s, c, 0), (0, 0, 1))
    return r


def _rpy(r, p, y):
    return _rot_z(y) @ _rot_y(p) @ _rot_x(r)


def _axis_rotation(axis, angle):
    ax = np.asarray(axis, dtype=float)
    ax = ax / np.linalg.norm(ax)
    x, y, z = ax
    c, s = math.cos(angle), math.sin(angle)
    C = 1.0 - c
    R = np.array([
        [x*x*C+c, x*y*C-z*s, x*z*C+y*s],
        [y*x*C+z*s, y*y*C+c, y*z*C-x*s],
        [z*x*C-y*s, z*y*C+x*s, z*z*C+c],
    ])
    t = np.eye(4, dtype=float)
    t[:3, :3] = R
    return t


def _quat_from_matrix(R):
    tr = float(np.trace(R))
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    else:
        i = int(np.argmax(np.diag(R)))
        if i == 0:
            S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            qw = (R[2, 1] - R[1, 2]) / S; qx = 0.25 * S
            qy = (R[0, 1] + R[1, 0]) / S; qz = (R[0, 2] + R[2, 0]) / S
        elif i == 1:
            S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            qw = (R[0, 2] - R[2, 0]) / S; qx = (R[0, 1] + R[1, 0]) / S
            qy = 0.25 * S; qz = (R[1, 2] + R[2, 1]) / S
        else:
            S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            qw = (R[1, 0] - R[0, 1]) / S; qx = (R[0, 2] + R[2, 0]) / S
            qy = (R[1, 2] + R[2, 1]) / S; qz = 0.25 * S
    q = np.array([qx, qy, qz, qw], dtype=float)
    q /= np.linalg.norm(q)
    return tuple(float(v) for v in q)


class RoArmKinematics:
    """3-DOF base_link -> hand_tcp FK and position-only numerical IK."""
    def __init__(self):
        self.limits = JOINT_LIMITS_3.copy()

    def _matrix(self, joints):
        q0, q1, q2 = [float(v) for v in joints]
        T = np.eye(4, dtype=float)
        T = T @ _translation(0.0100000008759151, 0, 0.123059270461044) @ _axis_rotation((0, 0, -1), q0)
        T = T @ _rpy(-math.pi/2, -math.pi/2, 0) @ _axis_rotation((0, 0, -1), q1)
        T = T @ _translation(0.236815132922094, 0.0300023995170449, 0) @ _rpy(0, 0, math.pi/2) @ _axis_rotation((0, 0, 1), q2)
        T = T @ _translation(0.002, -0.2802, 0) @ _rpy(-math.pi/2, 0, -math.pi/2)
        return T

    def forward(self, joints):
        if len(joints) != 3:
            raise ValueError("FK expects 3 positioning joints")
        q = np.asarray(joints, dtype=float)
        if not np.all(np.isfinite(q)):
            raise ValueError("FK joints must be finite")
        T = self._matrix(q)
        return FKResult(tuple(float(v) for v in T[:3, 3]), _quat_from_matrix(T[:3, :3]), T)

    def inverse(self, target_xyz, seed=(0.0, 0.0, 1.5708), tolerance=1e-4,
                max_iterations=120, damping=2e-3):
        target = np.asarray(target_xyz, dtype=float)
        if target.shape != (3,) or not np.all(np.isfinite(target)):
            raise ValueError("IK target must contain 3 finite values")
        q = np.asarray(seed, dtype=float).copy()
        if q.shape != (3,) or not np.all(np.isfinite(q)):
            raise ValueError("IK seed must contain 3 finite values")
        q = np.clip(q, self.limits[:, 0], self.limits[:, 1])
        eps = 1e-5
        for iteration in range(1, max_iterations + 1):
            p = np.asarray(self.forward(q).position)
            err = target - p
            residual = float(np.linalg.norm(err))
            if residual <= tolerance:
                return IKResult(True, tuple(float(v) for v in q), residual, iteration)
            J = np.zeros((3, 3), dtype=float)
            for j in range(3):
                qp = q.copy(); qm = q.copy()
                qp[j] += eps; qm[j] -= eps
                J[:, j] = (np.asarray(self.forward(qp).position) - np.asarray(self.forward(qm).position)) / (2 * eps)
            A = J @ J.T + (damping * damping) * np.eye(3)
            dq = J.T @ np.linalg.solve(A, err)
            max_abs = float(np.max(np.abs(dq)))
            if max_abs > 0.2:
                dq *= 0.2 / max_abs
            q = np.clip(q + dq, self.limits[:, 0], self.limits[:, 1])
        residual = float(np.linalg.norm(target - np.asarray(self.forward(q).position)))
        return IKResult(False, tuple(float(v) for v in q), residual, max_iterations)
