import math
from .vla_arm_adapter import ArmAction, VLAActionRejected


class SmolVLAActionMapper:
    def __init__(self, mode="cartesian_delta"):
        if mode not in ("cartesian_delta", "joint"):
            raise ValueError("unsupported action mode")
        self.mode = mode

    def map(self, vector):
        vals = tuple(float(v) for v in vector)
        if len(vals) != 4 or not all(math.isfinite(v) for v in vals):
            raise VLAActionRejected("SmolVLA action must be four finite values")
        if self.mode == "cartesian_delta":
            return ArmAction.cartesian_delta(vals[0], vals[1], vals[2], vals[3], source="smolvla")
        return ArmAction.joint(*vals, source="smolvla")


class SmolVLAPredictorAdapter:
    """Dependency-injected predictor wrapper; importing ROS does not require LeRobot/Torch."""
    def __init__(self, predictor, mapper=None):
        if not callable(predictor):
            raise TypeError("predictor must be callable")
        self.predictor = predictor
        self.mapper = mapper or SmolVLAActionMapper()

    def predict(self, observation):
        return self.mapper.map(self.predictor(observation))
