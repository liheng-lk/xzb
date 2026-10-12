import math

JOINT_NAMES = (
    "base_link_to_link1",
    "link1_to_link2",
    "link2_to_link3",
    "link3_to_gripper_link",
)


def controller_to_ros(values):
    b, s, e, t = [float(v) for v in values]
    return (-b, -s, e, math.pi - t)


def ros_to_controller(values):
    base, shoulder, elbow, gripper = [float(v) for v in values]
    return (-base, -shoulder, elbow, math.pi - gripper)
