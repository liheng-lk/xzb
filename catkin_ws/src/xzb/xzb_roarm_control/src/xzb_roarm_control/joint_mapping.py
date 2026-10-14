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


def ros_joint_to_controller(index, value):
    index = int(index)
    value = float(value)
    if index == 0:
        return -value
    if index == 1:
        return -value
    if index == 2:
        return value
    if index == 3:
        return math.pi - value
    raise ValueError("joint index must be in [0,3]")


def ros_to_controller(values):
    base, shoulder, elbow, gripper = [float(v) for v in values]
    return (
        ros_joint_to_controller(0, base),
        ros_joint_to_controller(1, shoulder),
        ros_joint_to_controller(2, elbow),
        ros_joint_to_controller(3, gripper),
    )
