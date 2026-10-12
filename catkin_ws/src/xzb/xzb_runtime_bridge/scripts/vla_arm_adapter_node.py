#!/usr/bin/env python3
import json
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String
from xzb_interfaces.msg import ArmAction as RosArmAction
from xzb_interfaces.srv import MoveArmCartesian, MoveArmJoints, SetArmGripper
from xzb_runtime_bridge.vla_arm_adapter import ArmAction, CARTESIAN_DELTA, JOINT, VLAActionRejected, VLAArmAdapter


class VLAArmAdapterNode:
    def __init__(self):
        self.move_cart = rospy.ServiceProxy("/roarm/move_cartesian", MoveArmCartesian)
        self.move_joints = rospy.ServiceProxy("/roarm/move_joints", MoveArmJoints)
        self.set_gripper = rospy.ServiceProxy("/roarm/set_gripper", SetArmGripper)
        self.adapter = VLAArmAdapter(
            dispatcher=self._dispatch,
            dry_run=rospy.get_param("~dry_run", True),
            hardware_enable=rospy.get_param("~hardware_enable", False),
            allow_joint_mode=rospy.get_param("~allow_joint_mode", False),
            max_axis_delta=rospy.get_param("~max_axis_delta", 0.01),
            max_delta_norm=rospy.get_param("~max_delta_norm", 0.015),
        )
        self.checked_pub = rospy.Publisher("/vla/arm_action_checked", RosArmAction, queue_size=10)
        self.status_pub = rospy.Publisher("/vla/arm_status", String, queue_size=10)
        rospy.Subscriber("/vla/arm_action", RosArmAction, self._action_cb, queue_size=10)
        rospy.loginfo("VLA arm adapter: dry_run=%s hardware_enable=%s joint_mode=%s", self.adapter.dry_run, self.adapter.hardware_enable, self.adapter.allow_joint_mode)

    def _from_ros(self, msg):
        if msg.mode == RosArmAction.CARTESIAN_DELTA:
            return ArmAction.cartesian_delta(msg.cartesian_delta.x, msg.cartesian_delta.y, msg.cartesian_delta.z, msg.gripper, source=msg.source or "ros")
        if msg.mode == RosArmAction.JOINT:
            return ArmAction(JOINT, tuple(msg.joint_target), 0.0, msg.source or "ros")
        raise VLAActionRejected("unknown ROS ArmAction mode")

    def _to_ros(self, action):
        msg = RosArmAction(); msg.header.stamp = rospy.Time.now(); msg.mode = action.mode; msg.source = action.source
        if action.mode == CARTESIAN_DELTA:
            msg.cartesian_delta.x, msg.cartesian_delta.y, msg.cartesian_delta.z = action.values; msg.gripper = action.gripper
        else:
            msg.joint_target = list(action.values)
        return msg

    def _dispatch(self, action):
        if action.mode == CARTESIAN_DELTA:
            p = Point(*action.values)
            motion = self.move_cart(p, True, 0.1)
            if not motion.accepted:
                return False
            grip = self.set_gripper(action.gripper, 0.1)
            return bool(grip.accepted)
        response = self.move_joints(list(action.values), False, 0.1)
        return bool(response.accepted)

    def _action_cb(self, msg):
        try:
            checked, sent = self.adapter.process(self._from_ros(msg))
            self.checked_pub.publish(self._to_ros(checked))
            self.status_pub.publish(String(data=json.dumps({"accepted": True, "hardware_sent": sent, "dry_run": self.adapter.dry_run})))
        except Exception as exc:
            self.status_pub.publish(String(data=json.dumps({"accepted": False, "error": str(exc)})))
            rospy.logwarn_throttle(1.0, "VLA arm action rejected: %s", exc)


def main():
    rospy.init_node("vla_arm_adapter")
    VLAArmAdapterNode()
    rospy.spin()

if __name__ == "__main__": main()
