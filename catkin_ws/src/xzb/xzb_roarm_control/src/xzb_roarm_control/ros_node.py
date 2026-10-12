import math

from .control_core import ArmControlCore, ControlRejected
from .joint_mapping import JOINT_NAMES, controller_to_ros, ros_to_controller
from .safety import ArmSafety, SafetyError


def _load_ros():
    import rospy
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool
    from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
    from xzb_interfaces.msg import ArmStatus
    from xzb_interfaces.srv import (
        ComputeArmFK, ComputeArmFKResponse, DriverJointCommand,
        MoveArmCartesian, MoveArmCartesianResponse, MoveArmJoints, MoveArmJointsResponse,
        SetArmGripper, SetArmGripperResponse, SolveArmIK, SolveArmIKResponse,
    )
    return rospy, locals()


class RoArmControlNode:
    def __init__(self):
        rospy, types = _load_ros()
        self.rospy = rospy
        self.types = types
        safety = ArmSafety(
            max_joint_step=rospy.get_param("~max_joint_step", 0.05),
            max_cartesian_axis_step=rospy.get_param("~max_cartesian_axis_step", 0.01),
            max_cartesian_norm=rospy.get_param("~max_cartesian_norm", 0.015),
            state_limit_margin=rospy.get_param("~state_limit_margin", 0.1),
        )
        self.driver = rospy.ServiceProxy("/roarm/driver/joint_command", types["DriverJointCommand"])
        self.core = ArmControlCore(self._dispatch, safety=safety, state_timeout=rospy.get_param("~state_timeout", 1.0))
        self.joint_pub = rospy.Publisher("/joint_states", types["JointState"], queue_size=10)
        self.tool_pose_pub = rospy.Publisher("/roarm/tool_pose", types["PoseStamped"], queue_size=10)
        self.status_pub = rospy.Publisher("/roarm/status", types["ArmStatus"], queue_size=5, latch=True)
        rospy.Subscriber("/roarm/connected", types["Bool"], self._connected_cb, queue_size=5)
        rospy.Subscriber("/roarm/controller_joint_states", types["JointState"], self._controller_state_cb, queue_size=10)
        rospy.Service("/roarm/motion_enable", types["SetBool"], self._enable_cb)
        rospy.Service("/roarm/soft_stop", types["Trigger"], self._stop_cb)
        rospy.Service("/roarm/move_joints", types["MoveArmJoints"], self._move_joints_cb)
        rospy.Service("/roarm/move_cartesian", types["MoveArmCartesian"], self._move_cartesian_cb)
        rospy.Service("/roarm/set_gripper", types["SetArmGripper"], self._gripper_cb)
        rospy.Service("/roarm/compute_fk", types["ComputeArmFK"], self._fk_cb)
        rospy.Service("/roarm/solve_ik", types["SolveArmIK"], self._ik_cb)
        self._publish_status("idle", "motion disabled at startup")

    def _dispatch(self, target, speed_scale):
        c = ros_to_controller(target)
        # Protocol spd remains 0 until its physical scale is characterized.
        response = self.driver(base=c[0], shoulder=c[1], elbow=c[2], hand=c[3], speed=0.0, acceleration=10.0)
        return response.accepted

    def _publish_status(self, state="idle", detail=""):
        msg = self.types["ArmStatus"]()
        msg.header.stamp = self.rospy.Time.now()
        msg.connected = self.core.connected
        msg.motion_enabled = self.core.motion_enabled
        msg.stale = self.core.is_stale()
        msg.state = state
        msg.detail = detail
        self.status_pub.publish(msg)

    def _connected_cb(self, msg):
        self.core.set_connected(msg.data)
        self._publish_status("idle" if msg.data else "disconnected", "")

    def _controller_state_cb(self, msg):
        if len(msg.position) < 4:
            self.rospy.logwarn_throttle(5.0, "RoArm controller state has fewer than 4 positions")
            return
        try:
            model = controller_to_ros(msg.position[:4])
            self.core.update_joint_state(model)
        except (SafetyError, ValueError) as exc:
            self.rospy.logwarn_throttle(5.0, "Rejected RoArm state: %s", exc)
            return
        out = self.types["JointState"]()
        out.header.stamp = self.rospy.Time.now()
        out.header.frame_id = "base_link"
        out.name = list(JOINT_NAMES)
        out.position = list(model)
        out.velocity = []
        out.effort = []
        self.joint_pub.publish(out)
        fk = self.core.compute_fk(model[:3])
        pose = self.types["PoseStamped"]()
        pose.header.stamp = out.header.stamp
        pose.header.frame_id = "base_link"
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = fk.position
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = fk.quaternion
        self.tool_pose_pub.publish(pose)
        self._publish_status("ready" if not self.core.motion_enabled else "armed", "fresh state")

    def _enable_cb(self, req):
        try:
            self.core.set_motion_enabled(req.data)
            self._publish_status("armed" if req.data else "idle", "motion gate changed")
            return self.types["SetBoolResponse"](success=True, message="motion enabled" if req.data else "motion disabled")
        except (ControlRejected, SafetyError) as exc:
            return self.types["SetBoolResponse"](success=False, message=str(exc))

    def _stop_cb(self, _req):
        text = self.core.soft_stop()
        self._publish_status("stopped", text)
        return self.types["TriggerResponse"](success=True, message=text)

    def _move_joints_cb(self, req):
        response = self.types["MoveArmJointsResponse"]()
        try:
            result = self.core.move_joints(req.values, req.relative, req.speed_scale)
            response.accepted = result.accepted; response.message = result.message; response.target = list(result.target)
        except (ControlRejected, SafetyError, ValueError) as exc:
            response.accepted = False; response.message = str(exc)
        return response

    def _move_cartesian_cb(self, req):
        response = self.types["MoveArmCartesianResponse"]()
        try:
            result = self.core.move_cartesian((req.target.x, req.target.y, req.target.z), req.relative, req.speed_scale)
            response.accepted = result.accepted; response.message = result.message; response.joint_target = list(result.target)
        except (ControlRejected, SafetyError, ValueError) as exc:
            response.accepted = False; response.message = str(exc)
        return response

    def _gripper_cb(self, req):
        response = self.types["SetArmGripperResponse"]()
        try:
            result = self.core.set_gripper(req.opening, req.speed_scale)
            response.accepted = result.accepted; response.message = result.message; response.joint_target = result.target[3]
        except (ControlRejected, SafetyError, ValueError) as exc:
            response.accepted = False; response.message = str(exc)
        return response

    def _fk_cb(self, req):
        response = self.types["ComputeArmFKResponse"]()
        try:
            fk = self.core.compute_fk(req.joints)
            response.success = True; response.message = "ok"
            response.pose.position.x, response.pose.position.y, response.pose.position.z = fk.position
            response.pose.orientation.x, response.pose.orientation.y, response.pose.orientation.z, response.pose.orientation.w = fk.quaternion
        except Exception as exc:
            response.success = False; response.message = str(exc)
        return response

    def _ik_cb(self, req):
        response = self.types["SolveArmIKResponse"]()
        try:
            ik = self.core.solve_ik((req.target.x, req.target.y, req.target.z), seed=req.seed)
            response.success = ik.success; response.joints = list(ik.joints); response.residual = ik.residual
            response.message = "ok" if ik.success else "unreachable or solver did not converge"
        except Exception as exc:
            response.success = False; response.message = str(exc); response.residual = math.inf
        return response


def main():
    rospy, _ = _load_ros()
    rospy.init_node("xzb_roarm_control")
    RoArmControlNode()
    rospy.spin()
