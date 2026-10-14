from .driver_core import ReadOnlyDriverCore, joint_state_values, pose_values, position_values
from .serial_transport import SerialTransport


def _load_ros_dependencies():
    import rospy
    from geometry_msgs.msg import Point, Pose
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, String
    from xzb_interfaces.srv import (
        DriverJointCommand, DriverJointCommandResponse,
        DriverSingleJointCommand, DriverSingleJointCommandResponse,
    )
    return rospy, {
        "Bool": Bool, "String": String, "JointState": JointState, "Point": Point, "Pose": Pose,
        "DriverJointCommand": DriverJointCommand, "DriverJointCommandResponse": DriverJointCommandResponse,
        "DriverSingleJointCommand": DriverSingleJointCommand, "DriverSingleJointCommandResponse": DriverSingleJointCommandResponse,
    }


class RoArmRosNode:
    def __init__(self, rospy_module=None, message_types=None, transport_factory=SerialTransport):
        if rospy_module is None or message_types is None:
            loaded_rospy, loaded_types = _load_ros_dependencies()
            rospy_module = rospy_module or loaded_rospy
            message_types = message_types or loaded_types
        self.rospy = rospy_module
        self.message_types = message_types
        self.port = self.rospy.get_param("~port", "/dev/ttyUSB0")
        self.baud_rate = int(self.rospy.get_param("~baud_rate", 115200))
        self.query_rate = float(self.rospy.get_param("~query_rate", 2.0))
        self.reconnect_interval = float(self.rospy.get_param("~reconnect_interval", 1.0))
        self.serial_timeout = float(self.rospy.get_param("~serial_timeout", 0.2))
        self.joint_frame_id = self.rospy.get_param("~joint_frame_id", "roarm_base_link")
        self.command_enable = bool(self.rospy.get_param("~command_enable", False))
        if self.query_rate <= 0.0:
            self.rospy.logwarn("~query_rate must be > 0; using 2.0 Hz")
            self.query_rate = 2.0

        self.connected_pub = self.rospy.Publisher("/roarm/connected", self.message_types["Bool"], queue_size=1, latch=True)
        self.joint_pub = self.rospy.Publisher("/roarm/joint_states", self.message_types["JointState"], queue_size=10)
        self.controller_joint_pub = self.rospy.Publisher("/roarm/controller_joint_states", self.message_types["JointState"], queue_size=10)
        self.pose_pub = self.rospy.Publisher("/roarm/pose", self.message_types["Pose"], queue_size=10)
        self.position_pub = None
        if "Point" in self.message_types:
            self.position_pub = self.rospy.Publisher("/roarm/position", self.message_types["Point"], queue_size=10)
        self.raw_pub = self.rospy.Publisher("/roarm/raw_response", self.message_types["String"], queue_size=50)
        self.tx_pub = self.rospy.Publisher("/roarm/raw_tx", self.message_types["String"], queue_size=50)
        self.connected_pub.publish(self.message_types["Bool"](data=False))

        self.transport = transport_factory(self.port, baud_rate=self.baud_rate, timeout=self.serial_timeout, reconnect_interval=self.reconnect_interval)
        self.core = ReadOnlyDriverCore(self.transport, on_raw=self._publish_raw, on_state=self._publish_state)
        self.transport.on_line = self._on_line
        self.transport.on_connection = self._on_connection
        self.transport.on_write = self._publish_tx
        self.command_service = None
        self.single_command_service = None
        if "DriverJointCommand" in self.message_types and hasattr(self.rospy, "Service"):
            self.command_service = self.rospy.Service("/roarm/driver/joint_command", self.message_types["DriverJointCommand"], self._handle_joint_command)
        if "DriverSingleJointCommand" in self.message_types and hasattr(self.rospy, "Service"):
            self.single_command_service = self.rospy.Service("/roarm/driver/single_joint_command", self.message_types["DriverSingleJointCommand"], self._handle_single_joint_command)
        self.transport.start()
        self.query_timer = self.rospy.Timer(self.rospy.Duration(1.0 / self.query_rate), self._query_timer)
        self.rospy.on_shutdown(self.shutdown)
        mode = "CONTROL-ENABLED" if self.command_enable else "READ-ONLY"
        self.rospy.loginfo("RoArm driver started: port=%s baud=%d query_rate=%.2fHz mode=%s", self.port, self.baud_rate, self.query_rate, mode)

    def _publish_raw(self, line):
        self.raw_pub.publish(self.message_types["String"](data=line))

    def _publish_tx(self, payload):
        if isinstance(payload, bytes):
            text = payload.decode("utf-8", errors="replace")
        else:
            text = str(payload)
        self.tx_pub.publish(self.message_types["String"](data=text.rstrip("\r\n")))

    def _publish_state(self, state):
        names, positions = joint_state_values(state)
        joint_msg = self.message_types["JointState"]()
        joint_msg.header.stamp = self.rospy.Time.now()
        joint_msg.header.frame_id = self.joint_frame_id
        joint_msg.name = names
        joint_msg.position = positions
        joint_msg.velocity = []
        joint_msg.effort = []
        self.joint_pub.publish(joint_msg)
        self.controller_joint_pub.publish(joint_msg)

        x, y, z, qx, qy, qz, qw = pose_values(state)
        pose_msg = self.message_types["Pose"]()
        pose_msg.position.x = x; pose_msg.position.y = y; pose_msg.position.z = z
        pose_msg.orientation.x = qx; pose_msg.orientation.y = qy; pose_msg.orientation.z = qz; pose_msg.orientation.w = qw
        self.pose_pub.publish(pose_msg)
        if self.position_pub is not None:
            px, py, pz = position_values(state)
            point = self.message_types["Point"](); point.x = px; point.y = py; point.z = pz
            self.position_pub.publish(point)

    def _command_disabled_response(self, response):
        response.accepted = False
        response.message = "driver command path disabled; enable only behind xzb_roarm_control"
        return response

    def _handle_single_joint_command(self, req):
        response = self.message_types["DriverSingleJointCommandResponse"]()
        if not self.command_enable:
            return self._command_disabled_response(response)
        try:
            accepted = self.core.send_single_joint_target(req.joint, req.rad, speed=req.speed, acceleration=req.acceleration)
        except ValueError as exc:
            response.accepted = False; response.message = str(exc); return response
        response.accepted = bool(accepted)
        response.message = "accepted" if accepted else "serial disconnected"
        return response

    def _handle_joint_command(self, req):
        response = self.message_types["DriverJointCommandResponse"]()
        if not self.command_enable:
            return self._command_disabled_response(response)
        try:
            accepted = self.core.send_joint_target(req.base, req.shoulder, req.elbow, req.hand, speed=req.speed, acceleration=req.acceleration)
        except ValueError as exc:
            response.accepted = False; response.message = str(exc); return response
        response.accepted = bool(accepted)
        response.message = "accepted" if accepted else "serial disconnected"
        return response

    def _on_line(self, line):
        event = self.core.handle_line(line)
        if event.kind == "invalid_state":
            self.rospy.logwarn_throttle(5.0, "Invalid RoArm state: %s", event.error)
        elif event.kind == "firmware_log":
            self.rospy.logdebug("RoArm firmware: %s", event.raw)

    def _on_connection(self, connected):
        self.connected_pub.publish(self.message_types["Bool"](data=bool(connected)))
        if connected: self.rospy.loginfo("RoArm serial connected: %s", self.port)
        else: self.rospy.logwarn("RoArm serial disconnected: %s", self.port)

    def _query_timer(self, _event):
        self.core.query_state()

    def shutdown(self):
        self.transport.stop()


def main():
    rospy, message_types = _load_ros_dependencies()
    rospy.init_node("xzb_roarm_driver")
    RoArmRosNode(rospy_module=rospy, message_types=message_types)
    rospy.spin()
