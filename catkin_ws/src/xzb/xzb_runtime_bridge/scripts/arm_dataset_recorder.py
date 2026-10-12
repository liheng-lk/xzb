#!/usr/bin/env python3
import json
import os
import threading
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, JointState
from xzb_interfaces.msg import ArmAction, ArmStatus


class Recorder:
    def __init__(self):
        path = os.path.expanduser(rospy.get_param("~output", "~/xzb_arm_dataset.jsonl"))
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        self.file = open(path, "a", buffering=1)
        self.lock = threading.Lock()
        self.joints = None; self.pose = None; self.status = None; self.action = None
        self.rgb_stamp = None; self.depth_stamp = None
        rospy.Subscriber("/joint_states", JointState, self._joints, queue_size=10)
        rospy.Subscriber("/roarm/tool_pose", PoseStamped, self._pose, queue_size=10)
        rospy.Subscriber("/roarm/status", ArmStatus, self._status, queue_size=10)
        rospy.Subscriber("/vla/arm_action_checked", ArmAction, self._action, queue_size=10)
        rospy.Subscriber(rospy.get_param("~rgb_topic", "/arm_camera/color/image_raw"), Image, lambda m: self._image_stamp("rgb", m), queue_size=1)
        rospy.Subscriber(rospy.get_param("~depth_topic", "/arm_camera/aligned_depth_to_color/image_raw"), Image, lambda m: self._image_stamp("depth", m), queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / float(rospy.get_param("~rate", 10.0))), self._record)
        rospy.on_shutdown(self.file.close)

    def _joints(self,m): self.joints={"name":list(m.name),"position":list(m.position)}
    def _pose(self,m): self.pose={"frame":m.header.frame_id,"position":[m.pose.position.x,m.pose.position.y,m.pose.position.z],"quaternion":[m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z,m.pose.orientation.w]}
    def _status(self,m): self.status={"connected":m.connected,"motion_enabled":m.motion_enabled,"stale":m.stale,"state":m.state,"detail":m.detail}
    def _action(self,m): self.action={"mode":int(m.mode),"delta":[m.cartesian_delta.x,m.cartesian_delta.y,m.cartesian_delta.z],"joint_target":list(m.joint_target),"gripper":m.gripper,"source":m.source}
    def _image_stamp(self,kind,m): setattr(self,kind+"_stamp",m.header.stamp.to_sec())
    def _record(self,_event):
        row={"timestamp":rospy.Time.now().to_sec(),"joint_state":self.joints,"tool_pose":self.pose,"arm_status":self.status,"checked_action":self.action,"rgb_stamp":self.rgb_stamp,"depth_stamp":self.depth_stamp}
        with self.lock: self.file.write(json.dumps(row,separators=(",",":"))+"\n")


def main():
    rospy.init_node("arm_dataset_recorder")
    Recorder(); rospy.spin()

if __name__ == "__main__": main()
