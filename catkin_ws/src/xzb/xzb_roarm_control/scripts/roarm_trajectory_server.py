#!/usr/bin/env python3
import math
import threading
import time
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryResult
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from xzb_interfaces.srv import MoveArmJoints

ARM_JOINTS = ["base_link_to_link1", "link1_to_link2", "link2_to_link3"]


class TrajectoryServer:
    def __init__(self):
        self.state = None
        self.lock = threading.Lock()
        self.tolerance = float(rospy.get_param("~goal_tolerance", 0.03))
        self.timeout = float(rospy.get_param("~point_timeout", 3.0))
        self.move = rospy.ServiceProxy("/roarm/move_joints", MoveArmJoints)
        self.stop = rospy.ServiceProxy("/roarm/soft_stop", Trigger)
        rospy.Subscriber("/joint_states", JointState, self._state_cb, queue_size=10)
        self.server = actionlib.SimpleActionServer(
            "/roarm_arm_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction,
            execute_cb=self._execute,
            auto_start=False,
        )
        self.server.start()

    def _state_cb(self, msg):
        try:
            values = {name: msg.position[msg.name.index(name)] for name in ARM_JOINTS}
            gripper = msg.position[msg.name.index("link3_to_gripper_link")]
        except (ValueError, IndexError):
            return
        with self.lock:
            self.state = ([values[n] for n in ARM_JOINTS], gripper)

    def _current(self):
        with self.lock:
            return self.state

    def _fail(self, code, text):
        result = FollowJointTrajectoryResult()
        result.error_code = code
        result.error_string = text
        self.server.set_aborted(result, text)

    def _execute(self, goal):
        if list(goal.trajectory.joint_names) != ARM_JOINTS:
            self._fail(FollowJointTrajectoryResult.INVALID_JOINTS, "expected hand planning-group joint order")
            return
        if not goal.trajectory.points:
            self._fail(FollowJointTrajectoryResult.INVALID_GOAL, "empty trajectory")
            return
        for point in goal.trajectory.points:
            if self.server.is_preempt_requested():
                try: self.stop()
                except Exception: pass
                self.server.set_preempted()
                return
            if len(point.positions) != 3 or not all(math.isfinite(v) for v in point.positions):
                self._fail(FollowJointTrajectoryResult.INVALID_GOAL, "invalid trajectory point")
                return
            current = self._current()
            if current is None:
                self._fail(FollowJointTrajectoryResult.INVALID_GOAL, "no fresh joint state")
                return
            target = list(point.positions) + [current[1]]
            try:
                response = self.move(target, False, 0.1)
            except Exception as exc:
                self._fail(FollowJointTrajectoryResult.INVALID_GOAL, str(exc)); return
            if not response.accepted:
                self._fail(FollowJointTrajectoryResult.INVALID_GOAL, response.message); return
            deadline = time.monotonic() + self.timeout
            while time.monotonic() < deadline and not rospy.is_shutdown():
                state = self._current()
                if state and max(abs(a-b) for a,b in zip(state[0], point.positions)) <= self.tolerance:
                    break
                rospy.sleep(0.02)
            else:
                self._fail(FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED, "point feedback timeout"); return
        result = FollowJointTrajectoryResult(); result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
        self.server.set_succeeded(result)


def main():
    rospy.init_node("roarm_trajectory_server")
    TrajectoryServer()
    rospy.spin()

if __name__ == "__main__": main()
