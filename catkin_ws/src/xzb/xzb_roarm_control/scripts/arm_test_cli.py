#!/usr/bin/env python3
import argparse
import rospy
from geometry_msgs.msg import Point
from std_srvs.srv import SetBool, Trigger
from xzb_interfaces.msg import ArmStatus
from xzb_interfaces.srv import ComputeArmFK, MoveArmCartesian, MoveArmJoints, SetArmGripper, SolveArmIK


def main():
    p = argparse.ArgumentParser(description="Bounded RoArm hardware-test client")
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("status"); sub.add_parser("enable"); sub.add_parser("disable"); sub.add_parser("soft-stop")
    j = sub.add_parser("joint-delta")
    for name in ("base", "shoulder", "elbow", "gripper"):
        j.add_argument("--" + name, type=float, default=0.0)
    j.add_argument("--speed", type=float, default=0.1)
    c = sub.add_parser("cart-delta")
    c.add_argument("--x", type=float, default=0.0); c.add_argument("--y", type=float, default=0.0); c.add_argument("--z", type=float, default=0.0)
    c.add_argument("--speed", type=float, default=0.1)
    g = sub.add_parser("gripper"); g.add_argument("opening", type=float); g.add_argument("--speed", type=float, default=0.1)
    f = sub.add_parser("fk"); f.add_argument("base", type=float); f.add_argument("shoulder", type=float); f.add_argument("elbow", type=float)
    i = sub.add_parser("ik"); i.add_argument("x", type=float); i.add_argument("y", type=float); i.add_argument("z", type=float); i.add_argument("--seed", type=float, nargs=3, default=[0.0, 0.0, 1.5708])
    a = p.parse_args()
    rospy.init_node("roarm_test_cli", anonymous=True, disable_signals=True)
    if a.cmd == "status":
        print(rospy.wait_for_message("/roarm/status", ArmStatus, timeout=3.0)); return
    if a.cmd in ("enable", "disable"):
        print(rospy.ServiceProxy("/roarm/motion_enable", SetBool)(a.cmd == "enable")); return
    if a.cmd == "soft-stop":
        print(rospy.ServiceProxy("/roarm/soft_stop", Trigger)()); return
    if a.cmd == "joint-delta":
        print(rospy.ServiceProxy("/roarm/move_joints", MoveArmJoints)([a.base, a.shoulder, a.elbow, a.gripper], True, a.speed)); return
    if a.cmd == "cart-delta":
        print(rospy.ServiceProxy("/roarm/move_cartesian", MoveArmCartesian)(Point(a.x, a.y, a.z), True, a.speed)); return
    if a.cmd == "gripper":
        print(rospy.ServiceProxy("/roarm/set_gripper", SetArmGripper)(a.opening, a.speed)); return
    if a.cmd == "fk":
        print(rospy.ServiceProxy("/roarm/compute_fk", ComputeArmFK)([a.base, a.shoulder, a.elbow])); return
    if a.cmd == "ik":
        print(rospy.ServiceProxy("/roarm/solve_ik", SolveArmIK)(Point(a.x, a.y, a.z), a.seed)); return

if __name__ == "__main__": main()
