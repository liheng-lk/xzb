#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Keyboard publisher for linear_actuator_control.

Run in a real terminal:
  rosrun linear_actuator_control actuator_keyboard.py

Keys:
  u extend, d retract, s stop, f coast, ? state, q stop+quit
"""

import select
import sys
import termios
import tty

import rospy
from std_msgs.msg import String


def main():
    rospy.init_node("actuator_keyboard", anonymous=True)
    topic = rospy.get_param("~cmd_topic", "/linear_actuator/cmd")
    pub = rospy.Publisher(topic, String, queue_size=10)

    if not sys.stdin.isatty():
        rospy.logerr("actuator_keyboard.py must run in an interactive terminal")
        return 1

    print("\nLinear actuator keyboard")
    print("  u : extend")
    print("  d : retract")
    print("  s : brake stop")
    print("  f : coast")
    print("  ? : query state")
    print("  q : stop and quit\n")

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)

    try:
        tty.setcbreak(fd)
        while not rospy.is_shutdown():
            ready, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not ready:
                continue

            key = sys.stdin.read(1).lower()
            if key == "u":
                pub.publish(String(data="extend"))
                print("EXTEND")
            elif key == "d":
                pub.publish(String(data="retract"))
                print("RETRACT")
            elif key == "s":
                pub.publish(String(data="stop"))
                print("STOP")
            elif key == "f":
                pub.publish(String(data="coast"))
                print("COAST")
            elif key == "?":
                pub.publish(String(data="status"))
            elif key == "q":
                pub.publish(String(data="stop"))
                rospy.sleep(0.1)
                print("STOP + QUIT")
                break
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        try:
            pub.publish(String(data="stop"))
            rospy.sleep(0.05)
        except Exception:
            pass

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
