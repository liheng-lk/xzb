#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Interactive ROS publisher; the bridge remains the only serial-port owner."""
import select
import sys
import time

import rospy
from std_msgs.msg import String


def main():
    rospy.init_node("actuator_servo_console", anonymous=True)
    topic = rospy.get_param("~cmd_topic", "/linear_actuator/cmd")
    pub = rospy.Publisher(topic, String, queue_size=1)
    if not sys.stdin.isatty():
        rospy.logerr("Run actuator_servo_console.py in an interactive terminal")
        return 1
    print("\nActuator + ST3215 console (Enter after each command)")
    print("  U / D / S / F       actuator extend / retract / brake / coast")
    print("  PING / READ        query servo communication / current positions")
    print("  PAN <deg>          ID1; integer angle, 0..360 before mechanical limits")
    print("  TILT <deg>         ID2; integer angle, 0..360 before mechanical limits")
    print("  PT <pan> <tilt>    synchronized position command to both servos")
    print("  BAUD <rate>        STM32 servo-UART rate; PC UART stays 115200")
    print("  status / HELP      host request state + servo READ / firmware help")
    print("  q                  stop ACTUATOR and quit (does NOT stop servos)")
    print("Read current positions first. No automatic center/homing is performed.")
    print("Wait for CONTROLLER_READY on the status topic before moving.\n")
    try:
        while not rospy.is_shutdown():
            print("> ", end="", flush=True)
            while not rospy.is_shutdown():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    break
            if rospy.is_shutdown():
                break
            line = sys.stdin.readline()
            if not line or line.strip().lower() in {"q", "quit", "exit"}:
                break
            command = line.strip()
            if not command:
                continue
            if pub.get_num_connections() == 0:
                print("No ROS bridge subscriber; command NOT queued.")
                continue
            pub.publish(String(data=command))
    except (KeyboardInterrupt, rospy.ROSInterruptException):
        pass
    finally:
        if pub.get_num_connections():
            try:
                pub.publish(String(data="stop"))
                time.sleep(0.1)
            except Exception:
                pass
        print("\nActuator stop requested; servo motion is not cancelled by S.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
