#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
import os
from datetime import datetime
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from scipy.signal import find_peaks
import pyrealsense2 as rs


class LightDetectionNode:
    def __init__(self):
        rospy.init_node('light_detection_node', anonymous=True)

        # 参数
        self.min_valid = rospy.get_param('~min_valid', 100)
        self.max_valid = rospy.get_param('~max_valid', 5000)
        self.far_ratio = rospy.get_param('~far_ratio', 1.6)
        self.gray_thresh = rospy.get_param('~gray_thresh', 100)
        self.save_images = rospy.get_param('~save_images', True)

        self.bridge = CvBridge()
        self.waiting_for_trigger = False

        # 初始化 RealSense
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        try:
            self.pipeline.start(config)
            rospy.loginfo("RealSense camera started")
        except Exception as e:
            rospy.logerr(f"Failed to start RealSense: {e}")
            rospy.signal_shutdown("Camera error")
            return

        self.align = rs.align(rs.stream.color)
        self.spatial = rs.spatial_filter()
        self.spatial.set_option(rs.option.filter_magnitude, 2)
        self.spatial.set_option(rs.option.filter_smooth_alpha, 0.5)
        self.temporal = rs.temporal_filter()

        # 发布 / 订阅
        self.result_pub = rospy.Publisher('/vision/light_result', String, queue_size=1)
        self.image_pub = rospy.Publisher('/vision/light_image', Image, queue_size=1)
        self.trigger_sub = rospy.Subscriber('/vision/trigger', Bool, self.trigger_callback)

        if self.save_images:
            self.save_dir = "/home/bingda/catkin_ws/src/bingda_ros1_noetic/door_detection/saved_frames"
            os.makedirs(self.save_dir, exist_ok=True)

        rospy.loginfo("Light detection node started")
        rospy.loginfo(f"Parameters: far_ratio={self.far_ratio}, gray_thresh={self.gray_thresh}")
        rospy.loginfo("Waiting for trigger on /vision/trigger...")

    def find_wall_and_door(self, depth_image):
        """深度直方图峰值分析法：找到墙壁距离和门洞区域"""
        valid = depth_image[(depth_image > self.min_valid) & (depth_image < self.max_valid)]
        if len(valid) == 0:
            return None, None, None

        counts, edges = np.histogram(valid, bins=200, range=(self.min_valid, self.max_valid))
        kernel = np.ones(7) / 7
        smooth = np.convolve(counts.astype(float), kernel, mode='same')

        min_height = smooth.max() * 0.05
        peak_indices, _ = find_peaks(smooth, height=min_height, distance=10)

        if len(peak_indices) == 0:
            return None, None, None

        bin_width = (self.max_valid - self.min_valid) / 200
        peak_mm = [self.min_valid + (i + 0.5) * bin_width for i in peak_indices]
        wall_dist = min(peak_mm)

        far_thresh = wall_dist * self.far_ratio
        valid_mask = (depth_image > self.min_valid) & (depth_image < self.max_valid)
        far_mask = (depth_image > far_thresh) & valid_mask

        return wall_dist, far_thresh, far_mask

    def analyze(self, color_image, depth_image):
        """主分析函数：检测灯状态"""
        wall_dist, far_thresh, far_mask = self.find_wall_and_door(depth_image)

        if far_mask is None or np.sum(far_mask) < 100:
            return "NO_LIGHT", None, None, None, None, None

        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        door_brightness = np.mean(gray[far_mask])

        wall_mask = (depth_image > self.min_valid) & (depth_image < far_thresh)
        wall_brightness = np.mean(gray[wall_mask]) if np.sum(wall_mask) > 0 else 0

        door_depth = np.mean(depth_image[far_mask])
        wall_depth = np.mean(depth_image[wall_mask]) if np.sum(wall_mask) > 0 else 0

        if door_brightness > self.gray_thresh or door_brightness > wall_brightness:
            light_status = "ON"
        else:
            light_status = "OFF"

        return light_status, door_brightness, wall_brightness, door_depth, wall_depth, far_mask

    def capture_frames(self):
        frames = self.pipeline.wait_for_frames()
        frames = self.align.process(frames)
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            return None, None

        depth_frame = self.spatial.process(depth_frame)
        depth_frame = self.temporal.process(depth_frame)
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        return depth_image, color_image

    def trigger_callback(self, msg):
        if not msg.data:
            return

        if self.waiting_for_trigger:
            rospy.logwarn("Already processing, ignoring trigger")
            return

        self.waiting_for_trigger = True
        rospy.loginfo("Trigger received, detecting light state...")

        depth_image, color_image = self.capture_frames()

        if depth_image is None:
            rospy.logerr("Failed to capture depth image")
            self.result_pub.publish(String(data="error"))
            self.waiting_for_trigger = False
            return

        status, door_bright, wall_bright, door_depth, wall_depth, far_mask = self.analyze(color_image, depth_image)

        rospy.loginfo(f"Light status: {status}")
        if door_bright is not None:
            rospy.loginfo(f"Door gray: {door_bright:.1f} | Wall gray: {wall_bright:.1f}")
            rospy.loginfo(f"Door depth: {door_depth:.0f}mm | Wall depth: {wall_depth:.0f}mm")

        if color_image is not None:
            # 在图上标注结果
            display = color_image.copy()
            if status == "ON":
                status_color = (0, 255, 0)
            elif status == "OFF":
                status_color = (0, 0, 255)
            else:
                status_color = (255, 255, 0)

            cv2.putText(display, f"Light: {status}", (10, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, status_color, 2)
            if door_bright is not None:
                cv2.putText(display, f"Door Gray: {door_bright:.1f}  Depth: {door_depth:.0f}mm", (10, 80),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
                cv2.putText(display, f"Wall Gray: {wall_bright:.1f}  Depth: {wall_depth:.0f}mm", (10, 110),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(display, "bgr8"))
            except Exception as e:
                rospy.logerr(f"Publish image failed: {e}")

        if self.save_images and color_image is not None:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            cv2.imwrite(f"{self.save_dir}/light_{ts}_{status}.jpg", color_image)
            np.save(f"{self.save_dir}/depth_{ts}.npy", depth_image)
            rospy.loginfo(f"Saved to {self.save_dir}/light_{ts}_{status}.jpg")

        self.result_pub.publish(String(data=status))
        self.waiting_for_trigger = False

    def run(self):
        rospy.spin()

    def __del__(self):
        if hasattr(self, 'pipeline'):
            self.pipeline.stop()


if __name__ == '__main__':
    try:
        node = LightDetectionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")

