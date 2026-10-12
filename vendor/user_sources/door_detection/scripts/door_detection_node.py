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

class DoorDetectionNode:
    def __init__(self):
        rospy.init_node('door_detection_node', anonymous=True)

        # 参数 —— 门
        self.min_valid = rospy.get_param('~min_valid', 100)
        self.max_valid = rospy.get_param('~max_valid', 5000)
        self.hist_bins = rospy.get_param('~hist_bins', 200)
        self.far_ratio_thresh = rospy.get_param('~far_ratio_thresh', 1.6)
        self.open_pct_thresh = rospy.get_param('~open_pct_thresh', 60.0)
        self.save_images = rospy.get_param('~save_images', True)
        # 参数 —— 灯
        self.gray_thresh = rospy.get_param('~gray_thresh', 100)
        
        # 初始化
        self.bridge = CvBridge()
        self.last_depth = None
        self.last_color = None
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
        
        # 对齐和滤波
        self.align = rs.align(rs.stream.color)
        self.spatial = rs.spatial_filter()
        self.spatial.set_option(rs.option.filter_magnitude, 2)
        self.spatial.set_option(rs.option.filter_smooth_alpha, 0.5)
        self.temporal = rs.temporal_filter()
        
        # 创建发布者和订阅者
        self.result_pub = rospy.Publisher('/vision/result', String, queue_size=1)
        self.image_pub = rospy.Publisher('/vision/image', Image, queue_size=1)
        self.light_result_pub = rospy.Publisher('/vision/light_result', String, queue_size=1)
        self.light_image_pub = rospy.Publisher('/vision/light_image', Image, queue_size=1)
        self.trigger_sub = rospy.Subscriber('/vision/trigger', Bool, self.trigger_callback)
        
        # 保存目录（绝对路径）
        if self.save_images:
            self.save_dir = "/home/bingda/catkin_ws/src/bingda_ros1_noetic/door_detection/saved_frames"
            os.makedirs(self.save_dir, exist_ok=True)
        
        rospy.loginfo("Door + Light detection node started (depth histogram method)")
        rospy.loginfo(f"Door params: far_ratio_thresh={self.far_ratio_thresh}, open_pct_thresh={self.open_pct_thresh}%")
        rospy.loginfo(f"Light params: gray_thresh={self.gray_thresh}")
        rospy.loginfo("Waiting for trigger on /vision/trigger...")
    
    def find_lowest_peak(self, depth_image):
        valid = depth_image[(depth_image > self.min_valid) & (depth_image < self.max_valid)]
        if valid.size == 0:
            return None, None
        
        counts, edges = np.histogram(valid, bins=self.hist_bins, range=(self.min_valid, self.max_valid))
        kernel = np.ones(7) / 7
        smooth = np.convolve(counts.astype(float), kernel, mode='same')
        min_height = smooth.max() * 0.05
        min_distance = self.hist_bins // 20
        peak_indices, _ = find_peaks(smooth, height=min_height, distance=min_distance)
        
        if len(peak_indices) == 0:
            return None, None
        
        bin_width = (self.max_valid - self.min_valid) / self.hist_bins
        peak_mm = [self.min_valid + (i + 0.5) * bin_width for i in peak_indices]
        lowest_idx = np.argmin(peak_mm)
        lowest_peak_mm = peak_mm[lowest_idx]
        
        return lowest_peak_mm, smooth
    
    def analyze_door_state(self, depth_image):
        wall_dist, _ = self.find_lowest_peak(depth_image)
        
        if wall_dist is None:
            rospy.logwarn("Cannot find wall distance")
            return "closed", 0.0, None
        
        far_thresh = wall_dist * self.far_ratio_thresh
        valid_mask = (depth_image > self.min_valid) & (depth_image < self.max_valid)
        far_mask = (depth_image > far_thresh) & valid_mask
        valid_cnt = np.sum(valid_mask)
        far_cnt = np.sum(far_mask)
        far_ratio = far_cnt / max(valid_cnt, 1) * 100
        state = "open" if far_ratio > self.open_pct_thresh else "closed"
        
        info = {
            "wall_dist": wall_dist,
            "far_thresh": far_thresh,
            "far_ratio": far_ratio
        }
        
        return state, far_ratio, info

    def analyze_light_state(self, depth_image, color_image):
        """在门洞区域基础上做灰度分析，判断灯状态"""
        wall_dist, _ = self.find_lowest_peak(depth_image)
        if wall_dist is None:
            return "NO_LIGHT", None, None

        far_thresh = wall_dist * self.far_ratio_thresh
        valid_mask = (depth_image > self.min_valid) & (depth_image < self.max_valid)
        far_mask = (depth_image > far_thresh) & valid_mask

        if np.sum(far_mask) < 100:
            return "NO_LIGHT", None, None

        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        door_brightness = np.mean(gray[far_mask])
        wall_mask = (depth_image > self.min_valid) & (depth_image < far_thresh)
        wall_brightness = np.mean(gray[wall_mask]) if np.sum(wall_mask) > 0 else 0

        if door_brightness > self.gray_thresh or door_brightness > wall_brightness:
            return "ON", door_brightness, wall_brightness
        return "OFF", door_brightness, wall_brightness

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
        rospy.loginfo("Trigger received, detecting door state...")
        
        depth_image, color_image = self.capture_frames()
        
        if depth_image is None:
            rospy.logerr("Failed to capture depth image")
            self.result_pub.publish(String(data="error"))
            self.waiting_for_trigger = False
            return
        
        state, far_ratio, info = self.analyze_door_state(depth_image)

        rospy.loginfo(f"Wall distance: {info['wall_dist']:.0f}mm")
        rospy.loginfo(f"Far threshold: {info['far_thresh']:.0f}mm")
        rospy.loginfo(f"Far ratio: {far_ratio:.1f}%")
        rospy.loginfo(f"Door state: {state}")

        # ---- 灯检测（复用同一帧） ----
        light_status, door_bright, wall_bright = self.analyze_light_state(depth_image, color_image)
        rospy.loginfo(f"Light state: {light_status}")
        if door_bright is not None:
            rospy.loginfo(f"Door gray: {door_bright:.1f} | Wall gray: {wall_bright:.1f}")

        # ---- 发布图片 ----
        if color_image is not None:
            try:
                # 门检测图片
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(color_image, "bgr8"))
                self.last_color = color_image

                # 灯检测图片（叠加灯状态标注）
                display = color_image.copy()
                label = f"Light: {light_status}"
                if light_status == "ON":
                    sc = (0, 255, 0)
                elif light_status == "OFF":
                    sc = (0, 0, 255)
                else:
                    sc = (255, 255, 0)
                cv2.putText(display, label, (10, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, sc, 2)
                if door_bright is not None:
                    cv2.putText(display, f"Door Gray: {door_bright:.1f}", (10, 80),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
                    cv2.putText(display, f"Wall Gray: {wall_bright:.1f}", (10, 110),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
                self.light_image_pub.publish(self.bridge.cv2_to_imgmsg(display, "bgr8"))
            except Exception as e:
                rospy.logerr(f"Publish image failed: {e}")

        if self.save_images and color_image is not None:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            cv2.imwrite(f"{self.save_dir}/door_{ts}_{state}.jpg", color_image)
            cv2.imwrite(f"{self.save_dir}/light_{ts}_{light_status}.jpg", color_image)
            np.save(f"{self.save_dir}/depth_{ts}.npy", depth_image)
            rospy.loginfo(f"Saved to {self.save_dir}")

        self.result_pub.publish(String(data=state))
        self.light_result_pub.publish(String(data=light_status))
        self.waiting_for_trigger = False
    
    def run(self):
        rospy.spin()
    
    def __del__(self):
        if hasattr(self, 'pipeline'):
            self.pipeline.stop()

if __name__ == '__main__':
    try:
        node = DoorDetectionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")

