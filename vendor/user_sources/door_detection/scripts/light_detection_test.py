#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
灯检测独立测试脚本（无需ROS，直接在电脑上运行）
按 空格 检测，按 q 退出
"""
import pyrealsense2 as rs
import cv2
import numpy as np
from scipy.signal import find_peaks

# 参数
MIN_VALID = 100
MAX_VALID = 5000
FAR_RATIO = 1.6
GRAY_THRESH = 100

def find_wall_and_door(depth_image):
    valid = depth_image[(depth_image > MIN_VALID) & (depth_image < MAX_VALID)]
    if len(valid) == 0:
        return None, None, None
    counts, edges = np.histogram(valid, bins=200, range=(MIN_VALID, MAX_VALID))
    kernel = np.ones(7)/7
    smooth = np.convolve(counts.astype(float), kernel, mode='same')
    min_height = smooth.max() * 0.05
    peak_indices, _ = find_peaks(smooth, height=min_height, distance=10)
    if len(peak_indices) == 0:
        return None, None, None
    bin_width = (MAX_VALID - MIN_VALID) / 200
    peak_mm = [MIN_VALID + (i + 0.5) * bin_width for i in peak_indices]
    wall_dist = min(peak_mm)
    far_thresh = wall_dist * FAR_RATIO
    valid_mask = (depth_image > MIN_VALID) & (depth_image < MAX_VALID)
    far_mask = (depth_image > far_thresh) & valid_mask
    return wall_dist, far_thresh, far_mask

def analyze(color_image, depth_image):
    wall_dist, far_thresh, far_mask = find_wall_and_door(depth_image)
    if far_mask is None or np.sum(far_mask) < 100:
        return "NO_LIGHT", None, None, None, None, None
    gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
    door_brightness = np.mean(gray[far_mask])
    wall_mask = (depth_image > MIN_VALID) & (depth_image < far_thresh)
    wall_brightness = np.mean(gray[wall_mask]) if np.sum(wall_mask) > 0 else 0
    door_depth = np.mean(depth_image[far_mask])
    wall_depth = np.mean(depth_image[wall_mask]) if np.sum(wall_mask) > 0 else 0
    if door_brightness > GRAY_THRESH or door_brightness > wall_brightness:
        light_status = "ON"
    else:
        light_status = "OFF"
    return light_status, door_brightness, wall_brightness, door_depth, wall_depth, far_mask

# ---- 主程序 ----
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

align = rs.align(rs.stream.color)
spatial = rs.spatial_filter()
temporal = rs.temporal_filter()

print("=" * 50)
print("灯检测测试工具")
print("按 空格键 检测当前画面灯状态")
print("按 q 退出")
print("=" * 50)

while True:
    frames = pipeline.wait_for_frames()
    frames = align.process(frames)
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    if not depth_frame or not color_frame:
        continue

    depth_frame = spatial.process(depth_frame)
    depth_frame = temporal.process(depth_frame)
    depth = np.asanyarray(depth_frame.get_data())
    color = np.asanyarray(color_frame.get_data())

    key = cv2.waitKey(1) & 0xFF

    if key == ord(' '):
        status, door_bright, wall_bright, door_depth, wall_depth, far_mask = analyze(color, depth)
        print(f"\n--- 检测结果 ---")
        print(f"灯状态: {status}")
        if door_bright is not None:
            print(f"门洞区域灰度: {door_bright:.1f} | 深度: {door_depth:.0f}mm")
            print(f"墙壁区域灰度: {wall_bright:.1f} | 深度: {wall_depth:.0f}mm")
            print(f"条件1 (>{GRAY_THRESH}): {door_bright > GRAY_THRESH}")
            print(f"条件2 (>墙壁): {door_bright > wall_bright}")
        else:
            print("未检测到门洞/灯区域")
    elif key == ord('q'):
        break

    # 实时显示
    display = color.copy()
    status, door_bright, wall_bright, door_depth, wall_depth, far_mask = analyze(color, depth)
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

    cv2.imshow("Light Detection", display)

pipeline.stop()
cv2.destroyAllWindows()

