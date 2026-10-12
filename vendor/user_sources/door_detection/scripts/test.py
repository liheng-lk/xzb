import pyrealsense2 as rs
import cv2
import numpy as np
from scipy.signal import find_peaks

# 参数
MIN_VALID = 100
MAX_VALID = 5000
FAR_RATIO = 1.6
OPEN_PCT_THRESH = 40
GRAY_THRESH = 100

# ================== 深度峰值 ==================
def find_lowest_peak(depth_image):
    valid = depth_image[(depth_image > MIN_VALID) & (depth_image < MAX_VALID)]
    if valid.size == 0:
        return None

    counts, _ = np.histogram(valid, bins=200, range=(MIN_VALID, MAX_VALID))
    smooth = np.convolve(counts.astype(float), np.ones(7)/7, mode='same')

    peaks, _ = find_peaks(smooth, height=smooth.max()*0.05, distance=10)
    if len(peaks) == 0:
        return None

    bin_width = (MAX_VALID - MIN_VALID) / 200
    peak_mm = [MIN_VALID + (i + 0.5)*bin_width for i in peaks]

    return min(peak_mm)  # 最近峰 = 墙

# ================== 核心融合 ==================
def analyze_all(color_image, depth_image):
    wall_dist = find_lowest_peak(depth_image)
    if wall_dist is None:
        return "UNKNOWN", {}

    far_thresh = wall_dist * FAR_RATIO

    valid_mask = (depth_image > MIN_VALID) & (depth_image < MAX_VALID)
    far_mask = (depth_image > far_thresh) & valid_mask

    valid_cnt = np.sum(valid_mask)
    far_cnt = np.sum(far_mask)
    far_ratio = far_cnt / max(valid_cnt, 1) * 100

    # 门状态
    door_state = "OPEN" if far_ratio > OPEN_PCT_THRESH else "CLOSED"

    # 灯状态
    gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

    if np.sum(far_mask) < 100:
        light_state = "UNKNOWN"
        door_brightness = 0
        wall_brightness = 0
    else:
        door_brightness = np.mean(gray[far_mask])
        wall_mask = valid_mask & (depth_image < far_thresh)
        wall_brightness = np.mean(gray[wall_mask]) if np.sum(wall_mask) > 0 else 0

        if door_brightness > GRAY_THRESH or door_brightness > wall_brightness:
            light_state = "ON"
        else:
            light_state = "OFF"

    final_state = f"{door_state}_{light_state}"

    info = {
        "door_state": door_state,
        "light_state": light_state,
        "far_ratio": far_ratio,
        "door_brightness": door_brightness,
        "wall_brightness": wall_brightness
    }

    return final_state, info

# ================== 主程序 ==================
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

align = rs.align(rs.stream.color)

print("按Q退出")

while True:
    frames = pipeline.wait_for_frames()
    frames = align.process(frames)

    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()

    if not depth_frame or not color_frame:
        continue

    depth = np.asanyarray(depth_frame.get_data())
    color = np.asanyarray(color_frame.get_data())

    # ===== 分析 =====
    state, info = analyze_all(color, depth)

    # ===== 显示 =====
    display = color.copy()

    # 状态颜色
    color_map = {
        "OPEN": (0, 255, 0),
        "CLOSED": (0, 0, 255),
        "UNKNOWN": (0, 255, 255)
    }

    door_color = color_map.get(info.get("door_state", "UNKNOWN"), (255,255,255))
    light_color = (0,255,0) if info.get("light_state")=="ON" else (0,0,255)

    # 文字显示
    cv2.putText(display, f"Door: {info.get('door_state','-')}", (10,40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, door_color, 2)

    cv2.putText(display, f"Light: {info.get('light_state','-')}", (10,80),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, light_color, 2)

    cv2.putText(display, f"FarRatio: {info.get('far_ratio',0):.1f}%", (10,120),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200,200,200), 1)

    cv2.putText(display, f"DoorGray: {info.get('door_brightness',0):.1f}", (10,150),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200,200,200), 1)

    cv2.putText(display, f"WallGray: {info.get('wall_brightness',0):.1f}", (10,180),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200,200,200), 1)

    cv2.imshow("Fusion Detection", display)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

pipeline.stop()
cv2.destroyAllWindows()
