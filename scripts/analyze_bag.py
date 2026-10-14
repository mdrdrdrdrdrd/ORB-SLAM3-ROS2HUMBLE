#!/usr/bin/env python3
"""Analyze a D455 bag offline: per-frame ORB keypoints and depth-valid keypoints."""
import os
import statistics
import sys

import numpy as np
import cv2
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image

bag = sys.argv[1] if len(sys.argv) > 1 else "/home/ros/ros2_ws/data/d455_bag"
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"),
            rosbag2_py.ConverterOptions("cdr", "cdr"))

color_t = "/camera/camera/color/image_raw"
depth_t = "/camera/camera/aligned_depth_to_color/image_raw"

orb = cv2.ORB_create(nfeatures=2000)
colors = []
depths = []
os.makedirs("/home/ros/ros2_ws/.ros/frames", exist_ok=True)
saved = 0

while reader.has_next():
    topic, data, t = reader.read_next()
    if topic == color_t:
        msg = deserialize_message(data, Image)
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        img = arr.reshape(msg.height, msg.width, 3)
        bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if msg.encoding == "rgb8" else img
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        kps = orb.detect(gray, None)
        if kps:
            pts = np.array([[k.pt[0], k.pt[1]] for k in kps], dtype=np.float32)
        else:
            pts = np.zeros((0, 2), np.float32)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        colors.append((stamp, len(kps), float(gray.mean()), pts))
        if len(colors) % 80 == 1 and saved < 4:
            cv2.imwrite("/home/ros/ros2_ws/.ros/frames/color_%04d.png" % len(colors), bgr)
            saved += 1
    elif topic == depth_t:
        msg = deserialize_message(data, Image)
        d = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        depths.append((stamp, d))

print("color frames:", len(colors), "depth frames:", len(depths))
depths.sort(key=lambda x: x[0])
dstamps = np.array([s for s, _ in depths]) if depths else np.array([])

valid_counts = []
for i, (stamp, nkp, mean, pts) in enumerate(colors):
    vc = 0
    if depths:
        j = int(np.argmin(np.abs(dstamps - stamp)))
        if abs(dstamps[j] - stamp) < 0.1 and len(pts) > 0:
            d = depths[j][1]
            xs = np.clip(pts[:, 0].astype(int), 0, d.shape[1] - 1)
            ys = np.clip(pts[:, 1].astype(int), 0, d.shape[0] - 1)
            dd = d[ys, xs].astype(np.int32)
            vc = int(np.count_nonzero((dd > 0) & (dd <= 10000)))
    valid_counts.append(vc)
    if i % 15 == 0:
        print("frame %3d: keypoints=%4d valid_depth_kp=%4d mean=%5.1f"
              % (i, nkp, vc, mean))

print("---")
print("keypoints:      min=%d median=%d max=%d"
      % (min(c[1] for c in colors), int(statistics.median(c[1] for c in colors)), max(c[1] for c in colors)))
print("valid_depth_kp: min=%d median=%d max=%d"
      % (min(valid_counts), int(statistics.median(valid_counts)), max(valid_counts)))
