#!/usr/bin/env python3
"""Preflight / monitor for RealSense color + aligned depth.

Examples:
  python3 check_camera_streams.py                       # one-shot check
  python3 check_camera_streams.py --samples 10 --interval 1.0   # watch depth health

Exit codes:
  0 = OK
  1 = a stream produced no message
  2 = depth arrived but was all zeros in at least one sample
"""

import argparse
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class StreamCheck(Node):
    def __init__(self, color_topic: str, depth_topic: str):
        super().__init__("d455_stream_check")
        self.color = None
        self.depth = None
        self.color_count = 0
        self.depth_count = 0
        self.create_subscription(Image, color_topic, self._color_cb, 5)
        self.create_subscription(Image, depth_topic, self._depth_cb, 5)

    def _color_cb(self, msg):
        self.color = msg
        self.color_count += 1

    def _depth_cb(self, msg):
        self.depth = msg
        self.depth_count += 1

    def wait_for_depth(self, target: int, timeout: float, need_color: bool = False) -> bool:
        deadline = time.time() + timeout
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.depth_count >= target and (not need_color or self.color is not None):
                return True
        return False


def depth_stats(msg):
    arr = np.frombuffer(msg.data, dtype=np.uint16)
    total = max(1, int(arr.size))
    nz = int(np.count_nonzero(arr))
    return nz, total, (int(arr.max()) if arr.size else -1)


def orb_feature_count(msg) -> int:
    """Count ORB keypoints on a color image (uses OpenCV, independent of ORB-SLAM3)."""
    import cv2

    h, w = msg.height, msg.width
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding in ("bgr8", "rgb8"):
        img = np.ascontiguousarray(arr.reshape(h, w, 3))
        if msg.encoding == "rgb8":
            img = img[:, :, ::-1].copy()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = np.ascontiguousarray(arr.reshape(h, w))
    orb = cv2.ORB_create(nfeatures=2000)
    return len(orb.detect(gray, None))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--color-topic", default="/camera/camera/color/image_raw")
    ap.add_argument("--depth-topic", default="/camera/camera/aligned_depth_to_color/image_raw")
    ap.add_argument("--timeout", type=float, default=10.0, help="per-sample wait timeout")
    ap.add_argument("--samples", type=int, default=1)
    ap.add_argument("--interval", type=float, default=1.0)
    ap.add_argument("--features", action="store_true",
                    help="also count ORB keypoints on the color image")
    args = ap.parse_args()

    rclpy.init()
    node = StreamCheck(args.color_topic, args.depth_topic)

    rc = 0
    target = 0
    for i in range(max(1, args.samples)):
        target += 1
        if not node.wait_for_depth(target, args.timeout, need_color=args.features):
            print("sample %d: NO DEPTH MESSAGE on %s" % (i + 1, args.depth_topic))
            rc = 1
            break
        nz, total, mx = depth_stats(node.depth)
        c = node.color
        print("sample %d: color=%dx%d %s | depth=%dx%d %s nonzero=%d/%d (%.1f%%) max=%d"
              % (i + 1,
                 c.width if c else 0, c.height if c else 0, c.encoding if c else "?",
                 node.depth.width, node.depth.height, node.depth.encoding,
                 nz, total, 100.0 * nz / total, mx))
        if nz == 0:
            print("sample %d: WARNING depth all zeros" % (i + 1))
            rc = 2
        if args.features and c is not None:
            try:
                print("         ORB keypoints: %d (low values mean a texture-poor scene)"
                      % orb_feature_count(c))
            except Exception as exc:
                print("         ORB feature check failed: %s" % exc)
        if i < args.samples - 1:
            time.sleep(args.interval)

    node.destroy_node()
    rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
