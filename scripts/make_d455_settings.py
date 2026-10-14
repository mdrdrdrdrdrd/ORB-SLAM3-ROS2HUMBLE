#!/usr/bin/env python3
"""Generate an ORB-SLAM3 RGB-D settings file from live RealSense camera_info.

Reads the on-device intrinsics published by realsense2_camera and writes a copy
of the template with Camera1.* and Camera.width/height replaced, so ORB-SLAM3
uses the real calibration instead of nominal values.
"""

import argparse
import os
import re
import shutil
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo


def set_key(text: str, key: str, value) -> str:
    pat = re.compile(r"^" + re.escape(key) + r"\s*:.*$", re.MULTILINE)
    line = "%s: %s" % (key, value)
    if pat.search(text):
        return pat.sub(line, text)
    return text + "\n" + line + "\n"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--template", required=True)
    ap.add_argument("--output", required=True)
    ap.add_argument("--topic", default="/camera/camera/color/camera_info")
    ap.add_argument("--timeout", type=float, default=20.0)
    ap.add_argument("--fallback", action="store_true",
                    help="if no camera_info arrives, copy the template to the output")
    args = ap.parse_args()

    rclpy.init()
    node = Node("d455_settings_writer")
    holder = {}

    def cb(msg: CameraInfo) -> None:
        holder["msg"] = msg

    node.create_subscription(CameraInfo, args.topic, cb, 10)

    deadline = time.time() + args.timeout
    while rclpy.ok() and "msg" not in holder and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()

    if "msg" not in holder:
        if args.fallback:
            os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
            shutil.copyfile(args.template, args.output)
            print("WARN: no camera_info within %.1fs; using nominal template -> %s"
                  % (args.timeout, args.output), file=sys.stderr)
            return 0
        print("ERROR: no camera_info on %s within %.1fs" % (args.topic, args.timeout),
              file=sys.stderr)
        return 1

    msg = holder["msg"]
    k = list(msg.k) + [0.0] * 9
    d = list(msg.d)

    with open(args.template, "r") as f:
        text = f.read()

    text = set_key(text, "Camera1.fx", k[0])
    text = set_key(text, "Camera1.fy", k[4])
    text = set_key(text, "Camera1.cx", k[2])
    text = set_key(text, "Camera1.cy", k[5])
    text = set_key(text, "Camera.width", int(msg.width))
    text = set_key(text, "Camera.height", int(msg.height))
    text = set_key(text, "Camera1.k1", d[0] if len(d) > 0 else 0.0)
    text = set_key(text, "Camera1.k2", d[1] if len(d) > 1 else 0.0)
    text = set_key(text, "Camera1.p1", d[2] if len(d) > 2 else 0.0)
    text = set_key(text, "Camera1.p2", d[3] if len(d) > 3 else 0.0)
    text = set_key(text, "Camera1.k3", d[4] if len(d) > 4 else 0.0)

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, "w") as f:
        f.write(text)

    print("wrote %s from %s (fx=%.4f fy=%.4f cx=%.4f cy=%.4f)"
          % (args.output, args.topic, k[0], k[4], k[2], k[5]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
