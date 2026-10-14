#!/usr/bin/env python3
"""Save a sensor_msgs/PointCloud2 topic to a .pcd file.

RViz2 has no "export point cloud" button, so this subscribes to the topic and
writes a PCD that PCL, CloudCompare or pcl_viewer can open.

Examples:
  python3 save_cloud.py                                   # /cloudPointMapping -> data/map.pcd
  python3 save_cloud.py --topic /point_cloud_all --output data/orb.pcd
  python3 save_cloud.py --frames 5 --interval 2.0         # 5 snapshots map_000.pcd ...
"""

import argparse
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

_NP_TYPES = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
             5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64}
_PCD_TYPE = {1: "I", 2: "U", 3: "I", 4: "U", 5: "I", 6: "U", 7: "F", 8: "F"}


def _structured(msg):
    base = np.dtype([(f.name, _NP_TYPES[f.datatype]) for f in msg.fields])
    padding = msg.point_step - base.itemsize
    if padding > 0:
        base = np.dtype(base.descr + [("__pad__", "u1", padding)])
    return np.frombuffer(msg.data, dtype=base)


def write_pcd(path, msg):
    arr = _structured(msg)
    if not all(n in arr.dtype.names for n in ("x", "y", "z")):
        raise RuntimeError("PointCloud2 has no x/y/z fields")

    ok = np.isfinite(arr["x"]) & np.isfinite(arr["y"]) & np.isfinite(arr["z"])
    arr = arr[ok]
    n = int(arr.shape[0])

    names, cols, sizes, types, counts = [], [], [], [], []
    for name in ("x", "y", "z"):
        names.append(name)
        cols.append(arr[name].astype(np.float32))
        sizes.append(4)
        types.append("F")
        counts.append(1)
    if "rgb" in arr.dtype.names:
        rgb = arr["rgb"]
        rgb_u = rgb.view(np.uint32) if rgb.dtype == np.float32 else rgb.astype(np.uint32)
        names.append("rgb"); cols.append(rgb_u); sizes.append(4); types.append("U"); counts.append(1)
    elif "rgba" in arr.dtype.names:
        rgba = arr["rgba"]
        rgba_u = rgba.view(np.uint32) if rgba.dtype == np.float32 else rgba.astype(np.uint32)
        names.append("rgba"); cols.append(rgba_u); sizes.append(4); types.append("U"); counts.append(1)
    elif "intensity" in arr.dtype.names:
        names.append("intensity"); cols.append(arr["intensity"].astype(np.float32))
        sizes.append(4); types.append("F"); counts.append(1)

    with open(path, "w") as fh:
        fh.write("# .PCD v0.7 - Point Cloud Data file format\n")
        fh.write("VERSION 0.7\n")
        fh.write("FIELDS " + " ".join(names) + "\n")
        fh.write("SIZE " + " ".join(str(s) for s in sizes) + "\n")
        fh.write("TYPE " + " ".join(types) + "\n")
        fh.write("COUNT " + " ".join(str(c) for c in counts) + "\n")
        fh.write("WIDTH %d\nHEIGHT 1\n" % n)
        fh.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        fh.write("POINTS %d\n" % n)
        fh.write("DATA ascii\n")
        for i in range(n):
            row = []
            for j, name in enumerate(names):
                if types[j] == "U":
                    row.append(str(int(cols[j][i])))
                else:
                    row.append("%.6f" % float(cols[j][i]))
            fh.write(" ".join(row) + "\n")
    return n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/cloudPointMapping")
    ap.add_argument("--output", default="/home/ros/ros2_ws/data/map.pcd")
    ap.add_argument("--timeout", type=float, default=30.0,
                    help="seconds to wait for a message")
    ap.add_argument("--frames", type=int, default=1,
                    help="number of snapshots to save")
    ap.add_argument("--interval", type=float, default=2.0,
                    help="seconds between snapshots")
    args = ap.parse_args()

    rclpy.init()
    node = Node("save_cloud")
    holder = {}
    node.create_subscription(PointCloud2, args.topic,
                             lambda m: holder.__setitem__("msg", m), 5)

    saved = 0
    for idx in range(max(1, args.frames)):
        holder.pop("msg", None)
        t0 = time.time()
        while rclpy.ok() and "msg" not in holder and time.time() - t0 < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
        if "msg" not in holder:
            print("no message on %s within %.1fs" % (args.topic, args.timeout),
                  file=sys.stderr)
            break
        if args.frames == 1:
            out = args.output
        else:
            out = "%s_%03d.pcd" % (os.path.splitext(args.output)[0], idx)
        os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
        n = write_pcd(out, holder["msg"])
        print("saved %s (%d points)" % (out, n))
        saved += 1
        if idx + 1 < args.frames:
            time.sleep(args.interval)

    node.destroy_node()
    rclpy.shutdown()
    return 0 if saved > 0 else 1


if __name__ == "__main__":
    sys.exit(main())
