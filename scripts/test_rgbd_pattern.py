#!/usr/bin/env python3
"""Publish a synthetic, synchronized RGB + aligned-depth pair.

Used to verify orbslam3_dense_ros2 can subscribe to the RealSense topic names
and process frames when no physical D455 is available.

Topics (same as realsense2_camera with camera_namespace=camera_name=camera):
  /camera/camera/color/image_raw                  bgr8   640x480
  /camera/camera/aligned_depth_to_color/image_raw 16UC1  640x480  (mm)
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class SyntheticRgbd(Node):
    def __init__(self) -> None:
        super().__init__("synthetic_rgbd")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 20.0)
        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter(
            "depth_topic", "/camera/camera/aligned_depth_to_color/image_raw"
        )
        # After this many frames publish a featureless image to force tracking
        # loss / map reset (0 = never). Used to exercise the crash fix.
        self.declare_parameter("blank_after", 0)
        # If >0, alternate texture/blank every blank_cycle frames to force
        # repeated tracking loss / map resets (stress test).
        self.declare_parameter("blank_cycle", 0)
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        # "16UC1" (mm, realsense default) or "32FC1" (m) to test depth handling
        self.declare_parameter("depth_encoding", "16UC1")
        # Fraction of pixels set to 65535 (RealSense "no depth" marker)
        self.declare_parameter("depth_invalid_frac", 0.0)

        self.w = int(self.get_parameter("width").value)
        self.h = int(self.get_parameter("height").value)
        fps = float(self.get_parameter("fps").value)
        self.blank_after = int(self.get_parameter("blank_after").value)
        self.blank_cycle = int(self.get_parameter("blank_cycle").value)
        self.depth_encoding = self.get_parameter("depth_encoding").value
        self.depth_invalid_frac = float(self.get_parameter("depth_invalid_frac").value)

        # deterministic pseudo-random texture, larger than the image so we can shift it
        rng = np.random.default_rng(0)
        self.noise = rng.integers(0, 255, size=(self.h + 64, self.w + 64), dtype=np.uint8)

        yy, xx = np.mgrid[0 : self.h, 0 : self.w].astype(np.float32)
        self.checker = (
            (((yy // 20).astype(np.int32) + (xx // 20).astype(np.int32)) % 2) * 255
        ).astype(np.uint8)
        # strongly non-planar depth (mm) so RGB-D initialization is not degenerate
        depth = 900.0 + 250.0 * np.sin(xx / 45.0) + 250.0 * np.cos(yy / 55.0)
        self.depth = np.clip(depth, 300, 3000).astype(np.uint16)
        if self.depth_invalid_frac > 0.0:
            rng2 = np.random.default_rng(1)
            mask = rng2.random((self.h, self.w)) < self.depth_invalid_frac
            self.depth = np.where(mask, np.uint16(65535), self.depth).astype(np.uint16)

        qos = rclpy.qos.QoSProfile(
            depth=10, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE
        )
        self.color_pub = self.create_publisher(
            Image, self.get_parameter("color_topic").value, qos
        )
        self.depth_pub = self.create_publisher(
            Image, self.get_parameter("depth_topic").value, qos
        )
        self.info_pub = self.create_publisher(
            CameraInfo, self.get_parameter("camera_info_topic").value, qos
        )
        self.camera_info = CameraInfo()
        self.camera_info.width = self.w
        self.camera_info.height = self.h
        self.camera_info.distortion_model = "plumb_bob"
        self.camera_info.k = [381.362, 0.0, 320.0, 0.0, 381.362, 240.0, 0.0, 0.0, 1.0]
        self.camera_info.d = [0.1, -0.2, 0.001, -0.001, 0.05]

        self.n = 0
        self.create_timer(1.0 / fps, self.tick)
        self.get_logger().info(
            "synthetic RGB-D -> %s , %s"
            % (
                self.get_parameter("color_topic").value,
                self.get_parameter("depth_topic").value,
            )
        )

    def tick(self) -> None:
        blank_now = bool(self.blank_after and self.n >= self.blank_after)
        if self.blank_cycle > 0 and (self.n % self.blank_cycle) >= (self.blank_cycle // 2):
            blank_now = True
        if blank_now:
            # featureless frame: forces ORB-SLAM3 to lose tracking / reset the map
            gray = np.full((self.h, self.w), 128, dtype=np.uint8)
        else:
            dx = self.n % 48
            dy = (self.n // 48) % 32
            noise = self.noise[dy : dy + self.h, dx : dx + self.w]
            gray = np.clip(0.55 * noise.astype(np.float32) + 0.45 * self.checker, 0, 255).astype(
                np.uint8
            )
        color = np.dstack([gray, np.roll(gray, 7, axis=1), np.roll(gray, 13, axis=0)])

        stamp = self.get_clock().now().to_msg()

        color_msg = Image()
        color_msg.header.stamp = stamp
        color_msg.header.frame_id = "camera_color_optical_frame"
        color_msg.height = self.h
        color_msg.width = self.w
        color_msg.encoding = "bgr8"
        color_msg.is_bigendian = False
        color_msg.step = self.w * 3
        color_msg.data = color.tobytes()
        self.color_pub.publish(color_msg)

        depth_msg = Image()
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = "camera_depth_optical_frame"
        depth_msg.height = self.h
        depth_msg.width = self.w
        depth_msg.is_bigendian = False
        if self.depth_encoding == "32FC1":
            depth_msg.encoding = "32FC1"
            depth_msg.step = self.w * 4
            depth_msg.data = (self.depth.astype(np.float32) / 1000.0).tobytes()
        else:
            depth_msg.encoding = "16UC1"
            depth_msg.step = self.w * 2
            depth_msg.data = self.depth.tobytes()
        self.depth_pub.publish(depth_msg)

        self.camera_info.header.stamp = stamp
        self.info_pub.publish(self.camera_info)

        self.n += 1


def main() -> None:
    rclpy.init()
    node = SyntheticRgbd()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
