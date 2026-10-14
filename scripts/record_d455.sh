#!/usr/bin/env bash
# Record a short RealSense D455 rosbag so the failure can be reproduced offline.
#
# Usage:
#   DURATION=10 ./record_d455.sh          # record 10 seconds into data/d455_bag
#
# The bag is written under /home/ros/ros2_ws/data/ so it stays in the workspace.
set +u

DURATION=$(printenv DURATION || echo 10)
START_DRIVER=$(printenv START_DRIVER || echo auto)

source /opt/ros/humble/setup.bash
source /home/ros/ros2_ws/install/setup.bash

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PKG="$(dirname "$SCRIPT_DIR")"
OUT=/home/ros/ros2_ws/data/d455_bag
RGB_TOPIC=/camera/camera/color/image_raw
DEPTH_TOPIC=/camera/camera/aligned_depth_to_color/image_raw

# Start a driver unless streams are already live.
if ! python3 "$SCRIPT_DIR/check_camera_streams.py" \
       --color-topic "$RGB_TOPIC" --depth-topic "$DEPTH_TOPIC" --timeout 4 >/dev/null 2>&1; then
  if [ "$START_DRIVER" = "auto" ] || [ "$START_DRIVER" = "true" ]; then
    echo "[rec] starting realsense2_camera"
    ros2 launch realsense2_camera rs_launch.py \
      enable_color:=true rgb_camera.color_profile:=640x480x30 \
      enable_depth:=true depth_module.depth_profile:=640x480x30 \
      align_depth.enable:=true enable_sync:=true \
      enable_infra:=false enable_infra1:=false enable_infra2:=false \
      pointcloud.enable:=true initial_reset:=true &
    DRIVER_PID=$!
    trap 'kill $DRIVER_PID 2>/dev/null' EXIT
    sleep 6
  fi
fi

if ! python3 "$SCRIPT_DIR/check_camera_streams.py" \
       --color-topic "$RGB_TOPIC" --depth-topic "$DEPTH_TOPIC" --timeout 8; then
  echo "[rec] camera streams are not healthy, aborting"
  exit 1
fi

mkdir -p /home/ros/ros2_ws/data
rm -rf "$OUT"
echo "[rec] recording $DURATION seconds -> $OUT"
echo "[rec] point the camera at the scene and move it exactly like when it fails"
timeout --signal=INT "$DURATION" ros2 bag record -o "$OUT" \
  "$RGB_TOPIC" \
  "$DEPTH_TOPIC" \
  /camera/camera/color/camera_info \
  /camera/camera/depth/camera_info
echo "[rec] done: $OUT"
ls -la "$OUT"
