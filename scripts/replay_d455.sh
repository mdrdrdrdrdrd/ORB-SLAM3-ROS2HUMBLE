#!/usr/bin/env bash
# Replay a recorded RealSense bag through orbslam3_dense_ros2 (offline reproduction).
#
# Usage:
#   BAG=/home/ros/ros2_ws/data/d455_bag ./replay_d455.sh
#   BAG=... ENABLE_PANGOLIN=true ./replay_d455.sh
#
# Note: the bag is played ONCE (not looped). Looping makes header timestamps
# jump backwards and produces artificial tracking losses / map resets.
set +u

BAG=$(printenv BAG || echo /home/ros/ros2_ws/data/d455_bag)
ENABLE_PANGOLIN=$(printenv ENABLE_PANGOLIN || echo false)

source /opt/ros/humble/setup.bash
source /home/ros/ros2_ws/install/setup.bash

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PKG="$(dirname "$SCRIPT_DIR")"
VOC=$PKG/orb_slam3/Vocabulary/ORBvoc.txt.bin
TEMPLATE=$PKG/orb_slam3/config/RGB-D/RealSense_D455.yaml
AUTO=$PKG/orb_slam3/config/RGB-D/RealSense_D455_auto.yaml

if [ ! -d "$BAG" ]; then
  echo "[replay] bag not found: $BAG"
  exit 1
fi

SETTINGS="$TEMPLATE"
echo "[replay] reading camera_info from bag (brief playback)"
ros2 bag play "$BAG" >/dev/null 2>&1 & P=$!
if python3 "$SCRIPT_DIR/make_d455_settings.py" \
     --template "$TEMPLATE" --output "$AUTO" \
     --topic /camera/camera/color/camera_info --timeout 8; then
  SETTINGS="$AUTO"
else
  echo "[replay] WARN: no camera_info in bag, using nominal settings"
fi
kill -INT $P 2>/dev/null
sleep 1
kill -9 $P 2>/dev/null

echo "[replay] starting SLAM (settings=$SETTINGS)"
ros2 run orbslam3_dense_ros2 orb_slam3_main --ros-args \
  -p vocabulary_file:="$VOC" \
  -p settings_file:="$SETTINGS" \
  -p enable_pangolin:="$ENABLE_PANGOLIN" \
  -p rgb_topic:=/camera/camera/color/image_raw \
  -p depth_topic:=/camera/camera/aligned_depth_to_color/image_raw &
SLAM=$!

BAG_PID=""
cleanup() { kill "$SLAM" "$BAG_PID" 2>/dev/null; }
trap cleanup EXIT

sleep 3
echo "[replay] playing bag once"
ros2 bag play "$BAG" &
BAG_PID=$!
wait "$SLAM"
