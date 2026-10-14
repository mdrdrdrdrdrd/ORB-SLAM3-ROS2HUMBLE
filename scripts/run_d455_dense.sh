#!/usr/bin/env bash
# Run RealSense D455 + orbslam3_dense_ros2 (RGB-D dense mapping).
#
# Usage:
#   ./run_d455_dense.sh                         # headless
#   ENABLE_PANGOLIN=true ./run_d455_dense.sh    # with Pangolin viewer
#   START_DRIVER=false ./run_d455_dense.sh      # never start a driver
#   AUTO_INTRINSICS=false ./run_d455_dense.sh   # keep nominal intrinsics
#   FORCE_DRIVER_START=true ./run_d455_dense.sh # start a driver even if a
#                                               # stale topic is advertised
# NOTE: do NOT use `set -u` here. ROS 2's setup.bash references unset
# variables (e.g. AMENT_TRACE_SETUP_FILES) and would abort under nounset.
set +u

ENABLE_PANGOLIN=$(printenv ENABLE_PANGOLIN || echo false)
START_DRIVER=$(printenv START_DRIVER || echo true)
AUTO_INTRINSICS=$(printenv AUTO_INTRINSICS || echo true)
FORCE_DRIVER_START=$(printenv FORCE_DRIVER_START || echo false)
MONITOR_ONLY=$(printenv MONITOR_ONLY || echo false)
MONITOR_SAMPLES=$(printenv MONITOR_SAMPLES || echo 20)

source /opt/ros/humble/setup.bash
source /home/ros/ros2_ws/install/setup.bash

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PKG="$(dirname "$SCRIPT_DIR")"
VOC=$PKG/orb_slam3/Vocabulary/ORBvoc.txt.bin
TEMPLATE=$PKG/orb_slam3/config/RGB-D/RealSense_D455.yaml
AUTO_SETTINGS=$PKG/orb_slam3/config/RGB-D/RealSense_D455_auto.yaml
RGB_TOPIC=/camera/camera/color/image_raw
DEPTH_TOPIC=/camera/camera/aligned_depth_to_color/image_raw
CAMERA_INFO=/camera/camera/color/camera_info

# Drop a possibly stale ros2 daemon cache so "topic advertised" checks are real.
ros2 daemon stop >/dev/null 2>&1

wait_for_topic() {
  local topic="$1" tries="$2"
  for _ in $(seq 1 "$tries"); do
    if ros2 topic list 2>/dev/null | grep -qx "$topic"; then return 0; fi
    sleep 0.5
  done
  return 1
}

# A topic can be listed but deliver nothing (stale/hung publisher), so decide
# whether a driver is already working by attempting to receive real frames.
echo "[run] preflight: checking whether a driver is already delivering data (4s)"
if python3 "$SCRIPT_DIR/check_camera_streams.py" \
     --color-topic "$RGB_TOPIC" --depth-topic "$DEPTH_TOPIC" --timeout 4 >/dev/null 2>&1; then
  echo "[run] streams already live -> not starting another driver"
  START_DRIVER=false
fi

DRIVER_PID=""
if [ "$START_DRIVER" = "true" ]; then
  # Only treat the topic as a real driver if it actually has a PUBLISHER.
  # "ros2 topic list" also lists topics that only have subscribers, e.g. a
  # leftover SLAM node, which would otherwise look like a hung driver.
  pub_count=$(ros2 topic info "$RGB_TOPIC" 2>/dev/null | awk '/Publisher count:/ {print $3}')
  if [ "$pub_count" != "" ] && [ "$pub_count" != "0" ] && [ "$FORCE_DRIVER_START" != "true" ]; then
    echo "[run] ERROR: $RGB_TOPIC is advertised but NOT delivering data."
    echo "[run]        A previous driver is likely hung / the daemon cache is stale."
    echo "[run]        Clean it up and retry:"
    echo "[run]          pkill -f realsense2_camera_node"
    echo "[run]          ros2 daemon stop"
    echo "[run]        (or re-run with FORCE_DRIVER_START=true to start anyway)"
    exit 1
  fi
  echo "[run] starting realsense2_camera (aligned depth, 640x480x30, initial_reset)"
  ros2 launch realsense2_camera rs_launch.py \
    enable_color:=true rgb_camera.color_profile:=640x480x30 \
    enable_depth:=true depth_module.depth_profile:=640x480x30 \
    align_depth.enable:=true enable_sync:=true \
    enable_infra:=false enable_infra1:=false enable_infra2:=false \
    pointcloud.enable:=true initial_reset:=true &
  DRIVER_PID=$!
fi

cleanup() {
  if [ -n "$DRIVER_PID" ]; then kill "$DRIVER_PID" 2>/dev/null; fi
}
trap cleanup EXIT

echo "[run] waiting for camera topics (up to 30s)..."
wait_for_topic "$RGB_TOPIC" 60 && echo "[run] color ok" || echo "[run] WARN: $RGB_TOPIC not found"
wait_for_topic "$DEPTH_TOPIC" 60 && echo "[run] aligned depth ok" || echo "[run] WARN: $DEPTH_TOPIC not found"

echo "[run] checking stream validity"
python3 "$SCRIPT_DIR/check_camera_streams.py" \
  --color-topic "$RGB_TOPIC" --depth-topic "$DEPTH_TOPIC" --timeout 10
STREAM_RC=$?
if [ "$STREAM_RC" != "0" ]; then
  echo "[run] WARNING: camera streams are not healthy (rc=$STREAM_RC)."
  echo "[run]          Check realsense-viewer / USB3 port / cable."
fi

if [ "$MONITOR_ONLY" = "true" ]; then
  echo "[run] MONITOR_ONLY: watching depth health for $MONITOR_SAMPLES samples"
  python3 "$SCRIPT_DIR/check_camera_streams.py" \
    --color-topic "$RGB_TOPIC" --depth-topic "$DEPTH_TOPIC" \
    --samples "$MONITOR_SAMPLES" --interval 1.0
  exit $?
fi

SETTINGS="$TEMPLATE"
if [ "$AUTO_INTRINSICS" = "true" ]; then
  echo "[run] generating settings from $CAMERA_INFO"
  if python3 "$SCRIPT_DIR/make_d455_settings.py" \
       --template "$TEMPLATE" --output "$AUTO_SETTINGS" --topic "$CAMERA_INFO"; then
    SETTINGS="$AUTO_SETTINGS"
  else
    echo "[run] WARN: could not read camera_info, using nominal intrinsics"
  fi
fi

echo "[run] starting orbslam3_dense_ros2 (pangolin=$ENABLE_PANGOLIN, settings=$SETTINGS)"
ros2 run orbslam3_dense_ros2 orb_slam3_main --ros-args \
  -p vocabulary_file:="$VOC" \
  -p settings_file:="$SETTINGS" \
  -p enable_pangolin:="$ENABLE_PANGOLIN" \
  -p rgb_topic:="$RGB_TOPIC" \
  -p depth_topic:="$DEPTH_TOPIC"
