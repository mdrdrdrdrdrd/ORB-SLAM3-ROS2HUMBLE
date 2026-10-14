# ORB-SLAM3 稠密 RGB-D · ROS 2 Humble

基于 ORB-SLAM3 的 **RGB-D 视觉 SLAM + 稠密建图** ROS 2 功能包（`orbslam3_dense_ros2`），
在 **Intel RealSense D455** 上完成适配与实测：实时输出相机/车体位姿、轨迹与**稠密彩色点云**。

> - 上游项目英文 README：[`README_UPSTREAM.md`](README_UPSTREAM.md)
> - 上游简体中文 README：[`README_CN.md`](README_CN.md)
> - 本工作区完整安装/调试/排错记录：[`docs/orbslam3_dense_ros2_setup.md`](docs/orbslam3_dense_ros2_setup.md)

## 特性

- 基于 ORB-SLAM3 的 RGB-D 视觉 SLAM，支持局部 BA、回环与重定位
- 实时输出**稠密彩色点云** `/cloudPointMapping`
- **一键启动**：RealSense 驱动 + 自动读取相机内参 + Pangolin 可视化 + RViz
- 配套录制/回放/诊断/点云保存等脚本

## 话题

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/camera_pose`、`/vehicle_pose` | `geometry_msgs/PoseStamped` | 相机 / 车体位姿 |
| `/camera_path`、`/vehicle_path` | `nav_msgs/Path` | 轨迹 |
| `/point_cloud_all`、`/point_cloud_ref` | `sensor_msgs/PointCloud2` | ORB 地图点 / 当前帧点 |
| `/cloudPointMapping` | `sensor_msgs/PointCloud2` | 稠密建图点云 |
| `/current_frame` | `sensor_msgs/Image` | 当前帧图像 |

## 依赖

- Ubuntu 22.04 + **ROS 2 Humble**
- OpenCV、PCL、Eigen、**Pangolin**（可视化）、`cv_bridge`、`image_transport`、`pcl_ros`
- Intel RealSense：`librealsense2` + `realsense2_camera`

## 构建

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select orbslam3_dense_ros2
source install/setup.bash
```

## 运行（RealSense D455 一键启动）

```bash
# 驱动 + 自动内参 + SLAM（Pangolin 窗口）+ RViz
ros2 launch orbslam3_dense_ros2 orbslam3_d455.launch.py

# 不显示窗口（SSH / 无界面）
ros2 launch orbslam3_dense_ros2 orbslam3_d455.launch.py rviz:=false enable_pangolin:=false

# 驱动已在别处运行 / 用标称内参
ros2 launch orbslam3_dense_ros2 orbslam3_d455.launch.py start_driver:=false auto_intrinsics:=false
```

## 辅助脚本（`scripts/`）

- `run_d455_dense.sh` 一键运行
- `record_d455.sh` / `replay_d455.sh` 录制 / 离线回放
- `check_camera_streams.py` 流与特征诊断
- `save_cloud.py` 把点云话题保存为 PCD
- `make_d455_settings.py` 由 `camera_info` 生成内参配置
- `test_rgbd_pattern.py` 合成 RGB-D 图源（无相机时）
- `analyze_bag.py` 离线分析 bag

## 稠密点云稳定性

配置项 `DenseMap.freeze_pose`（默认 `true`）会**冻结关键帧插入时的位姿**，
避免局部 BA / 回环更新位姿时旧点被重新投影，从而出现“旧点被重采并偏移/重影”。
设为 `false` 则稠密图跟随回环整体校正（更一致，但会看到跳变）。

## 目录结构

```
orbslam3_dense_ros2/
├── CMakeLists.txt  package.xml  LICENSE  run_rgbd.sh
├── include/  src/                 # 节点与库
├── launch/orbslam3_d455.launch.py # 一键启动
├── rviz/orbslam3_d455.rviz        # RViz 配置
├── scripts/                       # 运行/录制/诊断/存点云脚本
├── assets/
├── orb_slam3/                     # ORB-SLAM3 源码 + Vocabulary + 配置
└── docs/                          # 本工作区安装/调试记录
```

## 许可

GPL-3.0（沿用 ORB-SLAM3）。

## 致谢

[ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)、[ros2_orb_slam3](https://github.com/Mechazo11/ros2_orb_slam3)、
[Map_ORBSLAM_ROS](https://github.com/LeonardoDiCaprio1/Map_ORBSLAM_ROS) 等开源项目。
