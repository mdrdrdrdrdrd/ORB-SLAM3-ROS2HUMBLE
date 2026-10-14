# orbslam3_dense_ros2 本机配置记录（ROS 2 Humble / 无 CUDA）

依据 [subooku/orbslam3_dense_ros2 README_CN](https://github.com/subooku/orbslam3_dense_ros2/blob/main/README_CN.md)，
按本机环境（Ubuntu 22.04、ROS 2 Humble、Intel 核显、无 CUDA）调整后的安装与运行记录。

## 1. 依赖状态（本机已全部满足）

| 项目 | 状态 |
| --- | --- |
| Eigen3 (`libeigen3-dev`) | 已安装 |
| Pangolin | 已安装（`/usr/local/lib/cmake/Pangolin/PangolinConfig.cmake`） |
| OpenCV | 用系统自带 4.5.4（README 的 4.10 是可选项，CPU 版 4.5.4 可编译运行） |
| PCL / pcl_ros / pcl_conversions | 已安装 |
| boost system/serialization、libssl、libsuitesparse、libglew、libepoxy | 已安装 |
| message_filters、cv_bridge、image_transport、tf2_geometry_msgs | 已安装 |
| Realsense D455 驱动 | `ros-humble-realsense2-camera` 4.58.4，可用 |

所以 **不需要再跑 README 里那一大段 `sudo apt install` 和 OpenCV 编译**。

## 2. 代码改动（针对 Humble + 本机）

1. `src/main.cpp` 和 `include/orbslam3_dense_ros2/ros2_slam_publisher.h`：
   把 `#include <cv_bridge/cv_bridge.hpp>` 改为 `#include <cv_bridge/cv_bridge.h>`
   （Humble 的 cv_bridge 只有 `.h`，没有 `.hpp`）。
2. `src/main.cpp` 里写死的作者路径 `/home/zgf/...` 改成本机路径，默认配置改成 D455。
3. 新增 `orb_slam3/config/RGB-D/RealSense_D455.yaml`（基于 D435i 模板，改内参、基线 95mm、`Camera.RGB: 0`）。
4. 新增一键脚本 `scripts/run_d455_dense.sh`。
5. 修复 `src/main.cpp` 的位置参数解析：改用 `rclcpp::remove_ros_arguments()`，否则 `--ros-args -p ...` 会被误当成文件路径（报 `Failed to open settings file at: -p`）。
6. 修复退出时崩溃：`~ORBSlamNode` 未 join 数据处理线程、`~ROS2SlamPublisher` 漏 join `pointcloud_mapping_thread_`，Ctrl+C 时会 `terminate called without an active exception`。

## 3. 编译

```bash
source /opt/ros/humble/setup.bash
cd /home/ros/ros2_ws
MAKEFLAGS=-j4 colcon build --packages-select orbslam3_dense_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

（15G 内存，用 `MAKEFLAGS=-j4` 限制并行，避免 OOM。）

## 4. 运行

一键：

```bash
cd /home/ros/ros2_ws/src/orbslam3_dense_ros2
./scripts/run_d455_dense.sh                       # 无 Pangolin 窗口
ENABLE_PANGOLIN=true ./scripts/run_d455_dense.sh  # 带 Pangolin 可视化
START_DRIVER=false ./scripts/run_d455_dense.sh    # 驱动已在别处运行
```

手动分两步：

```bash
# 终端 1：驱动，必须开 align_depth
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true rgb_camera.color_profile:=640x480x30 \
  enable_depth:=true depth_module.depth_profile:=640x480x30 \
  align_depth.enable:=true enable_sync:=true pointcloud.enable:=true

# 终端 2：SLAM
source /home/ros/ros2_ws/install/setup.bash
ros2 run orbslam3_dense_ros2 orb_slam3_main --ros-args \
  -p vocabulary_file:=/home/ros/ros2_ws/src/orbslam3_dense_ros2/orb_slam3/Vocabulary/ORBvoc.txt.bin \
  -p settings_file:=/home/ros/ros2_ws/src/orbslam3_dense_ros2/orb_slam3/config/RGB-D/RealSense_D455.yaml \
  -p enable_pangolin:=false \
  -p rgb_topic:=/camera/camera/color/image_raw \
  -p depth_topic:=/camera/camera/aligned_depth_to_color/image_raw
```

## 5. 输出话题

节点名 `orb_slam3_ros2_node` / `orbslam3_ros2_publisher`，发布：

- `/camera_pose`、`/vehicle_pose`（geometry_msgs/PoseStamped）
- `/camera_path`、`/vehicle_path`（nav_msgs/Path）
- `/point_cloud_all`、`/point_cloud_ref`、`/cloudPointMapping`（sensor_msgs/PointCloud2）
- `/current_frame`（sensor_msgs/Image）
- TF：`map -> slam_map` 等

## 7. 验证结果

- 编译成功（colcon，约 5 分钟），产物齐全、`ldd` 依赖全部解析。
- 启动自检：词袋加载、D455 配置解析、Atlas/SlamDataProcess/发布器启动、SIGINT 干净退出。
- **无相机时的端到端验证**：`scripts/test_rgbd_pattern.py` 用合成 RGB-D 发布到与 D455 完全同名的话题，
  节点收到同步帧并运行 `TrackRGBD`，`/camera_pose` 约 26 Hz、`/current_frame` 有数据。
- 真实 D455：本机驱动可用且话题名一致；沙箱无 `/dev/bus/usb`，真实相机运行请在你自己终端执行 `./scripts/run_d455_dense.sh`。

无相机自测：

```bash
source /home/ros/ros2_ws/install/setup.bash
python3 /home/ros/ros2_ws/src/orbslam3_dense_ros2/scripts/test_rgbd_pattern.py &
ros2 run orbslam3_dense_ros2 orb_slam3_main --ros-args -p enable_pangolin:=false
```

## 6. 注意事项

- 相机话题是 **双重命名空间** `/camera/camera/...`（`camera_namespace` 和 `camera_name` 都叫 camera）。
- 深度必须用 **对齐后** 的 `/camera/camera/aligned_depth_to_color/image_raw`，所以驱动要 `align_depth.enable:=true`。
- `RealSense_D455.yaml` 里的内参是标称值，建议用 `rs-enumerate-devices -c` 或相机 `camera_info` 里的实际值替换。
- 想用 Pangolin 窗口需要图形界面；通过 SSH 或无显示环境请保持 `enable_pangolin:=false`。
- 系统 OpenCV 4.5.4 与 README 的 4.10 有 API 差异，本项目仅用到通用接口，4.5.4 可正常编译（如后续模块报 OpenCV API 错误，再按主 README 编译 4.10）。
## 8. 真机运行问题与修复

真机首跑出现：驱动 `Depth stream start failure, Hardware Error` → SLAM 反复 `Fail to track local map` → `Creation of new map` 后 **Segmentation fault**。

根因与修复：

| 问题 | 位置 | 修复 |
| --- | --- | --- |
| 重建地图瞬间 `mpAtlas->GetAllKeyFrames()` 为空，`vpKFs[0]` 越界 → 崩溃 | `SlamDataProcess::UpdateTrajectory()` | 增加空判断提前返回 + 空指针保护 |
| 轨迹向量每轮只增不清，无限增长 | 同上 | 每轮 clear() 后重建 |
| 点云缓存存裸 `KeyFrame*`，地图重建后可能悬空 | `PointClouds::generatePointCloud()` | 改为按指针判断是否属于当前地图，并剔除陈旧项 |
| 深度流启动 Hardware Error | 驱动 | 启动加 `initial_reset:=true` |

验证（无相机）：`scripts/test_rgbd_pattern.py --ros-args -p blank_after:=80` 先给纹理、后给空白帧强制丢失跟踪 →
日志出现 `Creation of new map with last KF id: 13`，节点继续运行到 449 帧，进程存活，无 segfault。

真机排错建议：

1. 先用 `realsense-viewer` 确认 Depth/Color 都能正常出图（排查 Hardware Error 是否真的影响深度）。
2. 用 `rs-enumerate-devices -c` 的实际彩色内参替换 `RealSense_D455.yaml` 的 `Camera1.*`。
3. 建图时放慢移动、对准有纹理的场景；RGB-D 初始化成功会打印 `New Map created with N points`。
4. 若仍频繁丢失，用 `rqt_image_view` 看 `/camera/camera/aligned_depth_to_color/image_raw`，确认深度不是全 0。
## 9. 内参自动化与输出验证

- 新增 `scripts/make_d455_settings.py`：订阅 `/camera/camera/color/camera_info`，把真实 `fx/fy/cx/cy` 与畸变系数写入 `RealSense_D455_auto.yaml`（以 `RealSense_D455.yaml` 为模板）。
- `scripts/run_d455_dense.sh` 现在会：等待 color / aligned_depth 话题就绪（最多 30s）→ 自动生成实际内参配置 → 用该配置启动 SLAM。读取失败则回退到标称模板；`AUTO_INTRINSICS=false` 可关闭自动生成。
- 无相机验证：合成 RGB-D + camera_info 下，脚本打印 `[run] color ok / aligned depth ok / generating settings from ...`，SLAM 用 `RealSense_D455_auto.yaml` 启动并 `New Map created with 1253 points`。
- 输出话题实测（合成数据）：`/camera_pose` ~24 Hz、`/vehicle_pose` ~24 Hz、`/camera_path` ~6 Hz、`/point_cloud_all` ~2.5 Hz、`/current_frame` ~14 Hz；`/cloudPointMapping`（稠密点云）一帧 **282480 点**，正常发布（`ros2 topic hz` 因点太多显示不出频率）。
## 10. 预检与僵死驱动处理

- 新增 `scripts/check_camera_streams.py`：真实订阅 color/aligned-depth，报告分辨率、编码、深度非零比例；返回码 0=正常、1=无消息、2=深度全 0。
- `scripts/run_d455_dense.sh` 现在会先 `ros2 daemon stop` 刷新图缓存，再用**真实收帧**判断“驱动是否已在工作”，不再只看话题是否存在。

- 若话题挂着却收不到帧（僵死发布者 / 过期缓存），脚本**直接报错并退出**，提示：
  `pkill -f realsense2_camera_node` + `ros2 daemon stop`，不会重复起驱动去抢相机（抢相机会触发 Hardware Error）。`FORCE_DRIVER_START=true` 可强制启动。
- 压力测试：`scripts/test_rgbd_pattern.py -p blank_cycle:=160` 反复丢失跟踪，日志出现 6 张地图（id 0..5），节点全程存活，无 segfault。

### 关于重复安装的 librealsense

本机存在两份 librealsense2：

- `/usr/local/lib/librealsense2.so.2.58.4`（早前从 master 源码安装，回收站里还有对应 build 目录）
- `/opt/ros/humble/lib/x86_64-linux-gnu/librealsense2.so.2.58.4`（ROS deb）

`source /opt/ros/humble/setup.bash` 后，驱动与所有 `rs-*` 工具都加载 **ROS deb 版**（`LD_LIBRARY_PATH` 优先），所以那份 master 构建当前没被用到，不是 Hardware Error 的直接原因；但官方建议避免重复安装，可清理：
```bash
sudo rm -f /usr/local/lib/librealsense2.so*
sudo ldconfig
```

## 11. 真机跟踪失败定位（深度流疑似中途失效）

真机实测（自动内参已生效）：

- 深度编码 `16UC1`，启动时 `nonzero=41.5%`，有效
- 实际内参 `fx=387.41 fy=386.77 cx=318.80 cy=237.72`
- 仍然 `New Map created with 456 points` → 立刻 `TRACK_REF_KF: Less than 15 matches!!` → 持续 `Fail to track local map!`
- 出问题前姿态只动了 7mm，排除“快速运动”
- 驱动日志里仍有 `Depth stream start failure, Hardware Error`

推断：**深度流在初始化后失效**（启动时有效，几秒后变 0），导致局部地图跟踪崩溃。

### 如何确认

驱动起来后连续监测深度（每秒一次，共 20 次）：

```bash
cd /home/ros/ros2_ws/src/orbslam3_dense_ros2
MONITOR_ONLY=true MONITOR_SAMPLES=20 ./scripts/run_d455_dense.sh
```

或手动：

```bash
ros2 launch realsense2_camera rs_launch.py enable_color:=true rgb_camera.color_profile:=640x480x30 \
  enable_depth:=true depth_module.depth_profile:=640x480x30 align_depth.enable:=true \
  enable_sync:=true pointcloud.enable:=true initial_reset:=true
python3 scripts/check_camera_streams.py --samples 20 --interval 1.0
```

若中途出现 `nonzero=0` 或 `max=0`，就证实深度流掉了。

### 可能的处置

1. **升级 D455 固件**：当前 `5.15.1.55` 相对 SDK 2.58.4 偏旧，用 `realsense-viewer` 或 `rs-fw-update -f <bin>` 升级到 5.16+。
2. **换 USB3 口 / 原装线 / 不用 Hub / 单独供电**，排除供电与带宽。
3. **降低深度帧率**：`depth_module.depth_profile:=640x480x15`、`rgb_camera.color_profile:=640x480x15`。
4. 关掉暂不需要的点云：`pointcloud.enable:=false`。
5. 确认没有第二个进程占用相机（新脚本已避免重复起驱动）。
## 12. 无效深度 (65535) 修正

真机监测发现深度 `nonzero~61%` 且**每帧 max=65535**。`65535=0xFFFF` 是 RealSense 的“无深度”标记，
而 ORB-SLAM3 的 `Frame::ComputeStereoFromRGBD` 只判断 `d>0`，会把 65535 当成 **65.535 m** 有效深度，生成远距离垃圾地图点。

修复（`src/main.cpp`）：深度转 16UC1(mm) 后，把 `> 10000`（>10m，含 65535）的像素置 0。

验证（合成数据人为加 40% 65535）：

- 修复前：`New Map created with 2012 points`（含约 840 个 65m 垃圾点）
- 修复后：`New Map created with 1172 points`，0 次 `Fail to track`，`/camera_pose` ~23 Hz

该修正剔除垃圾点，但**不一定是真机跟踪失败的唯一原因**（40% 65535 在合成场景里并未导致失败）。
真机场景需用 bag 复现：新增 `scripts/record_d455.sh`（录制 10s 到 `data/d455_bag`），录制与回放流程已验证。

### 下一步：录 bag 复现

```bash
cd /home/ros/ros2_ws/src/orbslam3_dense_ros2
DURATION=10 ./scripts/record_d455.sh
```

把相机对着出问题的场景、按平时的方式移动。bag 落在工作区，可离线回放复现。

## 13. 诊断工具与压力测试

- `scripts/check_camera_streams.py --features`：额外用 OpenCV 统计彩色帧的 ORB 关键点数量，判断场景是否太缺纹理：
  ```bash
  python3 scripts/check_camera_streams.py --samples 3 --features
  ```
  合成纹理场景能到 2000（nfeatures 上限）；真机若只有几十~一两百，说明场景纹理太少，需要换到有纹理的场景。
- 叠加压力测试：`blank_cycle:=160`（反复丢跟踪重建地图）+ `depth_invalid_frac:=0.4`（40% 无效深度），
  75 秒内触发 6 张地图（id 0..5），节点全程存活，无 segfault。

## 14. 离线回放复现（scripts/replay_d455.sh）

- 新增 `scripts/replay_d455.sh`：先从 bag 短暂回放取 `camera_info` 生成真实内参，再启动节点、**无 loop 回放一次**。
  ```bash
  BAG=/home/ros/ros2_ws/data/d455_bag ENABLE_PANGOLIN=false ./scripts/replay_d455.sh
  ```
- 重要：**不要用 `--loop` 回放 bag**。回环会让 header 时间戳回跳，ORB-SLAM3 会反复重建地图并报
  `TRACK_REF_KF: Less than 15 matches`——那是回放假象（已实测：loop 触发假失败，无 loop 正常）。
- 验证（合成 bag）：`New Map created with 2012 points`、`Received synchronized RGB-D frame #60`、0 次 `Fail to track`。

## 15. RGB-D 时间同步队列

`src/main.cpp` 里 `ApproximateTime` 的队列原来是 `SyncPolicy(1)`，对 30 Hz 的两路流太紧：稍晚到的一帧会被丢、
彩色与深度可能配错（真机上会表现为 3D 点错乱、跟踪迅速丢失）。已改为 `SyncPolicy(10)`。

合成数据验证：`New Map created with 2012 points`、0 次 `Fail to track`、`/camera_pose` 25 Hz、进程存活。

## 16. 真机 bag 离线复现与结论（决定性）

用 `scripts/record_d455.sh` 录制 19.5s 真机 bag（420 MB，298 彩色 + 270 深度帧），在沙箱用 `scripts/replay_d455.sh` 复现。

### 根因链

- 场景本身**不缺纹理**：多数帧 ~1900 个 ORB 特征，其中 ~1600 个带有效深度。
- 第 45~120 帧有一段明显**运动模糊 / 对准过曝白桌面**的坏镜头：特征掉到 8~579，有效深度特征掉到 0~663。
- ORB-SLAM3 在坏段丢失跟踪并反复重建地图；**旧代码在此处 segfault**（已修）。

### 修复后表现（真实 bag 回放）

- 坏段之后自动恢复：`New Map created with 1649 points`，`TrackRGBD frame #196` 继续跟踪。
- `/camera_pose` ~28 Hz、`/vehicle_pose` ~28 Hz、`/camera_path` ~6 Hz。
- **稠密点云 `/cloudPointMapping` width = 8374 点**（真实数据）。
- 全程无 `Segmentation / Aborted / terminate`。

### 结论

- **Segmentation fault 是致命 bug，已修复**（多次重建地图 + 真实 bag 均不再崩溃）。
- 真机跟踪中断是**运动模糊导致的瞬时数据问题**；节点现在能自动恢复，而旧代码会崩溃。
- 回放 bag 不要用 `--loop`（时间戳回跳会造成假的跟踪失败）。

### 残留项

驱动仍打印 `Depth stream start failure, Hardware Error` 的 WARNING，但深度实测 61% 有效、并未真正失效；
已加 `initial_reset:=true` 缓解。要彻底消除，建议升级 D455 固件（5.15.1.55 → 新版），固件偏旧是官方指出的常见诱因。

## 17. scripts/run_d455_dense.sh 发布者判定修正 + 实机端到端成功

- **问题**：旧脚本用 `ros2 topic list` 判断“驱动是否在跑”，但它对**只有订阅者**的话题也会列出。
  之前回放 bag 残留的 `orb_slam3_main` 订阅了 `/camera/camera/...`，于是脚本误报
  `ERROR: ... advertised but NOT delivering data` 并拒绝启动驱动。
- **修复**：改用 `ros2 topic info <topic>` 的 **Publisher count**，只有存在真正发布者才视为驱动在跑。
- **实机端到端实测**（沙箱已可访问 USB 相机，用 `scripts/run_d455_dense.sh` 全流程）：
  - 驱动启动 → 流检查 `depth nonzero=81.6%`、`color rgb8 640x480`
  - 自动内参 `fx=387.52 fy=386.88 cx=318.80 cy=237.72`
  - `New Map created with 1655 points`，收到同步帧 #30..#180，**无 `TRACK_REF_KF` / `Fail to track`**
  - `/camera_pose` **27.2 Hz（真实相机）**，`RUN_ALIVE`，无 crash
- 稠密点云在真实 bag 回放中实测 `width=8374`。
## 18. 一键 launch（Pangolin 窗口 + RViz 点云）

新增 `launch/orbslam3_d455.launch.py`，一条命令启动：驱动 + 自动实际内参 + SLAM（Pangolin 窗口）+ RViz（订阅稠密点云）。

```bash
source /opt/ros/humble/setup.bash
source /home/ros/ros2_ws/install/setup.bash
ros2 launch orbslam3_dense_ros2 orbslam3_d455.launch.py
```

参数：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `start_driver` | true | 是否同时启动 realsense2_camera |
| `enable_pangolin` | true | 显示 Pangolin 窗口 |
| `rviz` | true | 启动 RViz2 |
| `auto_intrinsics` | true | 从 camera_info 生成实际内参 |
| `initial_reset` | true | 启动时复位相机 |
| `driver_delay` | 6.0 | 等 camera_info 的秒数 |
| `settings_file` | 标称模板 | `auto_intrinsics:=false` 时使用 |
| `rgb_topic` / `depth_topic` | `/camera/camera/...` | 话题覆盖 |

实现：驱动启动后延迟 `driver_delay` 秒运行 `scripts/make_d455_settings.py`（带 `--fallback`），
写出 `~/.ros/orbslam3_dense_ros2/RealSense_D455_auto.yaml`，进程退出后再启动 SLAM 与 RViz（`OnProcessExit` 链）。

RViz 配置 `rviz/orbslam3_d455.rviz`：Fixed Frame `odom`，订阅 `/cloudPointMapping`（稠密点云，RGB）、
`/point_cloud_all`、`/point_cloud_ref`、`/camera_path`、`/current_frame`。

CMake 已把 `orb_slam3/{config,Vocabulary}`、`launch/`、`rviz/`、`scripts/` install 到 share，`ros2 launch` 直接可用。

验证：`ros2 launch ... rviz:=false enable_pangolin:=false` 实机跑通（自动内参 `fx=387.10`、`New Map created with 1102 points`、连续收帧无失败）；
RViz 配置在真实显示上加载无报错。

## 19. 稠密点云漂移 / 重影修复

**现象**：建图时之前采集的点被“重新采集”并整体偏移。

**根因**：`PointClouds::generatePointCloud()` 在每次插入关键帧时，用每个关键帧**当前的（已被局部 BA / 回环优化过的）位姿**
把整张点云从头重建。位姿一更新，旧点全部被重新投影到新位置；RViz 的 PointCloud2 又会累积显示，于是看起来是“重复叠加 + 偏移”。

**修复**（`orb_slam3/{include,src}/PointClouds.*`、`SlamDataProcess.*`）：

- 每个关键帧的点云在插入时**冻结其相机→世界位姿**（`PointCloudData::T_wc`），重建时用冻结位姿，旧点不再随优化移动。
- 增加可选体素下采样（`DenseMap.voxel_leaf_size`）合并重叠/重影点，控制点云规模。

配置（`orb_slam3/config/RGB-D/RealSense_D455.yaml`）：

```yaml
DenseMap.freeze_pose: true       # false = 用最新位姿重建（回环会整体校正，但会看到跳变）
DenseMap.voxel_leaf_size: 0.01   # 米；0 = 关闭
```

## 20. 保存点云为 PCD

RViz2 没有导出点云的按钮，用脚本订阅话题转存。

### 方法 A：脚本直接存（推荐）

`scripts/save_cloud.py` 订阅一个 PointCloud2 话题并写成 PCD（ASCII，字段 `x y z [rgb]`）：

```bash
source /opt/ros/humble/setup.bash
source /home/ros/ros2_ws/install/setup.bash
python3 /home/ros/ros2_ws/src/orbslam3_dense_ros2/scripts/save_cloud.py \
  --topic /cloudPointMapping --output /home/ros/ros2_ws/data/dense_map.pcd
```

| 选项 | 默认 | 说明 |
| --- | --- | --- |
| `--topic` | /cloudPointMapping | 要保存的话题 |
| `--output` | data/map.pcd | 输出路径 |
| `--timeout` | 30 | 等消息的秒数 |
| `--frames` | 1 | 快照数量（多张时输出 `map_000.pcd` ...） |
| `--interval` | 2.0 | 快照间隔秒数 |

常用话题：`/cloudPointMapping`（稠密点云）、`/point_cloud_all`（ORB 地图点）、`/point_cloud_ref`。
注意 `/cloudPointMapping` 只在插入关键帧时发布，保存时请缓慢移动相机（或增大 `--timeout`）。

### 方法 B：先录 bag 再转

```bash
# 一边建图一边录
ros2 bag record -o /home/ros/ros2_ws/data/map_bag /cloudPointMapping
# 回放并转存
ros2 bag play /home/ros/ros2_ws/data/map_bag &
python3 /home/ros/ros2_ws/src/orbslam3_dense_ros2/scripts/save_cloud.py \
  --topic /cloudPointMapping --output /home/ros/ros2_ws/data/dense_map.pcd
```

### 查看 / 转换

```bash
pcl_viewer /home/ros/ros2_ws/data/dense_map.pcd          # 需 apt install pcl-tools
pcl_convert_pcd_ascii_binary in.pcd out.pcd 1            # 文本 <-> 二进制
```

验证：用 `pcl_ros pcd_to_pointcloud` 回灌保存的 PCD，再存一次点数一致（18919 -> 18919）。


**权衡**：冻结位姿后，回环/BA 对**稠密点云**的校正不再追溯（`/point_cloud_all` 里的 ORB 地图点仍会校正）。
若希望稠密图也跟随回环整体校正，把 `freeze_pose` 设为 `false`。

**验证**：实机运行，`/cloudPointMapping` 稳定在 ~45k~50k 点，无崩溃。

> 注意区分两类“漂移”：本修复解决的是“旧点被重新投影/重采”的视觉跳变；
> 若地图整体逐渐弯曲（相对真实世界的轨迹漂移），那是 SLAM 精度问题，需靠更好的纹理/照明、放慢运动与回环来改善。







