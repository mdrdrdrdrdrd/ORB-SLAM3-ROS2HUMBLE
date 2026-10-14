"""One-click bringup for orbslam3_dense_ros2 with a RealSense D455.

Starts:
  1. realsense2_camera (aligned depth, 640x480x30)
  2. a settings generator that derives the real intrinsics from camera_info
  3. orb_slam3_main with the Pangolin viewer window
  4. RViz2 with a config that subscribes to the dense point cloud

Usage:
  ros2 launch orbslam3_dense_ros2 orbslam3_d455.launch.py
  ros2 launch orbslam3_dense_ros2 orbslam3_d455.launch.py rviz:=false
  ros2 launch orbslam3_dense_ros2 orbslam3_d455.launch.py enable_pangolin:=false
  ros2 launch orbslam3_dense_ros2 orbslam3_d455.launch.py start_driver:=false
  ros2 launch orbslam3_dense_ros2 orbslam3_d455.launch.py auto_intrinsics:=false
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context, *args, **kwargs):
    pkg = FindPackageShare("orbslam3_dense_ros2")
    camera_pkg = FindPackageShare("realsense2_camera")

    vocab = PathJoinSubstitution([pkg, "orb_slam3", "Vocabulary", "ORBvoc.txt.bin"])
    template = PathJoinSubstitution(
        [pkg, "orb_slam3", "config", "RGB-D", "RealSense_D455.yaml"])
    auto_settings = PathJoinSubstitution(
        [EnvironmentVariable("HOME"), ".ros", "orbslam3_dense_ros2",
         "RealSense_D455_auto.yaml"])
    rviz_config = PathJoinSubstitution([pkg, "rviz", "orbslam3_d455.rviz"])
    settings_gen = PathJoinSubstitution([pkg, "scripts", "make_d455_settings.py"])
    camera_info_topic = PathJoinSubstitution(
        ["/", LaunchConfiguration("camera_namespace"), LaunchConfiguration("camera_name"),
         "color", "camera_info"])

    auto_intrinsics = context.launch_configurations.get("auto_intrinsics", "true") == "true"

    slam_node = Node(
        package="orbslam3_dense_ros2",
        executable="orb_slam3_main",
        name="orb_slam3_ros2_node",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "vocabulary_file": vocab,
            "settings_file": (auto_settings if auto_intrinsics
                              else LaunchConfiguration("settings_file")),
            "enable_pangolin": ParameterValue(
                LaunchConfiguration("enable_pangolin"), value_type=bool),
            "rgb_topic": LaunchConfiguration("rgb_topic"),
            "depth_topic": LaunchConfiguration("depth_topic"),
        }],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="orbslam3_rviz",
        arguments=["-d", rviz_config],
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([camera_pkg, "/launch/rs_launch.py"]),
        launch_arguments={
            "camera_namespace": LaunchConfiguration("camera_namespace"),
            "camera_name": LaunchConfiguration("camera_name"),
            "enable_color": "true",
            "rgb_camera.color_profile": "640x480x30",
            "enable_depth": "true",
            "depth_module.depth_profile": "640x480x30",
            "align_depth.enable": "true",
            "enable_sync": "true",
            "enable_infra": "false",
            "enable_infra1": "false",
            "enable_infra2": "false",
            "pointcloud.enable": "true",
            "publish_tf": "true",
            "initial_reset": LaunchConfiguration("initial_reset"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("start_driver")),
    )

    actions = [driver]

    if auto_intrinsics:
        gen = ExecuteProcess(
            cmd=[
                "python3", settings_gen,
                "--template", template,
                "--output", auto_settings,
                "--topic", camera_info_topic,
                "--timeout", "15",
                "--fallback",
            ],
            output="screen",
        )
        actions.append(TimerAction(
            period=LaunchConfiguration("driver_delay"), actions=[gen]))
        # Start SLAM + RViz only after the settings file has been produced.
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=gen, on_exit=[slam_node, rviz_node])))
    else:
        actions.append(slam_node)
        actions.append(rviz_node)

    return actions


def generate_launch_description():
    pkg = FindPackageShare("orbslam3_dense_ros2")
    return LaunchDescription([
        DeclareLaunchArgument("start_driver", default_value="true",
                              description="Also start the realsense2_camera driver."),
        DeclareLaunchArgument("camera_namespace", default_value="camera",
                              description="RealSense driver namespace."),
        DeclareLaunchArgument("camera_name", default_value="camera",
                              description="RealSense camera name."),
        DeclareLaunchArgument("enable_pangolin", default_value="true",
                              description="Show the ORB-SLAM3 Pangolin viewer window."),
        DeclareLaunchArgument("rviz", default_value="true",
                              description="Start RViz2 with the dense point cloud config."),
        DeclareLaunchArgument("auto_intrinsics", default_value="true",
                              description="Generate settings from live camera_info."),
        DeclareLaunchArgument("initial_reset", default_value="true",
                              description="Reset the RealSense device on start."),
        DeclareLaunchArgument("driver_delay", default_value="6.0",
                              description="Seconds to wait before reading camera_info."),
        DeclareLaunchArgument(
            "settings_file",
            default_value=PathJoinSubstitution(
                [pkg, "orb_slam3", "config", "RGB-D", "RealSense_D455.yaml"]),
            description="Settings used when auto_intrinsics:=false."),
        DeclareLaunchArgument("rgb_topic",
                              default_value="/camera/camera/color/image_raw",
                              description="RGB image topic."),
        DeclareLaunchArgument("depth_topic",
                              default_value="/camera/camera/aligned_depth_to_color/image_raw",
                              description="Aligned depth image topic."),
        OpaqueFunction(function=_launch_setup),
    ])
