"""
UniCalib 标定 Launch 文件
用法:
  ros2 launch unicalib calibrate.launch.py \
      config_path:=/path/to/config.yaml \
      data_path:=/path/to/data \
      output_dir:=/path/to/output
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("unicalib")
    default_config = os.path.join(pkg_share, "config", "unicalib_config.yaml")

    # 参数声明
    config_path_arg = DeclareLaunchArgument(
        "config_path",
        default_value=default_config,
        description="UniCalib 配置文件路径",
    )
    data_path_arg = DeclareLaunchArgument(
        "data_path",
        default_value="/root/calib_ws/data",
        description="数据目录或 rosbag 路径",
    )
    output_dir_arg = DeclareLaunchArgument(
        "output_dir",
        default_value="/root/calib_ws/results",
        description="标定结果输出目录",
    )
    stage_arg = DeclareLaunchArgument(
        "stage",
        default_value="all",
        description="执行阶段: all | intrinsic | coarse | fine | validate",
    )
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="INFO",
        description="日志级别",
    )

    config_path = LaunchConfiguration("config_path")
    data_path = LaunchConfiguration("data_path")
    output_dir = LaunchConfiguration("output_dir")
    stage = LaunchConfiguration("stage")
    log_level = LaunchConfiguration("log_level")

    # 运行标定脚本
    calib_process = ExecuteProcess(
        cmd=[
            "python3",
            os.path.join(pkg_share, "..", "..", "..", "..",
                         "lib", "unicalib", "run_calibration"),
            "--config", config_path,
            "--data", data_path,
            "--output", output_dir,
            "--stage", stage,
            "--log-level", log_level,
        ],
        output="screen",
        name="unicalib_calibration",
    )

    return LaunchDescription([
        config_path_arg,
        data_path_arg,
        output_dir_arg,
        stage_arg,
        log_level_arg,
        calib_process,
    ])
