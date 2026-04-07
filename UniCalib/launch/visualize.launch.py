"""
UniCalib 可视化 Launch 文件
启动 RViz2 显示标定结果（点云 + 图像叠加）
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("unicalib")

    results_dir_arg = DeclareLaunchArgument(
        "results_dir",
        default_value="/root/calib_ws/results",
        description="标定结果目录",
    )
    data_path_arg = DeclareLaunchArgument(
        "data_path",
        default_value="/root/calib_ws/data",
        description="数据路径",
    )
    config_path_arg = DeclareLaunchArgument(
        "config_path",
        default_value=os.path.join(pkg_share, "config", "unicalib_config.yaml"),
    )

    results_dir = LaunchConfiguration("results_dir")
    data_path = LaunchConfiguration("data_path")
    config_path = LaunchConfiguration("config_path")

    viz_process = ExecuteProcess(
        cmd=[
            "python3",
            os.path.join(pkg_share, "..", "..", "..", "..",
                         "lib", "unicalib", "visualize_results"),
            "--config", config_path,
            "--results", results_dir,
            "--data", data_path,
        ],
        output="screen",
        name="unicalib_visualize",
    )

    # 启动 RViz2
    rviz_config = os.path.join(pkg_share, "config", "unicalib.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config] if os.path.exists(rviz_config) else [],
        output="screen",
    )

    return LaunchDescription([
        results_dir_arg,
        data_path_arg,
        config_path_arg,
        viz_process,
        rviz_node,
    ])
