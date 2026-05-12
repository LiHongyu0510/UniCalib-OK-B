#!/usr/bin/env python3
import os
import sys
import csv
from datetime import datetime
import struct
import time

def write_mcap_header(f):
    magic = b'MCAP\x00\r\n\x00\n\x00'
    f.write(magic)

def convert_imu_to_mcap(ros1_bag_dir, output_dir):
    csv_file = os.path.join(ros1_bag_dir, 'imu-data.csv')
    if not os.path.exists(csv_file):
        print(f"IMU CSV not found: {csv_file}")
        return None

    output_mcap = os.path.join(output_dir, 'imu.mcap')
    os.makedirs(output_dir, exist_ok=True)

    with open(output_mcap, 'wb') as f:
        write_mcap_header(f)

        with open(csv_file, 'r') as csvf:
            reader = csv.DictReader(csvf)
            count = 0
            for row in reader:
                count += 1

        print(f"IMU: {count} messages")

    metadata = os.path.join(output_dir, 'metadata.yaml')
    with open(metadata, 'w') as f:
        f.write(f"""%YAML:1.0
version: 2
storage_identifier: mcap
relative_file_paths:
  - imu.mcap
topics:
  - name: /imu/data
    type: sensor_msgs/msg/Imu
ros_distro: humble
""")

    print(f"Created: {output_dir}")
    return output_dir

def create_unified_ros2_bag(data_dir, output_name):
    base_dir = os.path.dirname(data_dir)
    output_dir = os.path.join(base_dir, output_name + '_ros2')

    ros1_bag_dirs = {
        'imu': 'school_scooter1.synced.imu',
        'left_camera': 'school_scooter1.synced.left_camera',
        'right_camera': 'school_scooter1.synced.right_camera',
        'lidar': 'school_scooter1.synced.lidar'
    }

    os.makedirs(output_dir, exist_ok=True)

    topics_info = []
    for sensor, bag_dir in ros1_bag_dirs.items():
        full_path = os.path.join(base_dir, bag_dir)
        csv_path = os.path.join(full_path, f'{sensor.replace("_", "-")}-data.csv')
        if os.path.exists(csv_path):
            with open(csv_path, 'r') as f:
                reader = csv.DictReader(f)
                rows = sum(1 for _ in reader)
            topics_info.append((f'/{sensor}', 'sensor_msgs/msg/Imu', rows))

    metadata = os.path.join(output_dir, 'metadata.yaml')
    with open(metadata, 'w') as f:
        f.write(f"""%YAML:1.0
version: 2
storage_identifier: mcap
relative_file_paths: []
topics:
""")
        for topic, msg_type, count in topics_info:
            f.write(f"  - name: {topic}\n    type: {msg_type}\n    message_count: {count}\n")

    print(f"Created unified ROS2 bag structure: {output_dir}")
    print("Topics:")
    for topic, msg_type, count in topics_info:
        print(f"  {topic}: {msg_type} ({count} msgs)")

    return output_dir

if __name__ == '__main__':
    data_dir = '/home/lihongyu/Unicalib/UniCalib-OK/data/school_scooter'
    output_name = 'school_scooter1'

    result = create_unified_ros2_bag(data_dir, output_name)
    print(f"\nDone: {result}")
    print("\nNote: 详细内容需要用完整ROS2工具链转换")