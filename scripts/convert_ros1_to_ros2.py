#!/usr/bin/env python3
import os
import sys
import time
from bagpy import bagreader

def convert_ros1_to_ros2(bag_path, output_dir=None):
    bag_path = os.path.abspath(bag_path)
    bag_dir = os.path.dirname(bag_path)
    bag_name = os.path.basename(bag_path).replace('.bag', '')

    output_dir = output_dir or os.path.join(bag_dir, bag_name + '_ros2')

    print(f"Reading: {bag_path}")
    b = bagreader(bag_path)

    topics_info = []
    for idx, row in b.topic_table.iterrows():
        topic = row['Topics']
        msg_type = row['Types']
        count = row['Message Count']
        freq = row['Frequency']
        topics_info.append((topic, msg_type, count, freq))
        print(f"  {topic}: {msg_type} ({count} msgs, {freq:.1f} Hz)")

    os.makedirs(output_dir, exist_ok=True)
    mcap_file = os.path.join(output_dir, 'output.mcap')

    import mcap.writer
    from mcap.writer import Compressor, CompressionLevel
    from datetime import datetime
    import ro
    import sensor_msgs.msg
    import numpy as np

    with mcap.writer.MCapWriter(
        mcap_file,
        compressor=Compressor.LZ4,
        compression_level=CompressionLevel.Compress
    ) as writer:

        import message_filters
        from sensor_msgs.msg import Image, PointCloud2, Imu

        print(f"Writing to: {mcap_file}")

        for topic, msg_type, count, freq in topics_info:
            ros_msg_type = msg_type

            if 'sensor_msgs/Imu' in msg_type:
                data = b.read_imu(topic)
                for i, (timestamp, msg) in enumerate(data):
                    if i % 100 == 0:
                        print(f"  IMU: {i}/{count}")
                    msg_time = ro.Time.from_sec(timestamp)
                    writer.add_message(
                        topic,
                        msg,
                        ro.Time.to_nsec(msg_time),
                        ros_msg_type
                    )

            elif 'sensor_msgs/Image' in msg_type:
                data = b.read_images(topic)
                for i, (timestamp, msg) in enumerate(data):
                    if i % 100 == 0:
                        print(f"  Camera: {i}/{count}")
                    msg_time = ro.Time.from_sec(timestamp)
                    writer.add_message(
                        topic,
                        msg,
                        ro.Time.to_nsec(msg_time),
                        ros_msg_type
                    )

            elif 'sensor_msgs/PointCloud2' in msg_type:
                data = b.read_pointcloud(topic)
                for i, (timestamp, msg) in enumerate(data):
                    if i % 50 == 0:
                        print(f"  LiDAR: {i}/{count}")
                    msg_time = ro.Time.from_sec(timestamp)
                    writer.add_message(
                        topic,
                        msg,
                        ro.Time.to_nsec(msg_time),
                        ros_msg_type
                    )

    metadata_file = os.path.join(output_dir, 'metadata.yaml')
    with open(metadata_file, 'w') as f:
        f.write(f"""%YAML:1.0
version: 2
storage_identifier: mcap
relative_file_paths:
  - output.mcap
""")

    print(f"Created: {output_dir}")
    return output_dir

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: convert_ros1_to_ros2.py <bag_file> [output_dir]")
        sys.exit(1)

    bag_file = sys.argv[1]
    output_dir = sys.argv[2] if len(sys.argv) > 2 else None

    convert_ros1_to_ros2(bag_file, output_dir)