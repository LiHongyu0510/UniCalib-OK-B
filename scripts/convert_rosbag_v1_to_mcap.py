#!/usr/bin/env python3
"""
ROS1 bag V2.0 to ROS2 MCAP converter using rosbags

Dependencies: rosbags mcap
pip install rosbags mcap
"""

import os
import sys
from pathlib import Path
from rosbags.convert import convert

def convert_bag(bag_path, output_dir=None):
    bag_path = Path(bag_path).resolve()
    if not bag_path.exists():
        print(f"Error: file not found: {bag_path}")
        return False

    bag_name = bag_path.stem
    output_dir = Path(output_dir) if output_dir else bag_path.parent / f"{bag_name}_ros2"
    import time
    ts = int(time.time())
    unique_output_dir = output_dir.parent / f"{output_dir.name}_{ts}"
    output_dir = unique_output_dir

    print(f"Input:  {bag_path}")
    print(f"Output: {output_dir}")

    convert(
        [bag_path],
        output_dir,
        'rosbag2', 2,
        compress=None,
        compress_mode='message',
        default_typestore=None,
        typestore=None,
        exclude_topics=[],
        include_topics=[],
        exclude_msgtypes=[],
        include_msgtypes=[],
    )
    print(f"Created: {output_dir}")
    return True

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: convert_rosbag_v1_to_mcap.py <bag_file> [output_dir]")
        sys.exit(1)
    
    bag_file = sys.argv[1]
    output_dir = sys.argv[2] if len(sys.argv) > 2 else None
    
    convert_bag(bag_file, output_dir)