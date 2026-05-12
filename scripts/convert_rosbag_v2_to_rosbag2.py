#!/usr/bin/env python3
"""
ROS2 V2.0 Bag (单文件) 转 Rosbag2 目录格式转换器

此脚本尝试将ROS2 V2.0单文件格式转换为rosbag2目录格式。
注意: 完整的消息解析需要ROS2 bag插件支持，当前仅创建空结构。

用法:
    python3 scripts/convert_rosbag_v2_to_rosbag2.py <输入.bag> <输出目录>
"""

import os
import sys
import sqlite3
import yaml
from pathlib import Path

def check_bag_format(filepath):
    """检查bag文件格式"""
    with open(filepath, 'rb') as f:
        magic = f.read(20)
    
    if magic.startswith(b'#ROSBAG V2.0'):
        return 'rosbag_v2'
    elif magic.startswith(b'#ROSBAG V1'):
        return 'rosbag_v1'
    elif magic.startswith(b'MCAP'):
        return 'mcap'
    else:
        return 'unknown'

def create_rosbag2_structure(output_dir, topics=None):
    """创建rosbag2目录结构"""
    os.makedirs(output_dir, exist_ok=True)
    
    # metadata.yaml
    metadata = {
        'rosbag2_bagfile_information': {
            'version': 2,
            'storage_identifier': 'sqlite3',
            'duration': {'nanoseconds': 0},
            'starting_time': {'nanoseconds_since_epoch': 0},
            'message_count': 0,
            'topics_with_message_count': [],
            'compression_formats': [],
            'covered_topics': []
        }
    }
    
    with open(os.path.join(output_dir, 'metadata.yaml'), 'w') as f:
        yaml.dump(metadata, f, default_flow_style=False)
    
    # 创建SQLite数据库
    conn = sqlite3.connect(os.path.join(output_dir, 'rosbag2.db3'))
    c = conn.cursor()
    
    c.execute('''CREATE TABLE topics (
        id INTEGER PRIMARY KEY,
        name TEXT NOT NULL,
        type TEXT NOT NULL,
        serialization_format TEXT NOT NULL,
        offered_qos_profiles TEXT NOT NULL)''')
    
    c.execute('''CREATE TABLE messages (
        id INTEGER PRIMARY KEY,
        topic_id INTEGER NOT NULL,
        timestamp INTEGER NOT NULL,
        data BLOB NOT NULL,
        FOREIGN KEY(topic_id) REFERENCES topics(id))''')
    
    c.execute('''CREATE TABLE connections (
        id INTEGER PRIMARY KEY,
        topic_id INTEGER NOT NULL,
        metadata BLOB NOT NULL,
        FOREIGN KEY(topic_id) REFERENCES topics(id))''')
    
    if topics:
        for topic_name, msg_type in topics.items():
            c.execute('''INSERT INTO topics (name, type, serialization_format, offered_qos_profiles) 
                VALUES (?, ?, ?, ?)''',
                (topic_name, msg_type, 'cdr', '{}'))
    
    conn.commit()
    conn.close()
    
    return True

def convert_bag(input_file, output_dir):
    """转换bag文件"""
    
    # 检查输入
    if not os.path.exists(input_file):
        print(f"错误: 输入文件不存在: {input_file}")
        return False
    
    # 检查格式
    fmt = check_bag_format(input_file)
    print(f"检测到bag格式: {fmt}")
    
    if fmt not in ['rosbag_v2', 'rosbag_v1']:
        print(f"警告: 无法识别的格式 {fmt}")
    
    # 创建输出目录
    os.makedirs(output_dir, exist_ok=True)
    
    # 对于V2.0格式，需要特殊处理
    if fmt == 'rosbag_v2':
        print("\n注意: ROS2 V2.0单文件格式需要特殊解析")
        print("此格式可能来自:")
        print("  - rosbag API v2 (Python)")
        print("  - 旧版 rosbag2 写入插件")
        print("  - 非标准 bag 录制")
        print("\n建议:")
        print("  1. 找到原始录制环境的 rosbag2 插件")
        print("  2. 使用录制时的同一ROS版本重新录制")
        print("  3. 或联系数据提供方获取标准rosbag2格式")
        
        # 创建空结构作为模板
        create_rosbag2_structure(output_dir)
        print(f"\n已创建空rosbag2结构: {output_dir}")
        print("但需要外部工具进行实际数据转换")
        
        return False
    
    return True

def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)
    
    input_file = sys.argv[1]
    output_dir = sys.argv[2]
    
    convert_bag(input_file, output_dir)

if __name__ == '__main__':
    main()