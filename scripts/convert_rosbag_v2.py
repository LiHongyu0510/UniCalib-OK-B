#!/usr/bin/env python3
"""
ROS2 V2.0 Bag 转 Rosbag2 目录结构转换工具
用法: python3 convert_rosbag_v2.py <input.bag> <output_dir>
"""

import struct
import os
import sqlite3
import json
from datetime import datetime

def parse_rosbag_v2_field(data, pos):
    """解析一个header field"""
    if pos + 4 > len(data):
        return None, pos
    field_len = struct.unpack('<I', data[pos:pos+4])[0]
    if field_len == 0 or field_len > 10000000:
        return None, pos
    field = data[pos+4:pos+4+field_len]
    return field, pos + 4 + field_len

def read_varint(data, pos):
    """读取protobuf varint"""
    result = 0
    shift = 0
    while True:
        b = data[pos]
        pos += 1
        result |= (b & 0x7F) << shift
        if not (b & 0x80):
            break
        shift += 7
    return result, pos

def parse_rosbag_v2(filepath):
    """解析ROS2 V2 bag文件"""
    with open(filepath, 'rb') as f:
        data = f.read()
    
    info = {
        'file_size': len(data),
        'magic': data[:7].decode('utf-8', errors='ignore'),
        'topics': {},
        'messages': []
    }
    
    # 从文件末尾读取索引数据
    # V2.0格式的索引位于文件末尾
    # 搜索chunk数据
    
    # 查找所有可能的topic字符串
    topic_strings = [
        b'sensor_msgs/PointCloud2',
        b'sensor_msgs/Image', 
        b'sensor_msgs/Imu',
        b'geometry_msgs/Twist',
        b'nav_msgs/Odometry'
    ]
    
    for topic in topic_strings:
        pos = 0
        while True:
            idx = data.find(topic, pos)
            if idx == -1:
                break
            # 回溯找到topic name
            name_start = max(0, idx - 200)
            name_end = idx
            chunk = data[name_start:name_end]
            
            # 查找最后一个空字符前的字符串
            null_pos = chunk.rfind(b'\x00')
            if null_pos >= 0:
                topic_name = chunk[null_pos+1:idx].decode('utf-8', errors='ignore')
                if topic_name and '/' in topic_name:
                    info['topics'][topic_name] = topic.decode('utf-8', errors='ignore')
            
            pos = idx + len(topic)
    
    return info

def create_rosbag2_structure(output_dir, topic_info):
    """创建rosbag2目录结构"""
    os.makedirs(output_dir, exist_ok=True)
    
    # 创建metadata.yaml
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
    
    metadata_path = os.path.join(output_dir, 'metadata.yaml')
    with open(metadata_path, 'w') as f:
        import yaml
        yaml.dump(metadata, f, default_flow_style=False)
    
    # 创建SQLite数据库
    db_path = os.path.join(output_dir, 'rosbag2.db3')
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    
    # 创建表
    cursor.execute('''
        CREATE TABLE topics (
            id INTEGER PRIMARY KEY,
            name TEXT NOT NULL,
            type TEXT NOT NULL,
            serialization_format TEXT NOT NULL,
            offered_qos_profiles TEXT NOT NULL
        )
    ''')
    
    cursor.execute('''
        CREATE TABLE messages (
            id INTEGER PRIMARY KEY,
            topic_id INTEGER NOT NULL,
            timestamp INTEGER NOT NULL,
            data BLOB NOT NULL,
            FOREIGN KEY(topic_id) REFERENCES topics(id)
        )
    ''')
    
    # 插入topic信息
    topic_id = 1
    for topic_name, msg_type in topic_info.items():
        cursor.execute('''
            INSERT INTO topics (id, name, type, serialization_format, offered_qos_profiles)
            VALUES (?, ?, ?, ?, ?)
        ''', (topic_id, topic_name, msg_type, 'cdr', '{}'))
        topic_id += 1
    
    conn.commit()
    conn.close()
    
    print(f"Created rosbag2 structure at {output_dir}")
    return True

def main():
    import sys
    
    if len(sys.argv) < 3:
        print("用法: python3 convert_rosbag_v2.py <input.bag> <output_dir>")
        print("注意: 此脚本可能无法完全解析ROS2 V2格式，建议使用ROS2工具转换")
        sys.exit(1)
    
    input_file = sys.argv[1]
    output_dir = sys.argv[2]
    
    if not os.path.exists(input_file):
        print(f"输入文件不存在: {input_file}")
        sys.exit(1)
    
    print(f"解析 {input_file}...")
    info = parse_rosbag_v2(input_file)
    
    print(f"发现 {len(info['topics'])} 个话题:")
    for topic, msg_type in info['topics'].items():
        print(f"  {topic}: {msg_type}")
    
    print(f"\n创建rosbag2结构...")
    create_rosbag2_structure(output_dir, info['topics'])
    
    print(f"\n转换完成!")
    print(f"输出目录: {output_dir}")
    print(f"\n注意: 此脚本仅创建了空结构，内容解析需要更复杂的实现")
    print(f"建议使用ROS2工具进行完整转换")

if __name__ == '__main__':
    main()