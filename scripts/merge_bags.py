#!/usr/bin/env python3
import os
import shutil
from rosbags.rosbag2 import Reader, Writer

d435i_bag = '/home/lihongyu/Unicalib/UniCalib-OK/data/ntu_day_02/ntu_day_02_d435i'
os1_bag = '/home/lihongyu/Unicalib/UniCalib-OK/data/ntu_day_02/ntu_day_02_os1_128'
output_bag = '/home/lihongyu/Unicalib/UniCalib-OK/data/ntu_day_02/ntu_merged'

if os.path.exists(output_bag):
    shutil.rmtree(output_bag)

print("Reading bags...")
with Reader(d435i_bag) as d435i, Reader(os1_bag) as os1:
    d435i_topics = {t: d435i.topics[t].msgtype for t in d435i.topics}
    os1_topics = {t: os1.topics[t].msgtype for t in os1.topics}
    print(f"D435i topics: {d435i_topics}")
    print(f"OS1 topics: {os1_topics}")
    
    all_topics = {}
    all_topics.update(d435i_topics)
    all_topics.update(os1_topics)
    print(f"Topics to merge: {all_topics}")
    
    writer = Writer(output_bag, version=9)
    with writer:
        connections = {}
        for topic, msgtype in all_topics.items():
            connections[topic] = writer.add_connection(topic, msgtype)
        
        print("Writing D435i messages...")
        for conn, timestamp, data in d435i.messages():
            writer.write(connections[conn.topic], timestamp, data)
        
        print("Writing OS1 messages...")
        for conn, timestamp, data in os1.messages():
            writer.write(connections[conn.topic], timestamp, data)

print(f"Merged bag created at: {output_bag}")
print("Done!")