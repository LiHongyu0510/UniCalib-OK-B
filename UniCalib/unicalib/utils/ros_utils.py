"""
ROS2 工具函数
rosbag 写入、消息转换等
"""
from __future__ import annotations
import logging
import os
from typing import Dict, Optional

import numpy as np

logger = logging.getLogger(__name__)


def sensor_msgs_to_numpy(msg) -> Optional[np.ndarray]:
    """将 ROS2 传感器消息转换为 numpy 数组。"""
    msg_type = type(msg).__name__
    if "Image" in msg_type:
        return _image_msg_to_numpy(msg)
    elif "PointCloud2" in msg_type:
        return _pointcloud2_to_numpy(msg)
    elif "Imu" in msg_type:
        return _imu_msg_to_numpy(msg)
    return None


def _image_msg_to_numpy(msg) -> np.ndarray:
    import cv2
    enc = msg.encoding.lower()
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    img = data.reshape(msg.height, msg.width, -1)
    if enc in ("rgb8", "rgb"):
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    return img


def _pointcloud2_to_numpy(msg) -> np.ndarray:
    import struct
    offsets = {f.name: f.offset for f in msg.fields if f.name in ("x", "y", "z")}
    data = bytes(msg.data)
    ps = msg.point_step
    pts = []
    for i in range(msg.width * msg.height):
        base = i * ps
        x = struct.unpack_from("f", data, base + offsets.get("x", 0))[0]
        y = struct.unpack_from("f", data, base + offsets.get("y", 4))[0]
        z = struct.unpack_from("f", data, base + offsets.get("z", 8))[0]
        if not (np.isnan(x) or np.isnan(y) or np.isnan(z)):
            pts.append([x, y, z])
    return np.array(pts, dtype=np.float32)


def _imu_msg_to_numpy(msg) -> Dict:
    return {
        "gyro": np.array([msg.angular_velocity.x,
                          msg.angular_velocity.y,
                          msg.angular_velocity.z]),
        "accel": np.array([msg.linear_acceleration.x,
                           msg.linear_acceleration.y,
                           msg.linear_acceleration.z]),
    }


class ROSBagWriter:
    """
    ROS2 rosbag 写入工具。
    用于将标定验证数据写入 bag 文件，方便在 RViz2 中查看。
    """

    def __init__(self, output_path: str, storage_id: str = "mcap"):
        self.output_path = output_path
        self.storage_id = storage_id
        self._writer = None

    def open(self):
        try:
            import rosbag2_py
            storage_opts = rosbag2_py.StorageOptions(
                uri=self.output_path, storage_id=self.storage_id)
            converter_opts = rosbag2_py.ConverterOptions(
                input_serialization_format="cdr",
                output_serialization_format="cdr")
            self._writer = rosbag2_py.SequentialWriter()
            self._writer.open(storage_opts, converter_opts)
        except ImportError:
            logger.warning("rosbag2_py not available, bag writing disabled.")

    def close(self):
        if self._writer is not None:
            self._writer = None

    def write_colored_cloud(
        self, topic: str, pts: np.ndarray, colors: np.ndarray,
        timestamp_ns: int, frame_id: str = "map"
    ):
        """写入着色点云到 bag。"""
        if self._writer is None:
            return
        try:
            from rclpy.serialization import serialize_message
            from sensor_msgs.msg import PointCloud2, PointField
            import struct

            fields = [
                PointField(name="x", offset=0, datatype=7, count=1),
                PointField(name="y", offset=4, datatype=7, count=1),
                PointField(name="z", offset=8, datatype=7, count=1),
                PointField(name="rgb", offset=12, datatype=7, count=1),
            ]
            point_step = 16
            data = bytearray(len(pts) * point_step)

            for i, (pt, col) in enumerate(zip(pts, colors)):
                base = i * point_step
                struct.pack_into("fff", data, base, pt[0], pt[1], pt[2])
                r, g, b = int(col[2]), int(col[1]), int(col[0])
                rgb = (r << 16) | (g << 8) | b
                struct.pack_into("I", data, base + 12, rgb)

            from std_msgs.msg import Header
            msg = PointCloud2()
            msg.header.frame_id = frame_id
            msg.header.stamp.sec = timestamp_ns // 10**9
            msg.header.stamp.nanosec = timestamp_ns % 10**9
            msg.height = 1
            msg.width = len(pts)
            msg.fields = fields
            msg.is_bigendian = False
            msg.point_step = point_step
            msg.row_step = point_step * len(pts)
            msg.data = bytes(data)
            msg.is_dense = True

            serialized = serialize_message(msg)
            self._writer.write(topic, serialized, timestamp_ns)

        except Exception as e:
            logger.warning(f"Failed to write colored cloud: {e}")
