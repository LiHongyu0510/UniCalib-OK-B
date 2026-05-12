"""
数据管理模块 —— 负责 rosbag 解析、时间对齐、帧采样
支持 ROS2 rosbag (MCAP/sqlite3) 格式
"""
from __future__ import annotations
import logging
import os
from pathlib import Path
from typing import Dict, Generator, List, Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)


class DataManager:
    """
    统一数据访问接口。

    支持两种数据源:
    1. rosbag2 文件 (.db3 / .mcap)
    2. 目录结构 (images/, pointclouds/, imu.csv 等)
    """

    def __init__(self, data_path: str):
        self.data_path = Path(data_path)
        self._bag_reader = None
        self._is_bag = self._detect_bag()
        logger.info(f"DataManager: path={data_path}, is_bag={self._is_bag}")

    # ------------------------------------------------------------------
    # 数据源探测
    # ------------------------------------------------------------------

    def _detect_bag(self) -> bool:
        """判断数据源是否为 rosbag2 格式。"""
        if self.data_path.is_file():
            return self.data_path.suffix in (".db3", ".mcap")
        # 目录内含 metadata.yaml 则认为是 bag 目录
        return (self.data_path / "metadata.yaml").exists()

    def open(self):
        """打开数据源。"""
        if self._is_bag:
            self._open_bag()
        # 否则为目录模式，按需读取

    def close(self):
        """关闭数据源。"""
        if self._bag_reader is not None:
            try:
                self._bag_reader.close()
            except Exception as e:
                logger.debug("Error closing bag reader: %s", e)
            self._bag_reader = None

    def _open_bag(self):
        """打开 rosbag2。"""
        try:
            import rosbag2_py
            storage_options = rosbag2_py.StorageOptions(
                uri=str(self.data_path),
                storage_id="mcap" if self._is_mcap() else "sqlite3",
            )
            converter_options = rosbag2_py.ConverterOptions(
                input_serialization_format="cdr",
                output_serialization_format="cdr",
            )
            reader = rosbag2_py.SequentialReader()
            reader.open(storage_options, converter_options)
            self._bag_reader = reader
            logger.info("rosbag2 opened successfully.")
        except ImportError:
            logger.warning("rosbag2_py not available; falling back to directory mode.")
            self._is_bag = False

    def _is_mcap(self) -> bool:
        if self.data_path.is_file():
            return self.data_path.suffix == ".mcap"
        for f in self.data_path.iterdir():
            if f.suffix == ".mcap":
                return True
        return False

    # ------------------------------------------------------------------
    # 图像帧迭代
    # ------------------------------------------------------------------

    def iter_images(self, topic: str, max_frames: Optional[int] = None,
                    skip: int = 1) -> Generator[Tuple[float, np.ndarray], None, None]:
        """
        迭代图像帧。

        Yields:
            (timestamp_sec, image_bgr_np)
        """
        if self._is_bag:
            yield from self._iter_images_bag(topic, max_frames, skip)
        else:
            yield from self._iter_images_dir(topic, max_frames, skip)

    def _iter_images_bag(self, topic, max_frames, skip):
        import cv2
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import Image, CompressedImage

        count = 0
        emitted = 0
        for (topic_str, data, t) in self._bag_reader.read_messages([topic]):
            count += 1
            if count % skip != 0:
                continue
            try:
                msg = deserialize_message(data, Image)
                img = _ros_image_to_numpy(msg)
            except Exception as e:
                logger.debug("Image deserialize failed, trying CompressedImage: %s", e)
                try:
                    msg = deserialize_message(data, CompressedImage)
                    img = cv2.imdecode(
                        np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
                except Exception as e:
                    logger.warning(f"Failed to decode image: {e}")
                    continue

            yield t * 1e-9, img
            emitted += 1
            if max_frames and emitted >= max_frames:
                break

    def _iter_images_dir(self, topic, max_frames, skip):
        import cv2
        topic_dir = self.data_path / topic.replace("/", "_").lstrip("_")
        if not topic_dir.exists():
            # 尝试直接用 topic 最后一段作为子目录名
            topic_dir = self.data_path / topic.split("/")[-1]
        if not topic_dir.exists():
            logger.warning(f"Image directory not found: {topic_dir}")
            return

        image_files = sorted(
            [f for f in topic_dir.iterdir()
             if f.suffix.lower() in (".jpg", ".jpeg", ".png", ".bmp")]
        )
        emitted = 0
        for i, img_path in enumerate(image_files):
            if i % skip != 0:
                continue
            img = cv2.imread(str(img_path))
            if img is None:
                continue
            # 文件名当时间戳（秒）
            try:
                ts = float(img_path.stem)
            except ValueError:
                ts = float(i)
            yield ts, img
            emitted += 1
            if max_frames and emitted >= max_frames:
                break

    # ------------------------------------------------------------------
    # 点云帧迭代
    # ------------------------------------------------------------------

    def iter_pointclouds(self, topic: str, max_frames: Optional[int] = None,
                         skip: int = 1) -> Generator[Tuple[float, np.ndarray], None, None]:
        """
        迭代点云帧。

        Yields:
            (timestamp_sec, points_Nx3_or_Nx4)
        """
        if self._is_bag:
            yield from self._iter_pc_bag(topic, max_frames, skip)
        else:
            yield from self._iter_pc_dir(topic, max_frames, skip)

    def _iter_pc_bag(self, topic, max_frames, skip):
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import PointCloud2

        count = 0
        emitted = 0
        for (topic_str, data, t) in self._bag_reader.read_messages([topic]):
            count += 1
            if count % skip != 0:
                continue
            try:
                msg = deserialize_message(data, PointCloud2)
                pts = _pointcloud2_to_numpy(msg)
            except Exception as e:
                logger.warning(f"Failed to decode pointcloud: {e}")
                continue
            yield t * 1e-9, pts
            emitted += 1
            if max_frames and emitted >= max_frames:
                break

    def _iter_pc_dir(self, topic, max_frames, skip):
        import open3d as o3d

        pc_dir = self.data_path / topic.replace("/", "_").lstrip("_")
        if not pc_dir.exists():
            pc_dir = self.data_path / topic.split("/")[-1]
        if not pc_dir.exists():
            logger.warning(f"Pointcloud directory not found: {pc_dir}")
            return

        pcd_files = sorted(
            [f for f in pc_dir.iterdir()
             if f.suffix.lower() in (".pcd", ".bin", ".ply")]
        )
        emitted = 0
        for i, pcd_path in enumerate(pcd_files):
            if i % skip != 0:
                continue
            try:
                if pcd_path.suffix == ".bin":
                    pts = np.fromfile(str(pcd_path), dtype=np.float32).reshape(-1, 4)
                else:
                    pcd = o3d.io.read_point_cloud(str(pcd_path))
                    pts = np.asarray(pcd.points)
            except Exception as e:
                logger.warning(f"Failed to load pointcloud {pcd_path}: {e}")
                continue
            try:
                ts = float(pcd_path.stem)
            except ValueError:
                ts = float(i)
            yield ts, pts
            emitted += 1
            if max_frames and emitted >= max_frames:
                break

    # ------------------------------------------------------------------
    # IMU 数据读取
    # ------------------------------------------------------------------

    def load_imu_data(self, topic: str) -> Optional[Dict]:
        """
        加载完整 IMU 数据。

        Returns:
            {
                "timestamps": np.ndarray,  # 秒
                "gyro": np.ndarray (N×3),
                "accel": np.ndarray (N×3),
                "sample_rate": float,
            }
        """
        if self._is_bag:
            return self._load_imu_bag(topic)
        else:
            return self._load_imu_dir(topic)

    def _load_imu_bag(self, topic):
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import Imu

        timestamps, gyros, accels = [], [], []
        for (topic_str, data, t) in self._bag_reader.read_messages([topic]):
            try:
                msg = deserialize_message(data, Imu)
                timestamps.append(t * 1e-9)
                gyros.append([
                    msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z,
                ])
                accels.append([
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z,
                ])
            except Exception as e:
                logger.warning(f"IMU decode error: {e}")

        if not timestamps:
            return None

        ts = np.array(timestamps)
        sample_rate = 1.0 / np.median(np.diff(ts)) if len(ts) > 1 else 200.0
        return {
            "timestamps": ts,
            "gyro": np.array(gyros),
            "accel": np.array(accels),
            "sample_rate": sample_rate,
        }

    def _load_imu_dir(self, topic):
        import csv
        imu_csv = self.data_path / "imu.csv"
        if not imu_csv.exists():
            imu_csv = self.data_path / (topic.split("/")[-1] + ".csv")
        if not imu_csv.exists():
            logger.warning(f"IMU CSV not found: {imu_csv}")
            return None

        timestamps, gyros, accels = [], [], []
        with open(imu_csv) as f:
            reader = csv.DictReader(f)
            for row in reader:
                timestamps.append(float(row["timestamp"]))
                gyros.append([float(row["gx"]), float(row["gy"]), float(row["gz"])])
                accels.append([float(row["ax"]), float(row["ay"]), float(row["az"])])

        ts = np.array(timestamps)
        sample_rate = 1.0 / np.median(np.diff(ts)) if len(ts) > 1 else 200.0
        return {
            "timestamps": ts,
            "gyro": np.array(gyros),
            "accel": np.array(accels),
            "sample_rate": sample_rate,
        }

    # ------------------------------------------------------------------
    # 时间同步帧对
    # ------------------------------------------------------------------

    def iter_synced_frames(self, topic_a: str, topic_b: str,
                           sync_threshold_ms: float = 50.0,
                           max_pairs: Optional[int] = None
                           ) -> Generator[Tuple[float, object, object], None, None]:
        """
        迭代时间同步的双传感器帧对。
        使用最近时间戳匹配策略。

        Yields:
            (timestamp_sec, data_a, data_b)
        """
        from collections import deque

        buf_a: deque = deque(maxlen=50)
        buf_b: deque = deque(maxlen=50)
        threshold = sync_threshold_ms * 1e-3
        emitted = 0

        # 简化: 分别加载后按时间戳匹配
        frames_a = list(self._iter_any(topic_a))
        frames_b = list(self._iter_any(topic_b))

        if not frames_a or not frames_b:
            return

        ts_b = np.array([f[0] for f in frames_b])

        for ts_a, data_a in frames_a:
            idx = np.argmin(np.abs(ts_b - ts_a))
            if abs(ts_b[idx] - ts_a) <= threshold:
                yield ts_a, data_a, frames_b[idx][1]
                emitted += 1
                if max_pairs and emitted >= max_pairs:
                    break

    def _iter_any(self, topic: str):
        """根据 topic 格式自动选择图像/点云迭代器。"""
        # 简单启发：含 image 或 cam 则为图像
        t_lower = topic.lower()
        if any(k in t_lower for k in ("image", "cam", "rgb", "color")):
            yield from self.iter_images(topic)
        elif any(k in t_lower for k in ("lidar", "point", "velodyne", "livox", "scan")):
            yield from self.iter_pointclouds(topic)
        else:
            logger.warning(f"Cannot determine data type for topic: {topic}")


# ------------------------------------------------------------------
# 辅助函数
# ------------------------------------------------------------------

def _ros_image_to_numpy(msg) -> np.ndarray:
    """将 sensor_msgs/Image 转换为 BGR numpy 数组。"""
    import cv2
    enc = msg.encoding.lower()
    data = np.frombuffer(msg.data, dtype=np.uint8)
    data = data.reshape(msg.height, msg.width, -1)

    if enc in ("bgr8", "bgr"):
        return data
    elif enc in ("rgb8", "rgb"):
        return cv2.cvtColor(data, cv2.COLOR_RGB2BGR)
    elif enc in ("mono8", "8uc1"):
        return cv2.cvtColor(data[:, :, 0], cv2.COLOR_GRAY2BGR)
    elif enc in ("bayer_bggr8",):
        return cv2.cvtColor(data[:, :, 0], cv2.COLOR_BayerBG2BGR)
    else:
        return data


def _pointcloud2_to_numpy(msg) -> np.ndarray:
    """将 sensor_msgs/PointCloud2 转换为 N×3 numpy 数组 (xyz)。"""
    import struct

    points = []
    point_step = msg.point_step
    data = bytes(msg.data)

    # 查找 x, y, z 字段偏移
    offsets = {}
    for field in msg.fields:
        if field.name in ("x", "y", "z"):
            offsets[field.name] = field.offset

    fmt_char = "f"  # float32
    for i in range(msg.width * msg.height):
        base = i * point_step
        x = struct.unpack_from(fmt_char, data, base + offsets.get("x", 0))[0]
        y = struct.unpack_from(fmt_char, data, base + offsets.get("y", 4))[0]
        z = struct.unpack_from(fmt_char, data, base + offsets.get("z", 8))[0]
        if not (np.isnan(x) or np.isnan(y) or np.isnan(z)):
            points.append([x, y, z])

    return np.array(points, dtype=np.float32)
