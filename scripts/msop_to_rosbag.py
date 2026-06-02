#!/usr/bin/env python3
"""
Convert RoboSense RS-Helios MSOP/DIFOP lidar packet files to ROS2 MCAP bag.

Usage:
    source /opt/ros/jazzy/setup.bash
    python3 msop_to_rosbag.py <msop_file> <difop_file> -o output_bag_dir
    python3 msop_to_rosbag.py <input_dir> -o output_bag_dir   # auto pair MSOP/DIFOP
"""

import argparse
import os
import struct
import sys
import math
import time
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

# Try ROS2 imports
try:
    import rclpy
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import Header
    import rosbag2_py
except ImportError:
    rclpy = None
    print("ERROR: ROS2 Jazzy environment not sourced. Run: source /opt/ros/jazzy/setup.bash", file=sys.stderr)
    sys.exit(1)

# ============================================================
# RS-Helios Constants
# ============================================================
MSOP_PKT_SIZE = 1248        # 42 header + 12 * 100 blocks + 4 index + 2 tail
MSOP_HEADER_SIZE = 42
MSOP_BLOCK_SIZE = 100       # 2 id + 2 azimuth + 32 * 3 channels
BLOCKS_PER_PKT = 12
CHANNELS_PER_BLOCK = 32

DISTANCE_SCALE = 0.0025     # m per distance unit
AZIMUTH_SCALE = 0.01        # degrees per azimuth unit
RPM = 600                   # default RPM

# Firing time offsets (microseconds) for each of 32 channels
FIRING_TSS_US = np.array([
    0.00, 1.57, 3.15, 4.72, 6.30, 7.87, 9.45, 11.36,
    13.26, 15.17, 17.08, 18.99, 20.56, 22.14, 23.71, 25.29,
    26.53, 29.01, 27.77, 30.25, 31.49, 33.98, 32.73, 35.22,
    36.46, 37.70, 38.94, 40.18, 41.42, 42.67, 43.91, 45.15
], dtype=np.float64)

BLK_TS_US = 55.56  # block time in microseconds
BLOCK_DURATION_S = BLK_TS_US / 1e6

# Precompute CHAN_AZIS (azimuth interpolation factor per channel)
CHAN_AZIS = FIRING_TSS_US / BLK_TS_US

# Default Helios 32 vertical angles (0.01 degree units)
HELIOS_DEFAULT_VERT_ANGLES = np.array([
    -2500, -2250, -2000, -1750, -1500, -1250, -1000, -750,
    -500,  -250,   0,     250,   500,   750,   1000,  1250,
    1500,  1750,  2000,  2250,  2500,  2750,  3000,  3250,
    3500,  3750,  4000,  4250,  4500,  4750,  5000,  5250
], dtype=np.float64)


def rs_timestamp_to_us(sec_bytes: bytes, us_bytes: bytes) -> int:
    """Parse 6-byte seconds + 4-byte microseconds (big-endian) -> total microseconds."""
    sec = int.from_bytes(sec_bytes, 'big')
    us = int.from_bytes(us_bytes, 'big')
    return sec * 1_000_000 + us


def parse_difop_calibration(data: bytes, base_off: int) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """Try to parse calibration data at a given offset. Returns (vert, horiz) or None."""
    if base_off + 96 * 2 > len(data):
        return None
    try:
        v_angles = np.zeros(32, dtype=np.float64)
        h_angles = np.zeros(32, dtype=np.float64)
        
        for i in range(32):
            off_v = base_off + i * 3
            sign_v = data[off_v]
            val_v = struct.unpack('>H', data[off_v+1:off_v+3])[0]
            v = val_v if sign_v == 0 else -val_v
            v_angles[i] = float(v)
            
            off_h = base_off + 96 + i * 3
            sign_h = data[off_h]
            val_h = struct.unpack('>H', data[off_h+1:off_h+3])[0]
            h = val_h if sign_h == 0 else -val_h
            h_angles[i] = float(h)
        
        # Validate: check if angles are reasonable
        good = np.sum((np.abs(v_angles) > 30) & (np.abs(v_angles) < 3000))
        v_range = np.max(v_angles) - np.min(v_angles)
        if good >= 8 and v_range >= 300:
            return v_angles, h_angles
        return None
    except Exception:
        return None


def parse_difop(difop_path: str) -> Tuple[np.ndarray, np.ndarray]:
    """Parse DIFOP file to extract vertical and horizontal calibration angles."""
    vert_angles = HELIOS_DEFAULT_VERT_ANGLES.copy()
    horiz_angles = np.zeros(32, dtype=np.float64)

    with open(difop_path, 'rb') as f:
        data = f.read()

    magic = data[:4]

    # Try multiple candidate offsets for calibration data:
    # - Some DIFOP files start struct at offset 0
    # - Some have a 28-byte custom record header (struct at offset 28)
    # - The calibration is at struct_offset + 468
    payload_starts = [0]
    if magic[:2] in (b'\x55\xaa', b'\xa5\xff'):
        payload_starts.append(28)
    
    angles_found = False
    for ps in payload_starts:
        candidate = ps + 468
        result = parse_difop_calibration(data, candidate)
        if result is not None:
            vert_angles, horiz_angles = result
            angles_found = True
            print(f"[INFO] DIFOP: loaded calibration at file offset {candidate}")
            break
    
    if not angles_found:
        print("[WARN] DIFOP: no valid calibration found, using default Helios angles")
    
    return vert_angles, horiz_angles


def trigon_sin(angle: float) -> float:
    """Trigonometric sin for angle in 0.01 degree units."""
    rad = math.radians(angle * 0.01)
    return math.sin(rad)


def trigon_cos(angle: float) -> float:
    """Trigonometric cos for angle in 0.01 degree units."""
    rad = math.radians(angle * 0.01)
    return math.cos(rad)


def decode_msop_to_frames(
    msop_path: str,
    vert_angles: np.ndarray,
    horiz_angles: np.ndarray,
    max_frames: int = 0,
    points_per_frame: int = 57600,
    base_timestamp_s: float = 0.0,
) -> List[Tuple[float, np.ndarray]]:
    """
    Decode MSOP file into frames of point clouds.
    Returns list of (timestamp_sec, Nx4 array [x, y, z, intensity]).
    """
    frames: List[Tuple[float, np.ndarray]] = []
    current_points: List[Tuple[float, float, float, float]] = []
    first_pts_time_ms = -1.0
    frame_counter = 0
    
    with open(msop_path, 'rb') as f:
        # Check for and skip any custom record header
        magic = f.read(4)
        if magic[:2] == b'\x55\xaa':
            if magic == b'\x55\xaa\x05\x5a':
                f.seek(0)
            else:
                f.seek(28)
                print(f"[INFO] MSOP: detected custom record header, skipping 28 bytes")
        else:
            f.seek(0)
        
        pkt_buf = f.read(MSOP_PKT_SIZE)
        pkt_count = 0
        
        while pkt_buf and len(pkt_buf) == MSOP_PKT_SIZE:
            pkt_count += 1
            
            hdr = pkt_buf[:MSOP_HEADER_SIZE]
            sync_word = hdr[:4]
            if sync_word != b'\x55\xaa\x05\x5a':
                pkt_buf = pkt_buf[1:] + f.read(1)
                continue
            
            # Parse blocks
            for blk_idx in range(BLOCKS_PER_PKT):
                blk_off = MSOP_HEADER_SIZE + blk_idx * MSOP_BLOCK_SIZE
                blk_data = pkt_buf[blk_off:blk_off + MSOP_BLOCK_SIZE]
                
                blk_id = blk_data[:2]
                if blk_id != b'\xff\xee':
                    continue
                
                block_az = struct.unpack('>H', blk_data[2:4])[0]
                
                if blk_idx < BLOCKS_PER_PKT - 1:
                    next_blk_off = MSOP_HEADER_SIZE + (blk_idx + 1) * MSOP_BLOCK_SIZE
                    next_az = struct.unpack('>H', pkt_buf[next_blk_off + 2:next_blk_off + 4])[0]
                    az_diff = next_az - block_az
                    if az_diff < 0:
                        az_diff += 36000
                    if az_diff > 100:
                        az_diff = 20
                else:
                    az_diff = 20
                
                for chan in range(CHANNELS_PER_BLOCK):
                    chan_off = 4 + chan * 3
                    chan_data = blk_data[chan_off:chan_off + 3]
                    
                    distance_raw = struct.unpack('>H', chan_data[:2])[0]
                    intensity = chan_data[2]
                    
                    distance = distance_raw * DISTANCE_SCALE
                    if not (0.05 <= distance <= 200.0):
                        continue
                    
                    angle_horiz = block_az + az_diff * CHAN_AZIS[chan]
                    angle_horiz_final = angle_horiz + horiz_angles[chan]
                    angle_vert = vert_angles[chan]
                    
                    cos_v = trigon_cos(angle_vert)
                    sin_v = trigon_sin(angle_vert)
                    cos_h = trigon_cos(angle_horiz_final)
                    sin_h = trigon_sin(angle_horiz_final)
                    cos_h0 = trigon_cos(angle_horiz)
                    sin_h0 = trigon_sin(angle_horiz)
                    
                    x = distance * cos_v * cos_h + 0.03498 * cos_h0
                    y = -distance * cos_v * sin_h - 0.03498 * sin_h0
                    z = distance * sin_v
                    
                    if math.isnan(x) or math.isnan(y) or math.isnan(z):
                        continue
                    
                    r = math.sqrt(x*x + y*y + z*z)
                    if r < 0.05:
                        continue
                    
                    if first_pts_time_ms < 0:
                        first_pts_time_ms = (pkt_count * blk_idx + chan) * 0.1
                    
                    current_points.append((x, y, z, intensity))
                    
                    if len(current_points) >= points_per_frame:
                        frame_ts = base_timestamp_s + frame_counter * 0.1
                        pts = np.array(current_points[:points_per_frame], dtype=np.float32)
                        frames.append((frame_ts, pts))
                        current_points = current_points[points_per_frame:]
                        first_pts_time_ms = -1.0
                        frame_counter += 1
                        
                        if max_frames > 0 and len(frames) >= max_frames:
                            print(f"[INFO] Reached max_frames={max_frames}")
                            return frames
            
            pkt_buf = f.read(MSOP_PKT_SIZE)
        
        print(f"[INFO] Processed {pkt_count} MSOP packets")
    
    if current_points:
        frame_ts = base_timestamp_s + frame_counter * 0.1
        pts = np.array(current_points, dtype=np.float32)
        frames.append((frame_ts, pts))
        print(f"[INFO] Flushed final frame with {len(current_points)} points")
    
    return frames


def create_pointcloud2_msg(
    points_nx3: np.ndarray,
    intensities: np.ndarray,
    timestamp_ns: int,
    frame_id: str = "lidar_main",
) -> PointCloud2:
    """Create a sensor_msgs/PointCloud2 message from numpy array."""
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    point_step = 16
    
    data = bytearray(len(points_nx3) * point_step)
    for i in range(len(points_nx3)):
        base = i * point_step
        struct.pack_into("ffff", data, base,
                         points_nx3[i, 0], points_nx3[i, 1], points_nx3[i, 2], intensities[i])
    
    msg = PointCloud2()
    msg.header = Header()
    msg.header.frame_id = frame_id
    msg.header.stamp.sec = int(timestamp_ns // 1_000_000_000)
    msg.header.stamp.nanosec = int(timestamp_ns % 1_000_000_000)
    msg.height = 1
    msg.width = len(points_nx3)
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = point_step
    msg.row_step = point_step * len(points_nx3)
    msg.data = bytes(data)
    msg.is_dense = True
    
    return msg


def write_ros2_bag(
    frames: List[Tuple[float, np.ndarray]],
    output_dir: str,
    topic: str = "/lidar_main/points",
    frame_id: str = "lidar_main",
):
    """Write decoded point cloud frames to a ROS2 MCAP bag."""
    # rosbag2_py requires the directory to NOT exist for MCAP storage
    if os.path.exists(output_dir):
        import shutil
        shutil.rmtree(output_dir)
    
    storage_opts = rosbag2_py.StorageOptions(
        uri=output_dir,
        storage_id="mcap",
    )
    converter_opts = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    
    writer = rosbag2_py.SequentialWriter()
    writer.open(storage_opts, converter_opts)
    
    # Register topic
    topic_metadata = rosbag2_py.TopicMetadata(
        id=0,
        name=topic,
        type="sensor_msgs/msg/PointCloud2",
        serialization_format="cdr",
    )
    writer.create_topic(topic_metadata)
    
    print(f"[INFO] Writing {len(frames)} frames to {output_dir} ...")
    for i, (ts_s, pts) in enumerate(frames):
        if len(pts) == 0:
            continue
        timestamp_ns = int(ts_s * 1_000_000_000)
        points_xyz = pts[:, :3]
        intensities = pts[:, 3]
        msg = create_pointcloud2_msg(points_xyz, intensities, timestamp_ns, frame_id)
        serialized = serialize_message(msg)
        writer.write(topic, bytes(serialized), timestamp_ns)
        
        if (i + 1) % 20 == 0:
            print(f"[INFO]  ... wrote {i + 1} frames ({len(pts)} pts/frame avg)")
    
    writer.close()
    print(f"[INFO] Done! Wrote {len(frames)} frames to {output_dir}")


def auto_pair_files(input_dir: str) -> List[Tuple[str, str]]:
    """Auto-pair MSOP and DIFOP files in a directory by timestamp."""
    msop_files = sorted(Path(input_dir).glob("lidarMSOP*"))
    difop_files = sorted(Path(input_dir).glob("lidarDIFOP*"))
    
    pairs = []
    for msop in msop_files:
        # Try to find matching DIFOP by same timestamp
        ts = str(msop.name).replace("lidarMSOP", "lidarDIFOP")
        difop = Path(input_dir) / ts
        if difop.exists():
            pairs.append((str(msop), str(difop)))
        else:
            print(f"[WARN] No matching DIFOP found for {msop.name}")
    
    return pairs


def main():
    parser = argparse.ArgumentParser(description="Convert RS-Helios MSOP/DIFOP to ROS2 MCAP bag")
    parser.add_argument("input", help="MSOP file or directory containing lidarMSOP* files")
    parser.add_argument("-d", "--difop", help="DIFOP file (auto-paired if input is a directory)")
    parser.add_argument("-o", "--output", default="rosbag_output", help="Output bag directory")
    parser.add_argument("--topic", default="/lidar_main/points", help="ROS2 topic name")
    parser.add_argument("--frame-id", default="lidar_main", help="Frame ID")
    parser.add_argument("--max-frames", type=int, default=0, help="Max frames to decode")
    parser.add_argument("--points-per-frame", type=int, default=57600, help="Points per frame")
    args = parser.parse_args()
    
    input_path = Path(args.input)
    
    if input_path.is_dir():
        # Directory mode: auto-pair MSOP/DIFOP files
        pairs = auto_pair_files(args.input)
        if not pairs:
            print(f"[ERROR] No MSOP/DIFOP pairs found in {args.input}", file=sys.stderr)
            sys.exit(1)
        print(f"[INFO] Found {len(pairs)} MSOP/DIFOP pairs")
        
        all_frames = []
        total_frames = 0
        base_ts = time.time()
        file_idx = 0
        for msop_file, difop_file in pairs:
            print(f"\n{'='*60}")
            print(f"[INFO] Processing: {Path(msop_file).name}")
            vert_angles, horiz_angles = parse_difop(difop_file)
            frames = decode_msop_to_frames(
                msop_file, vert_angles, horiz_angles,
                max_frames=args.max_frames,
                points_per_frame=args.points_per_frame,
                base_timestamp_s=base_ts + file_idx * 3600,
            )
            print(f"[INFO] Decoded {len(frames)} frames from {Path(msop_file).name}")
            all_frames.extend(frames)
            total_frames += len(frames)
            file_idx += 1
            if args.max_frames > 0 and total_frames >= args.max_frames:
                break
        
        if all_frames:
            write_ros2_bag(all_frames, args.output, args.topic, args.frame_id)
    else:
        # Single file mode
        if not args.difop:
            # Try to auto-pair
            difop = str(input_path).replace("lidarMSOP", "lidarDIFOP")
            if Path(difop).exists():
                args.difop = difop
                print(f"[INFO] Auto-paired DIFOP: {difop}")
            else:
                print("[ERROR] DIFOP file required for single MSOP file mode", file=sys.stderr)
                sys.exit(1)
        
        vert_angles, horiz_angles = parse_difop(args.difop)
        frames = decode_msop_to_frames(
            args.input, vert_angles, horiz_angles,
            max_frames=args.max_frames,
            points_per_frame=args.points_per_frame,
            base_timestamp_s=time.time(),
        )
        print(f"[INFO] Decoded {len(frames)} frames")
        
        if frames:
            write_ros2_bag(frames, args.output, args.topic, args.frame_id)
    
    print("[INFO] Conversion complete!")


if __name__ == "__main__":
    main()
