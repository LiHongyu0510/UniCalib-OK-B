# lidar_decode_standalone

ROS2-free version of the lidar data parser.

## Build (no ROS2 required)

```bash
sudo apt install libpcl-dev
cd lidar_standalone
mkdir build && cd build
cmake ..
make -j
```

## Run (identical CLI to original)

```bash
./lidar_decode_standalone <input_directory> <save_path> [pcd] [bev_mode]
```

Example:

```bash
./lidar_decode_standalone /data/lidar_airy/left_front_airy /output pcd
```

All original functionality (Rslidar Helios32, Livox, Hesai/Airy) is preserved exactly.

The only change is removal of ROS2 dependencies (pcl_conversions, rclcpp, sensor_msgs). Output PCD/BIN files are identical in content and naming (Unix ms timestamp, no frame number prefix).
