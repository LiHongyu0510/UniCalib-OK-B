
# 工具说明
激光雷达点云解析工具目前支持解析速腾msop文件和LivoxMID360、禾赛激光雷达自定义二进制文件。

- 速腾激光雷达
车辆直接记录雷达输出的原始数据包msop格式写文件，数据结构定义参见：http://192.168.2.195/embedded/robosense_driver/-/blob/master/rslidar_sdk/src/rs_driver/src/rs_driver/driver/decoder/decoder_RSHELIOS.hpp

- livoxMID360和禾赛激光雷达
车端解析点云数据后，按照自定义的存储格式写文件，数据结构定义参见：
```
// 将 float x,y,z,i 类型由 4 字节改为 int16_t 类型的 2 个字节存储，该过程会导致雷达探测距离精度为 1cm.
struct PointXYZITO {
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t intensity;
    uint8_t tag;
    uint16_t offset;
};

struct LivoxPacketXYZIT {
    uint64_t timestamp;
    uint16_t points_num;
    std::vector<PointXYZITO> points;
};

struct HesaiPacketXYZIT {
    uint64_t timestamp;
    uint32_t points_num;
    std::vector<PointXYZITO> points;
};

```
注意：此工具依赖ROS2环境，可以直接使用isaac_ros_dev-cross_compiler:v1.6.6 交叉编译docker容器; 如果本地主机已经有ROS2环境，则可以直接使用。



## 编译依赖安装：
（使用isaac_ros_dev-cross_compiler:v1.6.6 docker容器无需安装）
```
sudo rosdep update
sudo apt-get install ros-humble-pcl-ros ros-humble-pcl-conversions
```
## 编译：

```
cd lidar_decode
colcon build
```

执行命令：

```
source install/setup.bash 
ros2 run lidar_decode lidar_decode 参数1 参数2 参数3 参数4
```
>
> - 参数1：需解析文件所在文件夹路径
> - 参数2：文件保存路径
> - 参数3：可选, 解析成bin文件不加，解析成pcd时需要加上pcd（每帧点云为一个bin 或 pcd文件）
> - 参数4：可选，需要与参数3一同提供，解析bev数据时加上 `bev_mode`

使用示例：
（1）准备好待解析的文件，文件来源车端每次自驾系统启动后激光雷达驱动程序会记录到record的目录。
（2）上述编译完成后，执行命令

```
source install/setup.bash 

# 只解析主雷达为bin文件
ros2 run lidar_decode lidar_decode ~/record/lidar_main/  ~/record/lidar_main_output  

# 同时解析record目录下所有的激光雷达为bin文件
ros2 run lidar_decode lidar_decode ~/record  ~/record/all_output  

# 同时解析record目录下所有的激光雷达为pcd文件
# $ ros2 run lidar_decode lidar_decode ~/record/  ~/record/all_output2 pcd bev_mode
```
（3）解析后的bin文件说明
```
record/lidar_main_output/bin/top_front$ ls -lrt
total 277632
-rw-r--r-- 1 docker docker 836736 Dec 30 16:01 2_1766548872299.bin
-rw-r--r-- 1 docker docker 835040 Dec 30 16:01 3_1766548872399.bin
-rw-r--r-- 1 docker docker 835360 Dec 30 16:01 4_1766548872499.bin
-rw-r--r-- 1 docker docker 834512 Dec 30 16:01 5_1766548872599.bin
-rw-r--r-- 1 docker docker 833856 Dec 30 16:01 6_1766548872699.bin
```
每一帧的bin文件名中包含当前帧解析顺序ID和时间戳，bin文件的数据格式为当前帧每个点云x/y/z/i顺序拼接而成。

| float x | float y | float z | float intensity | float x | float y | float z | float intensity | ...  |
| ------- | ------- | ------- | --------------- | ------- | ------- | ------- | --------------- | ---- |

