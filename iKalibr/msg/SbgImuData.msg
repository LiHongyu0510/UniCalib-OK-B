# A copy format from 'https://github.com/SBG-Systems/sbg_ros_driver' (more detail, open link)
# SBG Ellipse Messages
std_msgs/Header header

# Time since sensor is powered up [us]
uint32 time_stamp

# IMU Status
SbgImuStatus imu_status

# Filtered Accelerometer [m/s^2]
#
# NED convention:
#   x: X axis of device frame
#   y: Y axis of device frame
#   z: Z axis of device frame
#
# ENU convention:
#   x: X axis of device frame
#   y: -Y axis of device frame
#   z: -Z axis of device frame
geometry_msgs/Vector3 accel

# Filtered Gyroscope [rad/s]
#
# NED convention:
#   x: X axis of device frame
#   y: Y axis of device frame
#   z: Z axis of device frame
#
# ENU convention:
#   x: X axis of device frame
#   y: -Y axis of device frame
#   z: -Z axis of device frame
geometry_msgs/Vector3 gyro

# Internal Temperature [°C]
float32 temp

# Sculling output [m/s2]
#
# NED convention:
#   x: X axis of device frame
#   y: Y axis of device frame
#   z: Z axis of device frame
#
# ENU convention:
#   x: X axis of device frame
#   y: -Y axis of device frame
#   z: -Z axis of device frame
geometry_msgs/Vector3 delta_vel

# Coning output [rad/s]
#
# NED convention:
#   x: X axis of device frame
#   y: Y axis of device frame
#   z: Z axis of device frame
#
# ENU convention:
#   x: X axis of device frame
#   y: -Y axis of device frame
#   z: -Z axis of device frame
geometry_msgs/Vector3 delta_angle
