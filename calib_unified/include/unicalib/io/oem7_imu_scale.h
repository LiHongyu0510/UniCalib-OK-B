#pragma once

namespace ns_unicalib {

/// 与 bynav_ros_driver/oem7_imu.hpp 中 oem7_imu_type_t 一致
enum class Oem7ImuType : int {
    Unknown = 0,
    Hg1700Ag11 = 1,
    Hg1700Ag17 = 4,
    Hg1900Ca29 = 5,
    Ln200 = 8,
    Hg1700Ag58 = 11,
    Hg1700Ag62 = 12,
    ImarFsas = 13,
    KvhCots = 16,
    Hg1930Aa99 = 20,
    Isa100c = 26,
    Hg1900Ca50 = 27,
    Hg1930Ca50 = 28,
    Adis16488 = 31,
    Stim300 = 32,
    Kvh1750 = 33,
    EpsonG320 = 41,
    LitefMicroimu = 52,
    Stim300d = 56,
    Hg4930An01 = 58,
    EpsonG370 = 61,
    EpsonG320_200Hz = 62,
    Hg4930An04 = 68,
    Hg4930An04_400Hz = 69,
};

/// 默认输出频率（Hz），与 supported_imus.yaml 一致
int oem7_default_imu_rate_hz(Oem7ImuType type);

/// RAWIMUSX 原始计数 → rad/s、m/s²（与 oem7_imu.cpp getImuRawScaleFactors 一致）
bool oem7_get_imu_raw_scale_factors(Oem7ImuType type, int imu_rate_hz, double& gyro_scale, double& acc_scale);

}  // namespace ns_unicalib
