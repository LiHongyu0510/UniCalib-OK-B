#pragma once
/**
 * NovAtel OEM7 二进制报文布局（摘自 bynav_ros_driver/oem7_messages.h，仅 IMU 解析所需子集）。
 */
#include <cstdint>

namespace ns_unicalib::oem7 {

using oem7_enum_t = uint32_t;

struct __attribute__((packed)) CORRIMUSMem {
    uint32_t imu_data_count;
    double pitch_rate;
    double roll_rate;
    double yaw_rate;
    double lateral_acc;
    double longitudinal_acc;
    double vertical_acc;
    uint32_t reserved1;
    uint32_t reserved2;
};
static_assert(sizeof(CORRIMUSMem) == 60, "CORRIMUSMem");

struct __attribute__((packed)) CORRIMUDATASMem {
    uint32_t week;
    double seconds;
    double pitch_rate;
    double roll_rate;
    double yaw_rate;
    double lateral_acc;
    double longitudinal_acc;
    double vertical_acc;
};
static_assert(sizeof(CORRIMUDATASMem) == 60, "CORRIMUDATASMem");

struct __attribute__((packed)) RAWIMUSXMem {
    uint8_t imu_info;
    uint8_t imu_type;
    uint16_t gnss_week;
    double gnss_week_seconds;
    uint8_t imu_status[4];
    int32_t z_acc;
    int32_t y_acc;
    int32_t x_acc;
    int32_t z_gyro;
    int32_t y_gyro;
    int32_t x_gyro;
};
static_assert(sizeof(RAWIMUSXMem) == 40, "RAWIMUSXMem");

struct __attribute__((packed)) INSCONFIG_FixedMem {
    oem7_enum_t imu_type;
    uint8_t mapping;
    uint8_t initial_alignment_velocity;
    uint16_t heave_window;
    oem7_enum_t profile;
    uint8_t enabled_updates[4];
    oem7_enum_t alignment_mode;
    oem7_enum_t relative_ins_output_frame;
    oem7_enum_t relative_ins_output_direction;  // oem7_bool_t = uint32_t
    uint8_t ins_receiver_status[4];
    uint8_t ins_seed_enabled;
    uint8_t ins_seed_validation;
    uint16_t reserved_1;
    uint32_t reserved_2;
    uint32_t reserved_3;
    uint32_t reserved_4;
    uint32_t reserved_5;
    uint32_t reserved_6;
    uint32_t reserved_7;
};
static_assert(sizeof(INSCONFIG_FixedMem) == 60, "INSCONFIG_FixedMem");

constexpr uint16_t kCorrImuDataSMsgId = 813u;
constexpr uint16_t kImuRateCorrImuSMsgId = 1362u;
constexpr uint16_t kCorrImuSMsgId = 2264u;
constexpr uint16_t kRawImusXMsgId = 1462u;
constexpr uint16_t kInsConfigMsgId = 1945u;

constexpr size_t kShortHdr = 12u;
constexpr size_t kLongHdr = 28u;
constexpr size_t kCrcLen = 4u;

}  // namespace ns_unicalib::oem7
