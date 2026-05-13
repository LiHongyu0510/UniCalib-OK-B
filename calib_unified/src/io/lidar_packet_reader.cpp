#include "unicalib/io/lidar_packet_reader.h"

#include "unicalib/common/logger.h"

#include <arpa/inet.h>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <vector>
#include <zlib.h>

namespace ns_unicalib {

namespace {

constexpr double kPi = 3.14159265358979323846;

// RS雷达角度转换：协议中以0.01度为单位存储（如1234表示12.34度）
// 将0.01度单位转换为弧度
inline double deg001_to_rad(double angle_001deg) { return angle_001deg * (kPi / 18000.0); }
// 预计算cos和sin值以提高性能
inline float rs_cos_001(double a) { return static_cast<float>(std::cos(deg001_to_rad(a))); }
inline float rs_sin_001(double a) { return static_cast<float>(std::sin(deg001_to_rad(a))); }

// ---------- RoboSense Helios MSOP/DIFOP 数据包结构定义 ----------
// 参考自 sensor_decode/lidar_decode/src/main.cpp，无ROS依赖

#pragma pack(push, 1)  // 字节对齐为1，确保结构体与网络包完全一致

// 单个激光通道数据
struct RSChannel {
    uint16_t distance;   // 距离值，单位：0.0025m
    uint8_t intensity;   // 反射强度
};

// UTC时间戳结构
struct RSTimestampUTC {
    uint8_t sec[6];      // 秒数，6字节大端整数
    uint8_t ss[4];       // 微秒数，4字节大端整数
};

// MSOP包头部
struct RSHELIOSMsopHeader {
    uint8_t id[4];               // 包头标识
    uint16_t protocol_version;   // 协议版本号
    uint8_t reserved1[14];       // 保留字段
    RSTimestampUTC timestamp;    // UTC时间戳
    uint8_t lidar_type;          // 激光雷达类型
    uint8_t reserved2[11];       // 保留字段
};

// MSOP数据块（每个块包含32个通道的数据）
struct RSHELIOSMsopBlock {
    uint8_t id[2];               // 块标识 0xFFEE
    uint16_t azimuth;            // 方位角，单位：0.01度
    RSChannel channels[32];      // 32个激光通道数据
};

// 完整MSOP数据包
struct RSHELIOSMsopPkt {
    RSHELIOSMsopHeader header;   // 包头
    RSHELIOSMsopBlock blocks[12]; // 12个数据块
    unsigned int index;          // 包索引
    uint16_t tail;               // 包尾
};

// 网络配置信息（DIFOP）
struct RSEthNetV2 {
    uint8_t lidar_ip[4];         // 雷达IP地址
    uint8_t dest_ip[4];          // 目标IP地址
    uint8_t mac_addr[6];         // MAC地址
    uint16_t msop_port;          // MSOP端口
    uint16_t reserve_1;          // 保留
    uint16_t difop_port;         // DIFOP端口
    uint16_t reserve_2;          // 保留
};

// 视场角范围
struct RSFOV {
    uint16_t start_angle;        // 起始角度，单位：0.01度
    uint16_t end_angle;          // 结束角度，单位：0.01度
};

// 版本信息
struct RSVersionV2 {
    uint8_t top_ver[5];          // 顶部版本
    uint8_t bottom_ver[5];       // 底部版本
    uint8_t bot_soft_ver[5];     // 底部软件版本
    uint8_t motor_firmware_ver[5]; // 电机固件版本
    uint8_t hw_ver[3];           // 硬件版本
};

// 序列号
struct RSSN {
    uint8_t num[6];              // 序列号
};

// 时间同步信息
struct RSTimeInfo {
    uint8_t sync_mode;           // 同步模式
    uint8_t sync_sts;            // 同步状态
    RSTimestampUTC timestamp;    // 时间戳
};

// 设备状态
struct RSStatusV2 {
    uint16_t device_current;     // 设备电流
    uint16_t vol_fpga;           // FPGA电压
    uint16_t vol_12v;            // 12V电压
    uint16_t vol_dig_5v4;        // 数字5.4V电压
    uint16_t vol_sim_5v;         // 模拟5V电压
    uint16_t vol_apd;            // APD电压
    uint8_t reserved[12];        // 保留
};

// 诊断信息
struct RSDiagnoV2 {
    uint16_t bot_fpga_temperature;      // 底部FPGA温度
    uint16_t recv_A_temperature;        // A接收端温度
    uint16_t recv_B_temperature;        // B接收端温度
    uint16_t main_fpga_temperature;     // 主FPGA温度
    uint16_t main_fpga_core_temperature; // 主FPGA核心温度
    uint16_t real_rpm;                  // 实际转速
    uint8_t lane_up;                    // 通道状态
    uint16_t lane_up_cnt;               // 通道计数
    uint16_t main_status;               // 主状态
    uint8_t gps_status;                 // GPS状态
    uint8_t reserved[22];               // 保留
};

// 角度标定值
struct RSCalibrationAngle {
    uint8_t sign;               // 符号位（0x00为正，其他为负）
    uint16_t value;             // 角度值，单位：0.01度
};

// 完整DIFOP数据包
struct RSHELIOSDifopPkt {
    uint8_t id[8];                       // 包标识
    uint16_t rpm;                        // 转速
    RSEthNetV2 eth;                      // 网络配置
    RSFOV fov;                           // 视场角
    uint8_t reserved1[2];                // 保留
    uint16_t phase_lock_angle;           // 锁相角
    RSVersionV2 version;                 // 版本信息
    uint8_t reserved2[229];              // 保留
    RSSN sn;                             // 序列号
    uint16_t zero_cali;                  // 零位标定
    uint8_t return_mode;                 // 回波模式
    RSTimeInfo time_info;                // 时间信息
    RSStatusV2 status;                   // 设备状态
    uint8_t reserved3[5];                // 保留
    RSDiagnoV2 diagno;                   // 诊断信息
    uint8_t gprmc[86];                   // GPRMC数据
    RSCalibrationAngle vert_angle_cali[32];   // 垂直角度标定（32通道）
    RSCalibrationAngle horiz_angle_cali[32];  // 水平角度标定（32通道）
    uint8_t reserved4[586];              // 保留
    uint16_t tail;                       // 包尾
};
#pragma pack(pop)  // 恢复默认对齐方式

/**
 * 帧分割策略基类
 * 用于确定何时开始新的一帧点云
 */
class SplitStrategy {
public:
    virtual bool newBlock(int32_t angle) = 0;  // 判断是否开始新帧
    virtual ~SplitStrategy() = default;
};

/**
 * 基于角度的帧分割策略
 * 当方位角跨越指定角度时切分新帧（通常使用0°）
 */
class SplitStrategyByAngle : public SplitStrategy {
public:
    explicit SplitStrategyByAngle(int32_t split_angle) 
        : split_angle_(split_angle), prev_angle_(split_angle) {}

    bool newBlock(int32_t angle) override {
        // 处理角度回绕（360° -> 0°）
        if (angle < prev_angle_) {
            prev_angle_ -= 36000;  // 36000 = 360度 * 100
        }
        // 检查是否从小于分割角跨越到大于等于分割角
        const bool v = ((prev_angle_ < split_angle_) && (split_angle_ <= angle));
        prev_angle_ = angle;
        return v;
    }

private:
    const int32_t split_angle_;  // 分割角度（单位：0.01度）
    int32_t prev_angle_;         // 前一个角度
};

/**
 * 数据块迭代器
 * 计算每个数据块的时间偏移和角度差
 */
class BlockIterator {
public:
    static const int MAX_BLOCKS_PER_PKT = 12;  // 每包最多12个数据块

    /**
     * 获取指定块的角度差和时间偏移
     * @param blk 块索引
     * @param az_diff 输出：角度差
     * @param ts 输出：时间偏移（秒）
     */
    void get(uint16_t blk, int32_t& az_diff, double& ts) {
        az_diff = az_diffs[blk];
        ts = tss[blk];
    }

    /**
     * 构造函数，预计算所有块的角度差和时间偏移
     * @param pkt MSOP数据包
     * @param blocks_per_pkt 每包块数
     * @param block_duration 单个块持续时间
     * @param block_az_duration 块角度持续时间
     * @param fov_blind_duration 视场盲区持续时间
     */
    BlockIterator(const RSHELIOSMsopPkt& pkt, uint16_t blocks_per_pkt, 
                  double block_duration, uint16_t block_az_duration,
                  double fov_blind_duration)
        : pkt_(pkt),
          BLOCKS_PER_PKT(blocks_per_pkt),
          BLOCK_DURATION(block_duration),
          BLOCK_AZ_DURATION(block_az_duration),
          FOV_BLIND_DURATION(fov_blind_duration) {
        uint16_t blk = 0;
        double tss_acc = 0;
        
        // 计算除最后一个块外所有块的角度差和时间偏移
        for (; blk < (this->BLOCKS_PER_PKT - 1); blk++) {
            double ts_diff = this->BLOCK_DURATION;
            // 计算当前块和下一块的角度差
            int32_t az_diff = ntohs(this->pkt_.blocks[blk + 1].azimuth) - 
                             ntohs(this->pkt_.blocks[blk].azimuth);
            if (az_diff < 0) {
                az_diff += 36000;  // 处理回绕
            }
            // 异常角度差检测
            if (az_diff > 100) {
                az_diff = this->BLOCK_AZ_DURATION;
                ts_diff = this->FOV_BLIND_DURATION;
            }
            this->az_diffs[blk] = az_diff;
            this->tss[blk] = tss_acc;
            tss_acc += ts_diff;
        }
        // 最后一个块使用默认值
        this->az_diffs[blk] = this->BLOCK_AZ_DURATION;
    }

protected:
    const RSHELIOSMsopPkt& pkt_;                    // MSOP数据包引用
    const uint16_t BLOCKS_PER_PKT;                  // 每包块数
    const double BLOCK_DURATION;                    // 块持续时间
    const uint16_t BLOCK_AZ_DURATION;               // 块角度持续时间
    const double FOV_BLIND_DURATION;                // 视场盲区持续时间
    int32_t az_diffs[MAX_BLOCKS_PER_PKT];          // 角度差数组
    double tss[MAX_BLOCKS_PER_PKT];                 // 时间偏移数组
};

/**
 * 解析RS雷达角度标定数据
 * 从DIFOP包中提取水平和垂直角度标定值
 * @param pkt_difop DIFOP数据包
 * @param vert_angles 输出：垂直角度数组（32个）
 * @param horiz_angles 输出：水平角度数组（32个）
 */
void parse_rs_angle_calibration(const RSHELIOSDifopPkt& pkt_difop, 
                                double vert_angles[32], double horiz_angles[32]) {
    // 解析水平角度标定
    for (int i = 0; i < 32; ++i) {
        const uint8_t sign = pkt_difop.horiz_angle_cali[i].sign;
        const uint16_t value = ntohs(pkt_difop.horiz_angle_cali[i].value);
        const int16_t angle = (sign == 0x00) ? static_cast<int16_t>(value) : -static_cast<int16_t>(value);
        horiz_angles[i] = static_cast<double>(angle);
    }
    // 解析垂直角度标定
    for (int i = 0; i < 32; ++i) {
        const uint8_t sign = pkt_difop.vert_angle_cali[i].sign;
        const uint16_t value = ntohs(pkt_difop.vert_angle_cali[i].value);
        const int16_t angle = (sign == 0x00) ? static_cast<int16_t>(value) : -static_cast<int16_t>(value);
        vert_angles[i] = static_cast<double>(angle);
    }
}

/**
 * 从UTC时间戳结构获取微秒时间戳
 * @param tsUtc UTC时间戳结构
 * @return 微秒级时间戳
 */
uint64_t rs_get_timestamp_us(const RSTimestampUTC& tsUtc) {
    uint64_t sec = 0;
    // 解析6字节秒数（大端）
    for (int i = 0; i < 6; i++) {
        sec <<= 8;
        sec += tsUtc.sec[i];
    }
    uint64_t us = 0;
    // 解析4字节微秒数（大端）
    for (int i = 0; i < 4; i++) {
        us <<= 8;
        us += tsUtc.ss[i];
    }
    return sec * 1000000 + us;
}

/**
 * 检查距离值是否有效
 * @param distance 距离值（米）
 * @return true表示有效距离
 */
bool rs_distance_ok(float distance) { 
    return (0.2f <= distance) && (distance <= 200.f);  // 有效范围0.2-200米
}

/**
 * 检查方位角是否在有效范围内
 * @param angle 角度值（0.01度）
 * @return true表示有效角度
 */
bool rs_azimuth_section_ok(int angle) {
    const int start = 0 * 100;      // 起始角度0度
    const int end = 360 * 100;      // 结束角度360度
    const int start_ = start % 36000;
    const int end_ = end % 36000;
    const int cross_zero = (start_ > end_);
    if ((start ==0) && (end == 36000)) return true;
    if (cross_zero) {
        return (angle >= start_) || (angle < end_);
    }
    return (angle >= start_) && (angle < end_);
}

/**
 * 从MSOP文件路径推测对应的DIFOP文件路径
 * 支持两种命名约定：lidarMSOP->lidarDIFOP 或 MSOP->DIFOP
 * @param msop_path MSOP文件路径
 * @return 推测的DIFOP文件路径，若无法推测则返回空字符串
 */
std::string guess_difop_from_msop_path(const std::string& msop_path) {
    namespace fs = std::filesystem;
    fs::path p(msop_path);
    std::string name = p.filename().string();
    // 尝试替换 lidarMSOP 为 lidarDIFOP
    const std::string key = "lidarMSOP";
    auto pos = name.find(key);
    if (pos != std::string::npos) {
        name.replace(pos, key.size(), "lidarDIFOP");
        return (p.parent_path() / name).string();
    }
    // 尝试替换 MSOP 为 DIFOP
    pos = name.find("MSOP");
    if (pos != std::string::npos) {
        name.replace(pos, 4, "DIFOP");
        return (p.parent_path() / name).string();
    }
    return {};
}

/**
 * 检查数据包是否为有效的RS MSOP数据包
 * 通过检查数据块标识（0xFFEE）来判断
 * @param pkt MSOP数据包
 * @return true表示有效的RS MSOP包
 */
bool looks_like_rs_msop_packet(const RSHELIOSMsopPkt& pkt) {
    constexpr uint8_t blk0 = 0xFF;
    constexpr uint8_t blk1 = 0xEE;
    for (size_t bi = 0; bi < std::size(pkt.blocks); ++bi) {
        if (pkt.blocks[bi].id[0] == blk0 && pkt.blocks[bi].id[1] == blk1) {
            return true;
        }
    }
    return false;
}

/**
 * 解码RS MSOP数据流为点云帧
 * 使用与原始工具相同的方法：以方位角0°为帧分割点
 * @param in_msop MSOP文件输入流
 * @param vert_angles 垂直角度标定
 * @param horiz_angles 水平角度标定
 * @param frames 输出：解码后的帧列表
 * @param max_frames 最大帧数限制（0表示不限制）
 * @return true表示解码成功
 */
bool decode_rs_msop_stream(std::ifstream& in_msop, const double vert_angles[32], 
                           const double horiz_angles[32],
                           std::vector<DecodedLidarFrame>& frames, 
                           std::size_t max_frames) {
    // 创建点云对象
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    
    // 刷新当前点云到帧列表的lambda函数
    auto flush_cloud = [&](double timestamp_sec) -> bool {
        if (pcl_cloud->empty()) {
            pcl_cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
            return false;
        }
        DecodedLidarFrame frame;
        frame.timestamp_sec = timestamp_sec;
        pcl_cloud->width = pcl_cloud->size();
        pcl_cloud->height = 1;
        pcl_cloud->is_dense = false;
        frame.cloud = pcl_cloud;
        frames.push_back(std::move(frame));
        pcl_cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
        if (max_frames > 0 && frames.size() >= max_frames) {
            return true;  // 达到最大帧数，停止处理
        }
        return false;
    };

    // 使用0°作为帧分割点
    SplitStrategyByAngle split_strategy(0);
    bool first_run = false;
    double first_pts_time_ms = -1.0;

    // 激光发射时序参数
    constexpr float blk_ts = 55.56f;  // 每个块的时间（微秒）
    const double BLOCK_DURATION = static_cast<double>(blk_ts) / 1000000.0;
    // 32个通道的发射时间偏移（微秒）
    constexpr float firing_tss[] = {0.00f,  1.57f,  3.15f,  4.72f,  6.30f,  7.87f,  9.45f,  11.36f, 13.26f, 15.17f,
                                    17.08f, 18.99f, 20.56f, 22.14f, 23.71f, 25.29f, 26.53f, 29.01f, 27.77f, 30.25f,
                                    31.49f, 33.98f, 32.73f, 35.22f, 36.46f, 37.70f, 38.94f, 40.18f, 41.42f, 42.67f,
                                    43.91f, 45.15f};
    float CHAN_AZIS[32];      // 通道角度偏移系数
    double CHAN_TSS[32];      // 通道时间偏移（秒）
    for (uint16_t i = 0; i < sizeof(firing_tss) / sizeof(firing_tss[0]); i++) {
        CHAN_AZIS[i] = firing_tss[i] / blk_ts;
        CHAN_TSS[i] = static_cast<double>(firing_tss[i]) / 1000000.0;
    }

    RSHELIOSMsopPkt pkt{};
    // 循环读取MSOP包
    while (in_msop.read(reinterpret_cast<char*>(&pkt), sizeof(RSHELIOSMsopPkt))) {
        // 验证数据包有效性
        if (!looks_like_rs_msop_packet(pkt)) {
            UNICALIB_WARN("[LidarPacketReader] RS MSOP 包块标记异常，停止解析");
            break;
        }
        
        BlockIterator iter(pkt, 12, BLOCK_DURATION, 20, 0.0);

        // 处理每个数据块
        for (size_t blocks_i = 0; blocks_i < std::size(pkt.blocks); ++blocks_i) {
            const RSHELIOSMsopBlock& block = pkt.blocks[blocks_i];
            const float azimuth = static_cast<float>(ntohs(block.azimuth)) / 100.0f;
            const int32_t block_az = ntohs(block.azimuth);
            const int azimuth_int = static_cast<int>(std::floor(azimuth));
            int32_t az_diff = 0;
            double block_ts_off = 0.0;
            iter.get(static_cast<uint16_t>(blocks_i), az_diff, block_ts_off);

            // 检测到0°方位角时启动第一帧
            if (azimuth_int == 0) {
                first_run = true;
            }

            if (!first_run) {
                continue;  // 跳过第一帧开始前的数据
            }

            // 计算当前块的时间戳
            const double pkt_ts = static_cast<double>(rs_get_timestamp_us(pkt.header.timestamp)) * 1e-6;
            const double block_ts = pkt_ts + block_ts_off;

            // 检查是否需要开始新帧
            if (split_strategy.newBlock(block_az)) {
                // 使用当前帧首点时间作为帧时间戳
                const double frame_ts_sec = (first_pts_time_ms >= 0.0) ? (first_pts_time_ms * 0.001) : block_ts;
                if (flush_cloud(frame_ts_sec)) {
                    return true;  // 达到最大帧数
                }
                first_pts_time_ms = -1.0;
            }

            // 处理32个激光通道
            for (size_t chan = 0; chan < std::size(block.channels); ++chan) {
                const double point_timestamp = block_ts + CHAN_TSS[chan];
                const RSChannel& channel = block.channels[chan];
                const float distance = ntohs(channel.distance) * 0.0025f;  // 距离转换为米
                
                // 计算水平和垂直角度
                double angle_horiz = static_cast<double>(block_az + 
                    static_cast<int32_t>(static_cast<float>(az_diff) * CHAN_AZIS[chan]));
                const double angle_vert = vert_angles[chan];
                const double angle_horiz_final = angle_horiz + horiz_angles[chan];

                // 过滤无效距离和角度
                if (!rs_distance_ok(distance) || !rs_azimuth_section_ok(static_cast<int>(angle_horiz_final))) {
                    continue;
                }

                // 极坐标转直角坐标
                // 考虑雷达偏心距补偿（0.03498米）
                const float x = distance * rs_cos_001(angle_vert) * rs_cos_001(angle_horiz_final) + 
                               0.03498f * rs_cos_001(angle_horiz);
                const float y = -distance * rs_cos_001(angle_vert) * rs_sin_001(angle_horiz_final) - 
                                0.03498f * rs_sin_001(angle_horiz);
                const float z = distance * rs_sin_001(angle_vert);
                const float intensity = static_cast<float>(channel.intensity);
                
                // 检查数值有效性
                if (std::isnan(x) || std::isnan(y) || std::isnan(z) || std::isnan(intensity)) {
                    continue;
                }
                
                // 记录帧首点时间戳
                if (first_pts_time_ms < 0.0) {
                    first_pts_time_ms = point_timestamp * 1000.0;
                }
                
                // 添加点到点云
                pcl::PointXYZI point;
                point.x = x;
                point.y = y;
                point.z = z;
                point.intensity = intensity;
                pcl_cloud->push_back(point);
            }
        }
    }

    // 处理最后一帧
    if (!pcl_cloud->empty()) {
        const double frame_ts_sec = (first_pts_time_ms >= 0.0) ? (first_pts_time_ms * 0.001) : 0.0;
        flush_cloud(frame_ts_sec);
    }
    return !frames.empty();
}

// ---------- Livox / Hesai 雷达格式（原有实现）----------

#pragma pack(push, 1)
// Livox点格式（带时间偏移）
struct PointXYZIT {
    int16_t x;          // X坐标，单位：cm
    int16_t y;          // Y坐标，单位：cm
    int16_t z;          // Z坐标，单位：cm
    uint8_t intensity;  // 强度
    uint8_t tag;        // 标签
};

// Livox点格式（带时间偏移）
struct PointXYZITO {
    int16_t x;          // X坐标，单位：cm
    int16_t y;          // Y坐标，单位：cm
    int16_t z;          // Z坐标，单位：cm
    uint8_t intensity;  // 强度
    uint8_t tag;        // 标签
    uint16_t offset;    // 时间偏移
};

// Livox数据包头部
struct LivoxPacketXYZIT {
    uint64_t timestamp;    // 时间戳（纳秒）
    uint16_t points_num;   // 点数
};

// Hesai数据包头部
struct HesaiPacketXYZIT {
    uint64_t timestamp;    // 时间戳（纳秒）
    uint32_t points_num;   // 点数
};
#pragma pack(pop)

/**
 * 读取通用数据包头部的模板函数
 */
template <typename HeaderT>
bool read_header(std::ifstream& ifs, HeaderT& hdr) {
    return static_cast<bool>(ifs.read(reinterpret_cast<char*>(&hdr), sizeof(HeaderT)));
}

/**
 * 解码带时间偏移的数据包
 * @param ifs 文件输入流
 * @param data_id 数据标识（0xddbb或0xccaa）
 * @param packet_ts_ns 包时间戳（纳秒）
 * @param points_num 点数
 * @param out 输出帧
 * @return true表示解码成功
 */
bool decode_packet_with_offset(std::ifstream& ifs, uint16_t data_id, uint64_t packet_ts_ns,
                               std::size_t points_num, DecodedLidarFrame& out) {
    auto cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    cloud->reserve(points_num);

    if (data_id == 0xddbb) {  // Livox格式
        uint64_t last_point_ts_us = packet_ts_ns / 1000;
        for (std::size_t i = 0; i < points_num; ++i) {
            PointXYZITO p{};
            if (!ifs.read(reinterpret_cast<char*>(&p), sizeof(PointXYZITO))) return false;
            last_point_ts_us = last_point_ts_us + p.offset;
            (void)last_point_ts_us;  // 保留时间戳供未来使用
            pcl::PointXYZI pt;
            pt.x = static_cast<float>(p.x) / 100.0f;   // cm转m
            pt.y = static_cast<float>(p.y) / 100.0f;
            pt.z = static_cast<float>(p.z) / 100.0f;
            pt.intensity = static_cast<float>(p.intensity);
            cloud->push_back(pt);
        }
    } else if (data_id == 0xccaa) {  // Hesai格式
        for (std::size_t i = 0; i < points_num; ++i) {
            PointXYZIT p{};
            if (!ifs.read(reinterpret_cast<char*>(&p), sizeof(PointXYZIT))) return false;
            pcl::PointXYZI pt;
            pt.x = static_cast<float>(p.x) / 100.0f;   // cm转m
            pt.y = static_cast<float>(p.y) / 100.0f;
            pt.z = static_cast<float>(p.z) / 100.0f;
            pt.intensity = static_cast<float>(p.intensity);
            cloud->push_back(pt);
        }
    } else {
        return false;  // 未知格式
    }
    out.timestamp_sec = static_cast<double>(packet_ts_ns) * 1e-9;
    out.cloud = cloud;
    return true;
}

/**
 * 解码Livox或Hesai数据流
 * @param ifs 文件输入流
 * @param frames 输出帧列表
 * @param max_frames 最大帧数限制
 * @return true表示解码成功
 */
bool decode_livox_or_hesai_stream(std::ifstream& ifs, std::vector<DecodedLidarFrame>& frames, 
                                  std::size_t max_frames) {
    while (ifs.good()) {
        uint16_t data_id = 0;
        if (!ifs.read(reinterpret_cast<char*>(&data_id), sizeof(data_id))) break;

        std::streampos pos_after_data_id = ifs.tellg();
        LivoxPacketXYZIT livox_hdr{};
        if (!read_header(ifs, livox_hdr)) break;

        // 尝试按Livox格式解析
        if (livox_hdr.points_num > 0 && livox_hdr.points_num < 200000) {
            DecodedLidarFrame frame;
            if (decode_packet_with_offset(ifs, data_id, livox_hdr.timestamp, 
                                          livox_hdr.points_num, frame)) {
                frames.push_back(std::move(frame));
                if (max_frames > 0 && frames.size() >= max_frames) break;
                continue;
            }
        }

        // 回退到Hesai格式
        ifs.clear();
        ifs.seekg(pos_after_data_id);
        HesaiPacketXYZIT hesai_hdr{};
        if (!read_header(ifs, hesai_hdr)) break;
        if (hesai_hdr.points_num == 0 || hesai_hdr.points_num > 200000) break;

        DecodedLidarFrame frame;
        if (!decode_packet_with_offset(ifs, data_id, hesai_hdr.timestamp, 
                                       hesai_hdr.points_num, frame)) {
            break;
        }
        frames.push_back(std::move(frame));
        if (max_frames > 0 && frames.size() >= max_frames) break;
    }
    return !frames.empty();
}

// ---------- Blindspot AACC 自定义流格式 ----------
#pragma pack(push, 1)
struct BlindspotPointXYZIO {
    int16_t x;          // cm
    int16_t y;          // cm
    int16_t z;          // cm
    uint8_t intensity;  // 强度
    uint8_t tag;        // 标签
    uint16_t offset;    // 点内时间偏移（保留字段）
};

struct BlindspotStoragePacketHeader {
    uint64_t timestamp;   // 由上层约定单位（默认按 ns 处理）
    uint32_t points_num;  // 点数
};
#pragma pack(pop)

/**
 * 解码补盲雷达 AACC 帧流：
 * AACC | StoragePacketXYZIT | AACC | StoragePacketXYZIT | ...
 */
bool decode_blindspot_aacc_stream(std::ifstream& ifs, std::vector<DecodedLidarFrame>& frames,
                                  std::size_t max_frames, double timestamp_unit_scale) {
    constexpr std::array<char, 4> kAaccMarker = {'A', 'A', 'C', 'C'};
    constexpr uint32_t kMaxReasonablePoints = 1000000;
    constexpr uint32_t kMinReasonablePoints = 8;

    ifs.clear();
    ifs.seekg(0, std::ios::end);
    const std::streamoff file_size = static_cast<std::streamoff>(ifs.tellg());
    ifs.clear();
    ifs.seekg(0, std::ios::beg);

    while (ifs.good()) {
        std::array<char, 4> marker{};
        if (!ifs.read(marker.data(), marker.size())) {
            break;
        }

        // 未命中 marker 时，回退 3 字节继续扫描（滑动窗口）
        if (marker != kAaccMarker) {
            auto pos = ifs.tellg();
            if (pos == std::streampos(-1)) break;
            ifs.seekg(pos - static_cast<std::streamoff>(marker.size() - 1));
            continue;
        }

        BlindspotStoragePacketHeader hdr{};
        if (!ifs.read(reinterpret_cast<char*>(&hdr), sizeof(hdr))) {
            break;
        }
        if (hdr.points_num < kMinReasonablePoints || hdr.points_num > kMaxReasonablePoints) {
            // 可能是误命中 marker，回退到 marker 后 1 字节继续滑窗扫描
            const std::streampos pos = ifs.tellg();
            if (pos == std::streampos(-1)) break;
            ifs.seekg(pos - static_cast<std::streamoff>(sizeof(hdr)) - 3);
            continue;
        }

        const std::streamoff point_bytes = static_cast<std::streamoff>(hdr.points_num) *
                                           static_cast<std::streamoff>(sizeof(BlindspotPointXYZIO));
        const std::streamoff curr_pos = static_cast<std::streamoff>(ifs.tellg());
        if (curr_pos < 0 || (curr_pos + point_bytes) > file_size) {
            // 半包或截断包：直接结束，避免越界读
            break;
        }

        auto cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
        cloud->reserve(hdr.points_num);

        for (uint32_t i = 0; i < hdr.points_num; ++i) {
            BlindspotPointXYZIO p{};
            if (!ifs.read(reinterpret_cast<char*>(&p), sizeof(p))) {
                return !frames.empty();
            }
            pcl::PointXYZI pt;
            pt.x = static_cast<float>(p.x) / 100.0f;  // cm -> m
            pt.y = static_cast<float>(p.y) / 100.0f;
            pt.z = static_cast<float>(p.z) / 100.0f;
            pt.intensity = static_cast<float>(p.intensity);
            cloud->push_back(pt);
        }

        DecodedLidarFrame frame;
        frame.timestamp_sec = static_cast<double>(hdr.timestamp) * timestamp_unit_scale;
        cloud->width = cloud->size();
        cloud->height = 1;
        cloud->is_dense = false;
        frame.cloud = cloud;
        frames.push_back(std::move(frame));

        if (max_frames > 0 && frames.size() >= max_frames) {
            break;
        }
    }

    return !frames.empty();
}

}  // namespace

/**
 * 解码RS MSOP/DIFOP文件对
 * @param msop_path MSOP文件路径
 * @param difop_path DIFOP文件路径（可选）
 * @param frames 输出帧列表
 * @param max_frames 最大帧数限制
 * @return true表示解码成功
 */
bool LidarPacketReader::decode_rs_msop_difop(const std::string& msop_path, 
                                             const std::string& difop_path,
                                             std::vector<DecodedLidarFrame>& frames, 
                                             std::size_t max_frames) {
    frames.clear();
    double vert_angles[32]{};   // 垂直角度标定
    double horiz_angles[32]{};  // 水平角度标定

    // 打开MSOP文件
    std::ifstream in_msop(msop_path, std::ios::binary);
    if (!in_msop) {
        UNICALIB_WARN("[LidarPacketReader] 打开 MSOP 失败: {}", msop_path);
        return false;
    }

    // 验证MSOP文件格式
    RSHELIOSMsopPkt probe{};
    const std::streampos start = in_msop.tellg();
    if (!in_msop.read(reinterpret_cast<char*>(&probe), sizeof(RSHELIOSMsopPkt))) {
        UNICALIB_WARN("[LidarPacketReader] RS MSOP 为空: {}", msop_path);
        return false;
    }
    in_msop.clear();
    in_msop.seekg(start);
    if (!looks_like_rs_msop_packet(probe)) {
        UNICALIB_WARN("[LidarPacketReader] 非 RS Helios MSOP 包布局: {}", msop_path);
        return false;
    }

    // 如果提供了DIFOP文件，读取角度标定数据
    if (!difop_path.empty()) {
        std::ifstream in_difop(difop_path, std::ios::binary);
        if (!in_difop) {
            UNICALIB_WARN("[LidarPacketReader] 打开 DIFOP 失败，使用零角度标定: {}", difop_path);
        } else {
            RSHELIOSDifopPkt pkt_difop{};
            in_difop.read(reinterpret_cast<char*>(&pkt_difop), sizeof(RSHELIOSDifopPkt));
            parse_rs_angle_calibration(pkt_difop, vert_angles, horiz_angles);
        }
    } else {
        UNICALIB_INFO("[LidarPacketReader] RS 未指定 DIFOP，使用零角度标定");
    }

    // 解码MSOP流
    const bool ok = decode_rs_msop_stream(in_msop, vert_angles, horiz_angles, frames, max_frames);
    if (!ok) {
        UNICALIB_WARN("[LidarPacketReader] RS 解码未得到帧: {}", msop_path);
        return false;
    }
    UNICALIB_INFO("[LidarPacketReader] RS 解码完成: {} 帧 ({})", frames.size(), msop_path);
    return true;
}

/**
 * 自动解码任意格式的激光雷达文件
 * 支持格式：Livox、Hesai、Blindspot(AACC)、RS-Helios
 * @param file_path 文件路径
 * @param frames 输出帧列表
 * @param max_frames 最大帧数限制
 * @return true表示解码成功
 */
namespace {
bool decompress_gzip_to_temp(const std::string& gz_path, std::string& out_temp_path) {
    gzFile gz = gzopen(gz_path.c_str(), "rb");
    if (!gz) return false;
    std::filesystem::path tmp = std::filesystem::temp_directory_path() / ("unicalib_lidar_" + std::to_string(std::hash<std::string>{}(gz_path)) + ".tmp");
    std::ofstream ofs(tmp, std::ios::binary);
    if (!ofs) { gzclose(gz); return false; }
    std::vector<char> buf(4096);
    int n;
    while ((n = gzread(gz, buf.data(), buf.size())) > 0) {
        ofs.write(buf.data(), n);
    }
    gzclose(gz);
    ofs.close();
    out_temp_path = tmp.string();
    return true;
}
}  // namespace

bool LidarPacketReader::decode_file(const std::string& file_path, 
                                    std::vector<DecodedLidarFrame>& frames,
                                    std::size_t max_frames,
                                    double blindspot_timestamp_unit_scale) {
    frames.clear();
    std::string actual_path = file_path;
    std::string temp_path;
    bool is_compressed = false;

    if (file_path.size() > 3 && file_path.substr(file_path.size()-3) == ".gz") {
        if (decompress_gzip_to_temp(file_path, temp_path)) {
            actual_path = temp_path;
            is_compressed = true;
        } else {
            UNICALIB_WARN("[LidarPacketReader] gzip 解压失败: {}", file_path);
            return false;
        }
    }

    std::ifstream ifs(actual_path, std::ios::binary);
    if (!ifs.is_open()) {
        UNICALIB_WARN("[LidarPacketReader] 打开文件失败: {}", actual_path);
        if (is_compressed) std::filesystem::remove(temp_path);
        return false;
    }

    bool ok = false;
    if (decode_livox_or_hesai_stream(ifs, frames, max_frames)) {
        ok = true;
    } else {
        ifs.clear(); ifs.seekg(0);
        if (decode_blindspot_aacc_stream(ifs, frames, max_frames, blindspot_timestamp_unit_scale)) {
            ok = true;
        } else {
            ifs.clear(); ifs.seekg(0);
            std::string difop = guess_difop_from_msop_path(actual_path);
            namespace fs = std::filesystem;
            if (difop.empty() || !fs::exists(difop)) difop.clear();
            std::vector<DecodedLidarFrame> rs_frames;
            if (decode_rs_msop_difop(actual_path, difop, rs_frames, max_frames)) {
                frames = std::move(rs_frames);
                ok = true;
            }
        }
    }

    if (is_compressed) std::filesystem::remove(temp_path);
    if (!ok) UNICALIB_WARN("[LidarPacketReader] 解码失败: {}", file_path);
    return ok;
}

}  // namespace ns_unicalib