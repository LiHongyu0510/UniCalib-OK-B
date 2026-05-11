#include "unicalib/io/oem7_imu_reader.h"

#include "unicalib/common/logger.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <vector>

namespace ns_unicalib {
namespace {

// NovAtel OEM4/OEM7 二进制帧 CRC：与 novatel_edie novatelparser 中逐字节 CRC32 一致。
uint32_t crc32_oem7_table_value(uint32_t i) {
    uint32_t c = i;
    for (int k = 0; k < 8; ++k) {
        c = (c >> 1) ^ (0xEDB88320u & (uint32_t)-(int32_t)(c & 1u));
    }
    return c;
}

uint32_t crc32_oem7_update(uint32_t crc, uint8_t b) {
    static uint32_t tab[256];
    static bool tab_ok = false;
    if (!tab_ok) {
        for (uint32_t j = 0; j < 256; ++j) {
            tab[j] = crc32_oem7_table_value(j);
        }
        tab_ok = true;
    }
    const uint32_t t1 = (crc >> 8u) & 0x00FFFFFFu;
    const uint32_t t2 = tab[(crc ^ static_cast<uint32_t>(b)) & 0xFFu];
    return t1 ^ t2;
}

bool crc32_oem7_ok(const uint8_t* msg, size_t len) {
    if (len < 8u) {
        return false;
    }
    uint32_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
        crc = crc32_oem7_update(crc, msg[i]);
    }
    return crc == 0u;
}

uint16_t read_le_u16(const uint8_t* p) {
    uint16_t v;
    std::memcpy(&v, p, sizeof(v));
    return v;
}

double read_le_f64(const uint8_t* p) {
    double v;
    std::memcpy(&v, p, sizeof(v));
    return v;
}

uint32_t read_le_u32(const uint8_t* p) {
    uint32_t v;
    std::memcpy(&v, p, sizeof(v));
    return v;
}

int32_t read_le_i32(const uint8_t* p) {
    int32_t v;
    std::memcpy(&v, p, sizeof(v));
    return v;
}

// bynav oem7_message_ids.hpp
constexpr uint16_t kCorrImuDataSMsgId = 813u;       // CORRIMUDATAS
constexpr uint16_t kImuRateCorrImuSMsgId = 1362u;  // IMURATECORRIMUS
constexpr uint16_t kCorrImuSMsgId = 2264u;         // CORRIMUS

constexpr uint8_t kSync1 = 0xAAu;
constexpr uint8_t kSync2 = 0x44u;
constexpr uint8_t kSync3Short = 0x13u;
constexpr uint8_t kSync3Long = 0x12u;

constexpr size_t kShortHdr = 12u;
constexpr size_t kLongHdr = 28u;  // Oem7MessageHeaderMem（bynav oem7_messages.h）
constexpr size_t kCrcLen = 4u;
constexpr size_t kCorrPayload = 60u;

// 短头：week @+6 uint16, ms @+8 int32（与 Oem7MessgeShortHeaderMem 一致）
double gps_time_from_short_header(const uint8_t* sh) {
    const uint16_t w = read_le_u16(sh + 6);
    const int32_t ms = read_le_i32(sh + 8);
    return static_cast<double>(w) * 604800.0 + static_cast<double>(ms) * 1e-3;
}

// 长头：gps_week @+14 uint16, gps_milliseconds @+16 int32（Oem7MessageHeaderMem）
double gps_time_from_long_header(const uint8_t* lh) {
    const uint16_t w = read_le_u16(lh + 14);
    const int32_t ms = read_le_i32(lh + 16);
    return static_cast<double>(w) * 604800.0 + static_cast<double>(ms) * 1e-3;
}

// CORRIMUDATAS / IMURATECORRIMUS 载荷：周内秒在报文体（ins_handler CORRIMUDATA 分支 × imu_rate）
void payload_corr_imu_data_to_frame(const uint8_t* payload, double imu_rate_hz, IMUFrameRos& frame) {
    const uint32_t gps_week = read_le_u32(payload + 0);
    const double sec_in_week = read_le_f64(payload + 4);
    const double pitch_rate = read_le_f64(payload + 12);
    const double roll_rate = read_le_f64(payload + 20);
    const double yaw_rate = read_le_f64(payload + 28);
    const double lateral_acc = read_le_f64(payload + 36);
    const double longitudinal_acc = read_le_f64(payload + 44);
    const double vertical_acc = read_le_f64(payload + 52);

    frame.timestamp = static_cast<double>(gps_week) * 604800.0 + sec_in_week;
    frame.gyro[0] = pitch_rate * imu_rate_hz;
    frame.gyro[1] = roll_rate * imu_rate_hz;
    frame.gyro[2] = yaw_rate * imu_rate_hz;
    frame.accel[0] = lateral_acc * imu_rate_hz;
    frame.accel[1] = longitudinal_acc * imu_rate_hz;
    frame.accel[2] = vertical_acc * imu_rate_hz;
}

// CORRIMUS 载荷无周内绝对时标，时间用头；缩放 ins_handler：imu_rate / imu_data_count
void payload_corr_imus_to_frame(const uint8_t* payload,
                                double imu_rate_hz,
                                double header_gps_time_sec,
                                IMUFrameRos& frame) {
    const uint32_t imu_data_count = read_le_u32(payload + 0);
    const double pitch_rate = read_le_f64(payload + 4);
    const double roll_rate = read_le_f64(payload + 12);
    const double yaw_rate = read_le_f64(payload + 20);
    const double lateral_acc = read_le_f64(payload + 28);
    const double longitudinal_acc = read_le_f64(payload + 36);
    const double vertical_acc = read_le_f64(payload + 44);

    const double count = (imu_data_count > 0u) ? static_cast<double>(imu_data_count) : 1.0;
    const double factor = imu_rate_hz / count;

    frame.timestamp = header_gps_time_sec;
    frame.gyro[0] = pitch_rate * factor;
    frame.gyro[1] = roll_rate * factor;
    frame.gyro[2] = yaw_rate * factor;
    frame.accel[0] = lateral_acc * factor;
    frame.accel[1] = longitudinal_acc * factor;
    frame.accel[2] = vertical_acc * factor;
}

/**
 * 从 buf[0..) 尝试解一帧；成功则 decoded=true 且 consumed 为整帧字节数；
 * 失败且应向前滑 1 字节则 consumed=1；数据不足则 consumed=0。
 */
bool try_decode_one_frame(const uint8_t* buf,
                          size_t n,
                          double imu_rate_hz,
                          size_t& consumed,
                          IMUFrameRos& frame,
                          bool& decoded) {
    consumed = 0;
    decoded = false;
    if (n < 3u) {
        return false;
    }
    if (buf[0] != kSync1 || buf[1] != kSync2) {
        consumed = 1;
        return true;
    }

    if (buf[2] == kSync3Short) {
        if (n < kShortHdr + kCrcLen) {
            return false;
        }
        const uint8_t body_len = buf[3];
        const size_t total = kShortHdr + static_cast<size_t>(body_len) + kCrcLen;
        if (total < kShortHdr + kCrcLen || n < total) {
            return false;
        }
        if (!crc32_oem7_ok(buf, total)) {
            consumed = 1;
            return true;
        }
        const uint16_t msg_id = read_le_u16(buf + 4);
        if (body_len != kCorrPayload) {
            consumed = total;
            return true;
        }
        const double hdr_t = gps_time_from_short_header(buf);
        if (msg_id == kCorrImuDataSMsgId || msg_id == kImuRateCorrImuSMsgId) {
            payload_corr_imu_data_to_frame(buf + kShortHdr, imu_rate_hz, frame);
            decoded = true;
        } else if (msg_id == kCorrImuSMsgId) {
            payload_corr_imus_to_frame(buf + kShortHdr, imu_rate_hz, hdr_t, frame);
            decoded = true;
        }
        consumed = total;
        return true;
    }

    if (buf[2] == kSync3Long) {
        if (n < kLongHdr + kCrcLen) {
            return false;
        }
        const uint16_t msg_len = read_le_u16(buf + 8);
        const size_t total = kLongHdr + static_cast<size_t>(msg_len) + kCrcLen;
        if (n < total) {
            return false;
        }
        if (!crc32_oem7_ok(buf, total)) {
            consumed = 1;
            return true;
        }
        const uint16_t msg_id = read_le_u16(buf + 4);
        if (msg_len != kCorrPayload) {
            consumed = total;
            return true;
        }
        const double hdr_t = gps_time_from_long_header(buf);
        if (msg_id == kCorrImuDataSMsgId || msg_id == kImuRateCorrImuSMsgId) {
            payload_corr_imu_data_to_frame(buf + kLongHdr, imu_rate_hz, frame);
            decoded = true;
        } else if (msg_id == kCorrImuSMsgId) {
            payload_corr_imus_to_frame(buf + kLongHdr, imu_rate_hz, hdr_t, frame);
            decoded = true;
        }
        consumed = total;
        return true;
    }

    consumed = 1;
    return true;
}

// 1980-01-06 00:00:00 UTC（GPS 历元）对应的 Unix 秒（与常见 GNSS 库一致）
constexpr double kGpsEpochUnixSec = 315964800.0;

void apply_oem7_timestamps(std::vector<IMUFrameRos>& frames, const Oem7ImuDecodeParams& p) {
    const int leap = std::max(0, std::min(200, p.gps_minus_utc_leap_sec));
    for (auto& f : frames) {
        if (p.time_base == Oem7ImuTimeBase::UnixUtcApprox) {
            f.timestamp = kGpsEpochUnixSec + f.timestamp - static_cast<double>(leap) + p.time_offset_sec;
        } else {
            f.timestamp = f.timestamp + p.time_offset_sec;
        }
    }
}

}  // namespace

bool decode_oem7_imu_binary_file(const std::string& abs_path,
                                 const Oem7ImuDecodeParams& params,
                                 std::vector<IMUFrameRos>& out,
                                 std::string& err_msg) {
    out.clear();
    err_msg.clear();

    if (!(params.imu_output_rate_hz > 0.0) || !std::isfinite(params.imu_output_rate_hz)) {
        err_msg = "oem7_imu: imu_output_rate_hz 必须为正数（与 bynav INSHandler 的 imu_rate 一致）";
        return false;
    }

    std::ifstream ifs(abs_path, std::ios::binary);
    if (!ifs) {
        err_msg = "oem7_imu: 无法打开文件: " + abs_path;
        return false;
    }

    // 分块读入 + 滑动消费，避免整文件一次性载入内存（大 .bin 友好）
    constexpr size_t kReadChunk = 256u * 1024u;
    constexpr size_t kMaxBuffer = 16u * 1024u * 1024u;
    std::vector<uint8_t> buf;
    buf.reserve(kReadChunk + kLongHdr + kCorrPayload + kCrcLen + 64u);

    std::vector<char> chunk(kReadChunk);
    size_t decoded = 0;

    for (;;) {
        ifs.read(chunk.data(), static_cast<std::streamsize>(chunk.size()));
        const std::streamsize g = ifs.gcount();
        if (g <= 0) {
            break;
        }
        buf.insert(buf.end(), reinterpret_cast<const uint8_t*>(chunk.data()),
                   reinterpret_cast<const uint8_t*>(chunk.data()) + g);
        if (buf.size() > kMaxBuffer) {
            err_msg = "oem7_imu: 缓冲超过 16MB 仍无法完成同步，可能非 OEM7 二进制流: " + abs_path;
            return false;
        }

        size_t skip = 0;
        while (skip < buf.size()) {
            size_t adv = 0;
            IMUFrameRos frame{};
            bool got = false;
            const bool ok =
                try_decode_one_frame(buf.data() + skip, buf.size() - skip, params.imu_output_rate_hz, adv, frame, got);
            if (!ok) {
                break;
            }
            if (adv == 0u) {
                break;
            }
            skip += adv;
            if (got) {
                out.push_back(frame);
                ++decoded;
            }
        }
        if (skip > 0u) {
            buf.erase(buf.begin(), buf.begin() + static_cast<std::ptrdiff_t>(skip));
        }
    }

    // 排空尾部缓冲
    size_t skip = 0;
    while (skip < buf.size()) {
        size_t adv = 0;
        IMUFrameRos frame{};
        bool got = false;
        const bool ok =
            try_decode_one_frame(buf.data() + skip, buf.size() - skip, params.imu_output_rate_hz, adv, frame, got);
        if (!ok || adv == 0u) {
            break;
        }
        skip += adv;
        if (got) {
            out.push_back(frame);
            ++decoded;
        }
    }

    if (decoded == 0u) {
        err_msg =
            "oem7_imu: 未解析到 IMU 帧。支持短/长二进制 CORRIMUDATAS(813)、IMURATECORRIMUS(1362)、"
            "CORRIMUS(2264)，载荷 60 字节 + CRC32: " +
            abs_path;
        return false;
    }

    std::sort(out.begin(), out.end(), [](const IMUFrameRos& a, const IMUFrameRos& b) {
        return a.timestamp < b.timestamp;
    });

    apply_oem7_timestamps(out, params);

    const char* tb = (params.time_base == Oem7ImuTimeBase::UnixUtcApprox) ? "unix_utc_approx" : "gps_since_epoch";
    UNICALIB_INFO("[Oem7ImuReader] {} 解码 {} 帧 IMU（rate={} Hz，time_base={}，offset={} s，leap={}）", abs_path,
                  decoded, params.imu_output_rate_hz, tb, params.time_offset_sec, params.gps_minus_utc_leap_sec);
    return true;
}

bool decode_oem7_corr_imu_short_binary_file(const std::string& abs_path,
                                            double imu_output_rate_hz,
                                            std::vector<IMUFrameRos>& out,
                                            std::string& err_msg) {
    Oem7ImuDecodeParams p;
    p.imu_output_rate_hz = imu_output_rate_hz;
    return decode_oem7_imu_binary_file(abs_path, p, out, err_msg);
}

}  // namespace ns_unicalib
