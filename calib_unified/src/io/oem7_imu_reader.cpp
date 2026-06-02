#include "unicalib/io/oem7_imu_reader.h"

#include "unicalib/common/logger.h"
#include "unicalib/io/oem7_imu_scale.h"
#include "unicalib/io/oem7_messages.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace ns_unicalib {
namespace {

using namespace oem7;

// NovAtel OEM4/OEM7 二进制帧 CRC（与 novatel_edie novatelparser 一致）
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

constexpr uint8_t kSync1 = 0xAAu;
constexpr uint8_t kSync2 = 0x44u;
constexpr uint8_t kSync3Short = 0x13u;
constexpr uint8_t kSync3Long = 0x12u;

double gps_time_from_short_header(const uint8_t* sh) {
    const uint16_t w = read_le_u16(sh + 6);
    const int32_t ms = read_le_i32(sh + 8);
    return static_cast<double>(w) * 604800.0 + static_cast<double>(ms) * 1e-3;
}

double gps_time_from_long_header(const uint8_t* lh) {
    const uint16_t w = read_le_u16(lh + 14);
    const int32_t ms = read_le_i32(lh + 16);
    return static_cast<double>(w) * 604800.0 + static_cast<double>(ms) * 1e-3;
}

constexpr double kGpsEpochUnixSec = 315964800.0;
/// Bynav CORRIMU 文本为 GPST（~1.46e9）；RAWIMU 文本已是 Unix（~1.78e9）
constexpr double kTimestampLooksUnixSec = 1.65e9;

bool timestamp_looks_like_unix(double ts) {
    return std::isfinite(ts) && ts >= kTimestampLooksUnixSec;
}

/**
 * 北云 X2D 文本 log 在秒内小数越过 1.0 时不进位整秒（.95 → .105），需按序展开。
 */
struct BynavTimestampNormalizer {
    int64_t carry_sec = 0;
    double prev_frac = -1.0;
    bool initialized = false;

    double normalize(double ts) {
        const int64_t ibase = static_cast<int64_t>(ts);
        const double frac = ts - static_cast<double>(ibase);
        if (!initialized) {
            carry_sec = ibase;
            prev_frac = frac;
            initialized = true;
            return static_cast<double>(carry_sec) + frac;
        }
        if (frac < prev_frac - 0.05) {
            carry_sec += 1;
        } else if (ibase > carry_sec) {
            carry_sec = ibase;
        }
        prev_frac = frac;
        return static_cast<double>(carry_sec) + frac;
    }
};

void apply_oem7_timestamps(std::vector<IMUFrameRos>& frames, const Oem7ImuDecodeParams& p) {
    const int leap = std::max(0, std::min(200, p.gps_minus_utc_leap_sec));
    const bool want_unix = (p.time_base == Oem7ImuTimeBase::UnixUtcApprox);
    for (auto& f : frames) {
        const bool already_unix = timestamp_looks_like_unix(f.timestamp);
        if (want_unix) {
            if (!already_unix) {
                f.timestamp = kGpsEpochUnixSec + f.timestamp - static_cast<double>(leap);
            }
        } else if (already_unix) {
            f.timestamp = f.timestamp - kGpsEpochUnixSec + static_cast<double>(leap);
        }
        f.timestamp += p.time_offset_sec;
    }
}

struct BinaryDecodeContext {
    double imu_rate_hz = 200.0;
    Oem7ImuType imu_type = Oem7ImuType::Unknown;
    double gyro_scale = 0.0;
    double accel_scale = 0.0;
    bool scale_ready = false;
    double yaml_gyro_scale = 0.0;
    double yaml_accel_scale = 0.0;

    void apply_params(const Oem7ImuDecodeParams& p) {
        imu_rate_hz = p.imu_output_rate_hz;
        if (p.imu_type >= 0) {
            imu_type = static_cast<Oem7ImuType>(p.imu_type);
        }
        yaml_gyro_scale = p.imu_gyro_scale_factor;
        yaml_accel_scale = p.imu_accel_scale_factor;
        scale_ready = false;
        refresh_scale();
    }

    void refresh_scale() {
        if (scale_ready) {
            return;
        }
        // 与 bynav INSHandler 一致：launch/config 显式比例因子优先于 INSCONFIG 查表
        if (yaml_gyro_scale > 0.0 && yaml_accel_scale > 0.0) {
            gyro_scale = yaml_gyro_scale;
            accel_scale = yaml_accel_scale;
            scale_ready = true;
            return;
        }
        const int rate_hz = static_cast<int>(std::lround(imu_rate_hz));
        if (imu_type != Oem7ImuType::Unknown && rate_hz > 0) {
            scale_ready = oem7_get_imu_raw_scale_factors(imu_type, rate_hz, gyro_scale, accel_scale);
        }
    }

    void on_insconfig(const INSCONFIG_FixedMem* cfg) {
        imu_type = static_cast<Oem7ImuType>(cfg->imu_type);
        const int def_rate = oem7_default_imu_rate_hz(imu_type);
        if (def_rate > 0) {
            imu_rate_hz = static_cast<double>(def_rate);
        }
        scale_ready = false;
        refresh_scale();
    }
};

void payload_corr_imu_data_to_frame(const CORRIMUDATASMem* payload, double imu_rate_hz, IMUFrameRos& frame) {
    const double pitch_rate = payload->pitch_rate;
    const double roll_rate = payload->roll_rate;
    const double yaw_rate = payload->yaw_rate;
    frame.timestamp = static_cast<double>(payload->week) * 604800.0 + payload->seconds;
    frame.gyro[0] = pitch_rate * imu_rate_hz;
    frame.gyro[1] = roll_rate * imu_rate_hz;
    frame.gyro[2] = yaw_rate * imu_rate_hz;
    frame.accel[0] = payload->lateral_acc * imu_rate_hz;
    frame.accel[1] = payload->longitudinal_acc * imu_rate_hz;
    frame.accel[2] = payload->vertical_acc * imu_rate_hz;
}

void payload_corr_imus_to_frame(const CORRIMUSMem* payload,
                                double imu_rate_hz,
                                double header_gps_time_sec,
                                IMUFrameRos& frame) {
    const double count =
        (payload->imu_data_count > 0u) ? static_cast<double>(payload->imu_data_count) : 1.0;
    const double factor = imu_rate_hz / count;
    frame.timestamp = header_gps_time_sec;
    frame.gyro[0] = payload->pitch_rate * factor;
    frame.gyro[1] = payload->roll_rate * factor;
    frame.gyro[2] = payload->yaw_rate * factor;
    frame.accel[0] = payload->lateral_acc * factor;
    frame.accel[1] = payload->longitudinal_acc * factor;
    frame.accel[2] = payload->vertical_acc * factor;
}

bool payload_rawimusx_to_frame(const RAWIMUSXMem* raw, BinaryDecodeContext& ctx, IMUFrameRos& frame) {
    if (raw->imu_type != 0u) {
        ctx.imu_type = static_cast<Oem7ImuType>(raw->imu_type);
        ctx.scale_ready = false;
    }
    ctx.refresh_scale();
    if (!ctx.scale_ready) {
        return false;
    }
    frame.timestamp = static_cast<double>(raw->gnss_week) * 604800.0 + raw->gnss_week_seconds;
    frame.gyro[0] = static_cast<double>(raw->x_gyro) * ctx.gyro_scale;
    frame.gyro[1] = -static_cast<double>(raw->y_gyro) * ctx.gyro_scale;
    frame.gyro[2] = static_cast<double>(raw->z_gyro) * ctx.gyro_scale;
    frame.accel[0] = static_cast<double>(raw->x_acc) * ctx.accel_scale;
    frame.accel[1] = -static_cast<double>(raw->y_acc) * ctx.accel_scale;
    frame.accel[2] = static_cast<double>(raw->z_acc) * ctx.accel_scale;
    return true;
}

bool dispatch_imu_payload(uint16_t msg_id,
                          const uint8_t* payload,
                          size_t payload_len,
                          double header_gps_sec,
                          BinaryDecodeContext& ctx,
                          IMUFrameRos& frame) {
    if (msg_id == kCorrImuDataSMsgId || msg_id == kImuRateCorrImuSMsgId) {
        if (payload_len < sizeof(CORRIMUDATASMem)) {
            return false;
        }
        payload_corr_imu_data_to_frame(reinterpret_cast<const CORRIMUDATASMem*>(payload), ctx.imu_rate_hz, frame);
        return true;
    }
    if (msg_id == kCorrImuSMsgId) {
        if (payload_len < sizeof(CORRIMUSMem)) {
            return false;
        }
        payload_corr_imus_to_frame(reinterpret_cast<const CORRIMUSMem*>(payload), ctx.imu_rate_hz, header_gps_sec,
                                   frame);
        return true;
    }
    if (msg_id == kRawImusXMsgId) {
        if (payload_len < sizeof(RAWIMUSXMem)) {
            return false;
        }
        return payload_rawimusx_to_frame(reinterpret_cast<const RAWIMUSXMem*>(payload), ctx, frame);
    }
    if (msg_id == kInsConfigMsgId) {
        if (payload_len >= sizeof(INSCONFIG_FixedMem)) {
            ctx.on_insconfig(reinterpret_cast<const INSCONFIG_FixedMem*>(payload));
        }
        return false;
    }
    return false;
}

bool try_decode_one_frame(const uint8_t* buf,
                          size_t n,
                          BinaryDecodeContext& ctx,
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
        const double hdr_t = gps_time_from_short_header(buf);
        decoded = dispatch_imu_payload(msg_id, buf + kShortHdr, body_len, hdr_t, ctx, frame);
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
        const double hdr_t = gps_time_from_long_header(buf);
        decoded = dispatch_imu_payload(msg_id, buf + kLongHdr, msg_len, hdr_t, ctx, frame);
        consumed = total;
        return true;
    }

    consumed = 1;
    return true;
}

bool looks_like_oem7_binary(const std::string& path) {
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs) {
        return false;
    }
    std::vector<uint8_t> probe(64u * 1024u);
    ifs.read(reinterpret_cast<char*>(probe.data()), static_cast<std::streamsize>(probe.size()));
    const size_t n = static_cast<size_t>(ifs.gcount());
    for (size_t i = 0; i + 2 < n; ++i) {
        if (probe[i] == kSync1 && probe[i + 1] == kSync2 &&
            (probe[i + 2] == kSync3Short || probe[i + 2] == kSync3Long)) {
            return true;
        }
    }
    return false;
}

enum class BynavTextLogKind { RawImu, CorrImu, Unknown };

/** 11 字段文本 log 中后 6 列物理量顺序（与 UniCalib CSV gx,gy,gz,ax,ay,az 对应） */
enum class BynavElevenFieldLayout {
    AccelThenGyro,    ///< ax,ay,az,gx,gy,gz（北云 X2D driver 文本 log）
    CorrImuSemantic,  ///< roll,pitch,yaw,lon,lat,vert（OEM7 publishCorrImuAsImuMsg 语义）
};

void assign_eleven_field_to_frame(const double v[6], BynavElevenFieldLayout layout, IMUFrameRos& frame) {
    if (layout == BynavElevenFieldLayout::AccelThenGyro) {
        frame.accel[0] = v[0];
        frame.accel[1] = v[1];
        frame.accel[2] = v[2];
        frame.gyro[0] = v[3];
        frame.gyro[1] = v[4];
        frame.gyro[2] = v[5];
    } else {
        frame.gyro[0] = v[0];
        frame.gyro[1] = v[1];
        frame.gyro[2] = v[2];
        frame.accel[0] = v[3];
        frame.accel[1] = v[4];
        frame.accel[2] = v[5];
    }
}

BynavElevenFieldLayout probe_eleven_field_layout(const double v[6]) {
    if (std::abs(v[2]) > 3.0 && std::abs(v[3]) < 3.0) {
        return BynavElevenFieldLayout::AccelThenGyro;
    }
    if (std::abs(v[5]) > 3.0 && std::abs(v[2]) < 3.0) {
        return BynavElevenFieldLayout::CorrImuSemantic;
    }
    return BynavElevenFieldLayout::AccelThenGyro;
}

const char* eleven_field_layout_str(BynavElevenFieldLayout layout) {
    return (layout == BynavElevenFieldLayout::AccelThenGyro) ? "ax,ay,az,gx,gy,gz" : "roll,pitch,yaw,lon,lat,vert";
}

/** 从 bynav spdlog 行中解析 accel:[a,b,c] 或 gyro:[g,h,i] */
bool parse_bracket_triple(const std::string& line, const char* key, double out[3]) {
    const std::string needle = std::string(key) + ":[";
    const size_t pos = line.find(needle);
    if (pos == std::string::npos) {
        return false;
    }
    size_t i = pos + needle.size();
    for (int k = 0; k < 3; ++k) {
        while (i < line.size() && (line[i] == ' ' || line[i] == ',')) {
            ++i;
        }
        if (i >= line.size()) {
            return false;
        }
        char* end = nullptr;
        out[k] = std::strtod(line.c_str() + i, &end);
        if (end == line.c_str() + i || !std::isfinite(out[k])) {
            return false;
        }
        i = static_cast<size_t>(end - line.c_str());
    }
    return true;
}

/** CORRIMU_data 日志 header:sec.nanosec 为 GPST 连续秒（与 publishCorrImuAsImuMsg 一致） */
bool parse_bynav_header_timestamp(const std::string& line, double& ts_out) {
    const char* key = "header:";
    const size_t pos = line.find(key);
    if (pos == std::string::npos) {
        return false;
    }
    size_t i = pos + std::strlen(key);
    char* end = nullptr;
    const double sec = std::strtod(line.c_str() + i, &end);
    if (end == line.c_str() + i || !std::isfinite(sec)) {
        return false;
    }
    i = static_cast<size_t>(end - line.c_str());
    if (i < line.size() && line[i] == '.') {
        const double frac = std::strtod(line.c_str() + i, &end);
        if (end != line.c_str() + i && std::isfinite(frac)) {
            ts_out = sec + frac;
            return true;
        }
    }
    ts_out = sec;
    return true;
}

/**
 * bynav logger simple_pattern 行：MMDD HH:MM:SS.ffffff pub_sys:... header:... accel:[...] gyro:[...]
 * 值为 publishCorrImuAsImuMsg / processRawImuMsg 已换算的物理量，勿再乘 imu_rate。
 */
bool parse_bynav_spdlog_imu_line(const std::string& line,
                                 BynavTextLogKind kind,
                                 BynavTimestampNormalizer& ts_norm,
                                 IMUFrameRos& frame) {
    if (line.find("accel:[") == std::string::npos || line.find("gyro:[") == std::string::npos) {
        return false;
    }
    double accel[3] = {};
    double gyro[3] = {};
    if (!parse_bracket_triple(line, "accel", accel) || !parse_bracket_triple(line, "gyro", gyro)) {
        return false;
    }
    double ts = 0.0;
    if (!parse_bynav_header_timestamp(line, ts)) {
        std::istringstream iss(line);
        std::string mmdd, hms, ts1s;
        if (!(iss >> mmdd >> hms >> ts1s)) {
            return false;
        }
        try {
            ts = ts_norm.normalize(std::stod(ts1s));
        } catch (...) {
            return false;
        }
    } else {
        ts = ts_norm.normalize(ts);
    }
    frame.timestamp = ts;
    frame.accel[0] = accel[0];
    frame.accel[1] = accel[1];
    frame.accel[2] = accel[2];
    frame.gyro[0] = gyro[0];
    frame.gyro[1] = gyro[1];
    frame.gyro[2] = gyro[2];
    (void)kind;
    return true;
}

/** 7 列 CSV：timestamp,gx,gy,gz,ax,ay,az（unicalib_imu_parse 输出） */
bool parse_bynav_csv7_imu_line(const std::string& line, IMUFrameRos& frame) {
    if (line.empty() || line[0] == '#') {
        return false;
    }
    std::istringstream iss(line);
    std::string tok;
    std::vector<double> vals;
    while (std::getline(iss, tok, ',')) {
        if (tok.empty()) {
            continue;
        }
        try {
            vals.push_back(std::stod(tok));
        } catch (...) {
            return false;
        }
    }
    if (vals.size() < 7u) {
        return false;
    }
    frame.timestamp = vals[0];
    frame.gyro[0] = vals[1];
    frame.gyro[1] = vals[2];
    frame.gyro[2] = vals[3];
    frame.accel[0] = vals[4];
    frame.accel[1] = vals[5];
    frame.accel[2] = vals[6];
    return std::isfinite(frame.timestamp) && std::isfinite(frame.gyro[0]) && std::isfinite(frame.accel[0]);
}

bool try_extract_eleven_field_samples(const std::string& line, double v[6]) {
    std::istringstream iss(line);
    std::string mmdd, hms, ts1s, ts2s, frame_id;
    if (!(iss >> mmdd >> hms >> ts1s >> ts2s >> frame_id)) {
        return false;
    }
    for (int i = 0; i < 6; ++i) {
        std::string tok;
        if (!(iss >> tok)) {
            return false;
        }
        try {
            v[i] = std::stod(tok);
        } catch (...) {
            return false;
        }
    }
    return true;
}

void refine_bynav_eleven_field_layout_from_content(std::istream& ifs, BynavElevenFieldLayout& layout) {
    const std::streampos pos = ifs.tellg();
    std::string line;
    while (std::getline(ifs, line)) {
        double v[6] = {};
        if (!try_extract_eleven_field_samples(line, v)) {
            continue;
        }
        layout = probe_eleven_field_layout(v);
        break;
    }
    ifs.clear();
    ifs.seekg(pos);
}

bool parse_bynav_text_imu_line(const std::string& line,
                               BynavElevenFieldLayout layout,
                               double imu_rate_hz,
                               BynavTimestampNormalizer& ts_norm,
                               IMUFrameRos& frame) {
    if (line.empty() || line[0] == '#') {
        return false;
    }
    if (parse_bynav_spdlog_imu_line(line, BynavTextLogKind::Unknown, ts_norm, frame)) {
        return true;
    }
    if (parse_bynav_csv7_imu_line(line, frame)) {
        return true;
    }

    // 兼容旧版 11 字段：MMDD TIME ts ts frame + 6 个数（已是物理量，勿乘 rate）
    std::istringstream iss(line);
    std::string mmdd;
    std::string hms;
    std::string ts1s;
    std::string ts2s;
    std::string frame_id;
    if (!(iss >> mmdd >> hms >> ts1s >> ts2s >> frame_id)) {
        return false;
    }
    double ts = 0.0;
    try {
        ts = ts_norm.normalize(std::stod(ts1s));
    } catch (...) {
        return false;
    }

    double v[6] = {};
    for (int i = 0; i < 6; ++i) {
        std::string tok;
        if (!(iss >> tok)) {
            return false;
        }
        try {
            v[i] = std::stod(tok);
        } catch (...) {
            return false;
        }
    }

    frame.timestamp = ts;
    assign_eleven_field_to_frame(v, layout, frame);
    (void)imu_rate_hz;
    (void)frame_id;
    (void)ts2s;
    (void)hms;
    (void)mmdd;
    return std::isfinite(frame.timestamp) && std::isfinite(frame.gyro[0]) && std::isfinite(frame.accel[0]);
}

bool decode_bynav_text_imu_log_file(const std::string& abs_path,
                                    const Oem7ImuDecodeParams& params,
                                    std::vector<IMUFrameRos>& out,
                                    std::string& err_msg) {
    std::ifstream ifs(abs_path);
    if (!ifs) {
        err_msg = "bynav_text_imu: 无法打开文件: " + abs_path;
        return false;
    }

    // 北云 X2D CORRIMU/RAWIMU 文本 log 后 6 列均为 ax,ay,az,gx,gy,gz（非 OEM7 roll/pitch/yaw 语义）
    BynavElevenFieldLayout layout = BynavElevenFieldLayout::AccelThenGyro;
    refine_bynav_eleven_field_layout_from_content(ifs, layout);

    std::string line;
    size_t line_no = 0;
    size_t decoded = 0;
    BynavTimestampNormalizer ts_norm;
    while (std::getline(ifs, line)) {
        ++line_no;
        while (!line.empty() && (line.back() == '\r' || line.back() == '\n')) {
            line.pop_back();
        }
        IMUFrameRos fr{};
        if (!parse_bynav_text_imu_line(line, layout, params.imu_output_rate_hz, ts_norm, fr)) {
            continue;
        }
        out.push_back(fr);
        ++decoded;
    }

    if (decoded == 0u) {
        err_msg = "bynav_text_imu: 未解析到有效 IMU 行（期望: MMDD TIME ts ts frame ax ay az gx gy gz）: " + abs_path;
        return false;
    }

    std::sort(out.begin(), out.end(), [](const IMUFrameRos& a, const IMUFrameRos& b) {
        return a.timestamp < b.timestamp;
    });
    apply_oem7_timestamps(out, params);

    const char* tb = (params.time_base == Oem7ImuTimeBase::UnixUtcApprox) ? "unix_utc_approx" : "gps_since_epoch";
    UNICALIB_INFO("[BynavTextImu] {} 解码 {} 帧（11-field layout={} rate={} Hz time_base={}）", abs_path, decoded,
                  eleven_field_layout_str(layout), params.imu_output_rate_hz, tb);
    return true;
}

bool decode_oem7_binary_stream(const std::string& abs_path,
                               const Oem7ImuDecodeParams& params,
                               std::vector<IMUFrameRos>& out,
                               std::string& err_msg) {
    std::ifstream ifs(abs_path, std::ios::binary);
    if (!ifs) {
        err_msg = "oem7_imu: 无法打开文件: " + abs_path;
        return false;
    }

    BinaryDecodeContext ctx;
    ctx.apply_params(params);
    ctx.refresh_scale();

    constexpr size_t kReadChunk = 256u * 1024u;
    constexpr size_t kMaxBuffer = 16u * 1024u * 1024u;
    std::vector<uint8_t> buf;
    buf.reserve(kReadChunk + kLongHdr + 64u);

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
            const bool ok = try_decode_one_frame(buf.data() + skip, buf.size() - skip, ctx, adv, frame, got);
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

    size_t skip = 0;
    while (skip < buf.size()) {
        size_t adv = 0;
        IMUFrameRos frame{};
        bool got = false;
        const bool ok = try_decode_one_frame(buf.data() + skip, buf.size() - skip, ctx, adv, frame, got);
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
            "oem7_imu: 未解析到 IMU 帧。支持 CORRIMUDATAS(813)、IMURATECORRIMUS(1362)、CORRIMUS(2264)、"
            "RAWIMUSX(1462)、INSCONFIG(1945): " +
            abs_path;
        return false;
    }

    std::sort(out.begin(), out.end(), [](const IMUFrameRos& a, const IMUFrameRos& b) {
        return a.timestamp < b.timestamp;
    });
    apply_oem7_timestamps(out, params);

    const char* tb = (params.time_base == Oem7ImuTimeBase::UnixUtcApprox) ? "unix_utc_approx" : "gps_since_epoch";
    UNICALIB_INFO("[Oem7ImuReader] {} 解码 {} 帧 IMU（rate={} Hz time_base={} offset={} s gyro_scale={} accel_scale={}）",
                  abs_path, decoded, ctx.imu_rate_hz, tb, params.time_offset_sec, ctx.gyro_scale, ctx.accel_scale);
    if (!out.empty()) {
        const auto& f0 = out.front();
        UNICALIB_INFO("[Oem7ImuReader] 首帧 ts={:.6f} gyro=[{:.6f},{:.6f},{:.6f}] accel=[{:.6f},{:.6f},{:.6f}]",
                      f0.timestamp, f0.gyro[0], f0.gyro[1], f0.gyro[2], f0.accel[0], f0.accel[1], f0.accel[2]);
    }
    return true;
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

    if (looks_like_oem7_binary(abs_path)) {
        return decode_oem7_binary_stream(abs_path, params, out, err_msg);
    }

    std::ifstream peek(abs_path);
    if (!peek) {
        err_msg = "oem7_imu: 无法打开文件: " + abs_path;
        return false;
    }
    char c0 = 0;
    peek.get(c0);
    const bool likely_text = (c0 == '#' || std::isdigit(static_cast<unsigned char>(c0)) || std::isspace(static_cast<unsigned char>(c0)));
    if (likely_text) {
        return decode_bynav_text_imu_log_file(abs_path, params, out, err_msg);
    }

    return decode_oem7_binary_stream(abs_path, params, out, err_msg);
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
