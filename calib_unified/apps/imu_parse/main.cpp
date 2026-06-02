/**
 * IMU 数据解析：OEM7 二进制 / Bynav 文本 .log → 7 列 CSV (timestamp,gx,gy,gz,ax,ay,az)
 */
#include "unicalib/io/oem7_imu_reader.h"

#include <algorithm>
#include <cstring>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using namespace ns_unicalib;

namespace {

void print_usage(const char* prog) {
    std::cerr
        << "用法:\n"
        << "  " << prog << " <input.log|bin> <output.csv> [rate_hz] [gps|unix]\n"
        << "  " << prog << " --batch <input_dir> <output_dir> [rate_hz] [gps|unix]\n"
        << "\n"
        << "  Bynav 文本: RAWIMU/CORRIMU_data_*.log（spdlog accel/gyro 括号格式，已是物理量）\n"
        << "  二进制 RAWIMUSX 建议配置 oem7_imu_gyro/accel_scale_factor（与 bynav config.yaml 一致）\n"
        << "  自动跳过: INSPVAX、空文件、非 IMU 日志\n";
}

Oem7ImuDecodeParams make_params(int argc, char** argv, int base_idx) {
    Oem7ImuDecodeParams p;
    if (argc > base_idx) {
        try {
            p.imu_output_rate_hz = std::stod(argv[base_idx]);
        } catch (...) {
        }
    }
    if (argc > base_idx + 1) {
        std::string tb = argv[base_idx + 1];
        for (char& c : tb) {
            c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        }
        if (tb == "unix" || tb == "unix_utc" || tb == "utc") {
            p.time_base = Oem7ImuTimeBase::UnixUtcApprox;
        }
    }
    return p;
}

bool write_csv(const std::string& out_path, const std::vector<IMUFrameRos>& frames) {
    std::ofstream ofs(out_path);
    if (!ofs) {
        return false;
    }
    ofs << "# timestamp,gx,gy,gz,ax,ay,az\n";
    ofs << std::fixed << std::setprecision(9);
    for (const auto& fr : frames) {
        ofs << fr.timestamp << ',' << fr.gyro[0] << ',' << fr.gyro[1] << ',' << fr.gyro[2] << ','
            << fr.accel[0] << ',' << fr.accel[1] << ',' << fr.accel[2] << '\n';
    }
    return true;
}

bool decode_one_file(const std::string& in_path, const std::string& out_path,
                     const Oem7ImuDecodeParams& params) {
    std::vector<IMUFrameRos> frames;
    std::string err;
    if (!decode_oem7_imu_binary_file(in_path, params, frames, err)) {
        std::cerr << "[SKIP] " << in_path << ": " << err << '\n';
        return false;
    }
    if (!write_csv(out_path, frames)) {
        std::cerr << "[FAIL] 写入 CSV: " << out_path << '\n';
        return false;
    }
    std::cout << "[OK] " << in_path << " → " << out_path << " (" << frames.size() << " 帧)\n";
    return true;
}

bool ends_with_ci(const std::string& s, const char* suf) {
    const size_t n = std::strlen(suf);
    if (s.size() < n) {
        return false;
    }
    for (size_t i = 0; i < n; ++i) {
        const char a = static_cast<char>(std::tolower(static_cast<unsigned char>(s[s.size() - n + i])));
        const char b = static_cast<char>(std::tolower(static_cast<unsigned char>(suf[i])));
        if (a != b) {
            return false;
        }
    }
    return true;
}

bool should_parse_imu_log(const fs::path& p) {
    if (!fs::is_regular_file(p)) {
        return false;
    }
    if (fs::file_size(p) == 0u) {
        return false;
    }
    const std::string name = p.filename().string();
    std::string lower = name;
    for (char& c : lower) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    if (lower.find("inspvax") != std::string::npos) {
        return false;
    }
    if (lower.find("bestgnss") != std::string::npos) {
        return false;
    }
    if (lower.find("bynav_log") != std::string::npos && lower.find("data") == std::string::npos) {
        return false;
    }
    return lower.find("rawimu") != std::string::npos || lower.find("corrimu") != std::string::npos ||
           ends_with_ci(lower, ".log") || ends_with_ci(lower, ".bin");
}

int run_batch(const fs::path& in_dir, const fs::path& out_dir, const Oem7ImuDecodeParams& params) {
    fs::create_directories(out_dir);
    int ok = 0;
    int fail = 0;
    std::vector<fs::path> files;
    for (const auto& ent : fs::directory_iterator(in_dir)) {
        if (!should_parse_imu_log(ent.path())) {
            continue;
        }
        files.push_back(ent.path());
    }
    std::sort(files.begin(), files.end());
    if (files.empty()) {
        std::cerr << "[FAIL] 目录内无 RAWIMU/CORRIMU .log: " << in_dir << '\n';
        return 1;
    }
    for (const auto& f : files) {
        const fs::path out_csv = out_dir / (f.stem().string() + ".csv");
        if (decode_one_file(f.string(), out_csv.string(), params)) {
            ++ok;
        } else {
            ++fail;
        }
    }
    std::cout << "[OK] 批量完成: 成功 " << ok << " / " << (ok + fail) << " → " << out_dir << '\n';
    return ok > 0 ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        print_usage(argv[0]);
        return 1;
    }

    if (std::string(argv[1]) == "--batch") {
        if (argc < 4) {
            print_usage(argv[0]);
            return 1;
        }
        const fs::path in_dir(argv[2]);
        const fs::path out_dir(argv[3]);
        if (!fs::is_directory(in_dir)) {
            std::cerr << "[FAIL] 输入不是目录: " << in_dir << '\n';
            return 1;
        }
        return run_batch(in_dir, out_dir, make_params(argc, argv, 4));
    }

    const std::string in_path = argv[1];
    const std::string out_path = argv[2];
    const Oem7ImuDecodeParams params = make_params(argc, argv, 3);

    if (fs::is_directory(in_path)) {
        return run_batch(in_path, fs::path(out_path), params);
    }

    fs::create_directories(fs::path(out_path).parent_path());
    const std::string out_lower = fs::path(out_path).filename().string();
    if (params.time_base != Oem7ImuTimeBase::UnixUtcApprox) {
        if (out_lower.find("unix") != std::string::npos) {
            std::cerr << "[WARN] 输出文件名含 unix 但时间基为 gps；标定 file-mode 请使用: ... unix\n";
        }
    }
    return decode_one_file(in_path, out_path, params) ? 0 : 1;
}
