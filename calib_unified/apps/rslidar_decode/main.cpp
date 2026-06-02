/**
 * RoboSense Helios MSOP/DIFOP 批量解码 — 实现统一走 unicalib::LidarPacketReader
 * （与 thirdparty/sensor_decode/lidar_decode 逻辑对齐）
 */
#include "unicalib/io/lidar_packet_reader.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <chrono>
#include <filesystem>
#include <iostream>
#include <map>
#include <regex>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using ns_unicalib::DecodedLidarFrame;
using ns_unicalib::LidarPacketReader;

static void classifyFiles(
    const fs::path& directory_path,
    std::map<std::string, std::vector<std::pair<std::string, std::string>>>& rslidar_files) {
    const std::regex msop_pattern(R"(lidarMSOP(\d+)_(\d{6,8}_\d{6}))");
    const std::regex difop_pattern(R"(lidarDIFOP(\d+)_(\d{6,8}_\d{6}))");
    std::smatch match;

    std::vector<fs::path> files;
    for (const auto& entry : fs::recursive_directory_iterator(directory_path)) {
        if (entry.is_regular_file()) {
            files.push_back(entry.path());
        }
    }
    std::sort(files.begin(), files.end());

    for (const auto& fp : files) {
        const std::string name = fp.filename().string();
        if (std::regex_search(name, match, msop_pattern)) {
            const std::string key = match[1].str() + "_" + match[2].str();
            rslidar_files[key].emplace_back(fp.string(), "");
        } else if (std::regex_search(name, match, difop_pattern)) {
            const std::string key = match[1].str() + "_" + match[2].str();
            rslidar_files[key].emplace_back("", fp.string());
        }
    }
}

static bool decode_and_save_pcd(const std::string& msop_file, const std::string& difop_file,
                                const fs::path& output_dir) {
    std::vector<DecodedLidarFrame> frames;
    if (!LidarPacketReader::decode_rs_msop_difop(msop_file, difop_file, frames, 0) || frames.empty()) {
        std::cerr << "[ERROR] 解码失败: " << msop_file << std::endl;
        return false;
    }

    const std::string sensor_name = fs::path(msop_file).parent_path().filename().string();
    const fs::path pcd_dir = output_dir / "pcd" / sensor_name;
    fs::create_directories(pcd_dir);

    uint64_t total_points = 0;
    int frame_count = 0;
    for (const auto& fr : frames) {
        if (!fr.cloud || fr.cloud->empty()) {
            continue;
        }
        const uint64_t stamp_ms =
            static_cast<uint64_t>(fr.timestamp_sec * 1000.0 + 0.5);
        const std::string pcd_file =
            pcd_dir.string() + "/" + std::to_string(frame_count) + "_" + std::to_string(stamp_ms) + ".pcd";
        if (pcl::io::savePCDFileBinary(pcd_file, *fr.cloud) == 0) {
            std::cout << "  [PCD] " << pcd_file << " (" << fr.cloud->size() << " pts, ts=" << stamp_ms
                      << "ms)" << std::endl;
        }
        total_points += fr.cloud->size();
        ++frame_count;
    }

    std::cout << "  解码完成: " << msop_file << std::endl;
    std::cout << "  帧数: " << frame_count << ", 总点数: " << total_points << std::endl;
    return frame_count > 0;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "用法: unicalib_rslidar_decode <输入目录> [输出目录]" << std::endl;
        std::cerr << "  递归扫描 lidarMSOP* / lidarDIFOP* 并解码为 PCD（与 sensor_decode 对齐）" << std::endl;
        return 1;
    }

    const fs::path input_dir = argv[1];
    const fs::path output_dir = (argc > 2) ? fs::path(argv[2]) : (input_dir / ".." / ".." / "..");
    fs::create_directories(output_dir);

    std::map<std::string, std::vector<std::pair<std::string, std::string>>> rslidar_files;
    classifyFiles(input_dir, rslidar_files);

    std::cout << "找到 " << rslidar_files.size() << " 组 RS 雷达文件" << std::endl;

    const auto start = std::chrono::system_clock::now();
    int ok_groups = 0;

    for (const auto& [key, files] : rslidar_files) {
        std::string msop_file;
        std::string difop_file;
        for (const auto& [msop, difop] : files) {
            if (!msop.empty()) {
                msop_file = msop;
            }
            if (!difop.empty()) {
                difop_file = difop;
            }
        }
        if (msop_file.empty()) {
            std::cerr << "[SKIP] " << key << " 缺少 MSOP 文件" << std::endl;
            continue;
        }
        std::cout << "\n--- " << key << " ---" << std::endl;
        std::cout << "  MSOP: " << msop_file << std::endl;
        std::cout << "  DIFOP: " << (difop_file.empty() ? "(无)" : difop_file) << std::endl;

        if (decode_and_save_pcd(msop_file, difop_file, output_dir)) {
            ++ok_groups;
        }
    }

    const auto elapsed =
        std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start).count();
    std::cout << "\n全部完成! 成功 " << ok_groups << " / " << rslidar_files.size() << ", 耗时: " << elapsed
              << "s" << std::endl;
    return ok_groups > 0 ? 0 : 1;
}
