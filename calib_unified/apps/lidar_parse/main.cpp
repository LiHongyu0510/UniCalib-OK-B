#include "unicalib/io/lidar_packet_reader.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <algorithm>
#include <cctype>
#include <filesystem>
#include <iostream>
#include <string>
#include <string_view>
#include <vector>

namespace fs = std::filesystem;
using namespace ns_unicalib;

namespace {

/** 与 lidar_packet_reader.cpp 中 guess_difop_from_msop_path 一致：lidarMSOP* -> lidarDIFOP* */
std::string guess_difop_from_msop_path(const std::string& msop_path) {
    fs::path p(msop_path);
    std::string name = p.filename().string();
    const std::string key = "lidarMSOP";
    auto pos = name.find(key);
    if (pos != std::string::npos) {
        name.replace(pos, key.size(), "lidarDIFOP");
        return (p.parent_path() / name).string();
    }
    pos = name.find("MSOP");
    if (pos != std::string::npos) {
        name.replace(pos, 4, "DIFOP");
        return (p.parent_path() / name).string();
    }
    return {};
}

bool ends_with_ci(std::string_view s, std::string_view suf) {
    if (s.size() < suf.size()) return false;
    for (size_t i = 0; i < suf.size(); ++i) {
        const char a = static_cast<char>(std::tolower(static_cast<unsigned char>(s[s.size() - suf.size() + i])));
        const char b = static_cast<char>(std::tolower(static_cast<unsigned char>(suf[i])));
        if (a != b) return false;
    }
    return true;
}

/**
 * 文件名含 lidarMSOP 且同目录存在配对 DIFOP 时，直接走 RS 解码，避免 decode_file 先扫 Livox 在大 MSOP 上极慢。
 */
bool decode_lidar_file_fast(const std::string& in_path, std::vector<DecodedLidarFrame>& frames,
                            std::size_t max_frames) {
    frames.clear();
    const std::string base = fs::path(in_path).filename().string();

    // 旧的 RoboSense Helios 机械雷达（有 MSOP/DIFOP）
    if (base.find("lidarMSOP") != std::string::npos || base.find("MSOP") != std::string::npos) {
        const std::string difop = guess_difop_from_msop_path(in_path);
        if (!difop.empty() && fs::exists(difop)) {
            if (LidarPacketReader::decode_rs_msop_difop(in_path, difop, frames, max_frames)) {
                return true;
            }
        }
        if (LidarPacketReader::decode_rs_msop_difop(in_path, "", frames, max_frames)) {
            return true;
        }
    }

    // 新的补盲雷达（Airy / 自定义二进制格式，无 MSOP/DIFOP），直接走通用解码器
    // 参考 sensor_decode/lidar_decode 中的 Livox/Hesai/Blindspot 解析逻辑
    return LidarPacketReader::decode_file(in_path, frames, max_frames);
}

/** 批量输出基名：默认与 MSOP 文件名一一对应（lidarMSOP0_xxx.pcd）；--name-difop 时用配对 DIFOP 基名 */
fs::path output_stem_for_msop(const fs::path& msop_path, const fs::path& out_dir, bool name_as_difop) {
    if (name_as_difop) {
        const std::string difop = guess_difop_from_msop_path(msop_path.string());
        if (!difop.empty() && fs::exists(difop)) {
            return out_dir / fs::path(difop).stem();
        }
    }
    return out_dir / msop_path.stem();
}

pcl::PointCloud<pcl::PointXYZI>::Ptr merge_all_frames(const std::vector<DecodedLidarFrame>& frames) {
    auto out = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    std::size_t total = 0;
    for (const auto& f : frames) {
        if (f.cloud) total += f.cloud->size();
    }
    out->points.reserve(total);
    for (const auto& f : frames) {
        if (!f.cloud || f.cloud->empty()) continue;
        out->points.insert(out->points.end(), f.cloud->points.begin(), f.cloud->points.end());
    }
    out->width = static_cast<uint32_t>(out->size());
    out->height = 1;
    out->is_dense = false;
    return out;
}

void save_all_frames(const std::string& out_path_stem, const std::vector<DecodedLidarFrame>& frames) {
    for (size_t i = 0; i < frames.size(); ++i) {
        const auto& f = frames[i];
        if (!f.cloud || f.cloud->empty()) continue;
        std::string out = out_path_stem;
        if (frames.size() > 1) {
            out += "_frame" + std::to_string(i) + ".pcd";
        } else {
            if (!ends_with_ci(out, ".pcd")) {
                out += ".pcd";
            }
        }
        if (pcl::io::savePCDFileBinary(out, *f.cloud) != 0) {
            std::cerr << "[WARN] 保存失败: " << out << std::endl;
        } else {
            std::cout << "[OK] 已保存: " << out << " (" << f.cloud->size() << " 点)" << std::endl;
        }
    }
}

/** 默认合并多帧为一个 PCD；per_frame 时仍按 _frameN 拆分 */
void save_frames_default_merge(const fs::path& stem, const std::vector<DecodedLidarFrame>& frames,
                               bool per_frame) {
    if (per_frame) {
        save_all_frames(stem.string(), frames);
        return;
    }
    if (frames.size() == 1) {
        save_all_frames(stem.string(), frames);
        return;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr merged = merge_all_frames(frames);
    if (!merged || merged->empty()) {
        std::cerr << "[WARN] 合并后点云为空: " << stem << std::endl;
        return;
    }
    std::string out = stem.string();
    if (!ends_with_ci(out, ".pcd")) {
        out += ".pcd";
    }
    if (pcl::io::savePCDFileBinary(out, *merged) != 0) {
        std::cerr << "[WARN] 保存失败: " << out << std::endl;
    } else {
        std::cout << "[OK] 已合并保存: " << out << " (" << merged->size() << " 点, 共 " << frames.size()
                  << " 帧)" << std::endl;
    }
}

int run_batch_dir(const fs::path& in_dir, const fs::path& out_dir, std::size_t max_frames, bool per_frame,
                  bool name_as_difop) {
    std::error_code ec;
    fs::create_directories(out_dir, ec);
    if (ec) {
        std::cerr << "[FAIL] 无法创建输出目录: " << out_dir << " " << ec.message() << std::endl;
        return 1;
    }

    // 支持两种命名：旧的 RoboSense Helios (lidarMSOP*) 和新的补盲雷达 (lidar0_* / lidar1_* 等)
    std::vector<fs::path> lidar_files;
    for (const fs::directory_entry& ent : fs::directory_iterator(in_dir, ec)) {
        if (!ent.is_regular_file()) continue;
        const std::string name = ent.path().filename().string();
        if (name.find("lidarMSOP") != std::string::npos ||
            name.find("MSOP") != std::string::npos ||
            name.find("lidar0_") != std::string::npos ||
            name.find("lidar1_") != std::string::npos) {
            lidar_files.push_back(ent.path());
        }
    }
    std::sort(lidar_files.begin(), lidar_files.end());

    if (lidar_files.empty()) {
        std::cerr << "[FAIL] 目录中未找到支持的雷达文件 (lidarMSOP* / lidar0_* / lidar1_*) : " << in_dir << std::endl;
        return 1;
    }

    size_t ok_n = 0;
    for (const fs::path& f : lidar_files) {
        std::vector<DecodedLidarFrame> frames;
        if (!decode_lidar_file_fast(f.string(), frames, max_frames) || frames.empty()) {
            std::cerr << "[FAIL] 解码失败: " << f << std::endl;
            continue;
        }
        // 新格式（补盲雷达）没有 DIFOP，直接用输入文件名作为输出基名
        fs::path stem = out_dir / f.stem();
        std::cout << "[OK] " << f.filename() << " 帧数: " << frames.size() << std::endl;
        save_frames_default_merge(stem, frames, per_frame);
        ++ok_n;
    }

    std::cout << "[OK] 批量完成: 成功 " << ok_n << " / " << lidar_files.size() << std::endl;
    return ok_n > 0 ? 0 : 1;
}

void print_usage() {
    std::cerr
        << "用法:\n"
        << "  单文件: unicalib_lidar_parse [选项] <lidar_file> [output.pcd] [max_frames]\n"
        << "  批量:   unicalib_lidar_parse [选项] <目录> <输出目录> [max_frames]\n"
        << "选项:\n"
        << "  --per-frame   多帧时按帧写出 *_frameN.pcd；默认合并为一个 PCD\n"
        << "  --name-difop  仅对 RS Helios 有效：输出基名用配对 lidarDIFOP*\n"
        << "支持格式:\n"
        << "  - RoboSense Helios 机械雷达 (lidarMSOP* + lidarDIFOP*)\n"
        << "  - 补盲雷达 / Livox / Hesai 自定义二进制 (lidar0_* / lidar1_* 等)\n"
        << "  - .pcd / .gz\n"
        << "说明: 批量模式会自动识别文件名，旧格式走 RS 专用路径，新格式走通用解码器（参考 sensor_decode/lidar_decode）。\n";
}

}  // namespace

int main(int argc, char** argv) {
    bool per_frame = false;
    bool name_as_difop = false;
    std::vector<char*> pos;
    for (int i = 1; i < argc; ++i) {
        const std::string_view a(argv[i]);
        if (a == "--per-frame") {
            per_frame = true;
        } else if (a == "--name-difop") {
            name_as_difop = true;
        } else if (a == "-h" || a == "--help") {
            print_usage();
            return 0;
        } else {
            pos.push_back(argv[i]);
        }
    }

    if (pos.size() >= 2 && fs::is_directory(pos[0])) {
        const fs::path in_dir(pos[0]);
        const fs::path out_dir(pos[1]);
        std::size_t max_frames = 0;
        if (pos.size() >= 3) {
            try {
                max_frames = static_cast<std::size_t>(std::stoull(pos[2]));
            } catch (...) {
                std::cerr << "[FAIL] max_frames 无效: " << pos[2] << std::endl;
                return 1;
            }
        }
        std::cerr << "批量模式: " << in_dir << " -> " << out_dir
                  << (max_frames ? " (每文件最多 " + std::to_string(max_frames) + " 帧)" : " (每文件全量解码)")
                  << (per_frame ? " [--per-frame]" : " [每 MSOP 合并为一个 PCD]")
                  << (name_as_difop ? " [--name-difop]" : " [输出名=lidarMSOP 基名]") << std::endl;
        return run_batch_dir(in_dir, out_dir, max_frames, per_frame, name_as_difop);
    }

    if (pos.empty()) {
        print_usage();
        return 1;
    }

    std::string in_path = pos[0];
    std::string out_path = (pos.size() >= 2) ? pos[1] : "decoded.pcd";
    std::size_t max_frames = 0;
    if (pos.size() >= 3) {
        try {
            max_frames = static_cast<std::size_t>(std::stoull(pos[2]));
        } catch (...) {
            std::cerr << "[FAIL] max_frames 无效: " << pos[2] << std::endl;
            return 1;
        }
    }

    std::vector<DecodedLidarFrame> frames;
    if (!decode_lidar_file_fast(in_path, frames, max_frames) || frames.empty()) {
        std::cerr << "[FAIL] 解码失败: " << in_path << std::endl;
        return 1;
    }

    std::cout << "[OK] 解码成功: " << in_path << std::endl;
    std::cout << "  帧数: " << frames.size() << std::endl;
    for (size_t i = 0; i < frames.size(); ++i) {
        const auto& f = frames[i];
        size_t n = f.cloud ? f.cloud->size() : 0;
        std::cout << "  帧[" << i << "]  时间戳: " << f.timestamp_sec << " s  点数: " << n << std::endl;
    }

    if (frames.size() == 1 && frames[0].cloud && !frames[0].cloud->empty()) {
        std::string out = out_path;
        if (out_path == "decoded.pcd" && name_as_difop) {
            const std::string difop_guess = guess_difop_from_msop_path(in_path);
            if (!difop_guess.empty() && fs::exists(difop_guess)) {
                out = fs::path(difop_guess).stem().string() + ".pcd";
            }
        } else if (out_path == "decoded.pcd") {
            out = fs::path(in_path).stem().string() + ".pcd";
        }
        if (!ends_with_ci(out, ".pcd")) {
            out += ".pcd";
        }
        if (pcl::io::savePCDFileBinary(out, *frames[0].cloud) == 0) {
            std::cout << "[OK] 已保存: " << out << " (" << frames[0].cloud->size() << " 点)" << std::endl;
        } else {
            std::cerr << "[WARN] 保存失败: " << out << std::endl;
            return 1;
        }
    } else if (frames.size() > 1) {
        fs::path op(out_path);
        fs::path stem_path = op;
        if (ends_with_ci(op.filename().string(), ".pcd")) {
            stem_path = op.parent_path() / op.stem();
        }
        if (name_as_difop) {
            const std::string difop_guess = guess_difop_from_msop_path(in_path);
            if (!difop_guess.empty() && fs::exists(difop_guess)) {
                stem_path = stem_path.parent_path() / fs::path(difop_guess).stem();
            }
        } else {
            stem_path = stem_path.parent_path() / fs::path(in_path).stem();
        }
        save_frames_default_merge(stem_path, frames, per_frame);
    }

    return 0;
}
