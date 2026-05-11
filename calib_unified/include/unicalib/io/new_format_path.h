#pragma once

#include <cstdlib>
#include <filesystem>
#include <string>

namespace ns_unicalib {

/**
 * 将 YAML 中的 new_format.root_dir 转为可用于加载的绝对路径：
 * - 已为绝对路径则原样返回（保留占位符形式便于日志对照时可仍传原始串给上层）；
 * - 相对路径与 CALIB_DATA_DIR 拼接；
 * - 兼容示例占位符 /path/to/foo → CALIB_DATA_DIR/foo（与 lidar_camera_extrin 一致）。
 */
inline std::string resolve_new_format_root_dir(const std::string& root_raw) {
    if (root_raw.empty())
        return root_raw;
    namespace fs = std::filesystem;
    if (fs::path(root_raw).is_absolute())
        return root_raw;

    std::string work = root_raw;
    const char prefix[] = "/path/to/";
    if (work.size() > sizeof(prefix) - 1 &&
        work.compare(0, sizeof(prefix) - 1, prefix) == 0)
        work = work.substr(sizeof(prefix) - 1);

    const char* base = std::getenv("CALIB_DATA_DIR");
    if (!base || !base[0])
        return root_raw;
    return (fs::path(base) / work).lexically_normal().string();
}

}  // namespace ns_unicalib
