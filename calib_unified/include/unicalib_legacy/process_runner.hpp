/**
 * UniCalib C++ — 外部进程调用（深度融合：调用 DM-Calib / learn-to-calibrate / 等）
 * 跨平台：Linux 使用 fork/exec + pipe；可扩展 Windows CreateProcess。
 */
#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <optional>
#include <cstdint>

namespace unicalib {

/** 进程执行结果 */
struct ProcessResult {
  int exit_code = -1;
  std::string stdout_output;
  std::string stderr_output;
  bool timed_out = false;
  std::string error_message;  // 若启动失败等
};

/**
 * 运行外部命令。
 * \param argv  首个为可执行路径，后续为参数（不含 argv[0] 也可，由 caller 传完整 argv）
 * \param env   可选，追加/覆盖环境变量（如 PYTHONPATH）
 * \param timeout_seconds  超时秒数，≤0 表示不限制
 * \param cwd   可选工作目录，空则继承当前
 */
ProcessResult run_process(
  const std::vector<std::string>& argv,
  const std::unordered_map<std::string, std::string>& env = {},
  int timeout_seconds = 0,
  const std::string& cwd = "");

/** 检查可执行路径是否存在（文件且可执行或可读） */
bool check_executable(const std::string& path);

/** 查找 Python3：优先 python3，其次 python */
std::string find_python3();

}  // namespace unicalib
