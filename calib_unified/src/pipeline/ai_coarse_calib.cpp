/**
 * UniCalib Unified — AI 粗标定适配器实现
 *
 * 核心实现思路:
 *   - 通过 popen/system 调用 Python 子进程
 *   - 在 work_dir 下创建临时输入/输出文件
 *   - 解析 YAML/JSON 输出提取标定结果
 *   - 完整日志记录每次调用的命令行、耗时、返回码
 *
 * 安全注意:
 *   - 所有命令行参数都需要转义 (此处使用 std::filesystem::path 规范化)
 *   - 超时通过 timeout 命令实现
 */

#include "unicalib/pipeline/ai_coarse_calib.h"
#include "unicalib/common/logger.h"
#include "unicalib/common/exception.h"
#include <opencv2/imgcodecs.hpp>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cstdio>
#include <atomic>
#include <chrono>
#include <thread>
#include <yaml-cpp/yaml.h>
#include <cstdlib>
#include <unistd.h>
#include <sys/wait.h>
#include <signal.h>
#include <fcntl.h>

namespace fs = std::filesystem;

namespace ns_unicalib {

// ---------------------------------------------------------------------------
// 工具: 执行命令并捕获输出
// ---------------------------------------------------------------------------
struct ExecResult {
    int exit_code;
    std::string stdout_str;
    double elapsed_ms;
};

using EnvMap = std::unordered_map<std::string, std::string>;

// 带环境变量的执行：子进程中先 setenv 再 execvp，用于 DM-Calib 直接调 python3 + PYTHONPATH
// 若 stream_stdout_to_log 为 true 且 log_prefix 非空，子进程每行 stdout/stderr 会实时打日志，便于观察进度
static ExecResult exec_cmd_safe_with_env(std::vector<std::string> argv,
                                          const EnvMap& env_extra,
                                          int timeout_sec,
                                          bool stream_stdout_to_log = false,
                                          const char* log_prefix = nullptr,
                                          const std::string& output_log_file = "") {
    ExecResult r{};
    if (argv.empty()) {
        r.exit_code = -1;
        r.stdout_str = "exec_cmd_safe: argv empty";
        return r;
    }
    auto t0 = std::chrono::steady_clock::now();
    const bool do_stream = stream_stdout_to_log && log_prefix && log_prefix[0] != '\0';
    std::string line_buf;

    int pipe_fd[2];
    if (pipe(pipe_fd) != 0) {
        r.exit_code = -1;
        r.stdout_str = "pipe() failed";
        return r;
    }
    pid_t pid = fork();
    if (pid < 0) {
        close(pipe_fd[0]);
        close(pipe_fd[1]);
        r.exit_code = -1;
        r.stdout_str = "fork() failed";
        return r;
    }
    if (pid == 0) {
        close(pipe_fd[0]);
        dup2(pipe_fd[1], STDOUT_FILENO);
        dup2(pipe_fd[1], STDERR_FILENO);
        close(pipe_fd[1]);
        for (const auto& kv : env_extra) {
            setenv(kv.first.c_str(), kv.second.c_str(), 1);
        }
        std::vector<char*> ptrs;
        for (auto& s : argv) ptrs.push_back(const_cast<char*>(s.c_str()));
        ptrs.push_back(nullptr);
        execvp(argv[0].c_str(), ptrs.data());
        _exit(127);
    }
    close(pipe_fd[1]);
    // 超时线程：每秒检查子进程是否仍存在，避免子进程早退时主线程被 join 阻塞满 timeout_sec
    std::thread timeout_thread([pid, timeout_sec]() {
        for (int i = 0; i < timeout_sec; ++i) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            if (kill(pid, 0) != 0) break;  // 进程已退出 (ESRCH)
        }
        if (kill(pid, 0) == 0)
            kill(pid, SIGKILL);
    });
    char buf[4096];
    ssize_t n;
    while ((n = read(pipe_fd[0], buf, sizeof(buf))) > 0) {
        r.stdout_str.append(buf, static_cast<size_t>(n));
        if (do_stream) {
            line_buf.append(buf, static_cast<size_t>(n));
            for (;;) {
                size_t pos = line_buf.find('\n');
                if (pos == std::string::npos) break;
                std::string line = line_buf.substr(0, pos);
                line_buf.erase(0, pos + 1);
                // 去掉行尾 \r
                while (!line.empty() && line.back() == '\r') line.pop_back();
                if (!line.empty())
                    UNICALIB_INFO("{}{}", log_prefix, line);
            }
        }
    }
    if (do_stream && !line_buf.empty()) {
        while (!line_buf.empty() && line_buf.back() == '\r') line_buf.pop_back();
        if (!line_buf.empty())
            UNICALIB_INFO("{}{}", log_prefix, line_buf);
    }
    close(pipe_fd[0]);
    int status = 0;
    waitpid(pid, &status, 0);
    timeout_thread.join();
    r.exit_code = WIFEXITED(status) ? WEXITSTATUS(status) : (WIFSIGNALED(status) ? 128 + WTERMSIG(status) : -1);
    auto t1 = std::chrono::steady_clock::now();
    r.elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    UNICALIB_DEBUG("[AI-Exec] 退出码={} 耗时={:.0f}ms", r.exit_code, r.elapsed_ms);
    
    // 保存完整输出到临时文件，便于深度诊断
    if (!output_log_file.empty() && !r.stdout_str.empty()) {
        try {
            std::ofstream out_log(output_log_file);
            if (out_log.is_open()) {
                out_log << r.stdout_str;
                out_log.close();
                UNICALIB_INFO("[AI-Exec] 子进程完整输出已保存到: {}", output_log_file);
            }
        } catch (const std::exception& e) {
            UNICALIB_WARN("[AI-Exec] 保存子进程输出失败: {}", e.what());
        }
    }
    
    return r;
}

// 安全执行: 无 shell，参数列表直接传 execvp，避免命令注入
static ExecResult exec_cmd_safe(std::vector<std::string> argv, int timeout_sec) {
    return exec_cmd_safe_with_env(std::move(argv), {}, timeout_sec);
}

static ExecResult exec_cmd(const std::string& cmd, int timeout_sec) {
    // 兼容旧调用: 将单字符串拆成 argv (仅空格分隔，路径含空格会失败)
    std::vector<std::string> argv;
    std::istringstream iss(cmd);
    std::string tok;
    while (iss >> tok) argv.push_back(tok);
    if (argv.empty()) {
        ExecResult r;
        r.exit_code = -1;
        r.stdout_str = "exec_cmd: empty command";
        return r;
    }
    return exec_cmd_safe(std::move(argv), timeout_sec);
}

// ---------------------------------------------------------------------------
// 工具: 检查 Python 模块是否可用（可选捕获 stderr 用于诊断）
// ---------------------------------------------------------------------------
static bool check_python_module(const std::string& python_exe,
                                  const std::string& module,
                                  std::string* out_stderr = nullptr) {
    std::string cmd = python_exe + " -c \"import " + module + "\" 2>&1";
    FILE* p = popen(cmd.c_str(), "r");
    if (!p) return false;
    std::string capture;
    char buf[512];
    while (fgets(buf, sizeof(buf), p)) capture.append(buf);
    int wstat = pclose(p);
    bool ok = (wstat == 0);
    if (out_stderr && !capture.empty()) *out_stderr = capture;
    return ok;
}

// ---------------------------------------------------------------------------
// 工具: 检测 DM-Calib 子进程输出是否为环境/依赖错误（NumPy 版本、import 等）
// 此类错误重试无意义，应快速失败并回退先验
// ---------------------------------------------------------------------------
static bool is_dmcalib_env_error(const std::string& output) {
    if (output.empty()) return false;
    const std::string lower = [&output]() {
        std::string s = output;
        for (auto& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        return s;
    }();
    // NumPy 1.x vs 2.x 不兼容
    if (lower.find("numpy 1.x") != std::string::npos ||
        (lower.find("numpy 2") != std::string::npos && lower.find("cannot be run") != std::string::npos) ||
        (lower.find("numpy") != std::string::npos && lower.find("cannot be run in") != std::string::npos) ||
        lower.find("downgrade to 'numpy<2'") != std::string::npos)
        return true;
    // 常见 import 错误
    if (lower.find("modulenotfounderror") != std::string::npos ||
        lower.find("importerror") != std::string::npos ||
        (lower.find("no module named") != std::string::npos))
        return true;
    // wrapper 未传参：python3 "" 导致 "can't find '__main__' module in '...'"
    if (lower.find("can't find") != std::string::npos && lower.find("__main__") != std::string::npos)
        return true;
    // diffusers 在无 Intel XPU 的 PyTorch 上会访问 torch.xpu，导致 AttributeError
    if (lower.find("has no attribute 'xpu'") != std::string::npos ||
        (lower.find("torch") != std::string::npos && lower.find("xpu") != std::string::npos && lower.find("attributeerror") != std::string::npos))
        return true;
    return false;
}

// ---------------------------------------------------------------------------
// 工具: 序列化 IMU 数据为 NumPy npy 格式 (通过 YAML 中间文件)
// ---------------------------------------------------------------------------
static std::string serialize_imu_to_yaml(
    const std::vector<IMUFrame>& data, const std::string& path) {

    std::ofstream f(path);
    if (!f.is_open()) {
        UNICALIB_THROW_DATA(ErrorCode::FILE_WRITE_ERROR, 
                           "无法写入 IMU 数据文件: " + path);
    }

    f << "num_samples: " << data.size() << "\n";
    f << "imu_data:\n";
    for (const auto& d : data) {
        f << "  - ts: " << std::fixed << std::setprecision(9) << d.timestamp << "\n";
        f << "    gyro: [" << d.gyro.x() << ", " << d.gyro.y() << ", " << d.gyro.z() << "]\n";
        f << "    accel: [" << d.accel.x() << ", " << d.accel.y() << ", " << d.accel.z() << "]\n";
    }
    return path;
}

// ===========================================================================
// DMCalibAdapter
// ===========================================================================

bool DMCalibAdapter::is_available() const {
    const std::string script_path = cfg_.repo_dir + "/" + cfg_.infer_script;
    if (cfg_.repo_dir.empty()) {
        UNICALIB_INFO("[DM-Calib] 可用性检查 失败 原因=repo_dir 未配置 (third_party.dm_calib / UNICALIB_DM_CALIB)");
        return false;
    }
    if (!fs::exists(cfg_.repo_dir)) {
        UNICALIB_INFO("[DM-Calib] 可用性检查 失败 原因=repo_dir 不存在 路径={}", cfg_.repo_dir);
        return false;
    }
    if (!fs::exists(script_path)) {
        UNICALIB_INFO("[DM-Calib] 可用性检查 失败 原因=推理脚本不存在 路径={}", script_path);
        return false;
    }
    if (cfg_.model_path.empty()) {
        UNICALIB_INFO("[DM-Calib] 可用性检查 失败 原因=model_path 未配置 (third_party.dm_calib_model / 或 repo/model)");
        return false;
    }
    if (!fs::exists(cfg_.model_path)) {
        UNICALIB_INFO("[DM-Calib] 可用性检查 失败 原因=模型目录不存在 路径={}", cfg_.model_path);
        return false;
    }
    // venv 路径（含 .venv 或 /venv/ 且为可执行 python）：必须在该环境中通过检查，不退回系统 python
    const bool is_venv_path = (cfg_.python_exe.find(".venv") != std::string::npos ||
                               (cfg_.python_exe.find("venv") != std::string::npos && cfg_.python_exe.find("bin") != std::string::npos));
    std::string torch_err;
    bool torch_ok = check_python_module(cfg_.python_exe, "torch", &torch_err);
    if (!torch_ok && !is_venv_path && cfg_.python_exe != "python3") {
        // 仅对 wrapper（如 NumPy 1.x 覆盖层）回退：若系统 python3 有 torch 仍视为可用，推理时用 wrapper 保证 NumPy 1.x
        torch_ok = check_python_module("python3", "torch", nullptr);
        if (torch_ok) {
            static bool once = false;
            if (!once) {
                UNICALIB_INFO("[DM-Calib] 专用 Python 下 torch 检查未通过，系统 python3 可 import torch，视为可用（推理将使用专用 Python）");
                once = true;
            }
        }
    }
    if (!torch_ok) {
        UNICALIB_INFO("[DM-Calib] 可用性检查 失败 原因=Python 未安装 torch (请 pip install torch{} )",
                      is_venv_path ? " 于当前 venv 内" : "");
        if (!torch_err.empty()) {
            std::string line;
            std::istringstream iss(torch_err);
            while (std::getline(iss, line) && line.size() < 400) {
                if (line.find("Error") != std::string::npos || line.find("error") != std::string::npos ||
                    line.find("ModuleNotFoundError") != std::string::npos || line.find("No module") != std::string::npos)
                    UNICALIB_INFO("[DM-Calib] 详情: {}", line);
            }
        }
        return false;
    }
    if (is_venv_path) {
        std::string numpy_err;
        if (!check_python_module(cfg_.python_exe, "numpy", &numpy_err)) {
            UNICALIB_INFO("[DM-Calib] 可用性检查 失败 原因=venv 内未安装 numpy (请于 venv 中: pip install numpy torch diffusers transformers)");
            return false;
        }
    }
    UNICALIB_INFO("[DM-Calib] 可用性检查 通过  repo={}  model={}  script={}",
                  cfg_.repo_dir, cfg_.model_path, cfg_.infer_script);
    return true;
}

std::string DMCalibAdapter::build_cmd(const std::string& input_dir,
                                       const std::string& output_dir) const {
    std::ostringstream oss;
    oss << cfg_.python_exe << " " << cfg_.repo_dir << "/" << cfg_.infer_script
        << " --pretrained_model_path " << cfg_.model_path
        << " --input_dir "  << input_dir
        << " --output_dir " << output_dir
        << " --domain " << cfg_.domain;
    if (cfg_.scale_10)       oss << " --scale_10";
    if (cfg_.domain_specify) oss << " --domain_specify";
    oss << " --seed 666";
    return oss.str();
}

// 解析 MIAS-LCEC 粗标定子进程使用的 Python：优先 UNICALIB_MIAS_LCEC_PYTHON，其次配置的 python_exe，
// 若已配置 model_path 且当前为默认 python3 则尝试 python3.10（LcMatch_CApi 需 3.10）
static std::string resolve_mias_python_exe(const std::string& configured_exe, bool has_model_path) {
    const char* env_py = std::getenv("UNICALIB_MIAS_LCEC_PYTHON");
    if (env_py && env_py[0]) return std::string(env_py);
    if (!configured_exe.empty() && configured_exe != "python3") return configured_exe;
    if (has_model_path && (configured_exe.empty() || configured_exe == "python3"))
        return "python3.10";
    return configured_exe.empty() ? "python3" : configured_exe;
}

// 判断是否为「包装脚本」路径：若为 wrapper 则直接调 python3 + PYTHONPATH，避免传参丢失导致 can't find __main__
static bool is_wrapper_path(const std::string& python_exe) {
    if (python_exe.empty()) return false;
    std::string lower = python_exe;
    for (auto& c : lower) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return (lower.find("wrapper") != std::string::npos && lower.find("dmcalib") != std::string::npos) ||
           (lower.size() >= 4 && lower.compare(lower.size() - 4, 4, ".sh") == 0);
}

// 构建 DM-Calib 调用的 argv 与 env；当 python_exe 为 wrapper 时改为 python3 + PYTHONPATH，彻底避免 __main__ 报错
// 始终设置 PYTHONPATH 包含 DMCalib 包根目录，避免子进程出现 No module named 'pipeline'
static void build_dmcalib_argv_env(const DMCalibAdapter::Config& cfg,
                                   const std::string& input_dir,
                                   const std::string& output_dir,
                                   std::vector<std::string>* out_argv,
                                   EnvMap* out_env,
                                   int max_images_override = 0) {
    std::string script_path = cfg.repo_dir + "/" + cfg.infer_script;
    std::string python_exe = cfg.python_exe;
    std::string dmcalib_lib = cfg.repo_dir + "/DMCalib";  // pipeline/ 所在目录
    const char* dm_lib = std::getenv("DMCALIB_LIB");
    const bool need_numpy1_overlay = (dm_lib && dm_lib[0]);
    if (is_wrapper_path(cfg.python_exe)) {
        python_exe = "python3";
        (*out_env)["PYTHONPATH"] = need_numpy1_overlay
            ? (std::string(dm_lib) + ":" + dmcalib_lib) : dmcalib_lib;
    } else {
        const char* existing = std::getenv("PYTHONPATH");
        std::string pp = dmcalib_lib;
        if (existing && existing[0]) pp = dmcalib_lib + ":" + existing;
        if (need_numpy1_overlay) pp = std::string(dm_lib) + ":" + pp;
        (*out_env)["PYTHONPATH"] = pp;
    }
    out_argv->clear();
    out_argv->push_back(python_exe);
    out_argv->push_back(script_path);
    out_argv->push_back("--pretrained_model_path");
    out_argv->push_back(cfg.model_path);
    out_argv->push_back("--input_dir");
    out_argv->push_back(input_dir);
    out_argv->push_back("--output_dir");
    out_argv->push_back(output_dir);
    out_argv->push_back("--domain");
    out_argv->push_back(cfg.domain);
    if (cfg.scale_10)       { out_argv->push_back("--scale_10"); }
    if (cfg.domain_specify) { out_argv->push_back("--domain_specify"); }
    if (cfg.use_cpu)        { out_argv->push_back("--cpu"); }
    if (cfg.denoise_steps > 0) {
        out_argv->push_back("--denoise_steps");
        out_argv->push_back(std::to_string(cfg.denoise_steps));
    }
    if (cfg.ensemble_size > 0) {
        out_argv->push_back("--ensemble_size");
        out_argv->push_back(std::to_string(cfg.ensemble_size));
    }
    if (cfg.processing_res > 0) {
        out_argv->push_back("--processing_res");
        out_argv->push_back(std::to_string(cfg.processing_res));
    }
    if (max_images_override > 0) {
        out_argv->push_back("--max_images");
        out_argv->push_back(std::to_string(max_images_override));
    }
    out_argv->push_back("--seed");
    out_argv->push_back("666");
}

DMCalibResult DMCalibAdapter::estimate(const std::string& image_path,
                                        const std::string& output_dir_arg) const {
    DMCalibResult result;
    result.model_name = "DM-Calib";
    result.success = false;

    if (!is_available()) {
        result.error_msg = "DM-Calib 不可用 (检查 repo_dir 和 torch 安装)";
        UNICALIB_WARN("[DM-Calib] {}", result.error_msg);
        return result;
    }

    try {
        fs::create_directories(cfg_.work_dir);
        std::string input_dir  = cfg_.work_dir + "/input";
        std::string output_dir = output_dir_arg.empty() ?
            cfg_.work_dir + "/output" : output_dir_arg;
        fs::create_directories(input_dir);
        fs::create_directories(output_dir);

        // 软链接或复制图像到 input_dir
        std::string link_path = input_dir + "/" + fs::path(image_path).filename().string();
        if (!fs::exists(link_path)) {
            fs::copy_file(image_path, link_path,
                          fs::copy_options::overwrite_existing);
        }

        // 获取图像尺寸（使用 OpenCV，避免依赖 Python/PIL 环境）
        int img_w = 0, img_h = 0;
        {
            cv::Mat img = cv::imread(image_path);
            if (!img.empty()) {
                img_w = img.cols;
                img_h = img.rows;
            }
        }
        if (img_w == 0 || img_h == 0) {
            result.error_msg = "无法读取图像尺寸: " + image_path +
                " (文件不存在或非有效图像，请检查路径与格式)";
            UNICALIB_WARN("[DM-Calib] 推理跳过 原因=无法读取图像尺寸  path={}", image_path);
            return result;
        }

        std::vector<std::string> argv;
        EnvMap env_extra;
        build_dmcalib_argv_env(cfg_, input_dir, output_dir, &argv, &env_extra);

        std::string cmd_for_log = build_cmd(input_dir, output_dir);
        UNICALIB_INFO("[DM-Calib] 推理开始  input={}  size={}x{}  timeout_s={}  output_dir={}",
                      fs::path(image_path).filename().string(), img_w, img_h, cfg_.timeout_sec, output_dir);
        // 精准定位：子进程实际使用的可执行文件、脚本路径、PYTHONPATH（便于确认修改是否生效）
        UNICALIB_INFO("[DM-Calib] 子进程 argv[0]={}  argv[1]={}  (共 {} 项)",
                      argv.empty() ? "" : argv[0], argv.size() > 1 ? argv[1] : "", argv.size());
        if (env_extra.count("PYTHONPATH")) {
            UNICALIB_INFO("[DM-Calib] 子进程 PYTHONPATH={}", env_extra.at("PYTHONPATH"));
        } else {
            UNICALIB_WARN("[DM-Calib] 子进程未设置 PYTHONPATH（将继承当前环境，可能导致 No module named 'pipeline'/'tools'）");
        }
        if (is_wrapper_path(cfg_.python_exe)) {
            UNICALIB_INFO("[DM-Calib] 已绕过 wrapper，直接调用 python3");
        }

        // 推理可能较慢：心跳日志每 10s 打一次，子进程 stdout 实时按行打出，便于观察进度
        std::atomic<bool> dm_done{false};
        std::thread heartbeat([&dm_done, timeout_sec = cfg_.timeout_sec]() {
            auto t0_hb = std::chrono::steady_clock::now();
            while (!dm_done) {
                std::this_thread::sleep_for(std::chrono::seconds(10));
                if (dm_done) break;
                auto elapsed_s = std::chrono::duration_cast<std::chrono::seconds>(
                    std::chrono::steady_clock::now() - t0_hb).count();
                UNICALIB_INFO("[DM-Calib] 推理进行中 已耗时 {}s (timeout {}s)", elapsed_s, timeout_sec);
            }
        });
        auto exec_r = exec_cmd_safe_with_env(std::move(argv), env_extra, cfg_.timeout_sec,
                                             true, "[DM-Calib] 子进程: ");
        dm_done = true;
        if (heartbeat.joinable()) heartbeat.join();
        result.elapsed_ms = exec_r.elapsed_ms;
        log_call(cmd_for_log, exec_r.exit_code, exec_r.elapsed_ms);

        if (exec_r.exit_code != 0) {
            result.error_msg = "DM-Calib 推理失败 (exit=" +
                                std::to_string(exec_r.exit_code) + ")";
            const bool env_err = is_dmcalib_env_error(exec_r.stdout_str);
            const bool main_module_err = (exec_r.stdout_str.find("__main__") != std::string::npos &&
                                         exec_r.stdout_str.find("can't find") != std::string::npos);
            if (env_err) {
                result.env_error = true;
                result.error_msg += " [环境错误: NumPy/依赖不兼容，重试无意义]";
            }
            UNICALIB_ERROR("[DM-Calib] 推理失败  exit_code={}  elapsed_ms={:.0f}  env_error={}",
                           exec_r.exit_code, result.elapsed_ms, env_err);
            UNICALIB_ERROR("[DM-Calib] 命令行: {}", cmd_for_log);
            char cwd_buf[1024];
            if (getcwd(cwd_buf, sizeof(cwd_buf)))
                UNICALIB_ERROR("[DM-Calib] 子进程工作目录: {} (本进程 cwd，子进程继承)", cwd_buf);
            if (exec_r.stdout_str.find("No module named") != std::string::npos)
                UNICALIB_ERROR("[DM-Calib] 定位: 见上方「子进程 PYTHONPATH」与子进程 stdout 中的「诊断 __file__/sys.path」；若未出现 PYTHONPATH 请重新编译");
            const size_t max_log = 2800;
            if (!exec_r.stdout_str.empty()) {
                std::string out = exec_r.stdout_str.length() <= max_log
                    ? exec_r.stdout_str
                    : exec_r.stdout_str.substr(0, max_log) + "\n... (truncated)";
                UNICALIB_ERROR("[DM-Calib] 子进程 stdout/stderr:\n{}", out);
            }
            if (main_module_err) {
                UNICALIB_ERROR("[DM-Calib] 检测到「can't find __main__」: 若仍出现请确认已用 build_dmcalib_argv_env 直接调 python3（本版应已绕过 wrapper）");
            }
            if (env_err && !main_module_err) {
                const std::string& out = exec_r.stdout_str;
                if (out.find("'xpu'") != std::string::npos && out.find("attribute") != std::string::npos)
                    UNICALIB_ERROR("[DM-Calib] 检测到 torch.xpu 错误：请更新 DM-Calib/DMCalib/tools/infer_unicalib.py 在 import diffusers 前添加 torch.xpu 兼容补丁，或使用带 Intel XPU 的 PyTorch");
                else if (out.find("diffusers") != std::string::npos)
                    UNICALIB_ERROR("[DM-Calib] 检测到缺 diffusers：请使用 DM-Calib/.venv（推荐）或 pip install diffusers transformers");
                else
                    UNICALIB_ERROR("[DM-Calib] 检测到环境错误，建议: 降级 numpy (pip install 'numpy<2') 或升级 PyTorch 以支持 NumPy 2.x");
            }
            return result;
        }

        // 解析输出
        std::string out_json = output_dir + "/intrinsics.json";
        if (!fs::exists(out_json)) {
            // 尝试 YAML 格式
            out_json = output_dir + "/intrinsics.yaml";
        }
        if (fs::exists(out_json)) {
            result.coarse_intrin = parse_output(out_json, img_w, img_h);
            result.success    = true;
            result.confidence = 0.75;
            UNICALIB_INFO("[DM-Calib] 推理成功  fx={:.1f} fy={:.1f} cx={:.1f} cy={:.1f}  elapsed_ms={:.0f}  output={}",
                          result.coarse_intrin.fx, result.coarse_intrin.fy,
                          result.coarse_intrin.cx, result.coarse_intrin.cy, result.elapsed_ms, out_json);
        } else {
            result.error_msg = "未找到输出文件: " + out_json;
            UNICALIB_WARN("[DM-Calib] 推理完成但无输出文件  exit_code=0  expected={}  output_dir={}",
                          out_json, output_dir);
        }
    } catch (const std::exception& e) {
        result.error_msg = std::string("DM-Calib 异常: ") + e.what();
        UNICALIB_ERROR("[DM-Calib] 异常  path={}  what={}", image_path, e.what());
    }
    return result;
}

DMCalibResult DMCalibAdapter::estimate_multi(
    const std::vector<std::string>& image_paths,
    const std::string& output_dir) const {

    UNICALIB_INFO("[DM-Calib] 批量推理开始  num_images={}  output_dir={} (单进程批处理)",
                  image_paths.size(), output_dir);

    if (image_paths.empty()) {
        DMCalibResult r;
        r.model_name = "DM-Calib";
        r.error_msg  = "无图像路径";
        return r;
    }

    if (!is_available()) {
        DMCalibResult r;
        r.model_name = "DM-Calib";
        r.error_msg  = "DM-Calib 不可用 (检查 repo_dir 和 torch 安装)";
        UNICALIB_WARN("[DM-Calib] {}", r.error_msg);
        return r;
    }

    try {
        fs::create_directories(cfg_.work_dir);
        std::string input_dir  = cfg_.work_dir + "/input";
        std::string out_dir    = output_dir.empty() ? (cfg_.work_dir + "/output") : output_dir;
        if (fs::exists(input_dir)) {
            for (const auto& e : fs::directory_iterator(input_dir))
                fs::remove_all(e.path());
        }
        fs::create_directories(input_dir);
        fs::create_directories(out_dir);

        for (size_t i = 0; i < image_paths.size(); ++i) {
            std::string ext = fs::path(image_paths[i]).extension().string();
            if (ext.empty()) ext = ".png";
            std::string name = "frame_" + std::to_string(i) + ext;
            fs::copy_file(image_paths[i], input_dir + "/" + name, fs::copy_options::overwrite_existing);
        }

        int img_w = 0, img_h = 0;
        {
            cv::Mat img = cv::imread(image_paths[0]);
            if (!img.empty()) { img_w = img.cols; img_h = img.rows; }
        }
        if (img_w == 0 || img_h == 0) {
            DMCalibResult r;
            r.model_name = "DM-Calib";
            r.error_msg  = "无法读取首图尺寸: " + image_paths[0];
            return r;
        }

        std::vector<std::string> argv;
        EnvMap env_extra;
        build_dmcalib_argv_env(cfg_, input_dir, out_dir, &argv, &env_extra,
                              static_cast<int>(image_paths.size()));

        std::atomic<bool> dm_done{false};
        std::thread heartbeat([&]() {
            int timeout_sec = cfg_.timeout_sec;
            auto t0 = std::chrono::steady_clock::now();
            while (!dm_done) {
                std::this_thread::sleep_for(std::chrono::seconds(10));
                if (dm_done) break;
                auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                    std::chrono::steady_clock::now() - t0).count();
                UNICALIB_INFO("[DM-Calib] 推理进行中 已耗时 {}s (timeout {}s)", elapsed, timeout_sec);
            }
        });
        auto exec_r = exec_cmd_safe_with_env(std::move(argv), env_extra, cfg_.timeout_sec,
                                             true, "[DM-Calib] 子进程: ");
        dm_done = true;
        if (heartbeat.joinable()) heartbeat.join();

        if (exec_r.exit_code != 0) {
            DMCalibResult r;
            r.model_name = "DM-Calib";
            r.error_msg  = "DM-Calib 推理失败 (exit=" + std::to_string(exec_r.exit_code) + ")";
            UNICALIB_ERROR("[DM-Calib] 批量推理失败  exit_code={}", exec_r.exit_code);
            return r;
        }

        std::string out_json = out_dir + "/intrinsics.json";
        if (!fs::exists(out_json)) out_json = out_dir + "/intrinsics.yaml";
        if (!fs::exists(out_json)) {
            DMCalibResult r;
            r.model_name = "DM-Calib";
            r.error_msg  = "未找到输出文件: " + out_dir + "/intrinsics.json";
            return r;
        }

        DMCalibResult final_r;
        final_r.model_name   = "DM-Calib";
        final_r.success     = true;
        final_r.confidence  = 0.8;
        final_r.coarse_intrin = parse_output(out_json, img_w, img_h);
        final_r.elapsed_ms  = exec_r.elapsed_ms;
        UNICALIB_INFO("[DM-Calib] 批量推理结束  fx={:.1f} fy={:.1f} cx={:.1f} cy={:.1f}  ({} 张一次完成, {:.0f}ms)",
                      final_r.coarse_intrin.fx, final_r.coarse_intrin.fy,
                      final_r.coarse_intrin.cx, final_r.coarse_intrin.cy,
                      image_paths.size(), final_r.elapsed_ms);
        return final_r;
    } catch (const std::exception& e) {
        DMCalibResult r;
        r.model_name = "DM-Calib";
        r.error_msg  = std::string("DM-Calib 异常: ") + e.what();
        UNICALIB_ERROR("[DM-Calib] 批量异常  what={}", e.what());
        return r;
    }
}

CameraIntrinsics DMCalibAdapter::parse_output(const std::string& path,
                                               int img_w, int img_h) const {
    CameraIntrinsics intrin;
    intrin.width  = img_w;
    intrin.height = img_h;
    intrin.model  = CameraIntrinsics::Model::PINHOLE;

    try {
        auto ends_with = [](const std::string& s, const std::string& suffix) {
            return s.size() >= suffix.size() &&
                   s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
        };
        if (ends_with(path, ".yaml") || ends_with(path, ".yml")) {
            YAML::Node node = YAML::LoadFile(path);
            intrin.fx = node["fx"] ? node["fx"].as<double>() : 0.0;
            intrin.fy = node["fy"] ? node["fy"].as<double>() : intrin.fx;
            intrin.cx = node["cx"] ? node["cx"].as<double>() : img_w * 0.5;
            intrin.cy = node["cy"] ? node["cy"].as<double>() : img_h * 0.5;
        } else {
            // 简单 JSON 解析
            std::ifstream f(path);
            std::string content((std::istreambuf_iterator<char>(f)),
                                 std::istreambuf_iterator<char>());
            auto find_val = [&](const std::string& key) -> double {
                size_t pos = content.find("\"" + key + "\"");
                if (pos == std::string::npos) return 0.0;
                pos = content.find(':', pos);
                if (pos == std::string::npos) return 0.0;
                return std::stod(content.substr(pos + 1));
            };
            intrin.fx = find_val("fx");
            intrin.fy = find_val("fy");
            intrin.cx = find_val("cx");
            intrin.cy = find_val("cy");
        }
    } catch (const std::exception& e) {
        UNICALIB_WARN("[DM-Calib] 解析输出异常  path={}  what={}", path, e.what());
    }

    if (intrin.cx == 0.0) intrin.cx = img_w * 0.5;
    if (intrin.cy == 0.0) intrin.cy = img_h * 0.5;
    if (intrin.fy == 0.0) intrin.fy = intrin.fx;

    return intrin;
}

void DMCalibAdapter::log_call(const std::string& cmd,
                               int exit_code, double elapsed_ms) const {
    UNICALIB_INFO("[DM-Calib] 调用完成  exit_code={}  elapsed_ms={:.0f}", exit_code, elapsed_ms);
    if (exit_code != 0) {
        UNICALIB_ERROR("[DM-Calib] 失败时完整命令: {}", cmd);
    }
}

// ===========================================================================
// MIASLCECAdapter
// ===========================================================================

bool MIASLCECAdapter::is_available() const {
    if (cfg_.repo_dir.empty()) {
        UNICALIB_INFO("[MIAS-LCEC] 不可用: repo_dir 未配置 (请设置 ai_root 或 third_party.mias_lcec.repo_dir)");
        return false;
    }
    if (!fs::exists(cfg_.repo_dir)) {
        UNICALIB_INFO("[MIAS-LCEC] 不可用: 仓库目录不存在 repo_dir={}", cfg_.repo_dir);
        return false;
    }
    std::string script = cfg_.repo_dir + "/" + cfg_.calib_script;
    if (!fs::exists(script)) {
        UNICALIB_INFO("[MIAS-LCEC] 不可用: 标定脚本不存在 script={}", script);
        UNICALIB_INFO("[MIAS-LCEC] 粗标定模型通常位于 {}/model/pretrained_overlap_transformer.pth.tar", cfg_.repo_dir);
        UNICALIB_INFO("[MIAS-LCEC] 若仓库使用其他入口（如 bin/python、mias_lcec.sh），请在 config 的 third_party.mias_lcec.calib_script 中指定正确相对路径");
        return false;
    }
    std::string model_dir = cfg_.repo_dir + "/model";
    if (!fs::exists(model_dir))
        UNICALIB_DEBUG("[MIAS-LCEC] 提示: model 目录不存在 model_dir={}", model_dir);
    return true;
}

std::string MIASLCECAdapter::write_sensor_config(
    const CameraIntrinsics& intrin,
    const std::string& work_dir) const {
    std::string cfg_path = work_dir + "/sensor_config.yaml";
    double cx = intrin.cx;
    double cy = intrin.cy;
    const double w = static_cast<double>(intrin.width);
    const double h = static_cast<double>(intrin.height);
    bool intrin_ok = (intrin.width > 0 && intrin.height > 0 && intrin.fx > 0 && intrin.fy > 0);
    if (intrin_ok && (cx < 0 || cy < 0 || cx > w || cy > h)) {
        cx = w * 0.5;
        cy = h * 0.5;
        UNICALIB_WARN("[MIAS-LCEC] 主点 cx={} cy={} 超出图像范围 [0,{}]x[0,{}]，已回退到图像中心 cx={} cy={}；请检查相机内参标定",
                      intrin.cx, intrin.cy, intrin.width, intrin.height, cx, cy);
    }
    if (!intrin_ok) {
        UNICALIB_WARN("[MIAS-LCEC] sensor_config 内参无效 (width={} height={} fx={} fy={})，Python 端将无法正确投影；请配置 camera_intrinsic_file 或从首帧推断",
                      intrin.width, intrin.height, intrin.fx, intrin.fy);
    }
    std::ofstream f(cfg_path);
    f << "camera:\n";
    f << "  model: " << (intrin.model == CameraIntrinsics::Model::FISHEYE ?
                          "fisheye" : "pinhole") << "\n";
    f << "  width: " << intrin.width << "\n";
    f << "  height: " << intrin.height << "\n";
    f << "  fx: " << intrin.fx << "\n";
    f << "  fy: " << intrin.fy << "\n";
    f << "  cx: " << cx << "\n";
    f << "  cy: " << cy << "\n";
    f << "  dist_coeffs: [";
    for (size_t i = 0; i < intrin.dist_coeffs.size(); ++i) {
        f << intrin.dist_coeffs[i];
        if (i + 1 < intrin.dist_coeffs.size()) f << ", ";
    }
    f << "]\n";
    f << "lidar:\n  type: " << cfg_.lidar_type << "\n";
    return cfg_path;
}

MIASLCECResult MIASLCECAdapter::estimate(
    const std::string& pcd_file,
    const std::string& image_file,
    const CameraIntrinsics& cam_intrin,
    const std::string& output_dir_arg) const {

    MIASLCECResult result;
    result.model_name = "MIAS-LCEC";

    if (!is_available()) {
        result.error_msg = "MIAS-LCEC 不可用 (请检查 repo_dir 与 " + cfg_.repo_dir + "/" + cfg_.calib_script + " 及 model/pretrained_overlap_transformer.pth.tar)";
        UNICALIB_WARN("[MIAS-LCEC] {}", result.error_msg);
        return result;
    }

    std::string model_abs_log = cfg_.model_path.empty() ? "(未配置)" : cfg_.model_path;
    if (!cfg_.model_path.empty() && !fs::path(cfg_.model_path).is_absolute() && !cfg_.repo_dir.empty())
        model_abs_log = cfg_.repo_dir + "/" + cfg_.model_path;
    UNICALIB_INFO("[MIAS-LCEC] 仓库: {} 脚本: {} 实际传入模型路径: {}",
                  cfg_.repo_dir, cfg_.calib_script, model_abs_log);

    try {
        fs::create_directories(cfg_.work_dir);
        std::string output_dir = output_dir_arg.empty() ?
            cfg_.work_dir + "/output" : output_dir_arg;
        fs::create_directories(output_dir);

        std::string sensor_cfg = write_sensor_config(cam_intrin, cfg_.work_dir);

        // MIAS-LCEC 根目录（用于 PYTHONPATH）：repo_dir 为 calib_unified 时脚本在此、真实 MIAS 根从 model_path 推导
        std::string mias_root = cfg_.repo_dir;
        auto path_ends_with = [](const std::string& s, const std::string& suffix) {
            if (s.size() < suffix.size()) return false;
            if (s.size() == suffix.size()) return s == suffix;
            return s[s.size() - suffix.size() - 1] == '/' && s.substr(s.size() - suffix.size()) == suffix;
        };
        const bool repo_is_calib_unified = mias_root.empty() || mias_root == "calib_unified" ||
                                          path_ends_with(mias_root, "calib_unified");
        if (repo_is_calib_unified && !cfg_.model_path.empty()) {
            // model_path 已由 pipeline 解析为绝对路径（如 ai_models_root/MIAS-LCEC/model/xxx.pth.tar）
            fs::path mp(cfg_.model_path);
            if (mp.has_parent_path()) {
                fs::path parent = mp.parent_path();
                if (parent.filename() == "model")
                    mias_root = parent.parent_path().string();
                else
                    mias_root = parent.string();
            }
        } else if (mias_root.empty() && !cfg_.model_path.empty()) {
            fs::path mp(cfg_.model_path);
            if (mp.has_parent_path()) mias_root = mp.parent_path().parent_path().string();
        }
        const std::string script_path = cfg_.repo_dir + "/" + cfg_.calib_script;
        const bool use_wrapper = is_wrapper_path(cfg_.python_exe);
        // 关键修复：wrapper 路径时应绕过，用 python3；否则正常解析
        // 避免 wrapper 被当作 python_exe 导致参数解析错误 ("can't find '__main__'")
        const std::string mias_python_exe = use_wrapper ? "python3" : resolve_mias_python_exe(cfg_.python_exe, !cfg_.model_path.empty());
        if (mias_python_exe != cfg_.python_exe)
            UNICALIB_INFO("[MIAS-LCEC] 子进程 Python: {} ({}绕过 wrapper)", mias_python_exe, use_wrapper ? "已" : "未");
        EnvMap env_extra;
        if (!mias_root.empty()) {
            std::string mias_bin_python = mias_root + "/bin/python";
            const char* existing = std::getenv("PYTHONPATH");
            std::string pp = mias_bin_python;
            if (existing && existing[0]) pp += ":" + std::string(existing);
            if (use_wrapper) {
                const char* dm_lib = std::getenv("DMCALIB_LIB");
                if (dm_lib && dm_lib[0]) pp = std::string(dm_lib) + ":" + pp;
            }
            env_extra["PYTHONPATH"] = pp;
        } else if (use_wrapper) {
            const char* dm_lib = std::getenv("DMCALIB_LIB");
            if (dm_lib && dm_lib[0]) {
                std::string pp = dm_lib;
                const char* existing = std::getenv("PYTHONPATH");
                if (existing && existing[0]) pp += ":" + std::string(existing);
                env_extra["PYTHONPATH"] = pp;
            }
        }

        ExecResult exec_r;
        std::string out_yaml = output_dir + "/extrinsic_result.yaml";
        const size_t max_cmd_log = 512;
        bool retried_pnp = false;
        std::string log_file;  // 临时日志文件路径（循环外可访问）

        for (int attempt = 0; attempt < 2; ++attempt) {
            const bool add_use_pnp = (attempt == 1);
            if (add_use_pnp) {
                UNICALIB_WARN("[MIAS-LCEC] 粗标定结果为 identity，重试  attempt=2/2  显式附加 --use_pnp");
                retried_pnp = true;
            }
            UNICALIB_INFO("[MIAS-LCEC] 粗标定尝试 {}/2  ({}), 超时={}s",
                          attempt + 1, attempt == 0 ? "首次" : "identity 后重试", cfg_.timeout_sec);

            if (use_wrapper) {
                std::vector<std::string> argv;
                argv.push_back(mias_python_exe);
                argv.push_back(script_path);
                argv.push_back("--pcd");
                argv.push_back(pcd_file);
                argv.push_back("--image");
                argv.push_back(image_file);
                argv.push_back("--sensor_config");
                argv.push_back(sensor_cfg);
                argv.push_back("--output_dir");
                argv.push_back(output_dir);
                // C3M iterative refine / quality thresholds (YAML → C++ → CLI → Python)
                if (cfg_.c3m_use_iterative_refine) {
                    argv.push_back("--c3m-use-iterative-refine");
                    argv.push_back("true");
                }
                if (cfg_.c3m_iter_max > 0) {
                    argv.push_back("--c3m-iter-max");
                    argv.push_back(std::to_string(cfg_.c3m_iter_max));
                }
                if (cfg_.c3m_iter_thresh > 0.0) {
                    argv.push_back("--c3m-iter-thresh");
                    argv.push_back(std::to_string(cfg_.c3m_iter_thresh));
                }
                if (cfg_.c3m_similarity_threshold > 0.0) {
                    argv.push_back("--c3m-similarity-threshold");
                    argv.push_back(std::to_string(cfg_.c3m_similarity_threshold));
                }
                if (cfg_.quality_ncc_good > 0.0) {
                    argv.push_back("--quality-ncc-good");
                    argv.push_back(std::to_string(cfg_.quality_ncc_good));
                }
                if (cfg_.quality_ncc_acceptable > 0.0) {
                    argv.push_back("--quality-ncc-acceptable");
                    argv.push_back(std::to_string(cfg_.quality_ncc_acceptable));
                }
                if (cfg_.quality_rms_good_px > 0.0) {
                    argv.push_back("--quality-rms-good-px");
                    argv.push_back(std::to_string(cfg_.quality_rms_good_px));
                }
                if (cfg_.quality_rms_acceptable_px > 0.0) {
                    argv.push_back("--quality-rms-acceptable-px");
                    argv.push_back(std::to_string(cfg_.quality_rms_acceptable_px));
                }
                if (cfg_.quality_inlier_ratio_good > 0.0) {
                    argv.push_back("--quality-inlier-ratio-good");
                    argv.push_back(std::to_string(cfg_.quality_inlier_ratio_good));
                }
                if (cfg_.quality_inlier_ratio_acceptable > 0.0) {
                    argv.push_back("--quality-inlier-ratio-acceptable");
                    argv.push_back(std::to_string(cfg_.quality_inlier_ratio_acceptable));
                }
                if (!cfg_.model_path.empty()) {
                    std::string model_abs = cfg_.model_path;
                    if (!fs::path(model_abs).is_absolute() && !cfg_.repo_dir.empty())
                        model_abs = cfg_.repo_dir + "/" + cfg_.model_path;
                    argv.push_back("--model_path");
                    argv.push_back(model_abs);
                    argv.push_back("--mias_repo");
                    argv.push_back(cfg_.repo_dir);
                }
                if (cfg_.allow_pnp_fallback)
                    argv.push_back("--no-require-model");
                if (add_use_pnp)
                    argv.push_back("--use_pnp");
                if (argv.size() * 2u < max_cmd_log) {
                    std::string log_cmd;
                    for (const auto& a : argv) log_cmd += (log_cmd.empty() ? "" : " ") + a;
                    UNICALIB_INFO("[MIAS-LCEC] 执行命令 (python3+PYTHONPATH): {}", log_cmd);
                } else
                    UNICALIB_INFO("[MIAS-LCEC] 执行命令 (python3+PYTHONPATH): {} ... ({} 个参数)", script_path, argv.size());
                // 生成临时日志文件路径
                std::string log_file = cfg_.work_dir + "/subprocess_" + 
                    std::to_string(std::chrono::system_clock::to_time_t(std::chrono::system_clock::now())) + ".log";
                exec_r = exec_cmd_safe_with_env(std::move(argv), env_extra, cfg_.timeout_sec,
                                             true, "[MIAS-LCEC] 子进程: ", log_file);
            } else {
                std::vector<std::string> argv;
                argv.push_back(mias_python_exe);
                argv.push_back(script_path);
                argv.push_back("--pcd"); argv.push_back(pcd_file);
                argv.push_back("--image"); argv.push_back(image_file);
                argv.push_back("--sensor_config"); argv.push_back(sensor_cfg);
                argv.push_back("--output_dir"); argv.push_back(output_dir);
                // C3M iterative refine / quality thresholds (YAML → C++ → CLI → Python)
                if (cfg_.c3m_use_iterative_refine) {
                    argv.push_back("--c3m-use-iterative-refine");
                    argv.push_back("true");
                }
                if (cfg_.c3m_iter_max > 0) {
                    argv.push_back("--c3m-iter-max");
                    argv.push_back(std::to_string(cfg_.c3m_iter_max));
                }
                if (cfg_.c3m_iter_thresh > 0.0) {
                    argv.push_back("--c3m-iter-thresh");
                    argv.push_back(std::to_string(cfg_.c3m_iter_thresh));
                }
                if (cfg_.c3m_similarity_threshold > 0.0) {
                    argv.push_back("--c3m-similarity-threshold");
                    argv.push_back(std::to_string(cfg_.c3m_similarity_threshold));
                }
                if (cfg_.quality_ncc_good > 0.0) {
                    argv.push_back("--quality-ncc-good");
                    argv.push_back(std::to_string(cfg_.quality_ncc_good));
                }
                if (cfg_.quality_ncc_acceptable > 0.0) {
                    argv.push_back("--quality-ncc-acceptable");
                    argv.push_back(std::to_string(cfg_.quality_ncc_acceptable));
                }
                if (cfg_.quality_rms_good_px > 0.0) {
                    argv.push_back("--quality-rms-good-px");
                    argv.push_back(std::to_string(cfg_.quality_rms_good_px));
                }
                if (cfg_.quality_rms_acceptable_px > 0.0) {
                    argv.push_back("--quality-rms-acceptable-px");
                    argv.push_back(std::to_string(cfg_.quality_rms_acceptable_px));
                }
                if (cfg_.quality_inlier_ratio_good > 0.0) {
                    argv.push_back("--quality-inlier-ratio-good");
                    argv.push_back(std::to_string(cfg_.quality_inlier_ratio_good));
                }
                if (cfg_.quality_inlier_ratio_acceptable > 0.0) {
                    argv.push_back("--quality-inlier-ratio-acceptable");
                    argv.push_back(std::to_string(cfg_.quality_inlier_ratio_acceptable));
                }
                if (!cfg_.model_path.empty()) {
                    std::string model_abs = cfg_.model_path;
                    if (!fs::path(model_abs).is_absolute() && !cfg_.repo_dir.empty())
                        model_abs = cfg_.repo_dir + "/" + cfg_.model_path;
                    argv.push_back("--model_path"); argv.push_back(model_abs);
                    argv.push_back("--mias_repo"); argv.push_back(cfg_.repo_dir);
                }
                if (cfg_.allow_pnp_fallback)
                    argv.push_back("--no-require-model");
                if (add_use_pnp)
                    argv.push_back("--use_pnp");
                if (argv.size() * 2u < max_cmd_log) {
                    std::string log_cmd;
                    for (const auto& a : argv) log_cmd += (log_cmd.empty() ? "" : " ") + a;
                    UNICALIB_INFO("[MIAS-LCEC] 执行命令 (PYTHONPATH={}): {}", env_extra.count("PYTHONPATH") ? "已设置" : "未设置", log_cmd);
                } else
                    UNICALIB_INFO("[MIAS-LCEC] 执行命令 (PYTHONPATH={}): {} ... ({} 个参数)", env_extra.count("PYTHONPATH") ? "已设置" : "未设置", script_path, argv.size());
                // 生成临时日志文件路径
                std::string log_file = cfg_.work_dir + "/subprocess_" + 
                    std::to_string(std::chrono::system_clock::to_time_t(std::chrono::system_clock::now())) + ".log";
                exec_r = exec_cmd_safe_with_env(std::move(argv), env_extra, cfg_.timeout_sec,
                                             true, "[MIAS-LCEC] 子进程: ", log_file);
            }
            result.elapsed_ms = exec_r.elapsed_ms;
            UNICALIB_INFO("[MIAS-LCEC] 子进程结束  exit_code={}  elapsed_ms={:.0f}  output_exists={}",
                          exec_r.exit_code, exec_r.elapsed_ms, fs::exists(out_yaml) ? "yes" : "no");
            if (!log_file.empty()) {
                UNICALIB_INFO("[MIAS-LCEC] 子进程完整日志: {}", log_file);
            }
            if (exec_r.exit_code == 0 && !exec_r.stdout_str.empty()) {
                const std::string& out = exec_r.stdout_str;
                auto pos = out.find("method=");
                if (pos != std::string::npos) {
                    pos += 7;
                    auto end = out.find_first_of(" \n\r\t", pos);
                    std::string method = (end != std::string::npos) ? out.substr(pos, end - pos) : out.substr(pos);
                    UNICALIB_INFO("[MIAS-LCEC] 粗标定方法: {} (由子进程 stdout 解析)", method);
                    if (method != "mias_lcec") {
                        UNICALIB_WARN("[MIAS-LCEC] ★ 粗标定未使用深度模型，结果来自几何回退({})；原因见上方子进程日志或 {}", method, log_file.empty() ? "子进程完整输出" : log_file);
                        // 从子进程输出解析原因摘要，便于主日志一眼定位
                        auto reason_pos = out.find("COARSE_REASON=");
                        if (reason_pos != std::string::npos) {
                            reason_pos += 14;
                            auto reason_end = out.find_first_of("\n\r", reason_pos);
                            std::string reason = (reason_end != std::string::npos)
                                ? out.substr(reason_pos, reason_end - reason_pos) : out.substr(reason_pos);
                            if (reason.size() > 200) reason = reason.substr(0, 200) + "...";
                            UNICALIB_WARN("[MIAS-LCEC] 粗标定诊断 — 未使用深度模型原因: {}", reason);
                        } else {
                            auto summary_pos = out.find("原因摘要");
                            if (summary_pos != std::string::npos) {
                                summary_pos = out.find(":", summary_pos);
                                if (summary_pos != std::string::npos) {
                                    ++summary_pos;
                                    while (summary_pos < out.size() && (out[summary_pos] == ' ' || out[summary_pos] == '\t'))
                                        ++summary_pos;
                                    auto summary_end = out.find_first_of("\n\r", summary_pos);
                                    std::string summary = (summary_end != std::string::npos)
                                        ? out.substr(summary_pos, summary_end - summary_pos) : out.substr(summary_pos);
                                    if (summary.size() > 200) summary = summary.substr(0, 200) + "...";
                                    UNICALIB_WARN("[MIAS-LCEC] 粗标定诊断 — 未使用深度模型原因: {}", summary);
                                }
                            }
                        }
                    } else {
                        UNICALIB_INFO("[MIAS-LCEC] ★ 粗标定已使用深度模型 (MIAS-LCEC)");
                    }
                }
            }

            if (exec_r.exit_code != 0) {
                result.error_msg = "MIAS-LCEC 失败 (exit=" +
                                    std::to_string(exec_r.exit_code) + ")";
                UNICALIB_ERROR("[MIAS-LCEC] {}", result.error_msg);
                UNICALIB_WARN("[MIAS-LCEC] 可能原因: 1) sensor_config 内参为 0 → 配置 camera_intrinsic_file 或确保 C++ 从首帧推断 2) PCD/图像读取失败 3) Python 环境 NumPy/OpenCV 版本不兼容(见下方) 4) 见下方子进程完整输出");
                const size_t max_out = 5000;
                if (!exec_r.stdout_str.empty()) {
                    std::string snippet = exec_r.stdout_str.size() <= max_out
                        ? exec_r.stdout_str
                        : exec_r.stdout_str.substr(0, max_out) + "\n... (truncated, 共 " + std::to_string(exec_r.stdout_str.size()) + " 字符)";
                    UNICALIB_ERROR("[MIAS-LCEC] 子进程 stdout/stderr (完整):\n{}", snippet);
                    const std::string& out = exec_r.stdout_str;
                    if (out.find("NumPy 1.x") != std::string::npos || out.find("numpy<2") != std::string::npos ||
                        out.find("cannot be run in") != std::string::npos || out.find("NumPy 2.") != std::string::npos) {
                        UNICALIB_WARN("[MIAS-LCEC] 检测到 NumPy 版本冲突: 当前环境为 NumPy 2.x，子进程依赖（如 opencv）可能为 NumPy 1.x 编译。建议: 1) 在子进程环境中安装 numpy<2 或 2) 升级 opencv-python 到支持 NumPy 2 的版本 3) 粗标定脚本已支持优先 Pillow 读图、仅 identity 初值时不依赖 cv2，请确保已更新脚本并安装 Pillow");
                    }
                }
                return result;
            }

            if (!fs::exists(out_yaml)) {
                UNICALIB_WARN("[MIAS-LCEC] 本轮无输出文件  out_yaml={}", out_yaml);
                break;
            }
            result.coarse_extrin = parse_output(out_yaml);
            double tx = result.coarse_extrin.POS_TargetInRef.x();
            double ty = result.coarse_extrin.POS_TargetInRef.y();
            double tz = result.coarse_extrin.POS_TargetInRef.z();
            Eigen::Matrix3d R = result.coarse_extrin.SO3_TargetInRef.matrix();
            bool is_identity = (std::abs(tx) < 1e-9 && std::abs(ty) < 1e-9 && std::abs(tz) < 1e-9 &&
                               (R - Eigen::Matrix3d::Identity()).norm() < 1e-9);
            UNICALIB_INFO("[MIAS-LCEC] 解析结果  t=[{:.4f}, {:.4f}, {:.4f}]  is_identity={}",
                          tx, ty, tz, is_identity ? "true" : "false");
            if (!is_identity) {
                UNICALIB_INFO("[MIAS-LCEC] 得到非 identity 初值，停止重试");
                break;
            }
            if (attempt == 1) {
                UNICALIB_INFO("[MIAS-LCEC] 第 2 次尝试仍为 identity，使用该结果");
                break;
            }
        }

        if (fs::exists(out_yaml)) {
            result.coarse_extrin = parse_output(out_yaml);
            result.success    = true;
            result.confidence = 0.7;
            double tx = result.coarse_extrin.POS_TargetInRef.x();
            double ty = result.coarse_extrin.POS_TargetInRef.y();
            double tz = result.coarse_extrin.POS_TargetInRef.z();
            Eigen::Matrix3d R = result.coarse_extrin.SO3_TargetInRef.matrix();
            bool is_identity = (std::abs(tx) < 1e-9 && std::abs(ty) < 1e-9 && std::abs(tz) < 1e-9 &&
                               (R - Eigen::Matrix3d::Identity()).norm() < 1e-9);
            UNICALIB_INFO("[MIAS-LCEC] 粗估外参 最终: t=[{:.4f}, {:.4f}, {:.4f}]m  is_identity={}  output={}{}",
                          tx, ty, tz, is_identity ? "true" : "false", out_yaml, retried_pnp ? " (重试后)" : "");
            if (!exec_r.stdout_str.empty()) {
                const std::string& out = exec_r.stdout_str;
                auto pos = out.find("method=");
                if (pos != std::string::npos) {
                    pos += 7;
                    auto end = out.find_first_of(" \n\r\t", pos);
                    std::string method = (end != std::string::npos) ? out.substr(pos, end - pos) : out.substr(pos);
                    UNICALIB_INFO("[MIAS-LCEC] 粗标定结果来源: {} (mias_lcec=深度模型, pnp=几何回退, identity=恒等)", method);
                }
            }
            if (is_identity) {
                UNICALIB_WARN("[MIAS-LCEC] 粗标定输出为 identity；精标定可能无法收敛。建议: 1) 查看上方 [PnP] 日志确认网格/回退是否成功 2) 配置 model_path 尝试深度模型 3) 检查内参与数据");
            }
        } else {
            result.error_msg = "未找到输出: " + out_yaml;
            UNICALIB_WARN("[MIAS-LCEC] {}", result.error_msg);
            if (!exec_r.stdout_str.empty()) {
                size_t max_out = 600;
                UNICALIB_DEBUG("[MIAS-LCEC] 子进程输出 (前 {} 字符): {}",
                               std::min(exec_r.stdout_str.size(), max_out),
                               exec_r.stdout_str.substr(0, max_out));
            }
        }
    } catch (const std::exception& e) {
        result.error_msg = std::string("MIAS-LCEC 异常: ") + e.what();
        UNICALIB_ERROR("[MIAS-LCEC] {}", result.error_msg);
    }
    return result;
}

ExtrinsicSE3 MIASLCECAdapter::parse_output(const std::string& path) const {
    ExtrinsicSE3 extrin;
    try {
        YAML::Node node = YAML::LoadFile(path);
        if (node["rotation_matrix"]) {
            Eigen::Matrix3d R;
            auto rm = node["rotation_matrix"];
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    R(i, j) = rm[i][j].as<double>();
            extrin.SO3_TargetInRef = Sophus::SO3d(R);
        }
        if (node["translation"]) {
            auto t = node["translation"];
            extrin.POS_TargetInRef = Eigen::Vector3d(
                t[0].as<double>(), t[1].as<double>(), t[2].as<double>());
        }
        if (node["lidar_id"])  extrin.ref_sensor_id    = node["lidar_id"].as<std::string>();
        if (node["camera_id"]) extrin.target_sensor_id = node["camera_id"].as<std::string>();
    } catch (const std::exception& e) {
        UNICALIB_WARN("[MIAS-LCEC] 解析输出异常: {}", e.what());
    }
    return extrin;
}

// ===========================================================================
// TransformerIMUAdapter
// ===========================================================================

bool TransformerIMUAdapter::is_available() const {
    if (cfg_.repo_dir.empty()) {
        UNICALIB_INFO("[TransformerIMU] 不可用: 未配置 third_party.transformer_imu 且环境变量 UNICALIB_TRANSFORMER_IMU 未设置");
        return false;
    }
    if (!fs::exists(cfg_.repo_dir)) {
        UNICALIB_INFO("[TransformerIMU] 不可用: 仓库目录不存在 repo_dir={}", cfg_.repo_dir);
        return false;
    }
    std::string script = cfg_.repo_dir + "/" + cfg_.eval_script;
    if (!fs::exists(script)) {
        UNICALIB_INFO("[TransformerIMU] 不可用: 脚本不存在 script={}", script);
        return false;
    }
    std::string model_path = cfg_.repo_dir + "/" + cfg_.model_weights;
    if (!fs::exists(model_path)) {
        UNICALIB_INFO("[TransformerIMU] 不可用: 模型文件不存在 model_path={}", model_path);
        return false;
    }
    if (!check_python_module(cfg_.python_exe, "torch")) {
        UNICALIB_INFO("[TransformerIMU] 不可用: Python 无法 import torch (python_exe={})", cfg_.python_exe);
        return false;
    }
    return true;
}

TransformerIMUResult TransformerIMUAdapter::estimate_native(
    const std::vector<IMUFrame>& imu_data) {

    TransformerIMUResult result;
    result.model_name = "Transformer-IMU-Calibrator(native)";

    if (imu_data.size() < 100) {
        result.error_msg = "IMU 帧数不足 (need ≥100)";
        return result;
    }

    Eigen::Vector3d sum_gyro = Eigen::Vector3d::Zero();
    Eigen::Vector3d sum_accel = Eigen::Vector3d::Zero();
    for (const auto& f : imu_data) {
        sum_gyro += f.gyro;
        sum_accel += f.accel;
    }
    double n = static_cast<double>(imu_data.size());
    result.coarse_intrin.bias_gyro = sum_gyro / n;
    Eigen::Vector3d mean_accel = sum_accel / n;
    double mean_norm = mean_accel.norm();
    Eigen::Vector3d g = Eigen::Vector3d(0., -9.81, 0.);
    if (mean_norm > 1e-6) {
        // 重力自检：g = (mean_accel/‖mean_accel‖)*9.81，与 eval_unicalib.py --gravity-auto 一致，避免 ‖ba‖≈13.9
        g = mean_accel.normalized() * 9.81;
        UNICALIB_INFO("[TransformerIMU] 原生零偏: 重力自检 g=({:.4f},{:.4f},{:.4f})", g.x(), g.y(), g.z());
    }
    result.coarse_intrin.bias_acce = mean_accel - g;
    result.success = true;
    result.confidence = 0.5;
    double ba_norm = result.coarse_intrin.bias_acce.norm();
    UNICALIB_INFO("[TransformerIMU] 使用 C++ 原生零偏估计 (无 Python/无模型推理) mean_accel‖={:.4f} ‖ba‖={:.4f} m/s²",
                  mean_norm, ba_norm);
    return result;
}

TransformerIMUResult TransformerIMUAdapter::estimate(
    const std::vector<IMUFrame>& imu_data,
    const std::string& output_dir_arg) const {

    TransformerIMUResult result;
    result.model_name = "Transformer-IMU-Calibrator";

    if (is_available()) {
        UNICALIB_INFO("[TransformerIMU] 从 {} 帧 IMU 数据估计内参 (Python 脚本)", imu_data.size());
        try {
            fs::create_directories(cfg_.work_dir);
            std::string output_dir = output_dir_arg.empty() ?
                cfg_.work_dir + "/output" : output_dir_arg;
            fs::create_directories(output_dir);

            std::string imu_yaml = cfg_.work_dir + "/imu_data.yaml";
            serialize_imu_to_yaml(imu_data, imu_yaml);

            std::ostringstream cmd;
            cmd << cfg_.python_exe
                << " " << cfg_.repo_dir << "/" << cfg_.eval_script
                << " --imu_data " << imu_yaml
                << " --weights " << cfg_.repo_dir << "/" << cfg_.model_weights
                << " --output_dir " << output_dir;
            if (cfg_.use_mean_only)
                cmd << " --mean-only";

            auto exec_r = exec_cmd(cmd.str(), cfg_.timeout_sec);
            result.elapsed_ms = exec_r.elapsed_ms;

            if (exec_r.exit_code == 0) {
                std::string out_json = output_dir + "/imu_intrinsics.json";
                if (fs::exists(out_json)) {
                    result.coarse_intrin = parse_output(out_json);
                    result.success    = true;
                    result.confidence = 0.6;
                    double ba_norm = result.coarse_intrin.bias_acce.norm();
                    UNICALIB_INFO("[TransformerIMU] 零偏来源: Python 脚本 ({} 模式)",
                                  cfg_.use_mean_only ? "mean-only" : "TIC 模型推理");
                    UNICALIB_INFO("[TransformerIMU] 粗估 bias_gyro=[{:.6f}, {:.6f}, {:.6f}]",
                                  result.coarse_intrin.bias_gyro.x(),
                                  result.coarse_intrin.bias_gyro.y(),
                                  result.coarse_intrin.bias_gyro.z());
                    UNICALIB_INFO("[TransformerIMU] 粗估 bias_acce=[{:.6f}, {:.6f}, {:.6f}] ‖ba‖={:.4f} m/s²",
                                  result.coarse_intrin.bias_acce.x(),
                                  result.coarse_intrin.bias_acce.y(),
                                  result.coarse_intrin.bias_acce.z(), ba_norm);
                    if (ba_norm > 15.0) {
                        UNICALIB_WARN("[TransformerIMU] ‖ba‖={:.2f} 远大于 g，可能为重力方向/坐标系假设不一致，建议配置 use_mean_only+gravity_auto 或补采六面法", ba_norm);
                    }
                    return result;
                }
            }
            result.error_msg = "Python 脚本失败或未产出 JSON，回退到 C++ 原生估计";
            UNICALIB_WARN("[TransformerIMU] {}", result.error_msg);
        } catch (const std::exception& e) {
            result.error_msg = std::string("TransformerIMU 异常: ") + e.what();
            UNICALIB_WARN("[TransformerIMU] {} 回退到 C++ 原生估计", result.error_msg);
        }
    } else {
        result.error_msg = "Transformer-IMU 不可用，使用 C++ 原生零偏估计（上方已打印具体原因）";
        UNICALIB_INFO("[TransformerIMU] {}", result.error_msg);
    }

    return estimate_native(imu_data);
}

IMUIntrinsics TransformerIMUAdapter::parse_output(const std::string& path) const {
    IMUIntrinsics intrin;
    try {
        std::ifstream f(path);
        std::string content((std::istreambuf_iterator<char>(f)),
                             std::istreambuf_iterator<char>());
        // 简单解析 JSON
        auto find_arr3 = [&](const std::string& key) -> Eigen::Vector3d {
            size_t pos = content.find("\"" + key + "\"");
            if (pos == std::string::npos) return Eigen::Vector3d::Zero();
            size_t lb = content.find('[', pos);
            size_t rb = content.find(']', lb);
            if (lb == std::string::npos || rb == std::string::npos)
                return Eigen::Vector3d::Zero();
            std::string arr_str = content.substr(lb+1, rb-lb-1);
            std::istringstream iss(arr_str);
            double x, y, z; char comma;
            iss >> x >> comma >> y >> comma >> z;
            return Eigen::Vector3d(x, y, z);
        };
        intrin.bias_gyro  = find_arr3("bias_gyro");
        intrin.bias_acce  = find_arr3("bias_acce");
    } catch (const std::exception& e) {
        UNICALIB_WARN("[TransformerIMU] 解析异常: {}", e.what());
    }
    return intrin;
}

std::string TransformerIMUAdapter::serialize_imu_data(
    const std::vector<IMUFrame>& data, const std::string& path) const {
    return serialize_imu_to_yaml(data, path);
}

// ===========================================================================
// L2CalibAdapter
// ===========================================================================

bool L2CalibAdapter::is_available() const {
    if (!fs::exists(cfg_.repo_dir)) return false;
    std::string script = cfg_.repo_dir + "/" + cfg_.train_script;
    return fs::exists(script) && check_python_module(cfg_.python_exe, "torch");
}

L2CalibResult L2CalibAdapter::estimate_from_bag(
    const std::string& bag_path,
    const std::string& output_dir_arg) const {

    L2CalibResult result;
    result.model_name = "L2Calib";

    if (!is_available()) {
        result.error_msg = "L2Calib 不可用";
        UNICALIB_WARN("[L2Calib] {}", result.error_msg);
        return result;
    }

    if (!fs::exists(bag_path)) {
        result.error_msg = "Bag 文件不存在: " + bag_path;
        UNICALIB_WARN("[L2Calib] {}", result.error_msg);
        return result;
    }

    UNICALIB_INFO("[L2Calib] RL强化学习 IMU-LiDAR 外参粗估");
    UNICALIB_INFO("[L2Calib] bag: {} epochs: {}", bag_path, cfg_.num_epochs);

    try {
        fs::create_directories(cfg_.work_dir);
        std::string output_dir = output_dir_arg.empty() ?
            cfg_.work_dir + "/output" : output_dir_arg;
        fs::create_directories(output_dir);

        // 写 FastLIO 配置 (如果没有提供)
        std::string fastlio_cfg = cfg_.fastlio_config;
        if (fastlio_cfg.empty()) {
            fastlio_cfg = cfg_.work_dir + "/fastlio_config.yaml";
            std::ofstream f(fastlio_cfg);
            f << "common:\n  imu_topic: " << cfg_.imu_topic << "\n";
        }

        std::ostringstream cmd;
        cmd << "cd " << cfg_.repo_dir << " && "
            << "export PYTHONPATH=" << cfg_.repo_dir << "/rl_solver:$PYTHONPATH && "
            << cfg_.python_exe << " " << cfg_.train_script
            << " --lio-config " << fastlio_cfg
            << " --bag-dir " << fs::path(bag_path).parent_path().string()
            << " --alg " << cfg_.alg
            << " --SO3-distribution " << cfg_.so3_dist
            << " --num-epochs " << cfg_.num_epochs
            << " --min " << (-cfg_.rough_trans_m)
            << " --max " << cfg_.rough_trans_m
            << " --output_dir " << output_dir;

        auto exec_r = exec_cmd(cmd.str(), cfg_.timeout_sec);
        result.elapsed_ms = exec_r.elapsed_ms;
        result.num_epochs  = cfg_.num_epochs;

        if (exec_r.exit_code != 0) {
            result.error_msg = "L2Calib 失败 (exit=" +
                                std::to_string(exec_r.exit_code) + ")";
            UNICALIB_ERROR("[L2Calib] {}", result.error_msg);
            UNICALIB_DEBUG("[L2Calib] stdout: {}", exec_r.stdout_str.substr(0, 500));
            return result;
        }

        std::string out_yaml = output_dir + "/calibration_result.yaml";
        if (fs::exists(out_yaml)) {
            result.coarse_extrin = parse_output(out_yaml);
            result.success    = true;
            result.confidence = 0.65;
            UNICALIB_INFO("[L2Calib] 粗估外参: t=[{:.3f}, {:.3f}, {:.3f}]m",
                          result.coarse_extrin.POS_TargetInRef.x(),
                          result.coarse_extrin.POS_TargetInRef.y(),
                          result.coarse_extrin.POS_TargetInRef.z());
        } else {
            result.error_msg = "未找到输出: " + out_yaml;
            UNICALIB_WARN("[L2Calib] {}", result.error_msg);
        }
    } catch (const std::exception& e) {
        result.error_msg = std::string("L2Calib 异常: ") + e.what();
        UNICALIB_ERROR("[L2Calib] {}", result.error_msg);
    }
    return result;
}

L2CalibResult L2CalibAdapter::estimate_from_trajectories(
    const std::string& lidar_traj_path,
    const std::string& imu_traj_path,
    const std::string& output_dir_arg) const {

    L2CalibResult result;
    result.model_name = "L2Calib";

    if (!is_available()) {
        result.error_msg = "L2Calib 不可用";
        UNICALIB_WARN("[L2Calib] {}", result.error_msg);
        return result;
    }

    if (!fs::exists(lidar_traj_path) || !fs::exists(imu_traj_path)) {
        result.error_msg = "轨迹文件不存在: " + lidar_traj_path + " / " + imu_traj_path;
        UNICALIB_WARN("[L2Calib] {}", result.error_msg);
        return result;
    }

    try {
        fs::create_directories(cfg_.work_dir);
        std::string output_dir = output_dir_arg.empty() ?
            cfg_.work_dir + "/output" : output_dir_arg;
        fs::create_directories(output_dir);

        std::ostringstream cmd;
        cmd << "cd " << cfg_.repo_dir << " && "
            << "export PYTHONPATH=" << cfg_.repo_dir << "/rl_solver:$PYTHONPATH && "
            << cfg_.python_exe << " " << cfg_.train_script
            << " --lidar-traj " << lidar_traj_path
            << " --imu-traj " << imu_traj_path
            << " --alg " << cfg_.alg
            << " --SO3-distribution " << cfg_.so3_dist
            << " --num-epochs " << cfg_.num_epochs
            << " --output_dir " << output_dir;

        auto exec_r = exec_cmd(cmd.str(), cfg_.timeout_sec);
        result.elapsed_ms = exec_r.elapsed_ms;
        result.num_epochs = cfg_.num_epochs;

        if (exec_r.exit_code != 0) {
            result.error_msg = "L2Calib (轨迹模式) 失败 exit=" + std::to_string(exec_r.exit_code);
            UNICALIB_WARN("[L2Calib] {}", result.error_msg);
            return result;
        }

        std::string out_yaml = output_dir + "/calibration_result.yaml";
        if (fs::exists(out_yaml)) {
            result.coarse_extrin = parse_output(out_yaml);
            result.success    = true;
            result.confidence = 0.65;
        } else {
            result.error_msg = "未找到输出: " + out_yaml;
        }
    } catch (const std::exception& e) {
        result.error_msg = std::string("L2Calib 异常: ") + e.what();
        UNICALIB_ERROR("[L2Calib] {}", result.error_msg);
    }
    return result;
}

ExtrinsicSE3 L2CalibAdapter::parse_output(const std::string& path) const {
    ExtrinsicSE3 extrin;
    extrin.ref_sensor_id    = "imu_0";
    extrin.target_sensor_id = "lidar_0";
    try {
        YAML::Node node = YAML::LoadFile(path);
        if (node["rotation_matrix"]) {
            Eigen::Matrix3d R;
            auto rm = node["rotation_matrix"];
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    R(i, j) = rm[i][j].as<double>();
            extrin.SO3_TargetInRef = Sophus::SO3d(R);
        } else if (node["rotation_euler_deg"]) {
            auto e = node["rotation_euler_deg"];
            double r = e[0].as<double>() * M_PI / 180.0;
            double p = e[1].as<double>() * M_PI / 180.0;
            double y = e[2].as<double>() * M_PI / 180.0;
            Eigen::AngleAxisd rx(r, Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd ry(p, Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd rz(y, Eigen::Vector3d::UnitZ());
            extrin.SO3_TargetInRef = Sophus::SO3d(
                (rz * ry * rx).toRotationMatrix());
        }
        if (node["translation"]) {
            auto t = node["translation"];
            extrin.POS_TargetInRef = Eigen::Vector3d(
                t[0].as<double>(), t[1].as<double>(), t[2].as<double>());
        }
        if (node["time_offset_s"])
            extrin.time_offset_s = node["time_offset_s"].as<double>();
    } catch (const std::exception& e) {
        UNICALIB_WARN("[L2Calib] 解析输出异常: {}", e.what());
    }
    return extrin;
}

// ===========================================================================
// AICoarseCalibManager
// ===========================================================================

AICoarseCalibManager::AICoarseCalibManager(const Config& cfg) : cfg_(cfg) {
    setup_adapters();
}

void AICoarseCalibManager::setup_adapters() {
    // DM-Calib：仅当 ai_root 非空时才用 ai_root 补全 repo_dir，避免空字符串 + "/DM-Calib" 得到 "/DM-Calib"
    DMCalibAdapter::Config dm_cfg = cfg_.dm_calib;
    if (dm_cfg.repo_dir.empty() && !cfg_.ai_root.empty())
        dm_cfg.repo_dir = cfg_.ai_root + "/DM-Calib";
    if (dm_cfg.model_path.empty() && !dm_cfg.repo_dir.empty())
        dm_cfg.model_path = dm_cfg.repo_dir + "/model";
    // 优先使用专用 Python（venv 等），以适配镜像 NumPy 2.x：DM-Calib/PyTorch 2.1 需 NumPy 1.x，脚本可设 UNICALIB_DM_CALIB_PYTHON
    const char* dm_python = std::getenv("UNICALIB_DM_CALIB_PYTHON");
    dm_cfg.python_exe = (dm_python && dm_python[0]) ? std::string(dm_python) : cfg_.python_exe;
    if (dm_python && dm_python[0])
        UNICALIB_INFO("[AI-Calib] DM-Calib 使用专用 Python (UNICALIB_DM_CALIB_PYTHON): {}", dm_cfg.python_exe);

    // 当配置的 Python（如 .venv）无 torch 时，自动改用已装 torch 的 Python（python3 / python）
    if (!check_python_module(dm_cfg.python_exe, "torch", nullptr)) {
        const std::vector<std::string> fallbacks = {"python3", "python"};
        for (const auto& candidate : fallbacks) {
            if (check_python_module(candidate, "torch", nullptr) && check_python_module(candidate, "numpy", nullptr)) {
                UNICALIB_INFO("[AI-Calib] DM-Calib 配置的 Python 无 torch，改用已装 torch 的 Python: {}", candidate);
                dm_cfg.python_exe = candidate;
                break;
            }
        }
    }

    dm_calib_ = std::make_unique<DMCalibAdapter>(dm_cfg);

    // MIAS-LCEC：repo_dir 为空则用 ai_root/MIAS-LCEC；若为相对路径（如 calib_unified）则相对 ai_root 解析
    MIASLCECAdapter::Config mias_cfg = cfg_.mias_lcec;
    if (mias_cfg.repo_dir.empty()) {
        mias_cfg.repo_dir = cfg_.ai_root + "/MIAS-LCEC";
    } else {
        fs::path rp(mias_cfg.repo_dir);
        if (!rp.is_absolute() && !cfg_.ai_root.empty()) {
            std::string resolved = cfg_.ai_root;
            if (resolved.back() != '/' && mias_cfg.repo_dir.front() != '/')
                resolved += "/";
            resolved += mias_cfg.repo_dir;
            mias_cfg.repo_dir = fs::path(resolved).lexically_normal().string();
        }
    }
    mias_cfg.python_exe = cfg_.python_exe;
    mias_lcec_ = std::make_unique<MIASLCECAdapter>(mias_cfg);

    // Transformer-IMU
    TransformerIMUAdapter::Config ti_cfg = cfg_.transformer_imu;
    if (ti_cfg.repo_dir.empty())
        ti_cfg.repo_dir = cfg_.ai_root + "/Transformer-IMU-Calibrator";
    ti_cfg.python_exe = cfg_.python_exe;
    transformer_imu_ = std::make_unique<TransformerIMUAdapter>(ti_cfg);

    // L2Calib
    L2CalibAdapter::Config l2_cfg = cfg_.l2calib;
    if (l2_cfg.repo_dir.empty())
        l2_cfg.repo_dir = cfg_.ai_root + "/learn-to-calibrate";
    l2_cfg.python_exe = cfg_.python_exe;
    l2calib_ = std::make_unique<L2CalibAdapter>(l2_cfg);
}

bool AICoarseCalibManager::check_dm_calib() const        { return dm_calib_->is_available(); }
bool AICoarseCalibManager::check_mias_lcec() const       { return mias_lcec_->is_available(); }
bool AICoarseCalibManager::check_transformer_imu() const { return transformer_imu_->is_available(); }
bool AICoarseCalibManager::check_l2calib() const         { return l2calib_->is_available(); }

void AICoarseCalibManager::print_availability() const {
    UNICALIB_INFO("[AI-Calib] ===== AI 模型可用性检查 =====");
    UNICALIB_INFO("  DM-Calib (相机内参):      {}",
                  check_dm_calib()        ? "✓ 可用" : "✗ 不可用");
    UNICALIB_INFO("  MIAS-LCEC (LiDAR-Cam):   {}",
                  check_mias_lcec()       ? "✓ 可用" : "✗ 不可用");
    UNICALIB_INFO("  Transformer-IMU (IMU内参): {}",
                  check_transformer_imu() ? "✓ 可用" : "✗ 不可用");
    UNICALIB_INFO("  L2Calib (IMU-LiDAR):     {}",
                  check_l2calib()         ? "✓ 可用" : "✗ 不可用");
    UNICALIB_INFO("[AI-Calib] ================================");
}

std::optional<CameraIntrinsics> AICoarseCalibManager::coarse_cam_intrinsic(
    const std::vector<std::string>& image_paths,
    int img_w, int img_h,
    const std::string& cam_id) const {

    UNICALIB_INFO("[AI-Calib] 相机内参粗估 (DM-Calib) cam_id={}", cam_id);

    if (!dm_calib_->is_available()) {
        UNICALIB_WARN("[AI-Calib] DM-Calib 不可用, 跳过粗估");
        return std::nullopt;
    }

    DMCalibResult r;
    if (image_paths.size() == 1) {
        r = dm_calib_->estimate(image_paths[0]);
    } else {
        r = dm_calib_->estimate_multi(image_paths);
    }

    if (!r.is_reliable(cfg_.min_confidence_threshold)) {
        UNICALIB_WARN("[AI-Calib] DM-Calib 置信度不足 ({:.2f} < {:.2f})",
                      r.confidence, cfg_.min_confidence_threshold);
        if (!cfg_.fallback_on_failure) return std::nullopt;
        UNICALIB_INFO("[AI-Calib] 降级: 使用默认内参初始值");
        CameraIntrinsics fallback;
        fallback.width = img_w; fallback.height = img_h;
        fallback.fx = fallback.fy = std::max(img_w, img_h) * 0.8;
        fallback.cx = img_w * 0.5; fallback.cy = img_h * 0.5;
        fallback.rms_reproj_error = -1.0;  // N/A，无棋盘格/未优化
        fallback.num_images_used = 0;
        return fallback;
    }

    r.coarse_intrin.width  = img_w;
    r.coarse_intrin.height = img_h;
    return r.coarse_intrin;
}

std::optional<IMUIntrinsics> AICoarseCalibManager::coarse_imu_intrinsic(
    const std::vector<IMUFrame>& imu_data,
    const std::string& imu_id) const {

    UNICALIB_INFO("[AI-Calib] IMU 内参粗估 (Transformer-IMU) imu_id={}", imu_id);

    if (!transformer_imu_->is_available()) {
        UNICALIB_WARN("[AI-Calib] Transformer-IMU 不可用, 跳过粗估");
        return std::nullopt;
    }

    auto r = transformer_imu_->estimate(imu_data);
    if (!r.is_reliable(cfg_.min_confidence_threshold)) {
        UNICALIB_WARN("[AI-Calib] Transformer-IMU 置信度不足");
        return std::nullopt;
    }
    return r.coarse_intrin;
}

std::optional<ExtrinsicSE3> AICoarseCalibManager::coarse_imu_lidar(
    const std::string& bag_path,
    const std::string& imu_id,
    const std::string& lidar_id) const {

    UNICALIB_INFO("[AI-Calib] IMU-LiDAR 粗估 (L2Calib) {}→{}", imu_id, lidar_id);

    if (!l2calib_->is_available()) {
        UNICALIB_WARN("[AI-Calib] L2Calib 不可用, 跳过粗估");
        return std::nullopt;
    }

    auto r = l2calib_->estimate_from_bag(bag_path);
    if (!r.is_reliable(cfg_.min_confidence_threshold)) {
        UNICALIB_WARN("[AI-Calib] L2Calib 置信度不足");
        return std::nullopt;
    }

    r.coarse_extrin.ref_sensor_id    = imu_id;
    r.coarse_extrin.target_sensor_id = lidar_id;
    return r.coarse_extrin;
}

std::optional<ExtrinsicSE3> AICoarseCalibManager::coarse_lidar_cam(
    const std::string& pcd_file,
    const std::string& image_file,
    const CameraIntrinsics& cam_intrin,
    const std::string& lidar_id,
    const std::string& cam_id) const {

    UNICALIB_INFO("[AI-Calib] LiDAR-Cam 粗估 (MIAS-LCEC) {}→{}", lidar_id, cam_id);

    if (!mias_lcec_->is_available()) {
        UNICALIB_WARN("[AI-Calib] MIAS-LCEC 不可用, 跳过粗估 (请检查 ai_root/MIAS-LCEC 或 third_party.mias_lcec.repo_dir，以及 model/pretrained_overlap_transformer.pth.tar)");
        return std::nullopt;
    }

    auto r = mias_lcec_->estimate(pcd_file, image_file, cam_intrin);
    if (!r.is_reliable(cfg_.min_confidence_threshold)) {
        UNICALIB_WARN("[AI-Calib] MIAS-LCEC 置信度不足");
        return std::nullopt;
    }

    r.coarse_extrin.ref_sensor_id    = lidar_id;
    r.coarse_extrin.target_sensor_id = cam_id;
    return r.coarse_extrin;
}

}  // namespace ns_unicalib
