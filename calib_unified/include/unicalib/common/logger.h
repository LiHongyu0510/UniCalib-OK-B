/**
 * UniCalib Unified — 增强版统一日志系统
 * 
 * 功能：
 *   - 多 Sink 输出（控制台 + 文件 + 轮转）
 *   - 自动包含文件名/行号/函数名
 *   - 结构化日志（可选 JSON 格式）
 *   - 性能计时宏
 *   - 模块级日志控制
 *   - 与 ErrorCode 集成
 * 
 * 用法：
 *   UNICALIB_INIT_LOGGER("calib.log", spdlog::level::debug);
 *   UNICALIB_INFO("Starting calibration");
 *   UNICALIB_LOG_ERROR(ErrorCode::FILE_NOT_FOUND, "config.yaml");
 */

#pragma once

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/fmt/ostr.h>
#include <memory>
#include <string>
#include <vector>
#include <filesystem>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <cstdlib>

#include "error_code.h"

namespace ns_unicalib {

namespace fs = std::filesystem;

// ===================================================================
// 日志目录与时间戳工具（编译/运行日志统一写入 logs/ 并带时间）
// ===================================================================
// 环境变量 UNICALIB_LOGS_DIR: 若设置则运行日志写入该目录（如项目根下 logs/）
// 否则使用 output_dir + "/logs"
inline std::string resolve_logs_dir(const std::string& output_dir) {
    const char* env = std::getenv("UNICALIB_LOGS_DIR");
    std::string dir = env ? std::string(env) : (output_dir + "/logs");
    fs::create_directories(dir);
    return dir;
}

// 用于日志文件名的时间戳，格式 YYYYMMDD_HHMMSS
inline std::string log_timestamp_filename() {
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);
    std::tm tm_buf{};
#ifdef _WIN32
    localtime_s(&tm_buf, &t);
#else
    localtime_r(&t, &tm_buf);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm_buf, "%Y%m%d_%H%M%S");
    return oss.str();
}

// ===================================================================
// LoggerConfig — 日志配置
// ===================================================================
struct LoggerConfig {
    std::string logger_name = "UniCalib";
    std::string log_file = "";              // 空 = 不写文件
    bool use_rotating_file = true;          // 使用轮转日志
    size_t max_file_size_mb = 10;           // 最大文件大小 (MB)
    size_t max_files = 5;                   // 保留文件数
    spdlog::level::level_enum console_level = spdlog::level::info;
    spdlog::level::level_enum file_level = spdlog::level::trace;
    std::string pattern = "[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [%n] [%s:%#] %v";
    bool flush_on_msg = true;               // 每条消息后刷新
    bool include_location = true;           // 包含文件位置
};

// ===================================================================
// Logger — 日志管理器
// ===================================================================
class Logger {
public:
    // 初始化日志系统
    static void init(const LoggerConfig& config = LoggerConfig()) {
        std::vector<spdlog::sink_ptr> sinks;
        
        // 控制台 Sink
        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        console_sink->set_level(config.console_level);
        console_sink->set_pattern(config.pattern);
        sinks.push_back(console_sink);
        
        // 文件 Sink (可选)
        if (!config.log_file.empty()) {
            // 确保目录存在
            fs::path log_path(config.log_file);
            if (log_path.has_parent_path()) {
                fs::create_directories(log_path.parent_path());
            }
            
            if (config.use_rotating_file) {
                auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
                    config.log_file,
                    config.max_file_size_mb * 1024 * 1024,
                    config.max_files);
                file_sink->set_level(config.file_level);
                file_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] [%n] %v");
                sinks.push_back(file_sink);
            } else {
                auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
                    config.log_file, true);
                file_sink->set_level(config.file_level);
                file_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] [%n] %v");
                sinks.push_back(file_sink);
            }
        }
        
        // 创建 Logger
        auto logger = std::make_shared<spdlog::logger>(config.logger_name, 
                                                        sinks.begin(), sinks.end());
        logger->set_level(spdlog::level::trace);
        
        if (config.flush_on_msg) {
            logger->flush_on(spdlog::level::info);
        }
        
        spdlog::register_logger(logger);
        spdlog::set_default_logger(logger);
        
        config_ = config;
        initialized_ = true;
    }
    
    // 简易初始化
    static void init(const std::string& name = "UniCalib",
                     const std::string& log_file = "",
                     spdlog::level::level_enum level = spdlog::level::info) {
        LoggerConfig config;
        config.logger_name = name;
        config.log_file = log_file;
        config.console_level = level;
        init(config);
    }
    
    // 设置全局日志级别
    static void setLevel(spdlog::level::level_enum level) {
        spdlog::set_level(level);
    }
    
    // 获取默认 Logger
    static spdlog::logger& get() { 
        return *spdlog::default_logger(); 
    }
    
    // 检查是否已初始化
    static bool isInitialized() { return initialized_; }
    
    // 刷新日志
    static void flush() {
        spdlog::default_logger()->flush();
    }
    
    // 关闭日志
    static void shutdown() {
        spdlog::shutdown();
        initialized_ = false;
    }
    
    // 设置模块日志级别
    static void setModuleLevel(const std::string& module, 
                               spdlog::level::level_enum level) {
        auto logger = spdlog::get(module);
        if (logger) {
            logger->set_level(level);
        }
    }

private:
    static LoggerConfig config_;
    static bool initialized_;
};

// 静态成员定义
inline LoggerConfig Logger::config_;
inline bool Logger::initialized_ = false;

// ===================================================================
// 模块级日志器
// ===================================================================
class ModuleLogger {
public:
    explicit ModuleLogger(const std::string& module_name,
                          spdlog::level::level_enum level = spdlog::level::info)
        : module_name_(module_name) {
        logger_ = spdlog::get(module_name);
        if (!logger_) {
            logger_ = std::make_shared<spdlog::logger>(module_name, 
                spdlog::default_logger()->sinks().begin(),
                spdlog::default_logger()->sinks().end());
            logger_->set_level(level);
            spdlog::register_logger(logger_);
        }
    }
    
    template<typename... Args>
    void trace(const char* fmt, Args&&... args) {
        logger_->trace("[{}] {}", module_name_, fmt::format(fmt, std::forward<Args>(args)...));
    }
    
    template<typename... Args>
    void debug(const char* fmt, Args&&... args) {
        logger_->debug("[{}] {}", module_name_, fmt::format(fmt, std::forward<Args>(args)...));
    }
    
    template<typename... Args>
    void info(const char* fmt, Args&&... args) {
        logger_->info("[{}] {}", module_name_, fmt::format(fmt, std::forward<Args>(args)...));
    }
    
    template<typename... Args>
    void warn(const char* fmt, Args&&... args) {
        logger_->warn("[{}] {}", module_name_, fmt::format(fmt, std::forward<Args>(args)...));
    }
    
    template<typename... Args>
    void error(const char* fmt, Args&&... args) {
        logger_->error("[{}] {}", module_name_, fmt::format(fmt, std::forward<Args>(args)...));
    }
    
    template<typename... Args>
    void critical(const char* fmt, Args&&... args) {
        logger_->critical("[{}] {}", module_name_, fmt::format(fmt, std::forward<Args>(args)...));
    }

private:
    std::string module_name_;
    std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace ns_unicalib

// ===================================================================
// 全局日志宏 — 基础版（保持兼容）
// ===================================================================
#define UNICALIB_TRACE(...)    spdlog::trace(__VA_ARGS__)
#define UNICALIB_DEBUG(...)    spdlog::debug(__VA_ARGS__)
#define UNICALIB_INFO(...)     spdlog::info(__VA_ARGS__)
#define UNICALIB_WARN(...)     spdlog::warn(__VA_ARGS__)
#define UNICALIB_ERROR(...)    spdlog::error(__VA_ARGS__)
#define UNICALIB_CRITICAL(...) spdlog::critical(__VA_ARGS__)

// ===================================================================
// 增强版日志宏 — 带位置信息
// ===================================================================

#define UNICALIB_TRACE_EX(msg, ...) \
    spdlog::trace("[{}:{}] " msg, \
                  std::filesystem::path(__FILE__).filename().string(), \
                  __LINE__, ##__VA_ARGS__)

#define UNICALIB_DEBUG_EX(msg, ...) \
    spdlog::debug("[{}:{}] " msg, \
                  std::filesystem::path(__FILE__).filename().string(), \
                  __LINE__, ##__VA_ARGS__)

#define UNICALIB_INFO_EX(msg, ...) \
    spdlog::info("[{}:{}] " msg, \
                 std::filesystem::path(__FILE__).filename().string(), \
                 __LINE__, ##__VA_ARGS__)

#define UNICALIB_WARN_EX(msg, ...) \
    spdlog::warn("[{}:{}] " msg, \
                 std::filesystem::path(__FILE__).filename().string(), \
                 __LINE__, ##__VA_ARGS__)

#define UNICALIB_ERROR_EX(msg, ...) \
    spdlog::error("[{}:{}] " msg, \
                  std::filesystem::path(__FILE__).filename().string(), \
                  __LINE__, ##__VA_ARGS__)

#define UNICALIB_CRITICAL_EX(msg, ...) \
    spdlog::critical("[{}:{}] " msg, \
                     std::filesystem::path(__FILE__).filename().string(), \
                     __LINE__, ##__VA_ARGS__)

// ===================================================================
// 带函数名的日志宏
// ===================================================================

#define UNICALIB_TRACE_FN(msg, ...) \
    spdlog::trace("[{}:{}] [{}] " msg, \
                  std::filesystem::path(__FILE__).filename().string(), \
                  __LINE__, __func__, ##__VA_ARGS__)

#define UNICALIB_DEBUG_FN(msg, ...) \
    spdlog::debug("[{}:{}] [{}] " msg, \
                  std::filesystem::path(__FILE__).filename().string(), \
                  __LINE__, __func__, ##__VA_ARGS__)

#define UNICALIB_INFO_FN(msg, ...) \
    spdlog::info("[{}:{}] [{}] " msg, \
                 std::filesystem::path(__FILE__).filename().string(), \
                 __LINE__, __func__, ##__VA_ARGS__)

#define UNICALIB_WARN_FN(msg, ...) \
    spdlog::warn("[{}:{}] [{}] " msg, \
                 std::filesystem::path(__FILE__).filename().string(), \
                 __LINE__, __func__, ##__VA_ARGS__)

#define UNICALIB_ERROR_FN(msg, ...) \
    spdlog::error("[{}:{}] [{}] " msg, \
                  std::filesystem::path(__FILE__).filename().string(), \
                  __LINE__, __func__, ##__VA_ARGS__)

// ===================================================================
// 带 ErrorCode 的日志宏
// ===================================================================

#define UNICALIB_LOG_ERROR(code, msg, ...) \
    do { \
        spdlog::error("[{}] [{}:{}] " msg, \
                      ::ns_unicalib::errorCodeName(code), \
                      std::filesystem::path(__FILE__).filename().string(), \
                      __LINE__, ##__VA_ARGS__); \
    } while(0)

#define UNICALIB_LOG_WARN_CODE(code, msg, ...) \
    do { \
        spdlog::warn("[{}] [{}:{}] " msg, \
                     ::ns_unicalib::errorCodeName(code), \
                     std::filesystem::path(__FILE__).filename().string(), \
                     __LINE__, ##__VA_ARGS__); \
    } while(0)

// ===================================================================
// 进度日志宏
// ===================================================================

#define UNICALIB_LOG_STEP(step_name, msg, ...) \
    spdlog::info("[{}] " msg, step_name, ##__VA_ARGS__)

#define UNICALIB_LOG_PROGRESS(current, total, msg, ...) \
    do { \
        double _pct = (total) > 0 ? 100.0 * (current) / (total) : 0.0; \
        spdlog::info("[{}/{} ({:.1f}%)] " msg, \
                     (current), (total), _pct, ##__VA_ARGS__); \
    } while(0)

// ===================================================================
// 计算环节日志（便于标定问题分析与排查，可用 grep '[CALC]' 过滤）
// ===================================================================
#define UNICALIB_CALC(msg, ...) \
    spdlog::info("[CALC] " msg, ##__VA_ARGS__)
#define UNICALIB_CALC_DEBUG(msg, ...) \
    spdlog::debug("[CALC] " msg, ##__VA_ARGS__)
#define UNICALIB_CALC_WARN(msg, ...) \
    spdlog::warn("[CALC] " msg, ##__VA_ARGS__)

// ===================================================================
// 初始化宏
// ===================================================================

#define UNICALIB_INIT_LOGGER(log_file, level) \
    ::ns_unicalib::Logger::init("UniCalib", log_file, level)

#define UNICALIB_INIT_LOGGER_FULL(config) \
    ::ns_unicalib::Logger::init(config)

// ===================================================================
// 条件日志宏
// ===================================================================

#define UNICALIB_LOG_IF(cond, level, msg, ...) \
    do { \
        if (cond) { \
            spdlog::level(msg, ##__VA_ARGS__); \
        } \
    } while(0)

#define UNICALIB_LOG_ONCE(level, msg, ...) \
    do { \
        static bool _logged = false; \
        if (!_logged) { \
            spdlog::level(msg, ##__VA_ARGS__); \
            _logged = true; \
        } \
    } while(0)

// ===================================================================
// 调试日志宏（仅 Debug 构建时有效）
// ===================================================================

#ifdef NDEBUG
#define UNICALIB_DLOG(...)
#define UNICALIB_DLOG_FN(...)
#else
#define UNICALIB_DLOG(msg, ...) UNICALIB_DEBUG(msg, ##__VA_ARGS__)
#define UNICALIB_DLOG_FN(msg, ...) UNICALIB_DEBUG_FN(msg, ##__VA_ARGS__)
#endif

// ===================================================================
// 进入/退出函数日志（调试用）
// ===================================================================

#define UNICALIB_FUNC_ENTER() \
    UNICALIB_TRACE_FN(">>> ENTER")

#define UNICALIB_FUNC_EXIT() \
    UNICALIB_TRACE_FN("<<< EXIT")

#define UNICALIB_SCOPED_LOG(name) \
    struct ScopedLog_##name { \
        ScopedLog_##name() { UNICALIB_TRACE_FN(">>> ENTER {}", #name); } \
        ~ScopedLog_##name() { UNICALIB_TRACE_FN("<<< EXIT {}", #name); } \
    } _scoped_log_##name
