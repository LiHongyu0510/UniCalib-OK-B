/**
 * UniCalib C++ — 日志系统（无外部依赖，基于 iostream）
 */
#pragma once

#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <mutex>
#include <chrono>
#include <map>
#include <ctime>
#include <cstdio>
#include <cstdarg>
#include "exceptions.hpp"

namespace unicalib {

enum class LogLevel { TRACE = 0, DEBUG = 1, INFO = 2, WARNING = 3, ERROR = 4, CRITICAL = 5, OFF = 6 };

class Logger {
 public:
  static Logger& instance() {
    static Logger inst;
    return inst;
  }

  void init(const std::string& name = "unicalib", LogLevel level = LogLevel::INFO,
            const std::string& log_file = "", bool enable_console = true) {
    std::lock_guard<std::mutex> lock(mutex_);
    logger_name_ = name;
    log_level_ = level;
    enable_console_ = enable_console;
    if (!log_file.empty()) {
      file_.open(log_file, std::ios::app);
      enable_file_ = file_.is_open();
    } else {
      enable_file_ = false;
    }
  }

  void set_level(LogLevel level) { log_level_ = level; }
  LogLevel get_level() const { return log_level_; }

  void trace(const std::string& msg)   { log(LogLevel::TRACE, msg); }
  void debug(const std::string& msg)   { log(LogLevel::DEBUG, msg); }
  void info(const std::string& msg)    { log(LogLevel::INFO, msg); }
  void warning(const std::string& msg) { log(LogLevel::WARNING, msg); }
  void error(const std::string& msg)   { log(LogLevel::ERROR, msg); }
  void critical(const std::string& msg){ log(LogLevel::CRITICAL, msg); }

  void log(LogLevel level, const std::string& message,
           const std::string& component = "", const std::string& sensor_id = "") {
    if (level < log_level_ || level == LogLevel::OFF) return;
    std::lock_guard<std::mutex> lock(mutex_);
    std::string line = format(level, message, component, sensor_id);
    if (enable_console_) {
      if (level >= LogLevel::ERROR) std::cerr << line << std::endl;
      else std::cout << line << std::endl;
    }
    if (enable_file_ && file_.is_open()) file_ << line << std::endl;
  }

  void log_exception(const std::exception& e, const std::string& context = "") {
    std::string msg = std::string("Exception: ") + e.what();
    if (!context.empty()) msg += " [context: " + context + "]";
    error(msg);
  }

  void log_exception(const UniCalibException& e, const std::string& context = "") {
    std::string msg = e.context().to_string();
    if (!context.empty()) msg += " [context: " + context + "]";
    if (e.severity() >= ErrorSeverity::CRITICAL) critical(msg);
    else error(msg);
  }

  void flush() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (file_.is_open()) file_.flush();
  }

 private:
  Logger() = default;
  std::mutex mutex_;
  LogLevel log_level_ = LogLevel::INFO;
  std::string logger_name_ = "unicalib";
  bool enable_console_ = true;
  bool enable_file_ = false;
  std::ofstream file_;

  static const char* level_str(LogLevel l) {
    switch (l) {
      case LogLevel::TRACE:   return "TRACE";
      case LogLevel::DEBUG:   return "DEBUG";
      case LogLevel::INFO:    return "INFO";
      case LogLevel::WARNING: return "WARN";
      case LogLevel::ERROR:   return "ERROR";
      case LogLevel::CRITICAL: return "CRITICAL";
      default: return "?";
    }
  }

  std::string format(LogLevel level, const std::string& message,
                     const std::string& component, const std::string& sensor_id) {
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", std::localtime(&t));
    std::ostringstream oss;
    oss << "[" << buf << "] [" << level_str(level) << "] [" << logger_name_ << "] ";
    if (!component.empty()) oss << "[" << component << "] ";
    if (!sensor_id.empty()) oss << "[" << sensor_id << "] ";
    oss << message;
    return oss.str();
  }
};

class PerformanceTimer {
 public:
  explicit PerformanceTimer(const std::string& name, LogLevel log_level = LogLevel::DEBUG)
      : name_(name), log_level_(log_level), start_(std::chrono::high_resolution_clock::now()) {}
  ~PerformanceTimer() {
    auto end = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(end - start_).count();
    Logger::instance().log(log_level_, name_ + " took " + std::to_string(ms) + " ms");
  }
 private:
  std::string name_;
  LogLevel log_level_;
  std::chrono::high_resolution_clock::time_point start_;
};

// ============================================================================
// 格式化日志辅助函数 (printf 风格)
// ============================================================================
inline std::string format_log_internal(const char* fmt, ...) {
  char buffer[1024];
  va_list args;
  va_start(args, fmt);
  std::vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);
  return std::string(buffer);
}

}  // namespace unicalib

// ============================================================================
// 基础日志宏 (单参数)
// ============================================================================
#define LOG_TRACE(msg)   unicalib::Logger::instance().trace(msg)
#define LOG_DEBUG(msg)   unicalib::Logger::instance().debug(msg)
#define LOG_INFO(msg)    unicalib::Logger::instance().info(msg)
#define LOG_WARNING(msg) unicalib::Logger::instance().warning(msg)
#define LOG_ERROR(msg)   unicalib::Logger::instance().error(msg)
#define LOG_CRITICAL(msg) unicalib::Logger::instance().critical(msg)
#define LOG_EXCEPTION(e) unicalib::Logger::instance().log_exception(e, __FUNCTION__)
#define PERF_TIMER(name) unicalib::PerformanceTimer _perf_##__LINE__(name)

// 向后兼容
#define LOG_WARN(msg) unicalib::Logger::instance().warning(msg)

// ============================================================================
// 格式化日志宏 (printf 风格)
// ============================================================================
#define LOG_INFO_FMT(fmt, ...)   unicalib::Logger::instance().info(unicalib::format_log_internal(fmt, ##__VA_ARGS__))
#define LOG_DEBUG_FMT(fmt, ...)  unicalib::Logger::instance().debug(unicalib::format_log_internal(fmt, ##__VA_ARGS__))
#define LOG_WARN_FMT(fmt, ...)   unicalib::Logger::instance().warning(unicalib::format_log_internal(fmt, ##__VA_ARGS__))
#define LOG_WARNING_FMT(fmt, ...) unicalib::Logger::instance().warning(unicalib::format_log_internal(fmt, ##__VA_ARGS__))
#define LOG_ERROR_FMT(fmt, ...)  unicalib::Logger::instance().error(unicalib::format_log_internal(fmt, ##__VA_ARGS__))
#define LOG_TRACE_FMT(fmt, ...)  unicalib::Logger::instance().trace(unicalib::format_log_internal(fmt, ##__VA_ARGS__))
#define LOG_CRITICAL_FMT(fmt, ...) unicalib::Logger::instance().critical(unicalib::format_log_internal(fmt, ##__VA_ARGS__))
