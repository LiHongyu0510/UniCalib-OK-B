/**
 * UniCalib Unified — 统一异常体系
 * 
 * 目的：
 *   - 提供统一的异常层次结构
 *   - 携带错误码、源码位置、上下文信息
 *   - 便于上层分类捕获和处理
 * 
 * 层次：
 *   UniCalibException (基类)
 *   ├── ConfigException     — 配置错误
 *   ├── DataException       — 数据加载/IO错误
 *   ├── CalibException      — 标定算法错误
 *   ├── NumericalException  — 数值计算错误
 *   ├── SystemException     — 系统资源错误
 *   └── VisualizationException — 可视化错误
 */

#pragma once

#include "error_code.h"
#include <stdexcept>
#include <string>
#include <sstream>
#include <memory>
#include <vector>
#include <chrono>
#include <iomanip>

namespace ns_unicalib {

// ===================================================================
// UniCalibException — 异常基类
// ===================================================================
class UniCalibException : public std::runtime_error {
public:
    // ─────────────────────────────────────────────────────────────
    // 构造函数
    // ─────────────────────────────────────────────────────────────
    
    UniCalibException(ErrorCode code, const std::string& message,
                      const char* file = nullptr, int line = 0,
                      const char* func = nullptr)
        : std::runtime_error(formatMessage(code, message, file, line, func)),
          code_(code),
          message_(message),
          file_(file ? file : ""),
          line_(line),
          func_(func ? func : ""),
          timestamp_(std::chrono::system_clock::now()) {}
    
    virtual ~UniCalibException() = default;
    
    // ─────────────────────────────────────────────────────────────
    // 访问器
    // ─────────────────────────────────────────────────────────────
    
    ErrorCode code() const { return code_; }
    int codeValue() const { return static_cast<int>(code_); }
    const std::string& message() const { return message_; }
    const std::string& file() const { return file_; }
    int line() const { return line_; }
    const std::string& func() const { return func_; }
    const std::string& context() const { return context_; }
    
    // 获取时间戳
    std::string timestampStr() const {
        auto tt = std::chrono::system_clock::to_time_t(timestamp_);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            timestamp_.time_since_epoch()) % 1000;
        std::ostringstream oss;
        oss << std::put_time(std::localtime(&tt), "%Y-%m-%d %H:%M:%S")
            << "." << std::setfill('0') << std::setw(3) << ms.count();
        return oss.str();
    }
    
    // 获取错误分类
    std::string category() const {
        return errorCodeCategory(code_);
    }
    
    // 添加上下文信息
    UniCalibException& withContext(const std::string& ctx) {
        context_ = ctx;
        return *this;
    }
    
    // 添加标签
    UniCalibException& withTag(const std::string& key, const std::string& value) {
        tags_.push_back({key, value});
        return *this;
    }
    
    const std::vector<std::pair<std::string, std::string>>& tags() const {
        return tags_;
    }
    
    // ─────────────────────────────────────────────────────────────
    // 格式化输出
    // ─────────────────────────────────────────────────────────────
    
    virtual std::string toString() const {
        std::ostringstream oss;
        oss << "[" << errorCodeName(code_) << "] " << message_;
        if (!file_.empty()) {
            oss << "\n  at " << file_ << ":" << line_;
            if (!func_.empty()) {
                oss << " in " << func_ << "()";
            }
        }
        if (!context_.empty()) {
            oss << "\n  context: " << context_;
        }
        for (const auto& tag : tags_) {
            oss << "\n  " << tag.first << ": " << tag.second;
        }
        oss << "\n  timestamp: " << timestampStr();
        return oss.str();
    }
    
    // JSON格式输出（用于日志系统）
    std::string toJson() const {
        std::ostringstream oss;
        oss << "{"
            << "\"code\":" << codeValue() << ","
            << "\"name\":\"" << errorCodeName(code_) << "\","
            << "\"category\":\"" << category() << "\","
            << "\"message\":\"" << escapeJson(message_) << "\"";
        if (!file_.empty()) {
            oss << "\"file\":\"" << file_ << "\","
                << "\"line\":" << line_ << ","
                << "\"function\":\"" << func_ << "\",";
        }
        if (!context_.empty()) {
            oss << "\"context\":\"" << escapeJson(context_) << "\",";
        }
        oss << "\"timestamp\":\"" << timestampStr() << "\""
            << "}";
        return oss.str();
    }

protected:
    ErrorCode code_;
    std::string message_;
    std::string file_;
    int line_;
    std::string func_;
    std::string context_;
    std::vector<std::pair<std::string, std::string>> tags_;
    std::chrono::system_clock::time_point timestamp_;
    
    static std::string formatMessage(ErrorCode code, const std::string& msg,
                                     const char* file, int line, const char* func) {
        std::ostringstream oss;
        oss << "[" << errorCodeName(code) << "] " << msg;
        if (file) {
            oss << " (" << file << ":" << line;
            if (func) oss << " in " << func;
            oss << ")";
        }
        return oss.str();
    }
    
    static std::string escapeJson(const std::string& s) {
        std::ostringstream oss;
        for (char c : s) {
            switch (c) {
                case '"': oss << "\\\""; break;
                case '\\': oss << "\\\\"; break;
                case '\n': oss << "\\n"; break;
                case '\r': oss << "\\r"; break;
                case '\t': oss << "\\t"; break;
                default: oss << c;
            }
        }
        return oss.str();
    }
};

// ===================================================================
// 派生异常类
// ===================================================================

class ConfigException : public UniCalibException {
public:
    ConfigException(ErrorCode code, const std::string& msg,
                    const char* file = nullptr, int line = 0, const char* func = nullptr)
        : UniCalibException(code, msg, file, line, func) {}
};

class DataException : public UniCalibException {
public:
    DataException(ErrorCode code, const std::string& msg,
                  const char* file = nullptr, int line = 0, const char* func = nullptr)
        : UniCalibException(code, msg, file, line, func) {}
};

class CalibException : public UniCalibException {
public:
    CalibException(ErrorCode code, const std::string& msg,
                   const char* file = nullptr, int line = 0, const char* func = nullptr)
        : UniCalibException(code, msg, file, line, func) {}
};

class NumericalException : public UniCalibException {
public:
    NumericalException(ErrorCode code, const std::string& msg,
                       const char* file = nullptr, int line = 0, const char* func = nullptr)
        : UniCalibException(code, msg, file, line, func) {}
};

class SystemException : public UniCalibException {
public:
    SystemException(ErrorCode code, const std::string& msg,
                    const char* file = nullptr, int line = 0, const char* func = nullptr)
        : UniCalibException(code, msg, file, line, func) {}
};

class VisualizationException : public UniCalibException {
public:
    VisualizationException(ErrorCode code, const std::string& msg,
                           const char* file = nullptr, int line = 0, const char* func = nullptr)
        : UniCalibException(code, msg, file, line, func) {}
};

// ===================================================================
// 异常工厂 — 便于根据错误码自动选择异常类型
// ===================================================================
inline std::unique_ptr<UniCalibException> createException(
    ErrorCode code, const std::string& msg,
    const char* file = nullptr, int line = 0, const char* func = nullptr) {
    
    int val = static_cast<int>(code);
    
    if (val >= 1001 && val < 2000) {
        return std::make_unique<ConfigException>(code, msg, file, line, func);
    }
    if (val >= 2001 && val < 3000) {
        return std::make_unique<DataException>(code, msg, file, line, func);
    }
    if (val >= 3001 && val < 4000) {
        return std::make_unique<CalibException>(code, msg, file, line, func);
    }
    if (val >= 4001 && val < 5000) {
        return std::make_unique<NumericalException>(code, msg, file, line, func);
    }
    if (val >= 5001 && val < 6000) {
        return std::make_unique<SystemException>(code, msg, file, line, func);
    }
    if (val >= 6001 && val < 7000) {
        return std::make_unique<VisualizationException>(code, msg, file, line, func);
    }
    
    return std::make_unique<UniCalibException>(code, msg, file, line, func);
}

// ===================================================================
// 便利宏
// ===================================================================

#define UNICALIB_THROW(code, msg) \
    do { \
        auto _unicalib_e = ::ns_unicalib::createException(code, msg, __FILE__, __LINE__, __func__); \
        throw *_unicalib_e; \
    } while(0)

#define UNICALIB_THROW_IF(cond, code, msg) \
    do { \
        if (cond) { \
            UNICALIB_THROW(code, msg); \
        } \
    } while(0)

#define UNICALIB_THROW_CONFIG(msg) \
    throw ::ns_unicalib::ConfigException(::ns_unicalib::ErrorCode::INVALID_CONFIG, \
                                         msg, __FILE__, __LINE__, __func__)

#define UNICALIB_THROW_DATA(code, msg) \
    throw ::ns_unicalib::DataException(code, msg, __FILE__, __LINE__, __func__)

#define UNICALIB_THROW_CALIB(code, msg) \
    throw ::ns_unicalib::CalibException(code, msg, __FILE__, __LINE__, __func__)

#define UNICALIB_THROW_NUMERICAL(code, msg) \
    throw ::ns_unicalib::NumericalException(code, msg, __FILE__, __LINE__, __func__)

#define UNICALIB_THROW_SYSTEM(code, msg) \
    throw ::ns_unicalib::SystemException(code, msg, __FILE__, __LINE__, __func__)

// 安全数值检查
#define UNICALIB_CHECK_FINITE(val, msg) \
    UNICALIB_THROW_IF(!std::isfinite(val), \
                      ::ns_unicalib::ErrorCode::NAN_DETECTED, msg)

#define UNICALIB_CHECK_POSITIVE(val, msg) \
    UNICALIB_THROW_IF((val) <= 0, \
                      ::ns_unicalib::ErrorCode::NUMERICAL_ERROR, msg)

#define UNICALIB_CHECK_NONEMPTY(container, msg) \
    UNICALIB_THROW_IF((container).empty(), \
                      ::ns_unicalib::ErrorCode::EMPTY_DATA, msg)

// ===================================================================
// 主程序异常报告（实现在 exception_report.cpp，避免头文件依赖 logger）
// ===================================================================
void reportUnhandledUniCalibException(const UniCalibException& e);
void reportUnhandledStdException(const std::exception& e);
void reportUnhandledUnknownException();

// ===================================================================
// 主程序异常捕获 — 避免未捕获异常导致进程崩溃，精确定位问题
// ===================================================================
#define UNICALIB_MAIN_TRY_BEGIN \
    try {

#define UNICALIB_MAIN_TRY_END(exitsuccess) \
    } \
    catch (const ::ns_unicalib::UniCalibException& e) { \
        ::ns_unicalib::reportUnhandledUniCalibException(e); \
        return static_cast<int>(e.codeValue() != 0 ? (e.codeValue() % 256) : 1); \
    } \
    catch (const std::exception& e) { \
        ::ns_unicalib::reportUnhandledStdException(e); \
        return 2; \
    } \
    catch (...) { \
        ::ns_unicalib::reportUnhandledUnknownException(); \
        return 3; \
    } \
    return (exitsuccess);

}  // namespace ns_unicalib
