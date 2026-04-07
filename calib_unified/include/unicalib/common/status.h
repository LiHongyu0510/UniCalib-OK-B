/**
 * UniCalib Unified — 统一状态返回类
 * 
 * 目的：
 *   - 提供非异常方式的错误返回（适用于性能敏感路径）
 *   - 支持链式错误信息传递
 *   - 兼容 Result<T> 模式
 * 
 * 用法：
 *   Status s = someFunction();
 *   if (!s.ok()) {
 *       UNICALIB_ERROR("Failed: {}", s.message());
 *       return s;
 *   }
 */

#pragma once

#include "error_code.h"
#include <string>
#include <optional>
#include <memory>
#include <sstream>
#include <vector>

namespace ns_unicalib {

// ===================================================================
// Status — 轻量级状态返回
// ===================================================================
class Status {
public:
    // ─────────────────────────────────────────────────────────────
    // 构造函数
    // ─────────────────────────────────────────────────────────────
    
    // 成功状态
    Status() : code_(ErrorCode::SUCCESS) {}
    
    // 错误状态
    Status(ErrorCode code, const std::string& message = "")
        : code_(code), message_(message) {}
    
    // 拷贝/移动
    Status(const Status&) = default;
    Status(Status&&) = default;
    Status& operator=(const Status&) = default;
    Status& operator=(Status&&) = default;
    
    // ─────────────────────────────────────────────────────────────
    // 静态工厂方法
    // ─────────────────────────────────────────────────────────────
    
    static Status OK() { return Status(); }
    
    static Status Error(ErrorCode code, const std::string& message = "") {
        return Status(code, message);
    }
    
    // 常用错误快捷构造
    static Status InvalidConfig(const std::string& msg = "") {
        return Status(ErrorCode::INVALID_CONFIG, msg);
    }
    static Status FileNotFound(const std::string& path = "") {
        return Status(ErrorCode::FILE_NOT_FOUND, 
            path.empty() ? "File not found" : "File not found: " + path);
    }
    static Status InsufficientData(const std::string& msg = "") {
        return Status(ErrorCode::INSUFFICIENT_DATA, msg);
    }
    static Status CalibrationFailed(const std::string& msg = "") {
        return Status(ErrorCode::CALIBRATION_FAILED, msg);
    }
    static Status NumericalError(const std::string& msg = "") {
        return Status(ErrorCode::NUMERICAL_ERROR, msg);
    }
    static Status NotImplemented(const std::string& msg = "") {
        return Status(ErrorCode::UNIMPLEMENTED, msg);
    }
    
    // ─────────────────────────────────────────────────────────────
    // 查询方法
    // ─────────────────────────────────────────────────────────────
    
    bool ok() const { return code_ == ErrorCode::SUCCESS; }
    bool IsSuccess() const { return ok(); }
    explicit operator bool() const { return ok(); }
    
    ErrorCode code() const { return code_; }
    const std::string& message() const { return message_; }
    
    std::string toString() const {
        if (ok()) {
            return "OK";
        }
        std::ostringstream oss;
        oss << "[" << errorCodeName(code_) << "] ";
        if (!message_.empty()) {
            oss << message_;
        } else {
            oss << errorCodeDescription(code_);
        }
        if (context_) {
            oss << " (context: " << *context_ << ")";
        }
        return oss.str();
    }
    
    // ─────────────────────────────────────────────────────────────
    // 错误链支持
    // ─────────────────────────────────────────────────────────────
    
    Status& WithContext(const std::string& ctx) {
        context_ = ctx;
        return *this;
    }
    
    Status& WithSourceLocation(const char* file, int line, const char* func) {
        std::ostringstream oss;
        if (context_) {
            oss << *context_ << " ";
        }
        oss << "at " << file << ":" << line << " in " << func;
        context_ = oss.str();
        return *this;
    }
    
    // ─────────────────────────────────────────────────────────────
    // 比较操作
    // ─────────────────────────────────────────────────────────────
    
    bool operator==(const Status& other) const {
        return code_ == other.code_;
    }
    bool operator!=(const Status& other) const {
        return code_ != other.code_;
    }
    bool operator==(ErrorCode code) const {
        return code_ == code;
    }
    bool operator!=(ErrorCode code) const {
        return code_ != code;
    }
    
    // ─────────────────────────────────────────────────────────────
    // 转换
    // ─────────────────────────────────────────────────────────────
    
    std::string ToJson() const {
        std::ostringstream oss;
        oss << "{\"code\":" << static_cast<int>(code_)
            << ",\"name\":\"" << errorCodeName(code_) << "\""
            << ",\"ok\":" << (ok() ? "true" : "false")
            << ",\"message\":\"" << message_ << "\"";
        if (context_) {
            oss << ",\"context\":\"" << *context_ << "\"";
        }
        oss << "}";
        return oss.str();
    }

private:
    ErrorCode code_;
    std::string message_;
    std::optional<std::string> context_;
};

// ===================================================================
// Result<T> — 带返回值的状态封装
// ===================================================================
template<typename T>
class Result {
public:
    // 成功构造
    Result(T value) : status_(), value_(std::move(value)) {}  // NOLINT
    
    // 错误构造
    Result(Status status) : status_(std::move(status)) {}  // NOLINT
    Result(ErrorCode code, const std::string& message = "")
        : status_(code, message) {}
    
    // 查询
    bool ok() const { return status_.ok(); }
    bool IsSuccess() const { return ok(); }
    explicit operator bool() const { return ok(); }
    
    const Status& status() const { return status_; }
    const T& value() const& {
        if (!ok()) {
            throw std::runtime_error("Attempted to get value from failed Result: " + 
                                    status_.toString());
        }
        return *value_;
    }
    T&& value() && {
        if (!ok()) {
            throw std::runtime_error("Attempted to get value from failed Result: " + 
                                    status_.toString());
        }
        return std::move(*value_);
    }
    
    // 带默认值的访问
    const T& value_or(const T& default_val) const& {
        return ok() ? *value_ : default_val;
    }
    T value_or(T&& default_val) && {
        return ok() ? std::move(*value_) : std::move(default_val);
    }
    
    // 转换
    std::string toString() const {
        if (ok()) {
            return "OK";
        }
        return status_.toString();
    }

private:
    Status status_;
    std::optional<T> value_;
};

// ===================================================================
// 便利宏
// ===================================================================

#define UNICALIB_RETURN_IF_ERROR(expr) \
    do { \
        ::ns_unicalib::Status _status = (expr); \
        if (!_status.ok()) { \
            return _status; \
        } \
    } while(0)

#define UNICALIB_ASSIGN_OR_RETURN(var, expr) \
    auto _result_##var = (expr); \
    if (!_result_##var.ok()) { \
        return _result_##var.status(); \
    } \
    var = std::move(_result_##var.value())

#define UNICALIB_STATUS_ERROR(code, msg) \
    ::ns_unicalib::Status::Error(code, msg)

// 带源码位置的状态
#define UNICALIB_STATUS_WITH_LOCATION(code, msg) \
    ::ns_unicalib::Status(code, msg).WithSourceLocation(__FILE__, __LINE__, __func__)

}  // namespace ns_unicalib
