/**
 * UniCalib C++ — 统一异常处理和错误码体系
 * 提供完整的异常分类、错误码定义和上下文信息
 */
#pragma once

#include <string>
#include <stdexcept>
#include <functional>
#include <unordered_map>

namespace unicalib {

// 前向声明枚举类型
enum class ErrorCode;
enum class ErrorSeverity;

// 前向声明工具函数（必须在 ErrorContext 构造函数之前声明）
std::string error_code_to_string(ErrorCode code);

/**
 * 错误码分类 - 用于错误处理和日志
 */
enum class ErrorCode {
  // 成功
  OK = 0,

  // 配置错误 (1000-1999)
  CONFIG_FILE_NOT_FOUND = 1001,
  CONFIG_PARSE_ERROR = 1002,
  CONFIG_INVALID_VALUE = 1003,
  CONFIG_MISSING_REQUIRED_FIELD = 1004,

  // 数据加载错误 (2000-2999)
  DATA_FILE_NOT_FOUND = 2001,
  DATA_DIRECTORY_NOT_FOUND = 2002,
  DATA_FORMAT_INVALID = 2003,
  DATA_INSUFFICIENT_SAMPLES = 2004,
  DATA_TIMESTAMP_INVALID = 2005,
  DATA_CORRUPTED = 2006,

  // 数据验证错误 (3000-3999)
  DATA_QUALITY_TOO_LOW = 3001,
  DATA_OUT_OF_RANGE = 3002,
  DATA_MISSING_SENSOR = 3003,
  DATA_RATE_INVALID = 3004,
  DATA_STATIC_INSUFFICIENT = 3005,

  // 标定计算错误 (4000-4999)
  CALIBRATION_FAILED = 4001,
  CALIBRATION_NOT_CONVERGED = 4002,
  CALIBRATION_SINGULAR_MATRIX = 4003,
  CALIBRATION_INVALID_RESULT = 4004,
  CALIBRATION_TIMEOUT = 4005,
  ALLAN_VARIANCE_FAILED = 4006,
  SIX_POSITION_FAILED = 4007,

  // 外参标定错误 (5000-5999)
  EXTRINSIC_MATCH_FAILED = 5001,
  EXTRINSIC_NO_FEATURES = 5002,
  EXTRINSIC_POOR_INITIALIZATION = 5003,
  EXTRINSIC_VALIDATION_FAILED = 5004,

  // 系统错误 (6000-6999)
  SYSTEM_INIT_FAILED = 6001,
  SYSTEM_INVALID_STATE = 6002,
  SYSTEM_RESOURCE_EXHAUSTED = 6003,

  // IO错误 (7000-7999)
  IO_WRITE_FAILED = 7001,
  IO_READ_FAILED = 7002,
  IO_PERMISSION_DENIED = 7003,
  IO_DISK_FULL = 7004,

  // 未知错误
  UNKNOWN = 9999
};

/**
 * 错误严重等级 - 用于日志和告警
 */
enum class ErrorSeverity {
  INFO = 0,      // 信息性，不影响运行
  WARNING = 1,   // 警告，可能影响结果但不致命
  ERROR = 2,     // 错误，当前操作失败
  CRITICAL = 3,  // 严重错误，系统无法继续运行
  FATAL = 4      // 致命错误，需要立即终止
};

/**
 * 错误上下文信息 - 提供详细的调试信息
 */
struct ErrorContext {
  ErrorCode code;
  ErrorSeverity severity;
  std::string message;
  std::string component;      // 模块名，如 "DataManager", "IMUCalibrator"
  std::string operation;      // 操作名，如 "load_imu_data", "allan_analysis"
  std::string sensor_id;      // 相关传感器ID
  std::string detail;         // 额外详细信息
  std::string suggestion;     // 修复建议
  int error_code_value;       // 错误码数值
  std::string error_code_name; // 错误码名称

  ErrorContext() : code(ErrorCode::OK), severity(ErrorSeverity::INFO), 
                    error_code_value(0) {}

  ErrorContext(ErrorCode ec, ErrorSeverity sev, const std::string& msg)
    : code(ec), severity(sev), message(msg), error_code_value(static_cast<int>(ec)),
      error_code_name(error_code_to_string(ec)) {}

  ErrorContext(ErrorCode ec, ErrorSeverity sev, const std::string& comp,
               const std::string& op, const std::string& msg)
    : code(ec), severity(sev), message(msg), component(comp), operation(op),
      error_code_value(static_cast<int>(ec)), error_code_name(error_code_to_string(ec)) {}

  ErrorContext& with_component(const std::string& comp) {
    component = comp;
    return *this;
  }

  ErrorContext& with_operation(const std::string& op) {
    operation = op;
    return *this;
  }

  ErrorContext& with_sensor(const std::string& sid) {
    sensor_id = sid;
    return *this;
  }

  ErrorContext& with_detail(const std::string& det) {
    detail = det;
    return *this;
  }

  ErrorContext& with_suggestion(const std::string& sug) {
    suggestion = sug;
    return *this;
  }

  std::string to_string() const;
};

/**
 * UniCalib异常基类 - 所有自定义异常的基类
 */
class UniCalibException : public std::exception {
 public:
  explicit UniCalibException(const ErrorContext& ctx);
  explicit UniCalibException(ErrorCode code, const std::string& message,
                              ErrorSeverity severity = ErrorSeverity::ERROR);
  explicit UniCalibException(ErrorCode code, const std::string& component,
                              const std::string& operation, const std::string& message);

  const char* what() const noexcept override;
  ErrorCode error_code() const { return context_.code; }
  ErrorSeverity severity() const { return context_.severity; }
  const ErrorContext& context() const { return context_; }
  const std::string& component() const { return context_.component; }
  const std::string& operation() const { return context_.operation; }
  const std::string& sensor_id() const { return context_.sensor_id; }

  virtual bool is_recoverable() const { return context_.severity < ErrorSeverity::CRITICAL; }

 protected:
  ErrorContext context_;
  mutable std::string what_message_;
};

/**
 * 配置异常
 */
class ConfigException : public UniCalibException {
 public:
  explicit ConfigException(ErrorCode code, const std::string& message);
  explicit ConfigException(ErrorCode code, const std::string& operation,
                           const std::string& message);
};

/**
 * 数据加载异常
 */
class DataException : public UniCalibException {
 public:
  explicit DataException(ErrorCode code, const std::string& message);
  explicit DataException(ErrorCode code, const std::string& sensor_id,
                         const std::string& message);
};

/**
 * 数据验证异常
 */
class ValidationException : public UniCalibException {
 public:
  explicit ValidationException(ErrorCode code, const std::string& message);
  explicit ValidationException(ErrorCode code, const std::string& metric_name,
                               double actual_value, double expected_value);
};

/**
 * 标定计算异常
 */
class CalibrationException : public UniCalibException {
 public:
  explicit CalibrationException(ErrorCode code, const std::string& message);
  explicit CalibrationException(ErrorCode code, const std::string& operation,
                               const std::string& message);
};

/**
 * 外参标定异常
 */
class ExtrinsicException : public UniCalibException {
 public:
  explicit ExtrinsicException(ErrorCode code, const std::string& message);
  explicit ExtrinsicException(ErrorCode code, const std::string& pair_name,
                               const std::string& message);
};

/**
 * IO异常
 */
class IOException : public UniCalibException {
 public:
  explicit IOException(ErrorCode code, const std::string& filepath,
                       const std::string& message);
};

// 工具函数
std::string error_code_to_string(ErrorCode code);
ErrorSeverity get_error_severity(ErrorCode code);
bool is_recoverable_error(ErrorCode code);
std::string get_fix_suggestion(ErrorCode code);

// RAII异常捕获器 - 用于自动捕获和记录异常
class ExceptionGuard {
 public:
  ExceptionGuard(const std::string& scope, const std::function<void(const UniCalibException&)>& handler = nullptr);
  ~ExceptionGuard() noexcept;

 private:
  std::string scope_;
  std::function<void(const UniCalibException&)> handler_;
};

} // namespace unicalib
