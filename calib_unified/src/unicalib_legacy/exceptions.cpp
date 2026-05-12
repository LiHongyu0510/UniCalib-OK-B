/**
 * UniCalib C++ — 异常处理实现
 */
#include "unicalib/exceptions.hpp"
#include <sstream>
#include <iostream>

namespace unicalib {

std::string ErrorContext::to_string() const {
  std::ostringstream oss;
  oss << "[" << error_code_name << "(" << error_code_value << ")] " << message;
  if (!component.empty()) oss << " | Component: " << component;
  if (!operation.empty()) oss << " | Operation: " << operation;
  if (!sensor_id.empty()) oss << " | Sensor: " << sensor_id;
  if (!detail.empty()) oss << " | Detail: " << detail;
  if (!suggestion.empty()) oss << " | Suggestion: " << suggestion;
  return oss.str();
}

UniCalibException::UniCalibException(const ErrorContext& ctx)
    : context_(ctx) {
  what_message_ = context_.to_string();
}

UniCalibException::UniCalibException(ErrorCode code, const std::string& message,
                                      ErrorSeverity severity)
    : context_(code, severity, message) {
  what_message_ = context_.to_string();
}

UniCalibException::UniCalibException(ErrorCode code, const std::string& component,
                                      const std::string& operation, const std::string& message)
    : context_(code, ErrorSeverity::ERROR, component, operation, message) {
  what_message_ = context_.to_string();
}

const char* UniCalibException::what() const noexcept {
  return what_message_.c_str();
}

ConfigException::ConfigException(ErrorCode code, const std::string& message)
    : UniCalibException(code, message, ErrorSeverity::ERROR) {
  context_.component = "Config";
}

ConfigException::ConfigException(ErrorCode code, const std::string& operation,
                                 const std::string& message)
    : UniCalibException(code, "Config", operation, message) {}

DataException::DataException(ErrorCode code, const std::string& message)
    : UniCalibException(code, message, ErrorSeverity::ERROR) {
  context_.component = "DataManager";
}

DataException::DataException(ErrorCode code, const std::string& sensor_id,
                             const std::string& message)
    : UniCalibException(code, message, ErrorSeverity::ERROR) {
  context_.component = "DataManager";
  context_.sensor_id = sensor_id;
}

ValidationException::ValidationException(ErrorCode code, const std::string& message)
    : UniCalibException(code, message, ErrorSeverity::WARNING) {
  context_.component = "DataValidator";
}

ValidationException::ValidationException(ErrorCode code, const std::string& metric_name,
                                         double actual_value, double expected_value)
    : UniCalibException(code, "", ErrorSeverity::WARNING) {
  context_.component = "DataValidator";
  std::ostringstream oss;
  oss << metric_name << " validation failed: actual=" << actual_value
      << ", expected=" << expected_value;
  context_.message = oss.str();
  context_.detail = "Metric value outside acceptable range";
}

CalibrationException::CalibrationException(ErrorCode code, const std::string& message)
    : UniCalibException(code, message, ErrorSeverity::ERROR) {
  context_.component = "Calibrator";
}

CalibrationException::CalibrationException(ErrorCode code, const std::string& operation,
                                           const std::string& message)
    : UniCalibException(code, "Calibrator", operation, message) {}

ExtrinsicException::ExtrinsicException(ErrorCode code, const std::string& message)
    : UniCalibException(code, message, ErrorSeverity::ERROR) {
  context_.component = "ExtrinsicCalibrator";
}

ExtrinsicException::ExtrinsicException(ErrorCode code, const std::string& pair_name,
                                        const std::string& message)
    : UniCalibException(code, message, ErrorSeverity::ERROR) {
  context_.component = "ExtrinsicCalibrator";
  context_.sensor_id = pair_name;
}

IOException::IOException(ErrorCode code, const std::string& filepath,
                         const std::string& message)
    : UniCalibException(code, message, ErrorSeverity::ERROR) {
  context_.component = "IO";
  context_.detail = "Filepath: " + filepath;
}

std::string error_code_to_string(ErrorCode code) {
  switch (code) {
    // 成功
    case ErrorCode::OK: return "OK";

    // 配置错误
    case ErrorCode::CONFIG_FILE_NOT_FOUND: return "CONFIG_FILE_NOT_FOUND";
    case ErrorCode::CONFIG_PARSE_ERROR: return "CONFIG_PARSE_ERROR";
    case ErrorCode::CONFIG_INVALID_VALUE: return "CONFIG_INVALID_VALUE";
    case ErrorCode::CONFIG_MISSING_REQUIRED_FIELD: return "CONFIG_MISSING_REQUIRED_FIELD";

    // 数据加载错误
    case ErrorCode::DATA_FILE_NOT_FOUND: return "DATA_FILE_NOT_FOUND";
    case ErrorCode::DATA_DIRECTORY_NOT_FOUND: return "DATA_DIRECTORY_NOT_FOUND";
    case ErrorCode::DATA_FORMAT_INVALID: return "DATA_FORMAT_INVALID";
    case ErrorCode::DATA_INSUFFICIENT_SAMPLES: return "DATA_INSUFFICIENT_SAMPLES";
    case ErrorCode::DATA_TIMESTAMP_INVALID: return "DATA_TIMESTAMP_INVALID";
    case ErrorCode::DATA_CORRUPTED: return "DATA_CORRUPTED";

    // 数据验证错误
    case ErrorCode::DATA_QUALITY_TOO_LOW: return "DATA_QUALITY_TOO_LOW";
    case ErrorCode::DATA_OUT_OF_RANGE: return "DATA_OUT_OF_RANGE";
    case ErrorCode::DATA_MISSING_SENSOR: return "DATA_MISSING_SENSOR";
    case ErrorCode::DATA_RATE_INVALID: return "DATA_RATE_INVALID";
    case ErrorCode::DATA_STATIC_INSUFFICIENT: return "DATA_STATIC_INSUFFICIENT";

    // 标定计算错误
    case ErrorCode::CALIBRATION_FAILED: return "CALIBRATION_FAILED";
    case ErrorCode::CALIBRATION_NOT_CONVERGED: return "CALIBRATION_NOT_CONVERGED";
    case ErrorCode::CALIBRATION_SINGULAR_MATRIX: return "CALIBRATION_SINGULAR_MATRIX";
    case ErrorCode::CALIBRATION_INVALID_RESULT: return "CALIBRATION_INVALID_RESULT";
    case ErrorCode::CALIBRATION_TIMEOUT: return "CALIBRATION_TIMEOUT";
    case ErrorCode::ALLAN_VARIANCE_FAILED: return "ALLAN_VARIANCE_FAILED";
    case ErrorCode::SIX_POSITION_FAILED: return "SIX_POSITION_FAILED";

    // 外参标定错误
    case ErrorCode::EXTRINSIC_MATCH_FAILED: return "EXTRINSIC_MATCH_FAILED";
    case ErrorCode::EXTRINSIC_NO_FEATURES: return "EXTRINSIC_NO_FEATURES";
    case ErrorCode::EXTRINSIC_POOR_INITIALIZATION: return "EXTRINSIC_POOR_INITIALIZATION";
    case ErrorCode::EXTRINSIC_VALIDATION_FAILED: return "EXTRINSIC_VALIDATION_FAILED";

    // 系统错误
    case ErrorCode::SYSTEM_INIT_FAILED: return "SYSTEM_INIT_FAILED";
    case ErrorCode::SYSTEM_INVALID_STATE: return "SYSTEM_INVALID_STATE";
    case ErrorCode::SYSTEM_RESOURCE_EXHAUSTED: return "SYSTEM_RESOURCE_EXHAUSTED";

    // IO错误
    case ErrorCode::IO_WRITE_FAILED: return "IO_WRITE_FAILED";
    case ErrorCode::IO_READ_FAILED: return "IO_READ_FAILED";
    case ErrorCode::IO_PERMISSION_DENIED: return "IO_PERMISSION_DENIED";
    case ErrorCode::IO_DISK_FULL: return "IO_DISK_FULL";

    default: return "UNKNOWN";
  }
}

ErrorSeverity get_error_severity(ErrorCode code) {
  switch (code) {
    // 可恢复的错误
    case ErrorCode::OK:
      return ErrorSeverity::INFO;
    case ErrorCode::DATA_QUALITY_TOO_LOW:
    case ErrorCode::DATA_OUT_OF_RANGE:
    case ErrorCode::CALIBRATION_NOT_CONVERGED:
      return ErrorSeverity::WARNING;

    // 错误但可处理
    case ErrorCode::CONFIG_FILE_NOT_FOUND:
    case ErrorCode::CONFIG_INVALID_VALUE:
    case ErrorCode::DATA_INSUFFICIENT_SAMPLES:
    case ErrorCode::DATA_RATE_INVALID:
    case ErrorCode::ALLAN_VARIANCE_FAILED:
    case ErrorCode::SIX_POSITION_FAILED:
    case ErrorCode::EXTRINSIC_NO_FEATURES:
    case ErrorCode::EXTRINSIC_POOR_INITIALIZATION:
      return ErrorSeverity::ERROR;

    // 严重错误
    case ErrorCode::CONFIG_PARSE_ERROR:
    case ErrorCode::CONFIG_MISSING_REQUIRED_FIELD:
    case ErrorCode::DATA_FILE_NOT_FOUND:
    case ErrorCode::DATA_DIRECTORY_NOT_FOUND:
    case ErrorCode::DATA_FORMAT_INVALID:
    case ErrorCode::DATA_CORRUPTED:
    case ErrorCode::CALIBRATION_FAILED:
    case ErrorCode::CALIBRATION_SINGULAR_MATRIX:
    case ErrorCode::CALIBRATION_INVALID_RESULT:
    case ErrorCode::EXTRINSIC_MATCH_FAILED:
    case ErrorCode::EXTRINSIC_VALIDATION_FAILED:
    case ErrorCode::IO_WRITE_FAILED:
    case ErrorCode::IO_READ_FAILED:
      return ErrorSeverity::CRITICAL;

    // 致命错误
    case ErrorCode::SYSTEM_INIT_FAILED:
    case ErrorCode::SYSTEM_INVALID_STATE:
    case ErrorCode::SYSTEM_RESOURCE_EXHAUSTED:
    case ErrorCode::CALIBRATION_TIMEOUT:
    case ErrorCode::IO_PERMISSION_DENIED:
    case ErrorCode::IO_DISK_FULL:
      return ErrorSeverity::FATAL;

    default:
      return ErrorSeverity::ERROR;
  }
}

bool is_recoverable_error(ErrorCode code) {
  ErrorSeverity severity = get_error_severity(code);
  return severity < ErrorSeverity::FATAL;
}

std::string get_fix_suggestion(ErrorCode code) {
  switch (code) {
    // 配置错误
    case ErrorCode::CONFIG_FILE_NOT_FOUND:
      return "检查配置文件路径是否正确，确认文件存在";
    case ErrorCode::CONFIG_PARSE_ERROR:
      return "检查YAML语法是否正确，使用YAML验证工具";
    case ErrorCode::CONFIG_INVALID_VALUE:
      return "检查配置参数是否在有效范围内";
    case ErrorCode::CONFIG_MISSING_REQUIRED_FIELD:
      return "补充缺失的必需字段，参考配置模板";

    // 数据加载错误
    case ErrorCode::DATA_FILE_NOT_FOUND:
      return "检查数据文件路径，确认文件存在且有读取权限";
    case ErrorCode::DATA_DIRECTORY_NOT_FOUND:
      return "检查数据目录路径，确认目录存在且有访问权限";
    case ErrorCode::DATA_FORMAT_INVALID:
      return "检查数据格式是否正确，参考数据格式文档";
    case ErrorCode::DATA_INSUFFICIENT_SAMPLES:
      return "增加数据采集时间或降低采样频率，确保数据量充足";
    case ErrorCode::DATA_TIMESTAMP_INVALID:
      return "检查时间戳格式，确保单调递增";
    case ErrorCode::DATA_CORRUPTED:
      return "数据文件损坏，请重新采集或从备份恢复";

    // 数据验证错误
    case ErrorCode::DATA_QUALITY_TOO_LOW:
      return "改善数据采集环境，减少干扰，重新采集数据";
    case ErrorCode::DATA_OUT_OF_RANGE:
      return "检查传感器是否正常工作，数据是否在合理范围内";
    case ErrorCode::DATA_MISSING_SENSOR:
      return "确认传感器配置正确，检查数据是否包含所有配置的传感器";
    case ErrorCode::DATA_RATE_INVALID:
      return "检查采样频率是否与传感器配置一致";
    case ErrorCode::DATA_STATIC_INSUFFICIENT:
      return "增加静态采集时间，确保有足够静态区段用于标定";

    // 标定计算错误
    case ErrorCode::CALIBRATION_FAILED:
      return "检查输入数据质量，尝试使用不同的初始化参数";
    case ErrorCode::CALIBRATION_NOT_CONVERGED:
      return "增加迭代次数，检查数据质量，尝试更好的初始化";
    case ErrorCode::CALIBRATION_SINGULAR_MATRIX:
      return "数据缺乏足够的激励，增加传感器运动幅度";
    case ErrorCode::CALIBRATION_INVALID_RESULT:
      return "标定结果不合理，检查数据和算法参数";
    case ErrorCode::CALIBRATION_TIMEOUT:
      return "标定超时，检查数据量或增加超时时间";
    case ErrorCode::ALLAN_VARIANCE_FAILED:
      return "检查IMU数据质量和采样率，确保有足够长度的静态数据";
    case ErrorCode::SIX_POSITION_FAILED:
      return "检查数据是否包含足够的静态区段，按照六面法要求重新采集";

    // 外参标定错误
    case ErrorCode::EXTRINSIC_MATCH_FAILED:
      return "检查传感器间是否有足够的公共特征，改善标定场景";
    case ErrorCode::EXTRINSIC_NO_FEATURES:
      return "增加场景中的特征点，改善光照和纹理";
    case ErrorCode::EXTRINSIC_POOR_INITIALIZATION:
      return "改善初始外参估计，使用更准确的初始值";
    case ErrorCode::EXTRINSIC_VALIDATION_FAILED:
      return "外参精度不足，重新标定或改善数据质量";

    // 系统错误
    case ErrorCode::SYSTEM_INIT_FAILED:
      return "检查系统配置和依赖，确保环境正确";
    case ErrorCode::SYSTEM_INVALID_STATE:
      return "重置系统状态，重新初始化";
    case ErrorCode::SYSTEM_RESOURCE_EXHAUSTED:
      return "释放系统资源，增加内存或磁盘空间";

    // IO错误
    case ErrorCode::IO_WRITE_FAILED:
      return "检查磁盘空间和写入权限，确认输出目录可写";
    case ErrorCode::IO_READ_FAILED:
      return "检查文件读取权限，确认文件未被占用";
    case ErrorCode::IO_PERMISSION_DENIED:
      return "检查文件和目录权限，确保有足够的访问权限";
    case ErrorCode::IO_DISK_FULL:
      return "清理磁盘空间或更换输出目录";

    default:
      return "参考日志和文档，联系技术支持";
  }
}

ExceptionGuard::ExceptionGuard(const std::string& scope,
                               const std::function<void(const UniCalibException&)>& handler)
    : scope_(scope), handler_(handler) {}

ExceptionGuard::~ExceptionGuard() noexcept {
  std::exception_ptr eptr = std::current_exception();
  if (eptr) {
    try {
      std::rethrow_exception(eptr);
    } catch (const UniCalibException& e) {
      std::cerr << "[ExceptionGuard] Caught exception in scope '" << scope_ << "': "
                << e.what() << std::endl;
      if (handler_) {
        handler_(e);
      }
    } catch (const std::exception& e) {
      std::cerr << "[ExceptionGuard] Caught standard exception in scope '" << scope_
                << "': " << e.what() << std::endl;
    } catch (...) {
      std::cerr << "[ExceptionGuard] Caught unknown exception in scope '" << scope_
                << "'" << std::endl;
    }
  }
}

} // namespace unicalib
