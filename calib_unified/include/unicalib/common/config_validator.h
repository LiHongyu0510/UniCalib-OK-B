/**
 * UniCalib Unified — 配置验证器
 * 
 * 功能：
 *   - 配置参数校验
 *   - 范围检查
 *   - 必需参数检查
 *   - 依赖关系验证
 *   - 生成验证报告
 *   - 详细的错误消息
 * 
 * 用法：
 *   ConfigValidator validator;
 *   validator.addRequired("imu_topic");
 *   validator.addRange("imu_frequency", 100, 1000);  // Hz
 *   validator.addFile("config.yaml");
 *   Status result = validator.validate(config);
 *   if (!result.ok()) {
 *       UNICALIB_ERROR("Config validation failed: {}", result.message());
 *   }
 */

#pragma once

#include "status.h"
#include "exception.h"
#include <string>
#include <map>
#include <vector>
#include <functional>
#include <memory>
#include <filesystem>
#include <sstream>
#include <variant>
#include <fmt/core.h>
#include <fmt/format.h>
#include <algorithm>

namespace fs = std::filesystem;

namespace ns_unicalib {

// ===================================================================
// ParameterValue — 参数值（支持多种类型）
// ===================================================================
using ParameterValue = std::variant<
    bool,
    int,
    double,
    std::string,
    std::vector<int>,
    std::vector<double>,
    std::vector<std::string>
>;

// 类型检查辅助函数
inline bool isBool(const ParameterValue& v) {
    return std::holds_alternative<bool>(v);
}
inline bool isInt(const ParameterValue& v) {
    return std::holds_alternative<int>(v);
}
inline bool isDouble(const ParameterValue& v) {
    return std::holds_alternative<double>(v);
}
inline bool isString(const ParameterValue& v) {
    return std::holds_alternative<std::string>(v);
}
inline bool isIntList(const ParameterValue& v) {
    return std::holds_alternative<std::vector<int>>(v);
}
inline bool isDoubleList(const ParameterValue& v) {
    return std::holds_alternative<std::vector<double>>(v);
}
inline bool isStringList(const ParameterValue& v) {
    return std::holds_alternative<std::vector<std::string>>(v);
}

// 值转字符串
inline std::string paramToString(const ParameterValue& v) {
    if (isBool(v)) return std::get<bool>(v) ? "true" : "false";
    if (isInt(v)) return std::to_string(std::get<int>(v));
    if (isDouble(v)) {
        std::ostringstream oss;
        oss << std::get<double>(v);
        return oss.str();
    }
    if (isString(v)) return std::get<std::string>(v);
    if (isIntList(v)) {
        auto list = std::get<std::vector<int>>(v);
        std::string s = "[";
        for (size_t i = 0; i < list.size(); ++i) {
            s += std::to_string(list[i]);
            if (i < list.size() - 1) s += ", ";
        }
        s += "]";
        return s;
    }
    if (isDoubleList(v)) {
        auto list = std::get<std::vector<double>>(v);
        std::string s = "[";
        for (size_t i = 0; i < list.size(); ++i) {
            std::ostringstream oss;
            oss << list[i];
            s += oss.str();
            if (i < list.size() - 1) s += ", ";
        }
        s += "]";
        return s;
    }
    if (isStringList(v)) {
        auto list = std::get<std::vector<std::string>>(v);
        std::string s = "[";
        for (size_t i = 0; i < list.size(); ++i) {
            s += "\"" + list[i] + "\"";
            if (i < list.size() - 1) s += ", ";
        }
        s += "]";
        return s;
    }
    return "unknown";
}

// ===================================================================
// ValidationRule — 验证规则
// ===================================================================
class ValidationRule {
public:
    using ValidatorFunc = std::function<Status(const std::string& name, 
                                              const ParameterValue& value)>;
    
    explicit ValidationRule(const std::string& description)
        : description_(description) {}
    
    virtual ~ValidationRule() = default;
    
    virtual Status validate(const std::string& name, 
                           const ParameterValue& value) const = 0;
    
    virtual std::string description() const { return description_; }

protected:
    std::string description_;
};

// ─────────────────────────────────────────────────────────────────
// 必需参数规则
// ─────────────────────────────────────────────────────────────────
class RequiredRule : public ValidationRule {
public:
    RequiredRule() : ValidationRule("Parameter is required") {}
    
    Status validate(const std::string& name, 
                   const ParameterValue& value) const override {
        // 检查是否提供了值（这个检查在 ConfigValidator 中完成）
        return Status::OK();
    }
};

// ─────────────────────────────────────────────────────────────────
// 范围规则（数值）
// ─────────────────────────────────────────────────────────────────
class RangeRule : public ValidationRule {
public:
    RangeRule(double min_val, double max_val, bool inclusive = true)
        : ValidationRule(fmt::format("Must be in range [{}, {}]", min_val, max_val)),
          min_val_(min_val),
          max_val_(max_val),
          inclusive_(inclusive) {}
    
    Status validate(const std::string& name, 
                   const ParameterValue& value) const override {
        if (isInt(value)) {
            int v = std::get<int>(value);
            double d = static_cast<double>(v);
            if (inclusive_) {
                if (d < min_val_ || d > max_val_) {
                    return Status::Error(ErrorCode::PARAM_OUT_OF_RANGE,
                        fmt::format("Parameter '{}'={} is out of range [{}, {}]",
                                    name, v, min_val_, max_val_));
                }
            } else {
                if (d <= min_val_ || d >= max_val_) {
                    return Status::Error(ErrorCode::PARAM_OUT_OF_RANGE,
                        fmt::format("Parameter '{}'={} is out of range ({}, {})",
                                    name, v, min_val_, max_val_));
                }
            }
        } else if (isDouble(value)) {
            double v = std::get<double>(value);
            if (inclusive_) {
                if (v < min_val_ || v > max_val_) {
                    return Status::Error(ErrorCode::PARAM_OUT_OF_RANGE,
                        fmt::format("Parameter '{}'={} is out of range [{}, {}]",
                                    name, v, min_val_, max_val_));
                }
            } else {
                if (v <= min_val_ || v >= max_val_) {
                    return Status::Error(ErrorCode::PARAM_OUT_OF_RANGE,
                        fmt::format("Parameter '{}'={} is out of range ({}, {})",
                                    name, v, min_val_, max_val_));
                }
            }
        } else {
            return Status::Error(ErrorCode::INVALID_PARAM_VALUE,
                fmt::format("Parameter '{}' must be numeric, got {}", 
                            name, paramToString(value)));
        }
        return Status::OK();
    }

private:
    double min_val_;
    double max_val_;
    bool inclusive_;
};

// ─────────────────────────────────────────────────────────────────
// 正值规则
// ─────────────────────────────────────────────────────────────────
class PositiveRule : public ValidationRule {
public:
    PositiveRule() : ValidationRule("Must be positive") {}
    
    Status validate(const std::string& name, 
                   const ParameterValue& value) const override {
        if (isInt(value)) {
            int v = std::get<int>(value);
            if (v <= 0) {
                return Status::Error(ErrorCode::PARAM_OUT_OF_RANGE,
                    fmt::format("Parameter '{}'={} must be positive", name, v));
            }
        } else if (isDouble(value)) {
            double v = std::get<double>(value);
            if (v <= 0.0) {
                return Status::Error(ErrorCode::PARAM_OUT_OF_RANGE,
                    fmt::format("Parameter '{}'={} must be positive", name, v));
            }
        } else {
            return Status::Error(ErrorCode::INVALID_PARAM_VALUE,
                fmt::format("Parameter '{}' must be numeric, got {}", 
                            name, paramToString(value)));
        }
        return Status::OK();
    }
};

// ─────────────────────────────────────────────────────────────────
// 枚举值规则
// ─────────────────────────────────────────────────────────────────
class EnumRule : public ValidationRule {
public:
    explicit EnumRule(const std::vector<std::string>& allowed_values)
        : ValidationRule(fmt::format("Must be one of: {}", 
                                     fmt::join(allowed_values, ", "))),
          allowed_values_(allowed_values) {}
    
    Status validate(const std::string& name, 
                   const ParameterValue& value) const override {
        if (isString(value)) {
            std::string v = std::get<std::string>(value);
            auto it = std::find(allowed_values_.begin(), 
                               allowed_values_.end(), v);
            if (it == allowed_values_.end()) {
                return Status::Error(ErrorCode::INVALID_PARAM_VALUE,
                    fmt::format("Parameter '{}'='{}' must be one of: {}",
                                name, v, fmt::join(allowed_values_, ", ")));
            }
        } else {
            return Status::Error(ErrorCode::INVALID_PARAM_VALUE,
                fmt::format("Parameter '{}' must be a string, got {}",
                            name, paramToString(value)));
        }
        return Status::OK();
    }

private:
    std::vector<std::string> allowed_values_;
};

// ─────────────────────────────────────────────────────────────────
// 文件存在规则
// ─────────────────────────────────────────────────────────────────
class FileExistsRule : public ValidationRule {
public:
    FileExistsRule(bool must_exist = true)
        : ValidationRule(must_exist ? "File must exist" : "File must not exist"),
          must_exist_(must_exist) {}
    
    Status validate(const std::string& name, 
                   const ParameterValue& value) const override {
        if (isString(value)) {
            std::string v = std::get<std::string>(value);
            if (v.empty()) {
                return Status::Error(ErrorCode::INVALID_PARAM_VALUE,
                    fmt::format("Parameter '{}' is empty file path", name));
            }
            bool exists = fs::exists(v);
            if (must_exist_ && !exists) {
                return Status::Error(ErrorCode::FILE_NOT_FOUND,
                    fmt::format("File '{}' does not exist for parameter '{}'",
                                v, name));
            } else if (!must_exist_ && exists) {
                return Status::Error(ErrorCode::INVALID_CONFIG,
                    fmt::format("File '{}' already exists for parameter '{}'",
                                v, name));
            }
        } else {
            return Status::Error(ErrorCode::INVALID_PARAM_VALUE,
                fmt::format("Parameter '{}' must be a file path, got {}",
                            name, paramToString(value)));
        }
        return Status::OK();
    }

private:
    bool must_exist_;
};

// ─────────────────────────────────────────────────────────────────
// 目录存在规则
// ─────────────────────────────────────────────────────────────────
class DirectoryExistsRule : public ValidationRule {
public:
    DirectoryExistsRule(bool must_exist = true, bool create_if_missing = false)
        : ValidationRule(must_exist ? "Directory must exist" : "Directory must not exist"),
          must_exist_(must_exist),
          create_if_missing_(create_if_missing) {}
    
    Status validate(const std::string& name, 
                   const ParameterValue& value) const override {
        if (isString(value)) {
            std::string v = std::get<std::string>(value);
            if (v.empty()) {
                return Status::Error(ErrorCode::INVALID_PARAM_VALUE,
                    fmt::format("Parameter '{}' is empty directory path", name));
            }
            bool exists = fs::exists(v);
            
            if (create_if_missing_ && !exists) {
                try {
                    fs::create_directories(v);
                    UNICALIB_INFO("Created directory: {}", v);
                    return Status::OK();
                } catch (const fs::filesystem_error& e) {
                    return Status::Error(ErrorCode::FILE_WRITE_ERROR,
                        fmt::format("Failed to create directory '{}': {}", v, e.what()));
                }
            }
            
            if (must_exist_ && !exists) {
                return Status::Error(ErrorCode::FILE_NOT_FOUND,
                    fmt::format("Directory '{}' does not exist for parameter '{}'",
                                v, name));
            } else if (!must_exist_ && exists) {
                return Status::Error(ErrorCode::INVALID_CONFIG,
                    fmt::format("Directory '{}' already exists for parameter '{}'",
                                v, name));
            }
        } else {
            return Status::Error(ErrorCode::INVALID_PARAM_VALUE,
                fmt::format("Parameter '{}' must be a directory path, got {}",
                            name, paramToString(value)));
        }
        return Status::OK();
    }

private:
    bool must_exist_;
    bool create_if_missing_;
};

// ─────────────────────────────────────────────────────────────────
// 自定义验证规则
// ─────────────────────────────────────────────────────────────────
class CustomRule : public ValidationRule {
public:
    CustomRule(const std::string& description, ValidationRule::ValidatorFunc func)
        : ValidationRule(description), func_(func) {}
    
    Status validate(const std::string& name, 
                   const ParameterValue& value) const override {
        return func_(name, value);
    }

private:
    ValidatorFunc func_;
};

// ===================================================================
// ConfigValidator — 配置验证器
// ===================================================================
class ConfigValidator {
public:
    ConfigValidator() = default;
    
    // 添加验证规则
    void addRequired(const std::string& name) {
        rules_[name].push_back(std::make_unique<RequiredRule>());
        required_.insert(name);
    }
    
    void addRange(const std::string& name, double min_val, double max_val, 
                  bool inclusive = true) {
        rules_[name].push_back(
            std::make_unique<RangeRule>(min_val, max_val, inclusive));
    }
    
    void addPositive(const std::string& name) {
        rules_[name].push_back(std::make_unique<PositiveRule>());
    }
    
    void addEnum(const std::string& name, 
                  const std::vector<std::string>& allowed_values) {
        rules_[name].push_back(
            std::make_unique<EnumRule>(allowed_values));
    }
    
    void addFileExists(const std::string& name, bool must_exist = true) {
        rules_[name].push_back(std::make_unique<FileExistsRule>(must_exist));
    }
    
    void addDirectoryExists(const std::string& name, 
                          bool must_exist = true, 
                          bool create_if_missing = false) {
        rules_[name].push_back(
            std::make_unique<DirectoryExistsRule>(must_exist, create_if_missing));
    }
    
    void addCustom(const std::string& name, 
                   const std::string& description,
                   ValidationRule::ValidatorFunc func) {
        rules_[name].push_back(std::make_unique<CustomRule>(description, func));
    }
    
    // 验证配置
    Status validate(const std::map<std::string, ParameterValue>& config) const {
        std::vector<std::string> errors;
        
        // 检查必需参数
        for (const auto& required_name : required_) {
            if (config.find(required_name) == config.end()) {
                errors.push_back(
                    fmt::format("Missing required parameter: '{}'", required_name));
            }
        }
        
        // 检查各参数的规则
        for (const auto& [name, value] : config) {
            auto it = rules_.find(name);
            if (it != rules_.end()) {
                for (const auto& rule : it->second) {
                    Status result = rule->validate(name, value);
                    if (!result.ok()) {
                        errors.push_back(
                            fmt::format("  {}: {} ({})", name, result.message(),
                                       rule->description()));
                    }
                }
            }
        }
        
        if (!errors.empty()) {
            std::ostringstream oss;
            oss << "Configuration validation failed:\n";
            for (const auto& error : errors) {
                oss << error << "\n";
            }
            return Status::Error(ErrorCode::INVALID_CONFIG, oss.str());
        }
        
        return Status::OK();
    }
    
    // 生成验证报告（即使失败也返回详细信息）
    std::string report(const std::map<std::string, ParameterValue>& config) const {
        std::ostringstream oss;
        oss << "========== Configuration Validation Report ==========\n\n";
        
        int passed = 0, failed = 0;
        
        for (const auto& [name, value] : config) {
            oss << fmt::format("Parameter '{}': {}", name, paramToString(value));
            
            auto it = rules_.find(name);
            if (it != rules_.end() && !it->second.empty()) {
                bool all_passed = true;
                for (const auto& rule : it->second) {
                    Status result = rule->validate(name, value);
                    if (!result.ok()) {
                        all_passed = false;
                        oss << fmt::format(" ❌ FAILED: {}", result.message());
                        failed++;
                    }
                }
                if (all_passed) {
                    oss << " ✓ PASSED";
                    passed++;
                }
            } else {
                oss << " ⚠ No rules";
            }
            oss << "\n";
        }
        
        // 检查缺失的必需参数
        for (const auto& required_name : required_) {
            if (config.find(required_name) == config.end()) {
                oss << fmt::format("Parameter '{}': ❌ MISSING (required)\n", 
                                   required_name);
                failed++;
            }
        }
        
        oss << "\nSummary:\n";
        oss << fmt::format("  Passed:  {}\n", passed);
        oss << fmt::format("  Failed:  {}\n", failed);
        oss << fmt::format("  Total:   {}\n", passed + failed);
        oss << "==================================================\n";
        
        return oss.str();
    }
    
    // 清除所有规则
    void clear() {
        rules_.clear();
        required_.clear();
    }

private:
    std::map<std::string, std::vector<std::unique_ptr<ValidationRule>>> rules_;
    std::set<std::string> required_;
};

// ===================================================================
// 辅助函数：从 YAML 值创建 ParameterValue
// ===================================================================
#ifdef UNICALIB_USE_YAML_CPP
#include <yaml-cpp/yaml.h>

inline ParameterValue yamlToParam(const YAML::Node& node) {
    if (node.IsScalar()) {
        // 尝试解析为不同类型
        try {
            if (node.Tag() == "!!bool") {
                return node.as<bool>();
            }
            if (node.Tag() == "!!int") {
                return node.as<int>();
            }
            if (node.Tag() == "!!float") {
                return node.as<double>();
            }
            // 尝试 int
            try {
                return node.as<int>();
            } catch (...) {
                // 尝试 double
                try {
                    return node.as<double>();
                } catch (...) {
                    // 最后尝试 string
                    return node.as<std::string>();
                }
            }
        } catch (const YAML::BadConversion&) {
            return node.as<std::string>();
        }
    } else if (node.IsSequence()) {
        if (node.size() == 0) {
            return std::vector<int>{};
        }
        // 检查元素类型
        bool all_int = true;
        bool all_double = true;
        bool all_string = true;
        
        for (const auto& item : node) {
            if (!item.IsScalar()) {
                all_int = all_double = all_string = false;
                break;
            }
            try {
                item.as<int>();
            } catch (...) {
                all_int = false;
                try {
                    item.as<double>();
                } catch (...) {
                    all_double = false;
                }
            }
            if (all_string && item.Tag() != "!!str") {
                all_string = false;
            }
        }
        
        if (all_int) {
            std::vector<int> result;
            for (const auto& item : node) {
                result.push_back(item.as<int>());
            }
            return result;
        } else if (all_double) {
            std::vector<double> result;
            for (const auto& item : node) {
                result.push_back(item.as<double>());
            }
            return result;
        } else {
            std::vector<std::string> result;
            for (const auto& item : node) {
                result.push_back(item.as<std::string>());
            }
            return result;
        }
    }
    
    return std::string("");
}
#endif  // UNICALIB_USE_YAML_CPP

}  // namespace ns_unicalib
