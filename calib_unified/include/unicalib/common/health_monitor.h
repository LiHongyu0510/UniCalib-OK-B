/**
 * UniCalib Unified — 模块健康监控
 * 
 * 功能：
 *   - 模块状态监控（健康/降级/故障）
 *   - 心跳检测
 *   - 资源使用监控（CPU/内存）
 *   - 告警阈值管理
 *   - 健康报告生成
 *   - 降级策略触发
 * 
 * 用法：
 *   HealthMonitor& monitor = HealthMonitor::instance();
 *   monitor.registerModule("data_loader", HealthStatus::HEALTHY);
 *   monitor.checkHeartbeat("data_loader");
 *   monitor.setMemoryThreshold(8 * 1024 * 1024 * 1024); // 8GB
 *   if (monitor.isDegraded("data_loader")) {
 *       // 采取降级策略
 *   }
 */

#pragma once

#include <string>
#include <map>
#include <mutex>
#include <chrono>
#include <memory>
#include <functional>
#include <sstream>
#include <vector>
#include <atomic>
#include <fmt/core.h>
#include <fmt/format.h>

#include "logger.h"

namespace ns_unicalib {

// ===================================================================
// 健康状态枚举
// ===================================================================
enum class HealthStatus : int {
    HEALTHY = 0,        // 健康：正常运行
    DEGRADED = 1,       // 降级：功能受限但可用
    WARNING = 2,        // 警告：潜在问题
    CRITICAL = 3,       // 危急：严重问题
    FAILED = 4          // 故障：无法工作
};

inline std::string healthStatusName(HealthStatus status) {
    switch (status) {
        case HealthStatus::HEALTHY:   return "HEALTHY";
        case HealthStatus::DEGRADED:  return "DEGRADED";
        case HealthStatus::WARNING:    return "WARNING";
        case HealthStatus::CRITICAL:   return "CRITICAL";
        case HealthStatus::FAILED:     return "FAILED";
        default: return "UNKNOWN";
    }
}

// ===================================================================
// 模块信息
// ===================================================================
struct ModuleInfo {
    std::string name;
    HealthStatus status = HealthStatus::HEALTHY;
    std::string message;
    std::chrono::steady_clock::time_point last_heartbeat;
    std::chrono::steady_clock::time_point last_status_change;
    int heartbeat_count = 0;
    int error_count = 0;
    int warning_count = 0;
    
    // 统计信息
    double cpu_usage_percent = 0.0;
    size_t memory_usage_bytes = 0;
    size_t memory_peak_bytes = 0;
    
    void setStatus(HealthStatus new_status, const std::string& msg = "") {
        if (new_status != status) {
            status = new_status;
            last_status_change = std::chrono::steady_clock::now();
            message = msg;
            
            if (new_status == HealthStatus::CRITICAL || new_status == HealthStatus::FAILED) {
                error_count++;
            } else if (new_status == HealthStatus::WARNING) {
                warning_count++;
            }
        }
    }
};

// ===================================================================
// 告警配置
// ===================================================================
struct AlertConfig {
    size_t memory_threshold_bytes = 8ULL * 1024 * 1024 * 1024;  // 8GB
    double cpu_threshold_percent = 90.0;
    std::chrono::seconds heartbeat_timeout{30};  // 30秒超时
    int max_error_count = 10;  // 最大错误数
    int max_warning_count = 50;  // 最大警告数
};

// ===================================================================
// 健康监控器
// ===================================================================
class HealthMonitor {
public:
    using AlertCallback = std::function<void(const std::string& module, 
                                          HealthStatus status,
                                          const std::string& message)>;
    
    static HealthMonitor& instance() {
        static HealthMonitor monitor;
        return monitor;
    }
    
    // 注册模块
    void registerModule(const std::string& name) {
        std::lock_guard<std::mutex> lock(mutex_);
        modules_[name] = std::make_shared<ModuleInfo>();
        modules_[name]->name = name;
        modules_[name]->last_heartbeat = std::chrono::steady_clock::now();
        modules_[name]->last_status_change = std::chrono::steady_clock::now();
        
        UNICALIB_INFO("[HealthMonitor] Module '{}' registered", name);
    }
    
    // 注销模块
    void unregisterModule(const std::string& name) {
        std::lock_guard<std::mutex> lock(mutex_);
        modules_.erase(name);
        UNICALIB_INFO("[HealthMonitor] Module '{}' unregistered", name);
    }
    
    // 心跳
    void heartbeat(const std::string& name, HealthStatus status = HealthStatus::HEALTHY,
                  const std::string& message = "") {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = modules_.find(name);
        if (it == modules_.end()) {
            UNICALIB_WARN("[HealthMonitor] Heartbeat from unknown module '{}'", name);
            return;
        }
        
        auto& info = *it->second;
        info.last_heartbeat = std::chrono::steady_clock::now();
        info.heartbeat_count++;
        info.setStatus(status, message);
        info.cpu_usage_percent = estimateCPUUsage(name);
        info.memory_usage_bytes = estimateMemoryUsage(name);
        info.memory_peak_bytes = std::max(info.memory_peak_bytes, info.memory_usage_bytes);
        
        // 检查告警条件
        checkAlerts(name, info);
    }
    
    // 获取模块状态
    std::optional<HealthStatus> getStatus(const std::string& name) const {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = modules_.find(name);
        if (it != modules_.end()) {
            return it->second->status;
        }
        return std::nullopt;
    }
    
    // 获取模块信息
    std::shared_ptr<ModuleInfo> getModuleInfo(const std::string& name) const {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = modules_.find(name);
        if (it != modules_.end()) {
            return it->second;
        }
        return nullptr;
    }
    
    // 判断模块是否健康
    bool isHealthy(const std::string& name) const {
        auto status = getStatus(name);
        return status && *status == HealthStatus::HEALTHY;
    }
    
    // 判断模块是否降级
    bool isDegraded(const std::string& name) const {
        auto status = getStatus(name);
        return status && (*status == HealthStatus::DEGRADED || 
                          *status == HealthStatus::WARNING);
    }
    
    // 判断模块是否故障
    bool isFailed(const std::string& name) const {
        auto status = getStatus(name);
        return status && (*status == HealthStatus::CRITICAL || 
                          *status == HealthStatus::FAILED);
    }
    
    // 判断所有模块是否健康
    bool allHealthy() const {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto& [name, info] : modules_) {
            if (info->status != HealthStatus::HEALTHY) {
                return false;
            }
        }
        return true;
    }
    
    // 设置告警配置
    void setAlertConfig(const AlertConfig& config) {
        alert_config_ = config;
    }
    
    const AlertConfig& getAlertConfig() const {
        return alert_config_;
    }
    
    // 设置告警回调
    void setAlertCallback(AlertCallback callback) {
        alert_callback_ = callback;
    }
    
    // 获取所有模块名称
    std::vector<std::string> getAllModules() const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<std::string> names;
        names.reserve(modules_.size());
        for (const auto& [name, _] : modules_) {
            names.push_back(name);
        }
        return names;
    }
    
    // 健康检查（定期调用）
    void performHealthCheck() {
        std::lock_guard<std::mutex> lock(mutex_);
        auto now = std::chrono::steady_clock::now();
        
        for (auto& [name, info] : modules_) {
            // 检查心跳超时
            auto heartbeat_age = std::chrono::duration_cast<std::chrono::seconds>(
                now - info->last_heartbeat);
            
            if (heartbeat_age > alert_config_.heartbeat_timeout) {
                info->setStatus(HealthStatus::FAILED, 
                              fmt::format("Heartbeat timeout ({}s)", heartbeat_age.count()));
                
                UNICALIB_WARN("[HealthMonitor] Module '{}' heartbeat timeout: {}s", 
                             name, heartbeat_age.count());
                
                if (alert_callback_) {
                    alert_callback_(name, HealthStatus::FAILED, info->message);
                }
            }
            
            // 检查内存使用
            if (info->memory_usage_bytes > alert_config_.memory_threshold_bytes) {
                if (info->status == HealthStatus::HEALTHY) {
                    info->setStatus(HealthStatus::DEGRADED,
                                  fmt::format("High memory usage: {:.2f} GB",
                                             info->memory_usage_bytes / 1e9));
                    UNICALIB_WARN("[HealthMonitor] Module '{}' high memory usage: {:.2f} GB",
                                 name, info->memory_usage_bytes / 1e9);
                }
            }
            
            // 检查 CPU 使用
            if (info->cpu_usage_percent > alert_config_.cpu_threshold_percent) {
                if (info->status == HealthStatus::HEALTHY) {
                    info->setStatus(HealthStatus::DEGRADED,
                                  fmt::format("High CPU usage: {:.1f}%",
                                             info->cpu_usage_percent));
                    UNICALIB_WARN("[HealthMonitor] Module '{}' high CPU usage: {:.1f}%",
                                 name, info->cpu_usage_percent);
                }
            }
            
            // 检查错误计数
            if (info->error_count > alert_config_.max_error_count) {
                info->setStatus(HealthStatus::CRITICAL,
                              fmt::format("Too many errors: {}", info->error_count));
                UNICALIB_ERROR("[HealthMonitor] Module '{}' too many errors: {}",
                              name, info->error_count);
            }
            
            // 检查警告计数
            if (info->warning_count > alert_config_.max_warning_count) {
                info->setStatus(HealthStatus::WARNING,
                              fmt::format("Too many warnings: {}", info->warning_count));
                UNICALIB_WARN("[HealthMonitor] Module '{}' too many warnings: {}",
                             name, info->warning_count);
            }
        }
    }
    
    // 生成健康报告
    std::string generateReport() const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::ostringstream oss;
        
        oss << "\n========== Health Monitor Report ==========\n";
        oss << "Total modules: " << modules_.size() << "\n";
        
        int healthy = 0, degraded = 0, warning = 0, critical = 0, failed = 0;
        for (const auto& [name, info] : modules_) {
            switch (info->status) {
                case HealthStatus::HEALTHY:   healthy++; break;
                case HealthStatus::DEGRADED:  degraded++; break;
                case HealthStatus::WARNING:    warning++; break;
                case HealthStatus::CRITICAL:   critical++; break;
                case HealthStatus::FAILED:     failed++; break;
            }
        }
        
        oss << "Status summary:\n";
        oss << "  Healthy:   " << healthy << "\n";
        oss << "  Degraded:  " << degraded << "\n";
        oss << "  Warning:   " << warning << "\n";
        oss << "  Critical:  " << critical << "\n";
        oss << "  Failed:    " << failed << "\n\n";
        
        oss << "Module details:\n";
        for (const auto& [name, info] : modules_) {
            oss << "  " << name << ":\n";
            oss << "    Status:       " << healthStatusName(info->status) << "\n";
            oss << "    Message:      " << info->message << "\n";
            oss << "    Heartbeats:   " << info->heartbeat_count << "\n";
            oss << "    Errors:       " << info->error_count << "\n";
            oss << "    Warnings:     " << info->warning_count << "\n";
            oss << "    CPU usage:    " << info->cpu_usage_percent << "%\n";
            oss << "    Memory usage: " << info->memory_usage_bytes / 1024.0 / 1024.0 << " MB";
            if (info->memory_peak_bytes > 0) {
                oss << " (peak: " << info->memory_peak_bytes / 1024.0 / 1024.0 << " MB)";
            }
            oss << "\n";
        }
        oss << "==========================================\n";
        
        return oss.str();
    }
    
    // 重置模块统计
    void resetModuleStats(const std::string& name) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = modules_.find(name);
        if (it != modules_.end()) {
            it->second->error_count = 0;
            it->second->warning_count = 0;
            it->second->heartbeat_count = 0;
            UNICALIB_INFO("[HealthMonitor] Module '{}' stats reset", name);
        }
    }

private:
    HealthMonitor() = default;
    mutable std::mutex mutex_;
    std::map<std::string, std::shared_ptr<ModuleInfo>> modules_;
    AlertConfig alert_config_;
    AlertCallback alert_callback_;
    
    void checkAlerts(const std::string& name, ModuleInfo& info) {
        if (alert_callback_) {
            // 严重状态变更时触发回调
            if (info.status == HealthStatus::CRITICAL || 
                info.status == HealthStatus::FAILED) {
                alert_callback_(name, info.status, info.message);
            }
        }
    }
    
    // 估算 CPU 使用率（占位：平台相关实现可读 /proc/self/stat 或 GetProcessTimes 等）
    double estimateCPUUsage(const std::string& /*name*/) const {
        return 0.0;
    }
    
    // 估算内存使用（占位：平台相关实现可读 /proc/self/status 或 GetProcessMemoryInfo 等）
    size_t estimateMemoryUsage(const std::string& /*name*/) const {
        return 0;
    }
};

// ===================================================================
// 健康监控 RAII 辅助
// ===================================================================
class ModuleHealthGuard {
public:
    ModuleHealthGuard(const std::string& name)
        : name_(name) {
        HealthMonitor::instance().registerModule(name_);
    }
    
    ~ModuleHealthGuard() {
        HealthMonitor::instance().unregisterModule(name_);
    }
    
    void update(HealthStatus status = HealthStatus::HEALTHY,
                const std::string& message = "") {
        HealthMonitor::instance().heartbeat(name_, status, message);
    }

private:
    std::string name_;
};

// ===================================================================
// 宏定义
// ===================================================================

#define UNICALIB_HEALTH_MODULE(name) \
    ::ns_unicalib::ModuleHealthGuard _health_guard_##name(#name)

#define UNICALIB_HEALTHY(msg) \
    ::ns_unicalib::HealthMonitor::instance().heartbeat(__FILE__, \
        ::ns_unicalib::HealthStatus::HEALTHY, msg)

#define UNICALIB_HEALTH_DEGRADED(msg) \
    ::ns_unicalib::HealthMonitor::instance().heartbeat(__FILE__, \
        ::ns_unicalib::HealthStatus::DEGRADED, msg)

#define UNICALIB_HEALTH_WARNING(msg) \
    ::ns_unicalib::HealthMonitor::instance().heartbeat(__FILE__, \
        ::ns_unicalib::HealthStatus::WARNING, msg)

#define UNICALIB_HEALTH_CRITICAL(msg) \
    ::ns_unicalib::HealthMonitor::instance().heartbeat(__FILE__, \
        ::ns_unicalib::HealthStatus::CRITICAL, msg)

#define UNICALIB_HEALTH_FAILED(msg) \
    ::ns_unicalib::HealthMonitor::instance().heartbeat(__FILE__, \
        ::ns_unicalib::HealthStatus::FAILED, msg)

#define UNICALIB_HEALTH_CHECK() \
    ::ns_unicalib::HealthMonitor::instance().performHealthCheck()

#define UNICALIB_HEALTH_REPORT() \
    UNICALIB_INFO("{}", ::ns_unicalib::HealthMonitor::instance().generateReport())

}  // namespace ns_unicalib
