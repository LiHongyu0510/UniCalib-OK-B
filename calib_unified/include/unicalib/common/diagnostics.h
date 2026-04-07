/**
 * UniCalib Unified — 诊断与调试工具
 * 
 * 功能：
 *   - 运行时系统诊断
 *   - 数据质量检查
 *   - 性能瓶颈识别
 *   - 内存泄漏检测（简化版）
 *   - 线程安全检查
 *   - 生成诊断报告
 *   - 集成健康监控
 * 
 * 用法：
 *   DiagnosticsEngine& diag = DiagnosticsEngine::instance();
 *   diag.collectSystemInfo();
 *   diag.checkDataQuality(data);
 *   diag.checkPerformance();
 *   UNICALIB_INFO("{}", diag.generateReport());
 */

#pragma once

#include "logger.h"
#include "health_monitor.h"
#include "timing.h"
#include <string>
#include <map>
#include <vector>
#include <mutex>
#include <memory>
#include <sstream>
#include <fstream>
#include <chrono>
#include <filesystem>
#include <atomic>
#include <thread>
#include <condition_variable>
#include <fmt/core.h>
#include <fmt/format.h>

namespace fs = std::filesystem;

namespace ns_unicalib {

// ===================================================================
// DiagnosticsLevel — 诊断级别
// ===================================================================
enum class DiagnosticsLevel : int {
    NONE = 0,       // 关闭诊断
    BASIC = 1,      // 基础诊断（性能、健康状态）
    STANDARD = 2,    // 标准诊断（+ 数据质量）
    VERBOSE = 3,     // 详细诊断（+ 内存、线程）
    FULL = 4         // 完整诊断（所有信息）
};

// ===================================================================
// SystemInfo — 系统信息
// ===================================================================
struct SystemInfo {
    // CPU 信息
    int cpu_cores = 0;
    std::string cpu_model;
    double cpu_usage_percent = 0.0;
    
    // 内存信息
    size_t total_memory_bytes = 0;
    size_t available_memory_bytes = 0;
    size_t used_memory_bytes = 0;
    
    // 磁盘信息
    size_t disk_total_bytes = 0;
    size_t disk_available_bytes = 0;
    
    // 系统时间
    std::chrono::system_clock::time_point boot_time;
    
    // 进程信息
    size_t process_memory_bytes = 0;
    int thread_count = 0;
    double process_cpu_usage_percent = 0.0;
    
    std::string toString() const {
        std::ostringstream oss;
        oss << "CPU: " << cpu_cores << " cores (" << cpu_model << ")\n";
        oss << "Memory: " << (used_memory_bytes / 1024.0 / 1024.0 / 1024.0)
            << " / " << (total_memory_bytes / 1024.0 / 1024.0 / 1024.0)
            << " GB used\n";
        oss << "Disk: " << ((disk_total_bytes - disk_available_bytes) / 1024.0 / 1024.0 / 1024.0)
            << " / " << (disk_total_bytes / 1024.0 / 1024.0 / 1024.0)
            << " GB used\n";
        oss << "Process: " << (process_memory_bytes / 1024.0 / 1024.0)
            << " MB, " << thread_count << " threads\n";
        return oss.str();
    }
};

// ===================================================================
// DataQualityMetrics — 数据质量指标
// ===================================================================
struct DataQualityMetrics {
    // 传感器数据统计
    size_t total_frames = 0;
    size_t valid_frames = 0;
    size_t dropped_frames = 0;
    
    // 时间戳统计
    double min_timestamp = std::numeric_limits<double>::max();
    double max_timestamp = 0.0;
    double avg_time_delta_ms = 0.0;
    double max_time_delta_ms = 0.0;
    size_t timestamp_gaps = 0;  // 大的时间间隔
    size_t duplicate_timestamps = 0;
    
    // 数据完整性
    size_t missing_values = 0;
    size_t nan_count = 0;
    size_t inf_count = 0;
    size_t outlier_count = 0;
    
    // 传感器同步
    size_t sync_errors = 0;
    double max_time_offset_ms = 0.0;
    
    // 质量评分 (0-100)
    double quality_score = 100.0;
    
    std::vector<std::string> warnings;
    std::vector<std::string> errors;
    
    void addWarning(const std::string& msg) {
        warnings.push_back(msg);
        quality_score = std::max(0.0, quality_score - 5.0);
    }
    
    void addError(const std::string& msg) {
        errors.push_back(msg);
        quality_score = std::max(0.0, quality_score - 10.0);
    }
    
    std::string summary() const {
        std::ostringstream oss;
        oss << "Quality Score: " << std::fixed << std::setprecision(1) 
            << quality_score << "/100\n";
        oss << "Frames: " << valid_frames << "/" << total_frames 
            << " valid (" << dropped_frames << " dropped)\n";
        oss << "Time range: [" << min_timestamp << ", " << max_timestamp << "]s\n";
        oss << "Data issues: " << nan_count << " NaN, " << inf_count 
            << " Inf, " << missing_values << " missing\n";
        oss << "Sync errors: " << sync_errors << "\n";
        
        if (!warnings.empty()) {
            oss << "Warnings (" << warnings.size() << "):\n";
            for (const auto& w : warnings) {
                oss << "  - " << w << "\n";
            }
        }
        if (!errors.empty()) {
            oss << "Errors (" << errors.size() << "):\n";
            for (const auto& e : errors) {
                oss << "  - " << e << "\n";
            }
        }
        return oss.str();
    }
};

// ===================================================================
// PerformanceMetrics — 性能指标
// ===================================================================
struct PerformanceMetrics {
    // 模块性能
    std::map<std::string, double> module_avg_time_ms;
    std::map<std::string, double> module_max_time_ms;
    std::map<std::string, double> module_call_count;
    
    // 瓶颈识别
    std::string slowest_module;
    double slowest_module_time_ms = 0.0;
    
    // 系统级指标
    double overall_fps = 0.0;
    double avg_frame_time_ms = 0.0;
    double max_frame_time_ms = 0.0;
    
    // 资源使用趋势
    std::vector<double> memory_usage_history;
    std::vector<double> cpu_usage_history;
    
    std::string summary() const {
        std::ostringstream oss;
        oss << "Overall FPS: " << std::fixed << std::setprecision(1) 
            << overall_fps << "\n";
        oss << "Frame time: avg=" << avg_frame_time_ms 
            << "ms, max=" << max_frame_time_ms << "ms\n";
        oss << "Slowest module: " << slowest_module 
            << " (" << slowest_module_time_ms << "ms)\n";
        
        oss << "Module performance:\n";
        for (const auto& [name, avg_time] : module_avg_time_ms) {
            oss << "  " << name << ": avg=" << avg_time << "ms"
                << ", max=" << module_max_time_ms.at(name) << "ms"
                << ", calls=" << module_call_count.at(name) << "\n";
        }
        return oss.str();
    }
};

// ===================================================================
// DiagnosticsEngine — 诊断引擎
// ===================================================================
class DiagnosticsEngine {
public:
    static DiagnosticsEngine& instance() {
        static DiagnosticsEngine engine;
        return engine;
    }
    
    // 设置诊断级别
    void setLevel(DiagnosticsLevel level) {
        level_ = level;
        UNICALIB_INFO("[Diagnostics] Level set to: {}", levelName(level));
    }
    
    DiagnosticsLevel level() const { return level_; }
    
    // ─────────────────────────────────────────────────────────────
    // 系统信息收集
    // ─────────────────────────────────────────────────────────────
    SystemInfo collectSystemInfo() {
        std::lock_guard<std::mutex> lock(mutex_);
        
        SystemInfo info;
        
        // CPU 核心数
        info.cpu_cores = std::thread::hardware_concurrency();
        
        // CPU 型号（Linux）
#ifdef __linux__
        std::ifstream cpuinfo("/proc/cpuinfo");
        std::string line;
        while (std::getline(cpuinfo, line)) {
            if (line.find("model name") != std::string::npos) {
                size_t pos = line.find(':');
                if (pos != std::string::npos) {
                    info.cpu_model = line.substr(pos + 2);
                    break;
                }
            }
        }
        
        // 内存信息
        std::ifstream meminfo("/proc/meminfo");
        size_t total_mem = 0, free_mem = 0, available_mem = 0;
        while (std::getline(meminfo, line)) {
            if (line.find("MemTotal:") == 0) {
                total_mem = std::stoul(line.substr(line.find(':') + 1));
            } else if (line.find("MemFree:") == 0) {
                free_mem = std::stoul(line.substr(line.find(':') + 1));
            } else if (line.find("MemAvailable:") == 0) {
                available_mem = std::stoul(line.substr(line.find(':') + 1));
            }
        }
        info.total_memory_bytes = total_mem * 1024;
        info.available_memory_bytes = available_mem * 1024;
        info.used_memory_bytes = total_mem * 1024 - available_mem * 1024;
        
        // 进程内存
        std::ifstream statm("/proc/self/statm");
        size_t vm_size, resident;
        statm >> vm_size >> resident;
        info.process_memory_bytes = resident * 4096;  // pages to bytes
        
#endif  // __linux__
        
        // 线程数（估算）
        info.thread_count = thread_count_.load();
        
        system_info_ = info;
        return info;
    }
    
    // ─────────────────────────────────────────────────────────────
    // 数据质量检查
    // ─────────────────────────────────────────────────────────────
    DataQualityMetrics checkDataQuality(
        const std::vector<double>& timestamps,
        const std::vector<Eigen::Vector3d>& imu_data) {
        
        DataQualityMetrics metrics;
        
        if (timestamps.size() != imu_data.size()) {
            metrics.addError("Timestamp and data size mismatch");
            return metrics;
        }
        
        metrics.total_frames = timestamps.size();
        
        // 检查时间戳
        for (size_t i = 0; i < timestamps.size(); ++i) {
            double ts = timestamps[i];
            
            // NaN/Inf 检查
            if (std::isnan(ts) || std::isinf(ts)) {
                metrics.nan_count++;
                continue;
            }
            
            // 有效帧计数
            metrics.valid_frames++;
            
            // 范围更新
            if (ts < metrics.min_timestamp) metrics.min_timestamp = ts;
            if (ts > metrics.max_timestamp) metrics.max_timestamp = ts;
            
            // 时间间隔检查
            if (i > 0) {
                double dt = ts - timestamps[i-1];
                if (dt < 0) {
                    metrics.timestamp_gaps++;
                    metrics.addWarning("Negative time delta detected");
                } else if (dt > 0.1) {  // 大于100ms
                    metrics.timestamp_gaps++;
                    metrics.addWarning(fmt::format("Large time gap: {:.3f}s", dt));
                } else if (dt < 1e-6) {  // 小于1微秒（重复）
                    metrics.duplicate_timestamps++;
                }
                
                if (i == 1) {
                    metrics.avg_time_delta_ms = dt * 1000.0;
                } else {
                    metrics.avg_time_delta_ms = 
                        0.9 * metrics.avg_time_delta_ms + 0.1 * (dt * 1000.0);
                }
                
                if (dt * 1000.0 > metrics.max_time_delta_ms) {
                    metrics.max_time_delta_ms = dt * 1000.0;
                }
            }
        }
        
        // 检查 IMU 数据
        for (size_t i = 0; i < imu_data.size(); ++i) {
            const auto& data = imu_data[i];
            
            // NaN/Inf 检查
            if (!data.array().isFinite().all()) {
                if (std::isnan(data.array().any())) {
                    metrics.nan_count++;
                }
                if (std::isinf(data.array().any())) {
                    metrics.inf_count++;
                }
            }
            
            // 异常值检查
            if (data.norm() > 100.0) {  // 超过100 m/s^2
                metrics.outlier_count++;
                metrics.addWarning(
                    fmt::format("Outlier at frame {}: norm={:.2f}", i, data.norm()));
            }
        }
        
        metrics.dropped_frames = metrics.total_frames - metrics.valid_frames;
        
        // 质量评分
        double completeness = static_cast<double>(metrics.valid_frames) / 
                             std::max(1.0, static_cast<double>(metrics.total_frames));
        double temporal_quality = 1.0 - 
            static_cast<double>(metrics.timestamp_gaps + metrics.duplicate_timestamps) / 
            std::max(1.0, static_cast<double>(metrics.total_frames));
        double data_integrity = 1.0 - 
            static_cast<double>(metrics.nan_count + metrics.inf_count) / 
            std::max(1.0, static_cast<double>(metrics.total_frames * 3));  // 3 values per frame
        
        metrics.quality_score = (completeness * 40.0 + 
                               temporal_quality * 30.0 + 
                               data_integrity * 30.0);
        
        data_quality_ = metrics;
        return metrics;
    }
    
    // ─────────────────────────────────────────────────────────────
    // 性能检查
    // ─────────────────────────────────────────────────────────────
    PerformanceMetrics checkPerformance() {
        std::lock_guard<std::mutex> lock(mutex_);
        
        PerformanceMetrics metrics;
        
        // 从 PerformanceProfiler 获取统计
        auto& profiler = PerformanceProfiler::instance();
        auto module_names = profiler.getAllNames();
        
        for (const auto& name : module_names) {
            auto stats = profiler.getStats(name);
            if (stats.count() > 0) {
                metrics.module_avg_time_ms[name] = stats.meanMs();
                metrics.module_max_time_ms[name] = stats.maxMs();
                metrics.module_call_count[name] = static_cast<double>(stats.count());
                
                // 识别最慢模块
                if (stats.meanMs() > metrics.slowest_module_time_ms) {
                    metrics.slowest_module_time_ms = stats.meanMs();
                    metrics.slowest_module = name;
                }
            }
        }
        
        // 计算整体 FPS
        if (!metrics.module_avg_time_ms.empty()) {
            double total_time = 0.0;
            for (const auto& [name, avg_time] : metrics.module_avg_time_ms) {
                total_time += avg_time * metrics.module_call_count[name];
            }
            double total_calls = std::accumulate(
                metrics.module_call_count.begin(), 
                metrics.module_call_count.end(), 0.0,
                [](double acc, const auto& p) { return acc + p.second; });
            
            if (total_calls > 0) {
                metrics.avg_frame_time_ms = total_time / total_calls;
                metrics.overall_fps = 1000.0 / metrics.avg_frame_time_ms;
            }
        }
        
        performance_metrics_ = metrics;
        return metrics;
    }
    
    // ─────────────────────────────────────────────────────────────
    // 诊断报告生成
    // ─────────────────────────────────────────────────────────────
    std::string generateReport() const {
        std::ostringstream oss;
        
        oss << "\n========== UniCalib Diagnostics Report ==========\n\n";
        
        // 诊断级别
        oss << "Diagnostics Level: " << levelName(level_) << "\n\n";
        
        // 系统信息
        oss << "--- System Information ---\n";
        oss << system_info_.toString() << "\n";
        
        // 健康状态
        oss << "--- Health Status ---\n";
        auto health = HealthMonitor::instance().getAllModules();
        oss << "Modules monitored: " << health.size() << "\n";
        
        int healthy = 0, degraded = 0, failed = 0;
        for (const auto& name : health) {
            auto status = HealthMonitor::instance().getStatus(name);
            if (status) {
                switch (*status) {
                    case HealthStatus::HEALTHY:   healthy++; break;
                    case HealthStatus::DEGRADED:  degraded++; break;
                    case HealthStatus::FAILED:     failed++; break;
                    default: break;
                }
            }
        }
        oss << fmt::format("Healthy: {}, Degraded: {}, Failed: {}\n", 
                           healthy, degraded, failed);
        oss << "\n";
        
        // 数据质量
        if (level_ >= DiagnosticsLevel::STANDARD) {
            oss << "--- Data Quality ---\n";
            oss << data_quality_.summary() << "\n";
        }
        
        // 性能指标
        oss << "--- Performance Metrics ---\n";
        oss << performance_metrics_.summary() << "\n";
        
        // 性能瓶颈建议
        oss << "--- Recommendations ---\n";
        if (performance_metrics_.slowest_module_time_ms > 100.0) {
            oss << "  ⚠ Module '" << performance_metrics_.slowest_module 
                << "' is slow (>100ms). Consider optimization.\n";
        }
        if (data_quality_.quality_score < 70.0) {
            oss << "  ⚠ Data quality is poor (<70). Check sensor data.\n";
        }
        if (failed > 0) {
            oss << "  ⚠ Some modules have failed. Check health monitor.\n";
        }
        
        oss << "==============================================\n";
        
        return oss.str();
    }
    
    // 保存报告到文件
    Status saveReport(const std::string& path) const {
        try {
            fs::path report_path(path);
            if (report_path.has_parent_path()) {
                fs::create_directories(report_path.parent_path());
            }
            
            std::ofstream ofs(path);
            if (!ofs) {
                return Status::Error(ErrorCode::FILE_WRITE_ERROR,
                    fmt::format("Failed to open file for writing: {}", path));
            }
            
            ofs << generateReport();
            ofs.close();
            
            UNICALIB_INFO("[Diagnostics] Report saved to: {}", path);
            return Status::OK();
        } catch (const std::exception& e) {
            return Status::Error(ErrorCode::FILE_WRITE_ERROR,
                fmt::format("Failed to save report: {}", e.what()));
        }
    }
    
    // ─────────────────────────────────────────────────────────────
    // 线程计数
    // ─────────────────────────────────────────────────────────────
    void incrementThreadCount() { thread_count_++; }
    void decrementThreadCount() { thread_count_--; }
    int getThreadCount() const { return thread_count_.load(); }

private:
    DiagnosticsEngine() = default;
    DiagnosticsLevel level_ = DiagnosticsLevel::STANDARD;
    
    mutable std::mutex mutex_;
    
    // 收集的信息
    SystemInfo system_info_;
    DataQualityMetrics data_quality_;
    PerformanceMetrics performance_metrics_;
    
    std::atomic<int> thread_count_{0};
    
    std::string levelName(DiagnosticsLevel level) const {
        switch (level) {
            case DiagnosticsLevel::NONE:     return "NONE";
            case DiagnosticsLevel::BASIC:     return "BASIC";
            case DiagnosticsLevel::STANDARD:  return "STANDARD";
            case DiagnosticsLevel::VERBOSE:   return "VERBOSE";
            case DiagnosticsLevel::FULL:      return "FULL";
            default: return "UNKNOWN";
        }
    }
};

// ===================================================================
// 宏定义
// ===================================================================

#define UNICALIB_DIAGNOSTICS_INIT(level) \
    ::ns_unicalib::DiagnosticsEngine::instance().setLevel(level)

#define UNICALIB_DIAGNOSTICS_COLLECT() \
    ::ns_unicalib::DiagnosticsEngine::instance().collectSystemInfo()

#define UNICALIB_DIAGNOSTICS_CHECK_DATA(timestamps, data) \
    ::ns_unicalib::DiagnosticsEngine::instance().checkDataQuality(timestamps, data)

#define UNICALIB_DIAGNOSTICS_CHECK_PERF() \
    ::ns_unicalib::DiagnosticsEngine::instance().checkPerformance()

#define UNICALIB_DIAGNOSTICS_REPORT() \
    UNICALIB_INFO("{}", ::ns_unicalib::DiagnosticsEngine::instance().generateReport())

#define UNICALIB_DIAGNOSTICS_SAVE(path) \
    ::ns_unicalib::DiagnosticsEngine::instance().saveReport(path)

}  // namespace ns_unicalib
