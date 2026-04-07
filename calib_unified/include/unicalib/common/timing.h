/**
 * UniCalib Unified — 性能计时与性能分析工具
 * 
 * 功能：
 *   - 精确耗时测量（纳秒级）
 *   - RAII 自动计时作用域
 *   - 性能统计（均值、标准差、百分位数）
 *   - 性能热点分析
 *   - 周期性任务调度检测
 * 
 * 用法：
 *   // 简单计时
 *   UNICALIB_TIMER(my_func);
 *   myFunction();
 *   UNICALIB_TIMER_END(my_func);
 * 
 *   // RAII 作用域计时
 *   UNICALIB_SCOPED_TIMER("optimize");
 *   optimize();
 *   // 自动输出耗时
 * 
 *   // 性能统计
 *   TimerStats stats;
 *   for (int i = 0; i < 100; ++i) {
 *       stats.start();
 *       processFrame();
 *       stats.stop();
 *   }
 *   UNICALIB_INFO("Stats: {}", stats.summary());
 */

#pragma once

#include <chrono>
#include <deque>
#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <functional>
#include <memory>
#include <fmt/core.h>
#include <fmt/format.h>

namespace ns_unicalib {

// ===================================================================
// HighResolutionClock — 高精度时钟
// ===================================================================
using Clock = std::chrono::high_resolution_clock;
using TimePoint = Clock::time_point;
using Duration = Clock::duration;
using Nanoseconds = std::chrono::nanoseconds;
using Microseconds = std::chrono::microseconds;
using Milliseconds = std::chrono::milliseconds;
using Seconds = std::chrono::seconds;

// ===================================================================
// TimingUtils — 时间转换工具
// ===================================================================
namespace TimingUtils {

inline double toNanoseconds(Duration d) {
    return std::chrono::duration<double, std::nano>(d).count();
}

inline double toMicroseconds(Duration d) {
    return std::chrono::duration<double, std::micro>(d).count();
}

inline double toMilliseconds(Duration d) {
    return std::chrono::duration<double, std::milli>(d).count();
}

inline double toSeconds(Duration d) {
    return std::chrono::duration<double>(d).count();
}

inline std::string formatDuration(Duration d, int precision = 3) {
    double ns = toNanoseconds(d);
    if (ns < 1000.0) {
        return fmt::format("{:.{}f}ns", ns, precision);
    } else if (ns < 1'000'000.0) {
        return fmt::format("{:.{}f}μs", toMicroseconds(d), precision);
    } else if (ns < 1'000'000'000.0) {
        return fmt::format("{:.{}f}ms", toMilliseconds(d), precision);
    } else {
        return fmt::format("{:.{}f}s", toSeconds(d), precision);
    }
}

inline std::string formatRate(double ops_per_sec, const std::string& unit = "ops") {
    if (ops_per_sec < 1000.0) {
        return fmt::format("{:.2f} {}/s", ops_per_sec, unit);
    } else if (ops_per_sec < 1'000'000.0) {
        return fmt::format("{:.2f} K{}/s", ops_per_sec / 1000.0, unit);
    } else if (ops_per_sec < 1'000'000'000.0) {
        return fmt::format("{:.2f} M{}/s", ops_per_sec / 1'000'000.0, unit);
    } else {
        return fmt::format("{:.2f} G{}/s", ops_per_sec / 1'000'000'000.0, unit);
    }
}

}  // namespace TimingUtils

// ===================================================================
// ScopedTimer — RAII 作用域计时器
// ===================================================================
class ScopedTimer {
public:
    using Callback = std::function<void(double elapsed_ms)>;
    
    ScopedTimer(const std::string& name, Callback cb = nullptr)
        : name_(name),
          start_(Clock::now()),
          callback_(cb) {}
    
    ~ScopedTimer() {
        auto end = Clock::now();
        double elapsed_ms = TimingUtils::toMilliseconds(end - start_);
        
        if (callback_) {
            callback_(elapsed_ms);
        } else {
            // 默认输出日志
            if (elapsed_ms < 1.0) {
                // 小于 1ms 输出微秒
                UNICALIB_TRACE("[TIMER] {} took {:.3f} μs", 
                              name_, elapsed_ms * 1000.0);
            } else if (elapsed_ms < 1000.0) {
                // 小于 1s 输出毫秒
                UNICALIB_DEBUG("[TIMER] {} took {:.3f} ms", 
                              name_, elapsed_ms);
            } else {
                // 大于等于 1s 输出秒
                UNICALIB_INFO("[TIMER] {} took {:.3f} s", 
                             name_, elapsed_ms / 1000.0);
            }
        }
    }
    
    double elapsed() const {
        return TimingUtils::toMilliseconds(Clock::now() - start_);
    }

private:
    std::string name_;
    TimePoint start_;
    Callback callback_;
};

// ===================================================================
// TimerStats — 性能统计
// ===================================================================
class TimerStats {
public:
    TimerStats() = default;
    
    void start() {
        start_time_ = Clock::now();
    }
    
    void stop() {
        auto end_time = Clock::now();
        double elapsed_ns = TimingUtils::toNanoseconds(end_time - start_time_);
        samples_.push_back(elapsed_ns);
        total_ns_ += elapsed_ns;
        if (elapsed_ns < min_ns_) min_ns_ = elapsed_ns;
        if (elapsed_ns > max_ns_) max_ns_ = elapsed_ns;
    }
    
    // 执行并计时
    void measure(const std::function<void()>& func) {
        start();
        func();
        stop();
    }
    
    template<typename T>
    T measureReturn(const std::function<T()>& func) {
        start();
        T result = func();
        stop();
        return result;
    }
    
    // 统计信息
    size_t count() const { return samples_.size(); }
    double totalMs() const { return total_ns_ / 1e6; }
    double meanMs() const { 
        return count() > 0 ? totalMs() / count() : 0.0;
    }
    double minMs() const { return min_ns_ / 1e6; }
    double maxMs() const { return max_ns_ / 1e6; }
    
    // 标准差
    double stdDevMs() const {
        if (count() < 2) return 0.0;
        
        double mean_ns = total_ns_ / count();
        double variance = 0.0;
        for (double ns : samples_) {
            double diff = ns - mean_ns;
            variance += diff * diff;
        }
        variance /= (count() - 1);
        
        return std::sqrt(variance) / 1e6;
    }
    
    // 百分位数
    double percentileMs(double p) const {
        if (samples_.empty()) return 0.0;
        
        std::vector<double> sorted = samples_;
        std::sort(sorted.begin(), sorted.end());
        
        double idx = p / 100.0 * (sorted.size() - 1);
        size_t i0 = static_cast<size_t>(std::floor(idx));
        size_t i1 = static_cast<size_t>(std::ceil(idx));
        
        if (i0 >= sorted.size() - 1) {
            return sorted.back() / 1e6;
        }
        
        double v0 = sorted[i0] / 1e6;
        double v1 = sorted[i1] / 1e6;
        double t = idx - i0;
        
        return v0 + t * (v1 - v0);
    }
    
    // 吞吐率（假设每次计时是一个操作）
    double opsPerSecond() const {
        double total_s = total_ns_ / 1e9;
        return total_s > 0 ? count() / total_s : 0.0;
    }
    
    // 重置
    void reset() {
        samples_.clear();
        total_ns_ = 0.0;
        min_ns_ = std::numeric_limits<double>::max();
        max_ns_ = 0.0;
    }
    
    // 摘要字符串
    std::string summary() const {
        std::ostringstream oss;
        oss << "count=" << count()
            << ", mean=" << std::fixed << std::setprecision(3) << meanMs() << "ms"
            << ", std=" << stdDevMs() << "ms"
            << ", min=" << minMs() << "ms"
            << ", max=" << maxMs() << "ms"
            << ", p50=" << percentileMs(50) << "ms"
            << ", p95=" << percentileMs(95) << "ms"
            << ", p99=" << percentileMs(99) << "ms"
            << ", rate=" << std::setprecision(2) << opsPerSecond() << " ops/s";
        return oss.str();
    }
    
    // JSON 输出
    std::string toJson() const {
        std::ostringstream oss;
        oss << "{"
            << "\"count\":" << count() << ","
            << "\"mean_ms\":" << meanMs() << ","
            << "\"std_ms\":" << stdDevMs() << ","
            << "\"min_ms\":" << minMs() << ","
            << "\"max_ms\":" << maxMs() << ","
            << "\"p50_ms\":" << percentileMs(50) << ","
            << "\"p95_ms\":" << percentileMs(95) << ","
            << "\"p99_ms\":" << percentileMs(99) << ","
            << "\"ops_per_sec\":" << opsPerSecond()
            << "}";
        return oss.str();
    }

    friend class PerformanceProfiler;

private:
    std::vector<double> samples_;
    double total_ns_ = 0.0;
    double min_ns_ = std::numeric_limits<double>::max();
    double max_ns_ = 0.0;
    TimePoint start_time_;
};

// ===================================================================
// PerformanceProfiler — 全局性能分析器
// ===================================================================
class PerformanceProfiler {
public:
    static PerformanceProfiler& instance() {
        static PerformanceProfiler profiler;
        return profiler;
    }
    
    // 记录一个计时
    void record(const std::string& name, double elapsed_ms) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto& stats = profiles_[name];
        stats.samples_.push_back(elapsed_ms);
        stats.total_ms_ += elapsed_ms;
        if (elapsed_ms < stats.min_ms_) stats.min_ms_ = elapsed_ms;
        if (elapsed_ms > stats.max_ms_) stats.max_ms_ = elapsed_ms;
    }
    
    // 获取统计信息
    TimerStats getStats(const std::string& name) const {
        std::lock_guard<std::mutex> lock(mutex_);
        TimerStats stats;
        auto it = profiles_.find(name);
        if (it != profiles_.end()) {
            const auto& data = it->second;
            stats.samples_ = data.samples_;
            for (double ms : data.samples_) {
                stats.total_ns_ += ms * 1'000'000.0;
            }
            if (!stats.samples_.empty()) {
                stats.min_ns_ = data.min_ms_ * 1'000'000.0;
                stats.max_ns_ = data.max_ms_ * 1'000'000.0;
            }
        }
        return stats;
    }
    
    // 获取所有名称
    std::vector<std::string> getAllNames() const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<std::string> names;
        names.reserve(profiles_.size());
        for (const auto& [name, _] : profiles_) {
            names.push_back(name);
        }
        std::sort(names.begin(), names.end());
        return names;
    }
    
    // 打印报告
    void printReport() const {
        std::lock_guard<std::mutex> lock(mutex_);
        UNICALIB_INFO("\n========== Performance Profile Report ==========");
        for (const auto& [name, data] : profiles_) {
            UNICALIB_INFO("  {}: count={}, mean={:.3f}ms, max={:.3f}ms, total={:.3f}s",
                         name, data.samples_.size(),
                         data.total_ms_ / data.samples_.size(),
                         data.max_ms_,
                         data.total_ms_ / 1000.0);
        }
        UNICALIB_INFO("============================================\n");
    }
    
    // 清除所有记录
    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        profiles_.clear();
    }
    
    struct ProfileData {
        std::vector<double> samples_;
        double total_ms_ = 0.0;
        double min_ms_ = std::numeric_limits<double>::max();
        double max_ms_ = 0.0;
    };

private:
    PerformanceProfiler() = default;
    mutable std::mutex mutex_;
    std::map<std::string, ProfileData> profiles_;
};

// ===================================================================
// 性能分析 RAII 包装器
// ===================================================================
class ScopedProfile {
public:
    explicit ScopedProfile(const std::string& name)
        : name_(name),
          start_(Clock::now()) {}
    
    ~ScopedProfile() {
        auto elapsed_ms = TimingUtils::toMilliseconds(Clock::now() - start_);
        PerformanceProfiler::instance().record(name_, elapsed_ms);
    }

private:
    std::string name_;
    TimePoint start_;
};

// ===================================================================
// FPS 计算器
// ===================================================================
class FPSCounter {
public:
    FPSCounter(size_t window_size = 30) : window_size_(window_size) {}
    
    void tick() {
        auto now = Clock::now();
        if (last_time_.time_since_epoch().count() > 0) {
            double dt_ms = TimingUtils::toMilliseconds(now - last_time_);
            frame_times_.push_back(dt_ms);
            
            if (frame_times_.size() > window_size_) {
                frame_times_.pop_front();
            }
        }
        last_time_ = now;
        total_frames_++;
    }
    
    double fps() const {
        if (frame_times_.empty()) return 0.0;
        
        double avg_dt_ms = 0.0;
        for (double dt : frame_times_) {
            avg_dt_ms += dt;
        }
        avg_dt_ms /= frame_times_.size();
        
        return avg_dt_ms > 0 ? 1000.0 / avg_dt_ms : 0.0;
    }
    
    double frameTimeMs() const {
        if (frame_times_.empty()) return 0.0;
        
        double avg_dt_ms = 0.0;
        for (double dt : frame_times_) {
            avg_dt_ms += dt;
        }
        return avg_dt_ms / frame_times_.size();
    }
    
    size_t totalFrames() const { return total_frames_; }
    
    void reset() {
        frame_times_.clear();
        last_time_ = TimePoint();
        total_frames_ = 0;
    }

private:
    std::deque<double> frame_times_;
    TimePoint last_time_;
    size_t total_frames_ = 0;
    size_t window_size_;
};

}  // namespace ns_unicalib

// ===================================================================
// 全局宏定义
// ===================================================================

// 简单计时（手动结束）
#define UNICALIB_TIMER(name) \
    auto _timer_##name##_start = ::ns_unicalib::Clock::now()

#define UNICALIB_TIMER_END(name) \
    do { \
        auto _timer_##name##_end = ::ns_unicalib::Clock::now(); \
        double _timer_##name##_ms = \
            ::ns_unicalib::TimingUtils::toMilliseconds(_timer_##name##_end - _timer_##name##_start); \
        UNICALIB_DEBUG("[TIMER] {} took {:.3f} ms", #name, _timer_##name##_ms); \
    } while(0)

// RAII 作用域计时
#define UNICALIB_SCOPED_TIMER(name) \
    ::ns_unicalib::ScopedTimer _scoped_timer_##name(name)

// 性能分析
#define UNICALIB_SCOPED_PROFILE(name) \
    ::ns_unicalib::ScopedProfile _scoped_profile_##name(name)

// 性能统计
#define UNICALIB_MEASURE(stats, ...) \
    (stats).measure([&]() { __VA_ARGS__; })

#define UNICALIB_MEASURE_RETURN(stats, var, ...) \
    var = (stats).measureReturn([&]() { return (__VA_ARGS__); })

// FPS 计数器
#define UNICALIB_FPS_COUNTER(name, window_size) \
    ::ns_unicalib::FPSCounter _fps_counter_##name(window_size)

#define UNICALIB_FPS_TICK(name) \
    (_fps_counter_##name).tick()

#define UNICALIB_FPS_PRINT(name) \
    UNICALIB_INFO("[FPS] {}: {:.2f} fps ({:.3f} ms/frame)", \
                  #name, (_fps_counter_##name).fps(), \
                  (_fps_counter_##name).frameTimeMs())
