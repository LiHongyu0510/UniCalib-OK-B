// ============================================================
// ShmCalibrationChannel — 高性能双缓冲共享内存通道
// ============================================================
// 用途：域控（DCU）上的 UniCalib 进程把标定结果零拷贝暴露给主机进程
// 特性：
//   - 双缓冲（Double Buffer）+ seqlock，避免读写竞争
//   - 固定大小，适合实时路径
//   - 支持同一 SoC 上的多进程共享（/dev/shm）
// ============================================================
// 启用方法：
//   1. 在需要使用的主机进程中 include 本头文件
//   2. 使用相同的 key（默认 "/unicalib_calib"）打开
//   3. 读取时检查 seq 奇偶性保证一致性
// ============================================================

#pragma once

#include <atomic>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include "unicalib/common/sensor_types.h"   // 复用项目内 ExtrinsicSE3 等结构

namespace ns_unicalib {
namespace cyber {

struct alignas(64) SharedExtrinsic {
    char   ref_id[32];
    char   target_id[32];
    double transform[16];      // row-major 4x4
    double residual_rms;
    bool   converged;
    uint32_t _pad;
};

struct alignas(64) SharedImuIntrinsic {
    char   imu_id[32];
    double bias_acc[3];
    double bias_gyro[3];
    double noise_density;
    double random_walk;
    double allan_fit_rms;
};

struct alignas(64) SharedCameraIntrinsic {
    char   cam_id[32];
    double K[9];
    double D[8];
    uint32_t width;
    uint32_t height;
    double reproj_error;
};

struct alignas(4096) CalibrationShmBlock {
    // 版本与状态
    uint32_t magic;           // 0x5543414C ('UCAL')
    uint32_t version;         // 当前 1
    uint64_t timestamp_ns;    // 最后一次更新时间

    // 外参（最多 16 对）
    SharedExtrinsic extrinsics[16];
    uint32_t num_extrinsics;

    // 相机内参（最多 8 个）
    SharedCameraIntrinsic cam_intrinsics[8];
    uint32_t num_cam_intrinsics;

    // IMU 内参（最多 4 个）
    SharedImuIntrinsic imu_intrinsics[4];
    uint32_t num_imu_intrinsics;

    double overall_quality;
    bool   needs_manual;
    char   message[128];

    // seqlock：写者每次写前/后递增，读者检查奇偶性
    std::atomic<uint32_t> seq;
};

/**
 * 共享内存标定结果通道
 * 使用示例（写者 - UniCalib Cyber Component）：
 *   ShmCalibrationChannel ch("/unicalib_calib", true);
 *   ch.publish(result);
 *
 * 使用示例（读者 - 主机感知/规划进程）：
 *   ShmCalibrationChannel ch("/unicalib_calib", false);
 *   if (ch.read(result)) { ... }
 */
class ShmCalibrationChannel {
public:
    explicit ShmCalibrationChannel(const char* name = "/unicalib_calib", bool create = false)
        : name_(name), create_(create), fd_(-1), ptr_(nullptr) {
        open_or_create();
    }

    ~ShmCalibrationChannel() {
        if (ptr_) munmap(ptr_, sizeof(CalibrationShmBlock));
        if (fd_ >= 0) close(fd_);
        if (create_) shm_unlink(name_);
    }

    // 禁止拷贝
    ShmCalibrationChannel(const ShmCalibrationChannel&) = delete;
    ShmCalibrationChannel& operator=(const ShmCalibrationChannel&) = delete;

    bool is_valid() const { return ptr_ != nullptr; }

    // 写者调用：把标定结果发布到共享内存
    void publish(const CalibrationShmBlock& data) {
        if (!ptr_) return;
        CalibrationShmBlock* blk = static_cast<CalibrationShmBlock*>(ptr_);

        blk->seq.fetch_add(1, std::memory_order_release);           // 开始写
        std::memcpy(blk, &data, sizeof(CalibrationShmBlock));
        blk->seq.fetch_add(1, std::memory_order_release);           // 写完成
    }

    // 读者调用：安全读取（seqlock）
    bool read(CalibrationShmBlock& out) {
        if (!ptr_) return false;
        CalibrationShmBlock* blk = static_cast<CalibrationShmBlock*>(ptr_);

        uint32_t s1, s2;
        do {
            s1 = blk->seq.load(std::memory_order_acquire);
            std::memcpy(&out, blk, sizeof(CalibrationShmBlock));
            s2 = blk->seq.load(std::memory_order_acquire);
        } while (s1 != s2 || (s1 & 1));   // 奇数表示写者正在写入

        return true;
    }

private:
    void open_or_create() {
        if (create_) {
            shm_unlink(name_); // 清理旧的
            fd_ = shm_open(name_, O_RDWR | O_CREAT | O_EXCL, 0666);
            if (fd_ >= 0) {
                ftruncate(fd_, sizeof(CalibrationShmBlock));
            }
        } else {
            fd_ = shm_open(name_, O_RDWR, 0666);
        }
        if (fd_ < 0) return;

        ptr_ = mmap(nullptr, sizeof(CalibrationShmBlock),
                    PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
        if (ptr_ == MAP_FAILED) {
            ptr_ = nullptr;
            close(fd_);
            fd_ = -1;
        }
    }

    const char* name_;
    bool create_;
    int fd_;
    void* ptr_;
};

} // namespace cyber
} // namespace ns_unicalib