#pragma once
/**
 * UniCalib — 标定精度 CSV 记录
 *
 * 五种标定各对应一个 CSV 文件，便于记录历史运行精度并绘制曲线：
 *   - calib_accuracy_cam_intrinsic.csv
 *   - calib_accuracy_imu_intrinsic.csv
 *   - calib_accuracy_lidar_cam_extrin.csv
 *   - calib_accuracy_cam_cam_extrin.csv
 *   - calib_accuracy_imu_lidar_extrin.csv
 *
 * 使用脚本 scripts/plot_calib_accuracy.py 可读取 CSV 并生成曲线图。
 */

#include <string>
#include <map>
#include <vector>

namespace ns_unicalib {

/** 标定类型标识，与 pipeline 的 CalibTaskType 对应 */
enum class CalibAccuracyTask {
    CAM_INTRINSIC,
    IMU_INTRINSIC,
    LIDAR_CAM_EXTRIN,
    CAM_CAM_EXTRIN,
    IMU_LIDAR_EXTRIN,
};

/** 将 CalibAccuracyTask 转为 CSV 文件名后缀 */
std::string accuracy_task_to_suffix(CalibAccuracyTask task);

/**
 * 追加一条标定精度记录到对应 CSV。
 * 若文件不存在会先写表头；若存在则追加一行。
 *
 * @param output_dir  输出目录，CSV 写在该目录下
 * @param task        标定类型
 * @param success     是否成功
 * @param residual_rms 残差 RMS（像素或无量纲，失败时可填 -1）
 * @param elapsed_ms  耗时 [ms]
 * @param extra       各类型额外列（键为列名，值为字符串）
 *                    例如 cam_intrinsic: num_images
 *                         imu_intrinsic: noise_gyro, bias_instab_gyro, noise_accel, bias_instab_accel
 */
void append_calib_accuracy(
    const std::string& output_dir,
    CalibAccuracyTask task,
    bool success,
    double residual_rms,
    double elapsed_ms,
    const std::map<std::string, std::string>& extra = {});

/**
 * 便捷：从 pipeline 的 StageResult 写入一条记录（用于外参/流水线统一出口）。
 * 仅写通用列：timestamp, success, residual_rms, elapsed_ms, message_short。
 */
void append_stage_result_accuracy(
    const std::string& output_dir,
    CalibAccuracyTask task,
    bool success,
    double residual_rms,
    double elapsed_ms,
    const std::string& message_short = "");

}  // namespace ns_unicalib
