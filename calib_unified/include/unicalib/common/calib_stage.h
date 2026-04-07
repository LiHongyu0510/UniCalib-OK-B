/**
 * UniCalib — 标定阶段标识（用于崩溃时日志定位）
 *
 * 在关键阶段入口设置 g_current_calib_stage，SIGSEGV/SIGABRT 等信号
 * 触发时 handler 可打出当前阶段，便于与分步日志配合定位崩溃点。
 */

#pragma once

namespace ns_unicalib {

/** 当前标定阶段名称（字符串字面量，如 "lidar_cam_fine_ceres"）。由各标定模块在进入/离开阶段时设置。 */
extern const char* g_current_calib_stage;

}  // namespace ns_unicalib
