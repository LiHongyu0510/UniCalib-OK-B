#pragma once
/**
 * 手动标定 Pangolin 左栏 UI 尺度（相对最初 180px 栏宽、18px 默认字体的累计放大）。
 * 栏宽 720 = 4×180。字号勿过大：Pangolin 内置字体纹理图集有限，过大（如 40+）会抛
 * "Unable to initialise font: run out of texture pixel space" 导致进程退出。
 */
namespace ns_unicalib {

inline constexpr int kManualPangolinPanelWidthPx = 720;
inline constexpr int kManualPangolinDefaultFontPx = 24;
inline constexpr int kManualPangolinPointSizeDefault = 4;
inline constexpr int kManualPangolinPointSizeMax = 24;

}  // namespace ns_unicalib
