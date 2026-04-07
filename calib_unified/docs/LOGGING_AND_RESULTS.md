# UniCalib — 结果记录、异常处理与日志改进清单

本文档汇总：**哪些代码需要记录计算结果**、**哪些需要增加异常处理**、**哪些需要增加日志**，以及**编译/运行日志落盘**的约定。

---

## 0. 本次已完成的改动（摘要）

| 类别 | 改动 |
|------|------|
| **结果记录** | 五种标定精度已写入 CSV（`calib_accuracy_*.csv`），Pipeline 与 camera_intrinsic/imu_intrinsic 独立 app 均会追加 |
| **运行日志落盘** | camera_intrinsic、imu_intrinsic 由仅控制台改为同时写入 `output_dir/logs/<app>_<时间戳>.log`；其余 app 原本已写 logs |
| **异常与失败日志** | accuracy_logger 写入 CSV 失败时打 UNICALIB_WARN；report_gen generate_yaml 的 catch 中打 UNICALIB_WARN；lidar_camera 从系统配置读 ROS2 话题失败打 UNICALIB_DEBUG |
| **文件写入失败** | camera_intrinsic/imu_intrinsic 保存 YAML 前检查 `is_open()`，失败时 UNICALIB_ERROR |
| **编译日志** | build.sh 已确保 `REPO_LOGS_DIR` 存在并写入 `build_*.log`；成功时也打印“编译完整日志”路径 |
| **文档** | README 增加「编译与运行日志（统一落盘）」和「标定精度记录与曲线」说明 |

---

## 1. 还需记录计算结果的代码位置

以下位置**已有部分落盘**，可按需扩展为“每次结果都落盘”或“额外指标落盘”：

| 位置 | 当前行为 | 建议 |
|------|----------|------|
| **joint_calib_solver_engineered** `diagnostics_report.txt` | 仅当启用时写入 `output_dir/diagnostics_report.txt` | 已写；可考虑把每次运行的 summary（error_code、elapsed、各阶段成功与否）追加到同一 CSV，便于与精度 CSV 对齐分析 |
| **JointCalibSolver::save_results** | 写 `calibration_result.yaml`，失败抛异常 | 已记录；若需“运行摘要”可在此处追加一行到 `calib_accuracy_joint.csv`（若引入 joint 类型） |
| **CalibPipeline::save_extrinsic_result** | LiDAR-Cam 外参写 YAML | 已写；精度已通过 StageResult 写入 `calib_accuracy_lidar_cam_extrin.csv` |
| **iKalibr solver main** | `paramMagr->Save(...)`、`SaveByProductsToDisk()` | 非当前统一入口；若通过 joint 调用，结果由 JointCalibSolver 保存即可 |
| **ManualCalibSession** 手动校准结果 | 依赖各 app 调用 session 后写出的 YAML/配置 | 若有“手动精化前后 RMS”等指标，可考虑追加到对应外参 CSV |

**结论**：核心标定结果与精度 CSV 已覆盖；后续只需按需增加“联合标定摘要 CSV”或“手动校准前后对比”等扩展。

---

## 2. 还需增加异常处理的位置

| 位置 | 当前 | 建议 |
|------|------|------|
| **unicalib_legacy/data_manager.cpp** | 多处 `catch (...)` 仅做 fallback（如 timestamp 用索引），无日志 | 可加 `UNICALIB_DEBUG` 或 `LOG_WARNING` 说明“某字段解析失败，使用默认” |
| **unicalib_legacy/ikalibr_wrapper.cpp** | `catch (...)` 仅 `LOG_ERROR("iKalibr: failed to create " + tmpdir)` | 已有一条日志；可考虑在 rethrow 或返回前把异常类型/消息一并写入 |
| **unicalib_legacy/mias_lcec_wrapper.cpp** | `catch (...) { return std::nullopt; }` | 建议至少 `UNICALIB_WARN("MIAS-LCEC 调用失败")`，便于区分“无结果”与“异常” |
| **io/ros2_data_source.cpp** | 部分 `catch (...)` 仅设 `ts = ...` 无日志（如 947、975 行） | 可加 `UNICALIB_DEBUG("时间戳解析失败，使用序号")` |
| **pipeline/calib_pipeline.cpp** | 阶段内 `catch (...)` 已设置 `StageResult` 并打 UNICALIB_ERROR | 已足够；可选在 catch(...) 中增加 `UNICALIB_ERROR("未知异常")` 以明确区分 |
| **intrinsic/camera_pinhole_calib.cpp** | `catch (...) { UNICALIB_ERROR("鱼眼标定失败"); }` | 已有日志；可改为 `catch (const std::exception& e)` 并输出 e.what()，再保留 `catch(...)` |
| **joint_calib_solver_engineered.cpp** | `catch (...) { ... }` 未知异常分支 | 建议在该分支内增加 `UNICALIB_CRITICAL("联合标定未知异常")` 并可选 flush |

**结论**：关键路径（apps、pipeline、solver）已有 try/catch 与日志；legacy 与 io 中少量静默 `catch(...)` 建议补一条日志或 DEBUG。

---

## 3. 还需增加日志打印的位置

| 位置 | 建议 |
|------|------|
| **PipelineReport::save_report** | 已有一行 UNICALIB_INFO；可在写入前加 `UNICALIB_DEBUG("写入 pipeline 报告: {}", path)` |
| **各 app 启动时** | 已打印 banner；可统一加一行 `UNICALIB_INFO("output_dir={}", output_dir)` 和 `UNICALIB_INFO("日志文件: {}", log_file)`（部分已有） |
| **CalibPipeline::run 末尾** | 在 `append_stage_result_accuracy` 循环后加 `UNICALIB_DEBUG("已追加 {} 条精度记录到 CSV", count)` 可选 |
| **JointCalibSolver::phase3** | 已有 UNICALIB_INFO 的 solver summary；可对“回写外参”循环加 DEBUG 级别“已回写 key=…” |
| **load_imu_csv / YAML::LoadFile** | 失败时已有 ERROR 或异常；可在成功路径加 `UNICALIB_DEBUG("已加载 N 条 IMU / 配置键 …")` 便于排查配置 |
| **accuracy_logger** | 已对“无法写入 CSV”打 WARN；首次创建文件时可打 `UNICALIB_DEBUG("创建精度 CSV: {}", path)` |

**结论**：以 DEBUG 为主做补充即可，避免刷屏；关键错误路径已覆盖 INFO/WARN/ERROR。

---

## 4. 编译与运行日志统一落盘（约定）

- **编译日志**  
  - 目录：`<项目根>/logs/`（由 `build.sh` 的 `REPO_LOGS_DIR` 决定，默认 `calib_unified/../logs`）。  
  - 文件：`build_YYYYMMDD_HHMMSS.log`（完整 cmake 构建输出）。  
  - 可通过环境变量 `REPO_LOGS_DIR` 覆盖目录。

- **运行日志**  
  - 目录：`<output_dir>/logs/`（各 app 的 `output_dir` 下的 `logs` 子目录）。  
  - 文件：`<app>_YYYYMMDD_HHMMSS.log`（如 `camera_intrinsic_*.log`、`joint_calib_*.log`）。  
  - 可通过环境变量 `UNICALIB_LOGS_DIR` 覆盖（见 `logger.h` 中 `resolve_logs_dir`）。

- **流水线阶段日志**  
  - 同一 `output_dir/logs/` 下，由 `CalibPipeline` 按阶段写入，例如 `Fine-Auto_<task>_<ts>.log`。

- **Makefile**  
  - `LOGS_DIR` 默认 `$(WORKSPACE_DIR)/logs`；容器内运行可设 `UNICALIB_LOGS_DIR` 将运行日志写到指定目录。

以上约定已在 README 的「编译与运行日志（统一落盘）」中说明。

---

## 5. 小结

- **结果记录**：五种标定精度 CSV 已实现；其余为可选扩展（joint 摘要、手动校准对比等）。  
- **异常处理**：主流程已覆盖；建议在 legacy/io 的静默 `catch(...)` 处补至少一条日志。  
- **日志**：关键路径已有 INFO/WARN/ERROR；可按需在关键步骤补 DEBUG。  
- **编译/运行日志**：已统一落盘到指定目录，并已在 README 与 build 脚本中说明。
