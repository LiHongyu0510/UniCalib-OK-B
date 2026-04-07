"""
UniCalib 系统主控调度器
"""
from __future__ import annotations
import logging
import os
import json
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import yaml

from .sensor_config import SensorConfig, SensorType, CalibPair, auto_infer_calib_pairs
from .calib_result import (
    CalibResult, CameraIntrinsic, FisheyeIntrinsic, IMUIntrinsic,
    IntrinsicResult, ValidationReport
)
from .data_manager import DataManager

logger = logging.getLogger(__name__)


class UniCalibSystem:
    """
    UniCalib 统一标定系统主控。

    执行顺序:
      Stage 1: 各传感器内参标定
      Stage 2: 基于学习的粗外参估计
      Stage 3: 基于优化的精外参标定
      Stage 4: 验证与质量评估
    """

    def __init__(self, config_path: str):
        self.config = self._load_config(config_path)
        self.sensors: Dict[str, SensorConfig] = {}
        self.calib_pairs: List[CalibPair] = []
        self.intrinsic_results: Dict[str, IntrinsicResult] = {}
        self.extrinsic_results: Dict[Tuple[str, str], CalibResult] = {}
        self.output_dir = Path(self.config.get("system", {}).get("output_dir", "./calib_results"))
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self._parse_config()
        self._init_modules()

    # ------------------------------------------------------------------
    # 初始化
    # ------------------------------------------------------------------

    def _load_config(self, path: str) -> dict:
        with open(path, "r") as f:
            return yaml.safe_load(f)

    def _parse_config(self):
        """解析传感器配置，自动推断标定对。"""
        for s_cfg in self.config.get("sensors", []):
            sensor = SensorConfig(**s_cfg)
            self.sensors[sensor.sensor_id] = sensor
            logger.info(f"  Sensor registered: {sensor.sensor_id} ({sensor.sensor_type.value})")

        # 自动推断标定对（按优先级排序）
        self.calib_pairs = auto_infer_calib_pairs(self.sensors)
        logger.info(f"  Inferred {len(self.calib_pairs)} calibration pairs:")
        for p in self.calib_pairs:
            logger.info(f"    [{p.sensor_a}] <-> [{p.sensor_b}]  "
                        f"coarse={p.method_coarse}  fine={p.method_fine}")

    def _init_modules(self):
        """延迟导入并实例化各功能模块。"""
        from ..intrinsic.camera_pinhole import CameraPinholeCalibrator
        from ..intrinsic.camera_fisheye import CameraFisheyeCalibrator
        from ..intrinsic.imu_intrinsic import IMUIntrinsicCalibrator
        from ..extrinsic.coarse.learn_to_calib_wrapper import LearnToCalibWrapper
        from ..extrinsic.coarse.imu_lidar_init import IMULidarInitCalibrator
        from ..extrinsic.coarse.feature_matching import FeatureMatchingCalibrator
        from ..extrinsic.fine.ikalibr_wrapper import IKalibrWrapper
        from ..extrinsic.fine.mias_lcec_wrapper import MIASLCECWrapper
        from ..validation.reprojection import ReprojectionValidator
        from ..validation.colorization import ColorizationValidator
        from ..validation.edge_alignment import EdgeAlignmentValidator
        from ..validation.report_generator import ReportGenerator

        self._cam_pinhole = CameraPinholeCalibrator(self.config)
        self._cam_fisheye = CameraFisheyeCalibrator(self.config)
        self._imu_calib = IMUIntrinsicCalibrator(self.config)
        self._learn_calib = LearnToCalibWrapper(self.config)
        self._imu_lidar_init = IMULidarInitCalibrator(self.config)
        self._feature_matching = FeatureMatchingCalibrator(self.config)
        self._ikalibr = IKalibrWrapper(self.config)
        self._mias_lcec = MIASLCECWrapper(self.config)
        self._reproj_val = ReprojectionValidator()
        self._color_val = ColorizationValidator()
        self._edge_val = EdgeAlignmentValidator()
        self._reporter = ReportGenerator(self.output_dir)

    # ------------------------------------------------------------------
    # 完整流水线
    # ------------------------------------------------------------------

    def run_full_pipeline(self, data_path: str) -> Dict:
        """执行完整四阶段标定流水线。"""
        logger.info("=" * 70)
        logger.info("UniCalib 多传感器自动标定系统启动")
        logger.info("=" * 70)

        data_mgr = DataManager(data_path)
        data_mgr.open()

        try:
            # Stage 1: 内参标定
            logger.info("\n[Stage 1/4] ===== 内参标定 =====")
            self.intrinsic_results = self._stage_intrinsic(data_mgr)

            # Stage 2: 粗外参标定
            logger.info("\n[Stage 2/4] ===== 粗外参标定 (Learning-based) =====")
            coarse_results = self._stage_coarse_extrinsic(data_mgr)

            # Stage 3: 精外参标定
            logger.info("\n[Stage 3/4] ===== 精外参标定 (Optimization-based) =====")
            self.extrinsic_results = self._stage_fine_extrinsic(data_mgr, coarse_results)

            # Stage 4: 验证
            logger.info("\n[Stage 4/4] ===== 标定验证 =====")
            validation = self._stage_validation(data_mgr)

        finally:
            data_mgr.close()

        # 保存结果并生成报告
        self.save_results(self.intrinsic_results, self.extrinsic_results, validation)

        logger.info("\n" + "=" * 70)
        logger.info(f"标定完成! 结果已保存至: {self.output_dir}")
        logger.info("=" * 70)

        return {
            "intrinsics": self.intrinsic_results,
            "extrinsics": self.extrinsic_results,
            "validation": validation,
        }

    # ------------------------------------------------------------------
    # Stage 1: 内参标定
    # ------------------------------------------------------------------

    def _stage_intrinsic(self, data_mgr: DataManager) -> Dict[str, IntrinsicResult]:
        results: Dict[str, IntrinsicResult] = {}

        for sid, sensor in self.sensors.items():
            logger.info(f"  Calibrating intrinsic: {sid} ({sensor.sensor_type.value})")

            if sensor.sensor_type == SensorType.CAMERA_PINHOLE:
                results[sid] = self._cam_pinhole.calibrate(data_mgr, sensor)

            elif sensor.sensor_type == SensorType.CAMERA_FISHEYE:
                results[sid] = self._cam_fisheye.calibrate(data_mgr, sensor)

            elif sensor.sensor_type == SensorType.IMU:
                results[sid] = self._imu_calib.calibrate(data_mgr, sensor)

            elif sensor.sensor_type == SensorType.LIDAR:
                logger.info(f"  LiDAR {sid}: 无内参标定步骤，跳过。")

        return results

    # ------------------------------------------------------------------
    # Stage 2: 粗外参标定
    # ------------------------------------------------------------------

    def _stage_coarse_extrinsic(self, data_mgr: DataManager) -> Dict[Tuple[str, str], CalibResult]:
        results: Dict[Tuple[str, str], CalibResult] = {}

        for pair in self.calib_pairs:
            logger.info(f"  Coarse calib: [{pair.sensor_a}] <-> [{pair.sensor_b}]  method={pair.method_coarse}")
            sensor_a = self.sensors[pair.sensor_a]
            sensor_b = self.sensors[pair.sensor_b]

            try:
                if pair.method_coarse == "l2calib_rl_init":
                    result = self._learn_calib.calibrate(
                        data_mgr, sensor_a, sensor_b, self.intrinsic_results)

                elif pair.method_coarse == "mias_lcec_coarse":
                    result = self._mias_lcec.calibrate_coarse(
                        data_mgr, sensor_a, sensor_b, self.intrinsic_results)

                elif pair.method_coarse == "feature_matching":
                    result = self._feature_matching.calibrate(
                        data_mgr, sensor_a, sensor_b, self.intrinsic_results)

                else:
                    logger.warning(f"  Unknown coarse method: {pair.method_coarse}, skipping.")
                    continue

                results[pair.key()] = result
                logger.info(f"    → R_euler={result.rotation_euler_deg()}, "
                            f"t={result.translation}, conf={result.confidence:.3f}")

            except Exception as e:
                logger.error(f"  Coarse calib failed for {pair.key()}: {e}", exc_info=True)

        return results

    # ------------------------------------------------------------------
    # Stage 3: 精外参标定
    # ------------------------------------------------------------------

    def _stage_fine_extrinsic(
        self, data_mgr: DataManager,
        coarse_results: Dict[Tuple[str, str], CalibResult]
    ) -> Dict[Tuple[str, str], CalibResult]:
        results: Dict[Tuple[str, str], CalibResult] = dict(coarse_results)

        # 3a: iKalibr 全局联合优化
        logger.info("  [3a] iKalibr 全局联合 B-spline 优化...")
        imu_lidar_pairs = [
            p for p in self.calib_pairs if p.method_fine == "ikalibr_bspline"
        ]
        if imu_lidar_pairs:
            try:
                global_result = self._ikalibr.joint_optimize(
                    data_mgr, self.sensors, self.intrinsic_results, coarse_results)
                results.update(global_result)
            except Exception as e:
                logger.error(f"  iKalibr joint optimize failed: {e}", exc_info=True)

        # 3b: MIAS-LCEC 精化 LiDAR-Camera
        logger.info("  [3b] MIAS-LCEC LiDAR-Camera 精化...")
        for pair in self.calib_pairs:
            if pair.method_fine == "mias_lcec_fine":
                sensor_a = self.sensors[pair.sensor_a]
                sensor_b = self.sensors[pair.sensor_b]
                initial = results.get(pair.key())
                if initial is None:
                    logger.warning(f"  No coarse result for {pair.key()}, skipping MIAS-LCEC fine.")
                    continue
                try:
                    refined = self._mias_lcec.calibrate_fine(
                        data_mgr, sensor_a, sensor_b,
                        self.intrinsic_results, initial)
                    results[pair.key()] = refined
                    logger.info(f"    → MIAS-LCEC refined {pair.key()}: "
                                f"reproj_err={refined.reprojection_error:.3f}px  "
                                f"td={refined.time_offset*1000:.2f}ms")
                except Exception as e:
                    logger.error(f"  MIAS-LCEC fine failed for {pair.key()}: {e}", exc_info=True)

        # 3c: click_calib 全局 BA (Camera-Camera)
        logger.info("  [3c] click_calib Camera-Camera 全局 BA...")
        for pair in self.calib_pairs:
            if pair.method_fine == "click_calib_ba":
                sensor_a = self.sensors[pair.sensor_a]
                sensor_b = self.sensors[pair.sensor_b]
                initial = results.get(pair.key())
                try:
                    from ..validation.click_calib_wrapper import ClickCalibWrapper
                    wrapper = ClickCalibWrapper(self.config)
                    refined = wrapper.global_ba(
                        data_mgr, sensor_a, sensor_b,
                        self.intrinsic_results, initial)
                    results[pair.key()] = refined
                    logger.info(f"    → click_calib BA refined {pair.key()}: "
                                f"reproj_err={refined.reprojection_error:.3f}px")
                except Exception as e:
                    logger.error(f"  click_calib BA failed for {pair.key()}: {e}", exc_info=True)

        return results

    # ------------------------------------------------------------------
    # Stage 4: 验证
    # ------------------------------------------------------------------

    def _stage_validation(self, data_mgr: DataManager) -> ValidationReport:
        report = ValidationReport()
        thresholds = (self.config.get("calibration", {})
                      .get("stage4_validation", {})
                      .get("thresholds", {}))
        reproj_thr = thresholds.get("reproj_error_px", 1.0)

        for (sa_id, sb_id), ext in self.extrinsic_results.items():
            sa = self.sensors[sa_id]
            sb = self.sensors[sb_id]

            if sa.is_lidar() and sb.is_camera():
                lidar_id, cam_id = sa_id, sb_id
            elif sb.is_lidar() and sa.is_camera():
                lidar_id, cam_id = sb_id, sa_id
            else:
                lidar_id, cam_id = None, None

            if lidar_id and cam_id and cam_id in self.intrinsic_results:
                # LiDAR-Camera 重投影误差验证
                metrics = self._reproj_val.validate(
                    data_mgr, lidar_id, cam_id,
                    self.sensors, self.intrinsic_results, ext)
                metrics["pass"] = metrics.get("mean_error_px", float('inf')) < reproj_thr
                report.add_metric(f"{lidar_id}-{cam_id}_reproj", metrics)

                # 点云着色一致性验证
                color_metrics = self._color_val.validate(
                    data_mgr, lidar_id, cam_id,
                    self.sensors, self.intrinsic_results, ext)
                report.add_metric(f"{lidar_id}-{cam_id}_colorization", color_metrics)

                # 边缘对齐验证
                edge_metrics = self._edge_val.validate(
                    data_mgr, lidar_id, cam_id,
                    self.sensors, self.intrinsic_results, ext)
                report.add_metric(f"{lidar_id}-{cam_id}_edge_align", edge_metrics)

        # 汇总通过状态
        all_pass = all(
            m.extra.get("pass", True)
            for m in report.metrics.values()
        )
        report.overall_pass = all_pass
        report.summary = "PASS" if all_pass else "FAIL - 部分指标超出阈值"
        logger.info(f"  验证结果: {report.summary}")
        return report

    # ------------------------------------------------------------------
    # 结果保存
    # ------------------------------------------------------------------

    def save_results(
        self,
        intrinsics: Dict[str, IntrinsicResult],
        extrinsics: Dict[Tuple[str, str], CalibResult],
        validation: ValidationReport,
    ):
        """保存所有标定结果到 output_dir。"""
        # 内参
        intr_out = {}
        for sid, result in intrinsics.items():
            if hasattr(result, "to_dict"):
                intr_out[sid] = result.to_dict()
        with open(self.output_dir / "intrinsics.yaml", "w") as f:
            yaml.dump(intr_out, f, default_flow_style=False)

        # 外参
        extr_out = {}
        for key, result in extrinsics.items():
            extr_out[f"{key[0]}_to_{key[1]}"] = result.to_dict()
        with open(self.output_dir / "extrinsics.yaml", "w") as f:
            yaml.dump(extr_out, f, default_flow_style=False)

        # 验证报告
        validation.save_yaml(str(self.output_dir / "validation_report.yaml"))

        # 打印汇总表格
        self._reporter.print_summary(intrinsics, extrinsics, validation)
        self._reporter.generate_html(intrinsics, extrinsics, validation)

        logger.info(f"  Results saved to: {self.output_dir}")

    # ------------------------------------------------------------------
    # 分阶段运行接口（供 scripts/ 使用）
    # ------------------------------------------------------------------

    def run_stage(self, stage: str, data_path: str):
        """运行单个阶段（用于调试/分步执行）。"""
        data_mgr = DataManager(data_path)
        data_mgr.open()
        try:
            if stage == "intrinsic":
                return self._stage_intrinsic(data_mgr)
            elif stage == "coarse":
                return self._stage_coarse_extrinsic(data_mgr)
            elif stage == "fine":
                coarse = self._load_coarse_from_disk()
                return self._stage_fine_extrinsic(data_mgr, coarse)
            elif stage == "validate":
                return self._stage_validation(data_mgr)
            else:
                raise ValueError(f"Unknown stage: {stage}")
        finally:
            data_mgr.close()

    def _load_coarse_from_disk(self) -> Dict[Tuple[str, str], CalibResult]:
        """从 output_dir 加载已保存的外参（用于分步执行）。"""
        path = self.output_dir / "extrinsics.yaml"
        if not path.exists():
            return {}
        with open(path) as f:
            data = yaml.safe_load(f)
        results = {}
        for key_str, d in data.items():
            a, b = key_str.split("_to_")
            r = CalibResult.from_dict(d)
            results[(a, b)] = r
        return results
