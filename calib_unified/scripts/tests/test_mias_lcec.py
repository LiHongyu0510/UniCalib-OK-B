"""
MIAS-LCEC 与初始值评估、质量评估模块的单元测试。

运行方式（在项目根目录）:
  PYTHONPATH=calib_unified/scripts python -m pytest calib_unified/scripts/tests/test_mias_lcec.py -v
  或: cd calib_unified/scripts && python -m pytest tests/test_mias_lcec.py -v
"""

from __future__ import annotations

import sys
from pathlib import Path

# 保证能导入 mias_lcec_pytorch
_scripts = Path(__file__).resolve().parent.parent
if str(_scripts) not in sys.path:
    sys.path.insert(0, str(_scripts))

import numpy as np
import pytest


class TestInitialGuessEstimator:
    """初始值评估模块测试。"""

    def test_search_range_from_prior(self):
        from mias_lcec_pytorch.initial_guess_estimator import (
            SearchRange,
            VehicleMountPrior,
        )
        prior = VehicleMountPrior(cam_forward=0.5, cam_left=0.0, cam_up=-0.3)
        sr = SearchRange.from_vehicle_prior(prior, multiplier=1.5)
        assert len(sr.tz_range) >= 1
        assert len(sr.tx_range) >= 1
        assert 0.5 <= min(sr.tz_range) <= max(sr.tz_range) <= 5.0

    def test_estimate_initial_guess_no_features(self):
        from mias_lcec_pytorch.initial_guess_estimator import estimate_initial_guess, InitialGuessConfig
        np.random.seed(42)
        n = 1000
        points = np.random.randn(n, 3).astype(np.float64) * 5 + np.array([0, 0, 1.5])
        image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        K = np.array([[500, 0, 320], [0, 500, 240], [0, 0, 1]], dtype=np.float64)
        config = InitialGuessConfig(use_mount_prior=True, enable_deep_matching=False)
        result = estimate_initial_guess(points, image, K, config=config)
        assert result.search_range is not None
        assert 0 <= result.confidence <= 1
        assert result.confidence_level is not None


class TestQualityAssessment:
    """质量评估模块测试。"""

    def test_assess_good(self):
        from mias_lcec_pytorch.quality_assessment import assess_calibration_quality, QualityVerdict
        report = assess_calibration_quality(ncc=0.35, rms_px=1.5, inlier_ratio=0.7)
        assert report.verdict == QualityVerdict.GOOD
        assert report.confidence >= 0.8

    def test_assess_acceptable(self):
        from mias_lcec_pytorch.quality_assessment import assess_calibration_quality, QualityVerdict
        report = assess_calibration_quality(ncc=0.25, rms_px=3.0, inlier_ratio=0.5)
        assert report.verdict in (QualityVerdict.GOOD, QualityVerdict.ACCEPTABLE)
        assert report.confidence >= 0.5 or report.verdict == QualityVerdict.ACCEPTABLE

    def test_assess_fail(self):
        from mias_lcec_pytorch.quality_assessment import assess_calibration_quality, QualityVerdict
        report = assess_calibration_quality(ncc=0.1, rms_px=8.0, inlier_ratio=0.2)
        assert report.verdict == QualityVerdict.FAIL
        assert len(report.suggestions) >= 1

    def test_assess_inlier_from_counts(self):
        from mias_lcec_pytorch.quality_assessment import assess_calibration_quality
        report = assess_calibration_quality(ncc=0.3, rms_px=2.0, n_inliers=60, n_total=100)
        assert report.inlier_ratio == pytest.approx(0.6)


class TestAdaptiveSearch:
    """自适应搜索配置测试。"""

    def test_adaptive_search_config_ranges(self):
        from mias_lcec_pytorch.adaptive_search import AdaptiveSearchConfig
        cfg = AdaptiveSearchConfig(tx_center=0.0, tx_half_width=0.5, coarse_step=0.25)
        tx = cfg.get_tx_range(coarse=True)
        assert len(tx) >= 1
        assert min(tx) <= 0 <= max(tx)
