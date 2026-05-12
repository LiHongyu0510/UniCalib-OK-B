"""
标定质量评估与诊断

根据 NCC、重投影 RMS、内点率等计算综合置信度并输出诊断建议。
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional

import numpy as np

_LOG = logging.getLogger(__name__)


class QualityVerdict(Enum):
    GOOD = "good"
    ACCEPTABLE = "acceptable"
    FAIL = "fail"


@dataclass
class QualityThresholds:
    ncc_good: float = 0.3
    ncc_acceptable: float = 0.2
    rms_good_px: float = 2.0
    rms_acceptable_px: float = 5.0
    inlier_ratio_good: float = 0.6
    inlier_ratio_acceptable: float = 0.4


@dataclass
class QualityReport:
    verdict: QualityVerdict
    confidence: float
    ncc: float
    rms_px: float
    inlier_ratio: float
    diagnostics: Dict = field(default_factory=dict)
    suggestions: List[str] = field(default_factory=list)

    def to_dict(self) -> Dict:
        return {
            "verdict": self.verdict.value,
            "confidence": self.confidence,
            "ncc": self.ncc,
            "rms_px": self.rms_px,
            "inlier_ratio": self.inlier_ratio,
            "diagnostics": self.diagnostics,
            "suggestions": self.suggestions,
        }


def assess_calibration_quality(
    ncc: float = -1.0,
    rms_px: float = -1.0,
    inlier_ratio: float = -1.0,
    n_inliers: int = -1,
    n_total: int = -1,
    thresholds: Optional[QualityThresholds] = None,
) -> QualityReport:
    """
    根据 NCC、RMS、内点率评估标定质量并给出诊断建议。

    Args:
        ncc: 边缘对齐 NCC 分数
        rms_px: 重投影 RMS (像素)
        inlier_ratio: 内点率 (0~1)，若未提供则用 n_inliers/n_total 计算
        n_inliers: 内点数
        n_total: 总点数
        thresholds: 质量阈值，默认使用 QualityThresholds()

    Returns:
        QualityReport
    """
    th = thresholds or QualityThresholds()
    if inlier_ratio < 0 and n_total > 0 and n_inliers >= 0:
        inlier_ratio = n_inliers / n_total

    verdict = QualityVerdict.FAIL
    suggestions: List[str] = []
    diagnostics: Dict = {}

    # NCC 判定
    if ncc >= th.ncc_good:
        ncc_ok = 2
    elif ncc >= th.ncc_acceptable:
        ncc_ok = 1
        suggestions.append("NCC 处于可接受范围，建议增加多帧或改善光照/边缘场景")
    else:
        ncc_ok = 0
        suggestions.append("NCC 偏低，请检查初值、光照或边缘丰富的场景")
    diagnostics["ncc_level"] = ncc_ok

    # RMS 判定
    if rms_px >= 0:
        if rms_px <= th.rms_good_px:
            rms_ok = 2
        elif rms_px <= th.rms_acceptable_px:
            rms_ok = 1
            suggestions.append("重投影 RMS 可接受，可尝试增加标定帧或精化角点")
        else:
            rms_ok = 0
            suggestions.append("重投影误差偏大，建议检查角点检测或初值")
        diagnostics["rms_level"] = rms_ok
    else:
        rms_ok = -1

    # 内点率判定
    if inlier_ratio >= 0:
        if inlier_ratio >= th.inlier_ratio_good:
            inlier_ok = 2
        elif inlier_ratio >= th.inlier_ratio_acceptable:
            inlier_ok = 1
        else:
            inlier_ok = 0
            suggestions.append("内点率较低，建议检查匹配或初值")
        diagnostics["inlier_level"] = inlier_ok
    else:
        inlier_ok = -1

    # 综合置信度：各指标加权
    parts = []
    if ncc_ok >= 0:
        parts.append((ncc_ok / 2.0) * 0.4)
    if rms_ok >= 0:
        parts.append((rms_ok / 2.0) * 0.35)
    if inlier_ok >= 0:
        parts.append((inlier_ok / 2.0) * 0.25)
    confidence = sum(parts) / len(parts) if parts else 0.0

    if confidence >= 0.8:
        verdict = QualityVerdict.GOOD
    elif confidence >= 0.5:
        verdict = QualityVerdict.ACCEPTABLE
    else:
        verdict = QualityVerdict.FAIL

    report = QualityReport(
        verdict=verdict,
        confidence=confidence,
        ncc=ncc,
        rms_px=rms_px,
        inlier_ratio=inlier_ratio,
        diagnostics=diagnostics,
        suggestions=suggestions,
    )
    _LOG.info(
        "[Quality] verdict=%s confidence=%.2f ncc=%.3f rms=%.2fpx inlier_ratio=%.2f",
        report.verdict.value, report.confidence, report.ncc, report.rms_px, report.inlier_ratio,
    )
    return report
