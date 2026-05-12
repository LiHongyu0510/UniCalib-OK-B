"""
标定结果报告生成器
生成终端汇总表格 + HTML 报告
"""
from __future__ import annotations
import logging
from datetime import datetime
from pathlib import Path
from typing import Dict, Tuple

import numpy as np
from scipy.spatial.transform import Rotation

from ..core.calib_result import (
    ValidationReport, CalibResult, CameraIntrinsic,
    FisheyeIntrinsic, IMUIntrinsic
)

logger = logging.getLogger(__name__)


class ReportGenerator:
    """标定报告生成器。"""

    def __init__(self, output_dir: Path):
        self.output_dir = Path(output_dir)

    def print_summary(self, intrinsics, extrinsics, validation: ValidationReport):
        """在终端打印汇总信息。"""
        line = "=" * 72
        logger.info(f"\n{line}")
        logger.info("  UniCalib 标定结果汇总")
        logger.info(line)

        # 内参汇总
        logger.info("\n【内参标定结果】")
        for sid, intr in intrinsics.items():
            if isinstance(intr, CameraIntrinsic):
                logger.info(f"  {sid:<20} fx={intr.K[0,0]:.1f} fy={intr.K[1,1]:.1f} "
                            f"cx={intr.K[0,2]:.1f} cy={intr.K[1,2]:.1f} "
                            f"reproj={intr.reprojection_error:.3f}px "
                            f"[{intr.method}]")
            elif isinstance(intr, FisheyeIntrinsic):
                p = intr.params
                logger.info(f"  {sid:<20} model={intr.model_type} "
                            f"fx={p.get('fx',0):.1f} reproj={intr.reprojection_error:.3f}px")
            elif isinstance(intr, IMUIntrinsic):
                logger.info(f"  {sid:<20} gyro_ARW={intr.gyro_noise:.2e} "
                            f"accel_VRW={intr.accel_noise:.2e} [{intr.method}]")

        # 外参汇总
        logger.info("\n【外参标定结果】")
        for (sa, sb), result in extrinsics.items():
            euler = Rotation.from_matrix(result.rotation).as_euler('zyx', degrees=True)
            logger.info(f"  {sa:<12} → {sb:<12} "
                        f"R=[{euler[0]:+.2f},{euler[1]:+.2f},{euler[2]:+.2f}]° "
                        f"t=[{result.translation[0]:+.3f},{result.translation[1]:+.3f},"
                        f"{result.translation[2]:+.3f}]m "
                        f"td={result.time_offset*1000:+.2f}ms "
                        f"[{result.method_used}]")

        # 验证汇总
        logger.info("\n【验证指标】")
        for key, metric in validation.metrics.items():
            status = "✓ PASS" if metric.extra.get("pass", True) else "✗ FAIL"
            logger.info(f"  {key:<35} "
                        f"mean={metric.mean_error_px:.2f}px  "
                        f"<1px={metric.pct_within_1px:.1f}%  {status}")

        logger.info(f"\n{'  总体结果':}: {validation.summary}")
        logger.info(line)

    def generate_html(self, intrinsics, extrinsics, validation: ValidationReport):
        """生成 HTML 格式的详细报告。"""
        html_path = self.output_dir / "calibration_report.html"
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        rows_intr = ""
        for sid, intr in intrinsics.items():
            if isinstance(intr, CameraIntrinsic):
                rows_intr += (
                    f"<tr><td>{sid}</td><td>pinhole</td>"
                    f"<td>{intr.K[0,0]:.2f}</td><td>{intr.K[1,1]:.2f}</td>"
                    f"<td>{intr.K[0,2]:.2f}</td><td>{intr.K[1,2]:.2f}</td>"
                    f"<td>{intr.reprojection_error:.4f}</td>"
                    f"<td>{intr.method}</td></tr>\n"
                )
            elif isinstance(intr, FisheyeIntrinsic):
                p = intr.params
                rows_intr += (
                    f"<tr><td>{sid}</td><td>{intr.model_type}</td>"
                    f"<td>{p.get('fx',0):.2f}</td><td>{p.get('fy',0):.2f}</td>"
                    f"<td>{p.get('cx',0):.2f}</td><td>{p.get('cy',0):.2f}</td>"
                    f"<td>{intr.reprojection_error:.4f}</td>"
                    f"<td>{intr.method}</td></tr>\n"
                )
            elif isinstance(intr, IMUIntrinsic):
                rows_intr += (
                    f"<tr><td>{sid}</td><td>IMU</td>"
                    f"<td colspan=4>ARW={intr.gyro_noise:.2e} "
                    f"VRW={intr.accel_noise:.2e}</td>"
                    f"<td>-</td><td>{intr.method}</td></tr>\n"
                )

        rows_extr = ""
        for (sa, sb), result in extrinsics.items():
            euler = Rotation.from_matrix(result.rotation).as_euler('zyx', degrees=True)
            rows_extr += (
                f"<tr><td>{sa}</td><td>{sb}</td>"
                f"<td>[{euler[0]:+.3f},{euler[1]:+.3f},{euler[2]:+.3f}]°</td>"
                f"<td>[{result.translation[0]:+.4f},{result.translation[1]:+.4f},"
                f"{result.translation[2]:+.4f}]m</td>"
                f"<td>{result.time_offset*1000:+.2f}ms</td>"
                f"<td>{result.reprojection_error:.4f}</td>"
                f"<td>{result.method_used}</td></tr>\n"
            )

        rows_val = ""
        for key, metric in validation.metrics.items():
            status_color = "green" if metric.extra.get("pass", True) else "red"
            rows_val += (
                f"<tr><td>{key}</td>"
                f"<td>{metric.mean_error_px:.3f}</td>"
                f"<td>{metric.median_error_px:.3f}</td>"
                f"<td>{metric.pct_within_1px:.1f}%</td>"
                f"<td style='color:{status_color}'>"
                f"{'PASS' if metric.extra.get('pass', True) else 'FAIL'}</td></tr>\n"
            )

        overall_color = "green" if validation.overall_pass else "red"

        html = f"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<title>UniCalib 标定报告</title>
<style>
  body {{ font-family: 'Arial', sans-serif; margin: 40px; background: #f5f5f5; }}
  h1 {{ color: #2c3e50; border-bottom: 3px solid #3498db; padding-bottom: 10px; }}
  h2 {{ color: #34495e; margin-top: 30px; }}
  table {{ border-collapse: collapse; width: 100%; background: white;
           box-shadow: 0 1px 3px rgba(0,0,0,0.1); margin-bottom: 20px; }}
  th {{ background: #3498db; color: white; padding: 10px; text-align: left; }}
  td {{ padding: 8px 10px; border-bottom: 1px solid #eee; }}
  tr:hover {{ background: #f8f9fa; }}
  .summary {{ background: {overall_color}; color: white; padding: 15px 25px;
              border-radius: 8px; font-size: 1.2em; display: inline-block; }}
  .meta {{ color: #7f8c8d; font-size: 0.9em; margin-bottom: 20px; }}
</style>
</head>
<body>
<h1>UniCalib 多传感器标定报告</h1>
<p class="meta">生成时间: {timestamp}</p>

<div class="summary">总体结果: {validation.summary}</div>

<h2>1. 内参标定结果</h2>
<table>
  <tr><th>传感器</th><th>模型</th><th>fx</th><th>fy</th>
      <th>cx</th><th>cy</th><th>重投影误差(px)</th><th>方法</th></tr>
  {rows_intr}
</table>

<h2>2. 外参标定结果</h2>
<table>
  <tr><th>传感器A</th><th>传感器B</th><th>旋转(ZYX欧拉角)</th>
      <th>平移</th><th>时间偏移</th><th>重投影误差(px)</th><th>方法</th></tr>
  {rows_extr}
</table>

<h2>3. 验证指标</h2>
<table>
  <tr><th>标定对</th><th>均值误差(px)</th><th>中值误差(px)</th>
      <th>&lt;1px比例</th><th>状态</th></tr>
  {rows_val}
</table>

<h2>4. 精度说明</h2>
<table>
  <tr><th>标定任务</th><th>粗标定精度</th><th>精标定精度</th><th>合格阈值</th></tr>
  <tr><td>相机内参(fx,fy)</td><td>±5%</td><td>±0.1% (reproj&lt;0.3px)</td><td>reproj&lt;1.0px</td></tr>
  <tr><td>IMU-LiDAR旋转</td><td>~1-3°</td><td>~0.05-0.1°</td><td>-</td></tr>
  <tr><td>IMU-LiDAR平移</td><td>~5-10cm</td><td>~1-3cm</td><td>-</td></tr>
  <tr><td>LiDAR-Camera旋转</td><td>~1-5°</td><td>~0.02-0.05°</td><td>-</td></tr>
  <tr><td>LiDAR-Camera平移</td><td>~5-20cm</td><td>~1-5mm</td><td>-</td></tr>
  <tr><td>时间偏移</td><td>N/A</td><td>~0.1-1ms</td><td>&lt;5ms</td></tr>
</table>

</body>
</html>"""

        with open(html_path, "w", encoding="utf-8") as f:
            f.write(html)
        logger.info(f"  HTML report saved: {html_path}")
