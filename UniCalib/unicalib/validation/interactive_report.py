"""
交互式标定报告生成器
使用 Plotly 生成可交互的图表和3D可视化
"""
from __future__ import annotations
import logging
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional
import json

import numpy as np
from scipy.spatial.transform import Rotation

logger = logging.getLogger(__name__)


class InteractiveReportGenerator:
    """交互式标定报告生成器。"""

    def __init__(self, output_dir: str):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

    def generate_interactive_html(
        self,
        intrinsics: Dict,
        extrinsics: Dict,
        validation_metrics: Dict,
        visualization_data: Optional[Dict] = None,
    ) -> str:
        """
        生成交互式 HTML 报告。
        
        Args:
            intrinsics: 内参标定结果
            extrinsics: 外参标定结果
            validation_metrics: 验证指标
            visualization_data: 可视化数据（误差分布、残差曲线等）
        
        Returns:
            HTML 文件路径
        """
        html_path = self.output_dir / "interactive_calibration_report.html"
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        # 生成各部分内容
        summary_section = self._generate_summary_section(validation_metrics)
        intrinsics_section = self._generate_intrinsics_section(intrinsics)
        extrinsics_section = self._generate_extrinsics_section(extrinsics)
        validation_section = self._generate_validation_section(
            validation_metrics, visualization_data
        )
        sensor_3d_section = self._generate_sensor_3d_section(extrinsics)

        html = f"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>UniCalib 交互式标定报告</title>
<script src="https://cdn.plot.ly/plotly-2.27.0.min.js"></script>
<link href="https://cdn.jsdelivr.net/npm/bootstrap@5.3.0/dist/css/bootstrap.min.css" rel="stylesheet">
<style>
  body {{ font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; margin: 0; padding: 0; background: #f8f9fa; }}
  .navbar {{ background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); padding: 15px 30px; }}
  .navbar h1 {{ color: white; margin: 0; font-size: 1.8em; }}
  .meta {{ color: rgba(255,255,255,0.8); margin-top: 5px; font-size: 0.9em; }}
  .container {{ max-width: 1400px; margin: 30px auto; padding: 0 20px; }}
  .card {{ border: none; border-radius: 12px; box-shadow: 0 4px 15px rgba(0,0,0,0.08); margin-bottom: 25px; }}
  .card-header {{ background: white; border-bottom: 2px solid #f0f0f0; padding: 15px 20px; }}
  .card-header h2 {{ margin: 0; color: #2c3e50; font-size: 1.3em; }}
  .card-body {{ padding: 20px; }}
  .summary-badge {{ display: inline-block; padding: 12px 25px; border-radius: 25px; 
                    font-size: 1.1em; font-weight: bold; }}
  .summary-pass {{ background: #10b981; color: white; }}
  .summary-fail {{ background: #ef4444; color: white; }}
  table {{ width: 100%; border-collapse: collapse; }}
  th {{ background: #f8f9fa; color: #495057; font-weight: 600; padding: 12px; text-align: left; border-bottom: 2px solid #dee2e6; }}
  td {{ padding: 10px 12px; border-bottom: 1px solid #e9ecef; }}
  tr:hover {{ background: #f8f9fa; }}
  .metric-pass {{ color: #10b981; font-weight: 600; }}
  .metric-fail {{ color: #ef4444; font-weight: 600; }}
  .plot-container {{ margin: 20px 0; border-radius: 8px; overflow: hidden; }}
  #sensor3d {{ width: 100%; height: 600px; background: #1a1a2e; border-radius: 12px; }}
  .section-divider {{ border-top: 3px solid #e9ecef; margin: 30px 0; }}
  .info-box {{ background: #e7f3ff; border-left: 4px solid #2196f3; padding: 15px; margin: 15px 0; border-radius: 4px; }}
  .nav-tabs .nav-link {{ border: none; color: #495057; padding: 10px 20px; }}
  .nav-tabs .nav-link.active {{ background: #667eea; color: white; border-radius: 8px; }}
</style>
</head>
<body>
<nav class="navbar">
  <div>
    <h1>🎯 UniCalib 多传感器标定报告</h1>
    <div class="meta">生成时间: {timestamp}</div>
  </div>
</nav>

<div class="container">
  <!-- 总体摘要 -->
  <div class="card">
    <div class="card-header"><h2>📊 总体摘要</h2></div>
    <div class="card-body">
      {summary_section}
    </div>
  </div>

  <!-- 内参标定结果 -->
  <div class="card">
    <div class="card-header"><h2>🎥 内参标定结果</h2></div>
    <div class="card-body">
      {intrinsics_section}
    </div>
  </div>

  <!-- 外参标定结果 -->
  <div class="card">
    <div class="card-header"><h2>🔗 外参标定结果</h2></div>
    <div class="card-body">
      {extrinsics_section}
    </div>
  </div>

  <!-- 验证指标与可视化 -->
  <div class="card">
    <div class="card-header"><h2>✓ 验证指标</h2></div>
    <div class="card-body">
      <ul class="nav nav-tabs" id="validationTabs" role="tablist">
        <li class="nav-item">
          <a class="nav-link active" data-bs-toggle="tab" href="#metrics">指标汇总</a>
        </li>
        <li class="nav-item">
          <a class="nav-link" data-bs-toggle="tab" href="#charts">交互式图表</a>
        </li>
        <li class="nav-item">
          <a class="nav-link" data-bs-toggle="tab" href="#sensor3d">3D传感器视图</a>
        </li>
      </ul>
      <div class="tab-content pt-3">
        <div class="tab-pane fade show active" id="metrics">
          {validation_section}
        </div>
        <div class="tab-pane fade" id="charts">
          {self._generate_charts_section(visualization_data)}
        </div>
        <div class="tab-pane fade" id="sensor3d">
          {sensor_3d_section}
        </div>
      </div>
    </div>
  </div>

  <!-- 精度说明 -->
  <div class="card">
    <div class="card-header"><h2>📏 精度说明</h2></div>
    <div class="card-body">
      <table class="table">
        <thead>
          <tr>
            <th>标定任务</th>
            <th>粗标定精度</th>
            <th>精标定精度</th>
            <th>合格阈值</th>
          </tr>
        </thead>
        <tbody>
          <tr>
            <td>相机内参(fx,fy)</td>
            <td>±5%</td>
            <td>±0.1% (reproj&lt;0.3px)</td>
            <td>reproj&lt;1.0px</td>
          </tr>
          <tr>
            <td>IMU-LiDAR旋转</td>
            <td>~1-3°</td>
            <td>~0.05-0.1°</td>
            <td>-</td>
          </tr>
          <tr>
            <td>IMU-LiDAR平移</td>
            <td>~5-10cm</td>
            <td>~1-3cm</td>
            <td>-</td>
          </tr>
          <tr>
            <td>LiDAR-Camera旋转</td>
            <td>~1-5°</td>
            <td>~0.02-0.05°</td>
            <td>-</td>
          </tr>
          <tr>
            <td>LiDAR-Camera平移</td>
            <td>~5-20cm</td>
            <td>~1-5mm</td>
            <td>-</td>
          </tr>
          <tr>
            <td>时间偏移</td>
            <td>N/A</td>
            <td>~0.1-1ms</td>
            <td>&lt;5ms</td>
          </tr>
        </tbody>
      </table>
    </div>
  </div>
</div>

<script src="https://cdn.jsdelivr.net/npm/bootstrap@5.3.0/dist/js/bootstrap.bundle.min.js"></script>
{self._generate_plotly_scripts(visualization_data, extrinsics)}
</body>
</html>"""

        with open(html_path, "w", encoding="utf-8") as f:
            f.write(html)
        logger.info(f"  Interactive HTML report saved: {html_path}")
        return str(html_path)

    def _generate_summary_section(self, validation_metrics: Dict) -> str:
        """生成总体摘要部分。"""
        overall_pass = validation_metrics.get('overall_pass', True)
        summary_text = validation_metrics.get('summary', 'Calibration Completed')
        
        badge_class = 'summary-pass' if overall_pass else 'summary-fail'
        status_text = '✓ PASS' if overall_pass else '✗ FAIL'
        
        return f"""
<div class="summary-badge {badge_class}">
  总体状态: {status_text} - {summary_text}
</div>
<div class="info-box mt-3">
  <strong>关键指标:</strong><br>
  平均重投影误差: {self._get_avg_reproj_error(validation_metrics):.3f} px<br>
  验证通过率: {self._get_pass_rate(validation_metrics):.1f}%
</div>"""

    def _generate_intrinsics_section(self, intrinsics: Dict) -> str:
        """生成内参表格部分。"""
        from ..core.calib_result import CameraIntrinsic, FisheyeIntrinsic, IMUIntrinsic
        
        rows = ""
        for sid, intr in intrinsics.items():
            if isinstance(intr, CameraIntrinsic):
                rows += f"""
                <tr>
                  <td>{sid}</td>
                  <td>针孔相机</td>
                  <td>{intr.K[0,0]:.2f}</td>
                  <td>{intr.K[1,1]:.2f}</td>
                  <td>{intr.K[0,2]:.2f}</td>
                  <td>{intr.K[1,2]:.2f}</td>
                  <td>{intr.reprojection_error:.4f}</td>
                  <td>{intr.method}</td>
                </tr>"""
            elif isinstance(intr, FisheyeIntrinsic):
                p = intr.params
                rows += f"""
                <tr>
                  <td>{sid}</td>
                  <td>{intr.model_type}</td>
                  <td>{p.get('fx',0):.2f}</td>
                  <td>{p.get('fy',0):.2f}</td>
                  <td>{p.get('cx',0):.2f}</td>
                  <td>{p.get('cy',0):.2f}</td>
                  <td>{intr.reprojection_error:.4f}</td>
                  <td>{intr.method}</td>
                </tr>"""
            elif isinstance(intr, IMUIntrinsic):
                rows += f"""
                <tr>
                  <td>{sid}</td>
                  <td>IMU</td>
                  <td colspan="4">ARW={intr.gyro_noise:.2e}, VRW={intr.accel_noise:.2e}</td>
                  <td>-</td>
                  <td>{intr.method}</td>
                </tr>"""
        
        return f"""
<table class="table">
  <thead>
    <tr>
      <th>传感器</th>
      <th>模型</th>
      <th>fx</th>
      <th>fy</th>
      <th>cx</th>
      <th>cy</th>
      <th>重投影误差(px)</th>
      <th>方法</th>
    </tr>
  </thead>
  <tbody>
    {rows}
  </tbody>
</table>"""

    def _generate_extrinsics_section(self, extrinsics: Dict) -> str:
        """生成外参表格部分。"""
        rows = ""
        for (sa, sb), result in extrinsics.items():
            euler = Rotation.from_matrix(result.rotation).as_euler('zyx', degrees=True)
            pass_status = "✓" if hasattr(result, 'reprojection_error') else "-"
            rows += f"""
            <tr>
              <td>{sa}</td>
              <td>{sb}</td>
              <td>[{euler[0]:+.3f}, {euler[1]:+.3f}, {euler[2]:+.3f}]°</td>
              <td>[{result.translation[0]:+.4f}, {result.translation[1]:+.4f}, {result.translation[2]:+.4f}]m</td>
              <td>{result.time_offset*1000:+.2f}ms</td>
              <td>{result.reprojection_error:.4f if hasattr(result, 'reprojection_error') else '-'}</td>
              <td>{result.method_used}</td>
            </tr>"""
        
        return f"""
<table class="table">
  <thead>
    <tr>
      <th>传感器A</th>
      <th>传感器B</th>
      <th>旋转(ZYX欧拉角)</th>
      <th>平移</th>
      <th>时间偏移</th>
      <th>重投影误差(px)</th>
      <th>方法</th>
    </tr>
  </thead>
  <tbody>
    {rows}
  </tbody>
</table>"""

    def _generate_validation_section(
        self, validation_metrics: Dict, visualization_data: Optional[Dict] = None
    ) -> str:
        """生成验证指标部分。"""
        rows = ""
        metrics = validation_metrics.get('metrics', {})
        
        for key, metric in metrics.items():
            pass_status = metric.extra.get("pass", True)
            status_class = "metric-pass" if pass_status else "metric-fail"
            status_text = "PASS" if pass_status else "FAIL"
            
            rows += f"""
            <tr>
              <td>{key}</td>
              <td>{metric.mean_error_px:.3f}</td>
              <td>{metric.median_error_px:.3f}</td>
              <td>{metric.pct_within_1px:.1f}%</td>
              <td class="{status_class}">{status_text}</td>
            </tr>"""
        
        return f"""
<table class="table">
  <thead>
    <tr>
      <th>标定对</th>
      <th>均值误差(px)</th>
      <th>中值误差(px)</th>
      <th>&lt;1px比例</th>
      <th>状态</th>
    </tr>
  </thead>
  <tbody>
    {rows}
  </tbody>
</table>"""

    def _generate_charts_section(self, visualization_data: Optional[Dict] = None) -> str:
        """生成交互式图表部分。"""
        if visualization_data is None:
            visualization_data = {}
        
        charts_html = """
<div class="row">
  <div class="col-md-6">
    <h5>误差分布直方图</h5>
    <div id="errorHistogram" class="plot-container"></div>
  </div>
  <div class="col-md-6">
    <h5>误差累积分布</h5>
    <div id="errorCDF" class="plot-container"></div>
  </div>
</div>
<div class="row mt-4">
  <div class="col-md-12">
    <h5>优化残差收敛曲线</h5>
    <div id="residualPlot" class="plot-container"></div>
  </div>
</div>"""
        
        return charts_html

    def _generate_sensor_3d_section(self, extrinsics: Dict) -> str:
        """生成3D传感器可视化部分。"""
        return """
<div id="sensor3d"></div>
<div class="info-box">
  <strong>操作说明:</strong><br>
  • 鼠标左键拖动: 旋转视角<br>
  • 鼠标右键拖动: 平移<br>
  • 鼠标滚轮: 缩放<br>
  • 双击传感器名称: 隐藏/显示该传感器
</div>"""

    def _generate_plotly_scripts(
        self, visualization_data: Optional[Dict] = None, extrinsics: Optional[Dict] = None
    ) -> str:
        """生成 Plotly 图表脚本。"""
        if visualization_data is None:
            visualization_data = {}
        
        scripts = "<script>\n"
        
        # 误差分布直方图
        error_data = visualization_data.get('error_distribution', None)
        if error_data is not None:
            errors = error_data.get('errors', [])
            if len(errors) > 0:
                scripts += self._create_error_histogram(errors)
                scripts += self._create_error_cdf(errors)
        
        # 残差曲线
        residual_data = visualization_data.get('residual_convergence', None)
        if residual_data is not None:
            iterations = residual_data.get('iterations', [])
            errors = residual_data.get('errors', [])
            if len(iterations) > 0:
                scripts += self._create_residual_plot(iterations, errors)
        
        # 3D传感器可视化
        if extrinsics is not None:
            scripts += self._create_sensor_3d_plot(extrinsics)
        
        scripts += "</script>\n"
        return scripts

    def _create_error_histogram(self, errors: np.ndarray) -> str:
        """创建误差直方图脚本。"""
        import json
        errors_json = json.dumps(errors.tolist())
        
        return f"""
// 误差分布直方图
const errorData = {errors_json};
const errorTrace = {{
  x: errorData,
  type: 'histogram',
  name: '误差分布',
  marker: {{
    color: 'rgba(102, 126, 234, 0.7)',
    line: {{ color: 'rgba(102, 126, 234, 1)', width: 1 }}
  }},
  nbinsx: 50
}};
const errorLayout = {{
  title: '重投影误差分布',
  xaxis: {{ title: '误差 (像素)' }},
  yaxis: {{ title: '频次' }},
  margin: {{ t: 30, b: 40, l: 50, r: 20 }},
  height: 350
}};
Plotly.newPlot('errorHistogram', [errorTrace], errorLayout, {{responsive: true}});
"""

    def _create_error_cdf(self, errors: np.ndarray) -> str:
        """创建误差CDF脚本。"""
        import json
        sorted_errors = np.sort(errors)
        cdf = np.arange(1, len(sorted_errors) + 1) / len(sorted_errors) * 100
        
        errors_json = json.dumps(sorted_errors.tolist())
        cdf_json = json.dumps(cdf.tolist())
        
        return f"""
// 误差CDF曲线
const cdfTrace = {{
  x: {errors_json},
  y: {cdf_json},
  type: 'scatter',
  mode: 'lines',
  name: '累积分布',
  line: {{ color: 'rgb(239, 68, 68)', width: 2 }}
}};
const cdfLayout = {{
  title: '误差累积分布函数',
  xaxis: {{ title: '误差 (像素)' }},
  yaxis: {{ title: '累积比例 (%)', range: [0, 100] }},
  margin: {{ t: 30, b: 40, l: 60, r: 20 }},
  height: 350
}};
Plotly.newPlot('errorCDF', [cdfTrace], cdfLayout, {{responsive: true}});
"""

    def _create_residual_plot(self, iterations: List[int], errors: List[float]) -> str:
        """创建残差收敛曲线脚本。"""
        import json
        iterations_json = json.dumps(iterations)
        errors_json = json.dumps(errors)
        
        return f"""
// 残差收敛曲线
const residualTrace = {{
  x: {iterations_json},
  y: {errors_json},
  type: 'scatter',
  mode: 'lines+markers',
  name: '残差',
  line: {{ color: 'rgb(16, 185, 129)', width: 2 }},
  marker: {{ size: 4 }}
}};
const residualLayout = {{
  title: '优化残差收敛曲线',
  xaxis: {{ title: '迭代次数' }},
  yaxis: {{ title: '残差' }},
  margin: {{ t: 30, b: 40, l: 50, r: 20 }},
  height: 400
}};
Plotly.newPlot('residualPlot', [residualTrace], residualLayout, {{responsive: true}});
"""

    def _create_sensor_3d_plot(self, extrinsics: Dict) -> str:
        """创建3D传感器可视化脚本。"""
        traces = []
        
        for (sa, sb), result in extrinsics.items():
            R = result.rotation
            t = result.translation
            pos = -R.T @ t
            
            # 绘制传感器位置
            if 'lidar' in sa.lower() or 'lidar' in sb.lower():
                color = 'red'
                name = f"{sa}-{sb}"
            elif 'camera' in sa.lower() or 'camera' in sb.lower():
                color = 'blue'
                name = f"{sa}-{sb}"
            else:
                color = 'green'
                name = f"{sa}-{sb}"
            
            traces.append(f"""
const trace{name} = {{
  x: [{pos[0]}],
  y: [{pos[1]}],
  z: [{pos[2]}],
  mode: 'markers',
  type: 'scatter3d',
  marker: {{ size: 10, color: '{color}', opacity: 0.8 }},
  name: '{name}'
}};""")
        
        traces_str = '\n'.join(traces)
        
        return f"""
// 3D传感器可视化
{traces_str}

const sensor3dData = [{', '.join([f"trace{sa}-{sb}" for (sa, sb) in extrinsics.keys()])}];
const sensor3dLayout = {{
  title: '传感器3D布局',
  scene: {{
    xaxis: {{ title: 'X (m)' }},
    yaxis: {{ title: 'Y (m)' }},
    zaxis: {{ title: 'Z (m)' }},
    aspectmode: 'cube'
  }},
  margin: {{ t: 30, b: 0, l: 0, r: 0 }}
}};
Plotly.newPlot('sensor3d', sensor3dData, sensor3dLayout, {{responsive: true}});
"""

    def _get_avg_reproj_error(self, validation_metrics: Dict) -> float:
        """获取平均重投影误差。"""
        metrics = validation_metrics.get('metrics', {})
        errors = [m.mean_error_px for m in metrics.values()]
        return np.mean(errors) if errors else 0.0

    def _get_pass_rate(self, validation_metrics: Dict) -> float:
        """获取验证通过率。"""
        metrics = validation_metrics.get('metrics', {})
        if len(metrics) == 0:
            return 0.0
        passes = sum(1 for m in metrics.values() if m.extra.get('pass', True))
        return passes / len(metrics) * 100
