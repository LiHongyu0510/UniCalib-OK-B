#!/usr/bin/env python3
"""
UniCalib — LiDAR-Camera 粗标定脚本（与 C++ MIASLCECAdapter 对接）

供 calib_unified 的 AI 粗标定管线调用：输入单帧 PCD + 图像 + 相机内参，
输出 T_camera_in_lidar 的 YAML（rotation_matrix + translation），供精标定初值。

接口约定（与 ai_coarse_calib.cpp MIASLCECAdapter 一致）:
  输入: --pcd, --image, --sensor_config, --output_dir
  输出: output_dir/extrinsic_result.yaml
    - rotation_matrix: 3x3 (double)
    - translation: [tx, ty, tz] (米)
    - 可选: lidar_id, camera_id

当 MIAS-LCEC 仓库无 headless 入口时，可将 third_party.mias_lcec.repo_dir
指向 calib_unified 安装目录、calib_script 指向本脚本，即可启用粗标定流程；
本脚本提供基于几何的初值（恒等或简单 PnP），精标定由 Ceres 边缘对齐完成。

依赖: PyYAML, numpy；图像读取优先 Pillow（避免 NumPy 2.x 下 opencv 崩溃）；可选 opencv-python（仅 --use_pnp 时需要）, open3d（读二进制 PCD）
  pip install PyYAML numpy Pillow
  pip install opencv-python  # 仅在使用 --use_pnp 时需要；若遇 NumPy 2.x 崩溃可仅用 identity 初值
  pip install open3d  # 可选，用于读二进制 .pcd
"""

from __future__ import annotations

import argparse
import logging
import os
import sys
import time
from pathlib import Path

# 统一日志：带时间戳与级别，输出到 stderr，便于与 C++ 管线日志合并
def _setup_logging(verbose: bool = True) -> logging.Logger:
    log = logging.getLogger("run_lidar_cam_coarse")
    log.setLevel(logging.DEBUG if verbose else logging.INFO)
    if not log.handlers:
        h = logging.StreamHandler(sys.stderr)
        h.setFormatter(logging.Formatter(
            "[%(asctime)s.%(msecs)03d] [%(levelname)s] %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S",
        ))
        log.addHandler(h)
    return log

LOG = _setup_logging()


def load_sensor_config(path: str) -> dict:
    """读取 sensor_config.yaml：camera.fx/fy/cx/cy/dist_coeffs/width/height"""
    import yaml
    LOG.info("读取 sensor_config: %s", path)
    with open(path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    if not cfg or "camera" not in cfg:
        raise ValueError("sensor_config 需包含 camera 段")
    c = cfg["camera"]
    fx, fy = float(c.get("fx", 0)), float(c.get("fy", 0))
    cx, cy = float(c.get("cx", 0)), float(c.get("cy", 0))
    w, h = c.get("width", ""), c.get("height", "")
    LOG.info("相机内参: fx=%.2f fy=%.2f cx=%.2f cy=%.2f width=%s height=%s",
             fx, fy, cx, cy, w, h)
    if not fx or not fy or (isinstance(w, int) and w <= 0) or (isinstance(h, int) and h <= 0):
        LOG.warning("sensor_config 内参为 0 或尺寸无效，将导致投影失败；请确保 C++ 端配置 camera_intrinsic_file 或从首帧推断内参")
    return cfg


def get_camera_k_d(cfg: dict):
    """K 3x3 与 dist_coeffs（OpenCV 格式）"""
    import numpy as np
    c = cfg["camera"]
    fx = float(c["fx"])
    fy = float(c["fy"])
    cx = float(c["cx"])
    cy = float(c["cy"])
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
    dist = c.get("dist_coeffs")
    if dist is None:
        dist = np.zeros(5, dtype=np.float64)
    else:
        dist = np.asarray(dist, dtype=np.float64)
    return K, dist


def _pcd_is_ascii(pcd_path: str, peek_bytes: int = 2048) -> bool:
    """检测 PCD 是否为 ASCII 格式，避免在 NumPy 2.x 下导入 Open3D 触发崩溃。"""
    path = Path(pcd_path)
    if not path.exists() or path.stat().st_size == 0:
        return False
    with open(path, "rb") as f:
        head = f.read(peek_bytes).decode("utf-8", errors="ignore")
    return "data ascii" in head.lower() or "data 0" in head.lower()


def load_pcd_points(pcd_path: str):
    """返回 Nx3 点云（lidar 系）。优先用 ASCII 解析（避免 NumPy 2.x 下 Open3D 崩溃），否则尝试 Open3D。"""
    import numpy as np
    path = Path(pcd_path)
    if not path.exists():
        raise FileNotFoundError(f"PCD 不存在: {pcd_path}")

    # 优先 ASCII：C++ 端已写 ASCII PCD，且 NumPy 2.x 下 Open3D 可能因 ABI 不兼容崩溃，不导入可避免
    if _pcd_is_ascii(pcd_path):
        try:
            pts = _read_pcd_ascii(pcd_path)
            if pts.size > 0:
                LOG.info("PCD 加载方式: ASCII (优先，避免 Open3D/NumPy 2.x 兼容问题)  路径=%s", pcd_path)
                n = len(pts)
                xmin, xmax = float(pts[:, 0].min()), float(pts[:, 0].max())
                ymin, ymax = float(pts[:, 1].min()), float(pts[:, 1].max())
                zmin, zmax = float(pts[:, 2].min()), float(pts[:, 2].max())
                LOG.info("点云规模: %d 点  范围 X[%.2f, %.2f] Y[%.2f, %.2f] Z[%.2f, %.2f]",
                         n, xmin, xmax, ymin, ymax, zmin, zmax)
                return pts
        except (ValueError, TypeError) as e:
            LOG.warning("ASCII 读 PCD 失败，将尝试 Open3D: %s", e)

    try:
        import open3d as o3d
        pcd = o3d.io.read_point_cloud(str(path))
        pts = np.asarray(pcd.points, dtype=np.float64)
        LOG.info("PCD 加载方式: open3d  路径=%s", pcd_path)
    except ImportError:
        pts = _read_pcd_ascii(pcd_path)
        LOG.info("PCD 加载方式: ASCII 回退 (无 open3d)  路径=%s", pcd_path)
    except Exception as e:
        err_msg = str(e).lower()
        if "numpy" in err_msg or "dtype" in err_msg or "binary" in err_msg or "incompatibility" in err_msg:
            LOG.warning("Open3D/NumPy 二进制不兼容，回退 ASCII 读 PCD: %s", e)
        else:
            LOG.warning("Open3D 读 PCD 失败，回退 ASCII: %s", e)
        try:
            pts = _read_pcd_ascii(pcd_path)
            LOG.info("PCD 加载方式: ASCII 回退  路径=%s", pcd_path)
        except (ValueError, TypeError) as ascii_err:
            LOG.error("ASCII 读 PCD 失败: %s", ascii_err)
            LOG.error("PCD 可能为二进制格式；C++ 端应使用 savePCDFile(path, cloud, false) 写 ASCII，便于 Python 无 Open3D 时解析")
            raise
    if pts.size == 0:
        raise ValueError(f"PCD 无有效点: {pcd_path}")
    n = len(pts)
    xmin, xmax = float(pts[:, 0].min()), float(pts[:, 0].max())
    ymin, ymax = float(pts[:, 1].min()), float(pts[:, 1].max())
    zmin, zmax = float(pts[:, 2].min()), float(pts[:, 2].max())
    LOG.info("点云规模: %d 点  范围 X[%.2f, %.2f] Y[%.2f, %.2f] Z[%.2f, %.2f]",
             n, xmin, xmax, ymin, ymax, zmin, zmax)
    return pts


def load_image_safe(image_path: str):
    """
    优先用 Pillow 读图，避免 NumPy 2.x 下 import cv2 触发 ABI 崩溃。
    返回 (h, w, 3) numpy 数组，通道顺序与 cv2.imread 一致 (BGR)，便于后续若使用 cv2 时兼容。
    """
    import numpy as np
    path = Path(image_path)
    if not path.exists():
        raise FileNotFoundError(f"图像不存在: {image_path}")
    try:
        from PIL import Image
        img = Image.open(path)
        if img.mode != "RGB":
            img = img.convert("RGB")
        arr = np.asarray(img, dtype=np.uint8)
        # BGR 顺序以与 cv2.imread 一致（若后续 --use_pnp 用 cv2）
        arr = arr[:, :, ::-1].copy()
        LOG.info("图像加载: Pillow  路径=%s  尺寸=%dx%d (避免 NumPy 2.x 下 cv2 崩溃)",
                 image_path, arr.shape[1], arr.shape[0])
        return arr
    except ImportError:
        pass
    try:
        import cv2
        image = cv2.imread(str(path))
        if image is None:
            raise FileNotFoundError(f"无法读取图像: {image_path}")
        LOG.info("图像加载: cv2  路径=%s  尺寸=%dx%d", image_path, image.shape[1], image.shape[0])
        return image
    except Exception as e:
        LOG.error("读取图像失败 (Pillow/cv2 均不可用或出错): %s", e)
        raise


def _read_pcd_ascii(pcd_path: str):
    """简单 ASCII PCD 读取（FIELDS x y z 或 x y z intensity 等）。"""
    import numpy as np
    with open(pcd_path, "r", encoding="utf-8", errors="ignore") as f:
        lines = f.readlines()
    i = 0
    while i < len(lines):
        line = lines[i].strip().lower()
        if line.startswith("points"):
            n = int(line.split()[-1])
            break
        i += 1
    else:
        raise ValueError("PCD 未找到 POINTS 行")
    # 下一行 DATA ascii，再后为数据
    while i < len(lines) and "data" not in lines[i].lower():
        i += 1
    i += 1
    rows = []
    for j in range(i, min(i + n, len(lines))):
        parts = lines[j].split()
        if len(parts) >= 3:
            rows.append([float(parts[0]), float(parts[1]), float(parts[2])])
    return np.array(rows, dtype=np.float64) if rows else np.zeros((0, 3), dtype=np.float64)


def compute_coarse_identity():
    """恒等外参：T_cam_lidar = I，即 R = I, t = 0。"""
    import numpy as np
    R = np.eye(3, dtype=np.float64)
    t = np.zeros(3, dtype=np.float64)
    return R, t


def _project_lidar_to_image(pts_lidar, R, t, K, dist):
    """将 LiDAR 系下的点用 T_cam_lidar (R, t) 投影到图像平面。"""
    import numpy as np
    import cv2
    pts_cam = (R.astype(np.float64) @ pts_lidar.T).T + np.asarray(t, dtype=np.float64).ravel()
    proj, _ = cv2.projectPoints(pts_cam, np.zeros(3), np.zeros(3), K, dist)
    return proj.reshape(-1, 2), pts_cam


def _build_2d3d_pairs_from_edges(pts_lidar, proj_2d, edge_pts_2d, max_pixel_dist=15):
    """
    对每个落在图像内的投影点，找最近图像边缘点，形成 (图像2D, 雷达3D) 对。
    edge_pts_2d: (N, 2) xy；proj_2d: (M, 2)。
    """
    import numpy as np
    h, w = 1e9, 1e9  # 不在此处裁剪，由调用方保证
    mask = np.ones(proj_2d.shape[0], dtype=bool)
    if mask.sum() < 6:
        return np.zeros((0, 3)), np.zeros((0, 2))
    obj_list, img_list = [], []
    for i in range(proj_2d.shape[0]):
        if not mask[i]:
            continue
        p = proj_2d[i]
        d = np.sqrt(np.sum((edge_pts_2d - p) ** 2, axis=1))
        j = np.argmin(d)
        if d[j] <= max_pixel_dist:
            obj_list.append(pts_lidar[i])
            img_list.append(edge_pts_2d[j])
    if len(obj_list) < 6:
        return np.zeros((0, 3)), np.zeros((0, 2))
    return np.array(obj_list, dtype=np.float64), np.array(img_list, dtype=np.float64)


def _check_cv2_available():
    """
    检测 cv2 是否可安全导入（避免 NumPy 2.x 与 NumPy 1.x 编译的 OpenCV ABI 冲突）。
    返回 True 表示可用，False 表示不可用并已打 WARNING。
    """
    try:
        import cv2  # noqa: F401
        _ = cv2.__version__
        return True
    except ImportError as e:
        LOG.warning("[粗标定] cv2 不可用 (ImportError)，跳过 PnP，将输出 identity。原因: %s", e)
        return False
    except AttributeError as e:
        LOG.warning(
            "[粗标定] cv2 导入后异常 (多为 NumPy 2.x 与 OpenCV ABI 不兼容，如 _ARRAY_API not found)，"
            "跳过 PnP，将输出 identity。建议: 安装 numpy<2 或支持 NumPy 2 的 opencv-python。异常: %s",
            e,
        )
        return False
    except Exception as e:
        LOG.warning("[粗标定] cv2 检测失败: %s  跳过 PnP，将输出 identity", e)
        return False


def compute_coarse_pnp(pts_lidar, image, K, dist):
    """
    通过图像边缘与点云投影建立 2D-3D 对应，用 PnP+RANSAC 求外参。
    对平移做粗网格搜索，用图像边缘点作为 2D、对应雷达点作为 3D，取 inlier 最多的解。
    依赖: cv2。若 cv2 不可用（如 NumPy 2.x 与 OpenCV 不兼容）则返回 None。
    """
    import numpy as np
    try:
        import cv2
    except (ImportError, AttributeError, Exception) as e:
        LOG.warning("[PnP] 无法导入 cv2，跳过 PnP: %s", e)
        return None
    h, w = image.shape[:2]
    n_pts = pts_lidar.shape[0]
    LOG.info("[PnP] 输入: 点云=%d 点  图像=%dx%d  内参 fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
             n_pts, w, h, K[0, 0], K[1, 1], K[0, 2], K[1, 2])
    if n_pts < 6:
        LOG.warning("[PnP] 点云不足 6 点，跳过  n_pts=%d", n_pts)
        return None
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    edges = cv2.Canny(gray, 50, 150)
    edge_ys, edge_xs = np.where(edges > 0)
    edge_pts_2d = np.column_stack([edge_xs.astype(np.float64), edge_ys.astype(np.float64)])
    n_edges = len(edge_pts_2d)
    if n_edges < 6:
        LOG.warning("[PnP] 图像边缘点过少，跳过  n_edges=%d", n_edges)
        return None
    LOG.info("[PnP] 图像边缘点数=%d  网格搜索 (tz×tx×ty) 开始", n_edges)
    best_R, best_t = None, None
    best_inliers = -1
    best_t_guess = None
    n_hypotheses = 0
    tz_range = [0.3, 0.6, 1.0, 1.5, 2.0]
    tx_range = [-0.3, 0.0, 0.3]
    ty_range = [-0.2, 0.0, 0.2]
    for tz in tz_range:
        for tx in tx_range:
            for ty in ty_range:
                t_guess = np.array([tx, ty, tz], dtype=np.float64)
                R_guess = np.eye(3, dtype=np.float64)
                proj_2d, _ = _project_lidar_to_image(pts_lidar, R_guess, t_guess, K, dist)
                mask = (
                    (proj_2d[:, 0] >= 0) & (proj_2d[:, 0] < w) &
                    (proj_2d[:, 1] >= 0) & (proj_2d[:, 1] < h)
                )
                if mask.sum() < 6:
                    continue
                obj_pts, img_pts = _build_2d3d_pairs_from_edges(
                    pts_lidar[mask], proj_2d[mask], edge_pts_2d, max_pixel_dist=15
                )
                if len(obj_pts) < 6:
                    continue
                n_hypotheses += 1
                ok, rvec, tvec, inliers = cv2.solvePnPRansac(
                    obj_pts, img_pts, K, dist,
                    flags=cv2.SOLVEPNP_ITERATIVE,
                    reprojectionError=8.0,
                    confidence=0.95,
                )
                if not ok or inliers is None:
                    continue
                nin = inliers.size
                if nin > best_inliers and nin >= 6:
                    best_inliers = nin
                    R, _ = cv2.Rodrigues(rvec)
                    best_R = R.astype(np.float64)
                    best_t = tvec.ravel().astype(np.float64)
                    best_t_guess = (tx, ty, tz)
    if best_R is not None and best_inliers >= 6:
        LOG.info("[PnP] 网格搜索完成  假设数=%d  最佳 inliers=%d  初值 t_guess=(%.2f,%.2f,%.2f) → 结果 t=[%.4f, %.4f, %.4f]",
                 n_hypotheses, best_inliers,
                 best_t_guess[0], best_t_guess[1], best_t_guess[2],
                 best_t[0], best_t[1], best_t[2])
        return best_R, best_t
    # 回退：恒等投影 + 单次 PnP
    LOG.info("[PnP] 网格搜索无有效解 (最佳 inliers=%d, 假设数=%d)，回退恒等投影 PnP", best_inliers, n_hypotheses)
    rvec = np.zeros(3, dtype=np.float64)
    tvec = np.zeros(3, dtype=np.float64)
    proj, _ = cv2.projectPoints(pts_lidar, rvec, tvec, K, dist)
    proj = proj.reshape(-1, 2)
    mask = (
        (proj[:, 0] >= 0) & (proj[:, 0] < w) &
        (proj[:, 1] >= 0) & (proj[:, 1] < h)
    )
    n_in_view = int(mask.sum())
    obj_pts = pts_lidar[mask]
    img_pts = proj[mask]
    if len(obj_pts) < 6:
        LOG.warning("[PnP] 回退路径: 恒等投影落在视野内点数=%d (<6)，失败", n_in_view)
        return None
    ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        LOG.warning("[PnP] 回退路径: solvePnP 失败  视野内点=%d", n_in_view)
        return None
    R, _ = cv2.Rodrigues(rvec)
    # OpenCV 返回 T_cam_lidar: p_cam = R@p_lidar + t，与 _project_lidar_to_image 约定一致
    t = tvec.ravel().astype(np.float64)
    LOG.info("[PnP] 回退路径成功  视野内点=%d  t=[%.4f, %.4f, %.4f]", n_in_view, t[0], t[1], t[2])
    return R.astype(np.float64), t


def write_extrinsic_result(output_dir: str, R, t, lidar_id: str = "", camera_id: str = ""):
    """写出 extrinsic_result.yaml；返回写入路径。"""
    import yaml
    os.makedirs(output_dir, exist_ok=True)
    out_path = os.path.join(output_dir, "extrinsic_result.yaml")
    rm = R.tolist()
    trans = t.tolist()
    data = {
        "rotation_matrix": rm,
        "translation": trans,
    }
    if lidar_id:
        data["lidar_id"] = lidar_id
    if camera_id:
        data["camera_id"] = camera_id
    with open(out_path, "w", encoding="utf-8") as f:
        yaml.dump(data, f, default_flow_style=False, allow_unicode=True)
    LOG.info("已写出 extrinsic_result.yaml: %s  translation=[%.4f, %.4f, %.4f]",
             out_path, trans[0], trans[1], trans[2])
    return out_path


def write_quality_report(output_dir: str, report: dict):
    """写出 quality_report.yaml（包含 NCC/RMS/内点率/置信度/诊断建议）。"""
    import yaml
    os.makedirs(output_dir, exist_ok=True)
    out_path = os.path.join(output_dir, "quality_report.yaml")
    with open(out_path, "w", encoding="utf-8") as f:
        yaml.dump(report, f, default_flow_style=False, allow_unicode=True)
    LOG.info("已写出 quality_report.yaml: %s", out_path)
    return out_path


def main() -> int:
    parser = argparse.ArgumentParser(
        description="UniCalib LiDAR-Camera 粗标定：PCD + 图像 + 内参 → extrinsic_result.yaml",
    )
    parser.add_argument("--pcd", required=True, help="点云 .pcd 路径")
    parser.add_argument("--image", required=True, help="图像路径")
    parser.add_argument("--sensor_config", required=True, help="相机/传感器配置 YAML")
    parser.add_argument("--output_dir", required=True, help="输出目录，将写入 extrinsic_result.yaml")
    parser.add_argument("--lidar_id", default="", help="可选 lidar_id")
    parser.add_argument("--camera_id", default="", help="可选 camera_id")
    parser.add_argument("--use_pnp", action="store_true", default=True, help="尝试用 PnP/边缘对齐求初值（默认启用）")
    parser.add_argument("--no_pnp", action="store_true", dest="no_pnp", help="禁用 PnP，仅输出恒等外参")
    parser.add_argument("--model_path", default="", help="可选 MIAS-LCEC/Overlap Transformer 模型路径 (.pth.tar)；有则优先深度模型，失败回退 PnP")
    parser.add_argument("--mias_repo", default="", help="可选 MIAS-LCEC 仓库目录，供深度模型推理导入")
    parser.add_argument("--require-model", action="store_true", dest="require_model", help="已配置 model_path 时必须使用深度模型，不可用时退出(不回退 PnP)")
    parser.add_argument("--no-require-model", action="store_false", dest="require_model", default=False, help="已配置 model_path 但推理不可用时允许回退 PnP（默认）")
    parser.add_argument("--device", choices=("auto", "cpu", "cuda"), default="auto", help="深度模型计算设备: auto=自动检测(不支持的本机 GPU 如 RTX 50 系会回退 CPU), cpu=强制 CPU(建议只读/容器环境), cuda=优先 GPU（默认 auto）")
    parser.add_argument("--quiet", action="store_true", help="减少日志输出")
    # C3M 迭代精化配置（通过环境变量 UNICALIB_C3M_* 也可传入）
    parser.add_argument("--c3m-use-iterative-refine", type=str, default="", help="启用 C3M 迭代精化: true/false (默认根据模型/环境自动)")
    parser.add_argument("--c3m-iter-max", type=int, default=0, help="C3M 迭代精化最大迭代次数 (默认 3)")
    parser.add_argument("--c3m-iter-thresh", type=float, default=0.0, help="C3M 迭代精化收敛阈值 (默认 0.01)")
    parser.add_argument("--c3m-similarity-threshold", type=float, default=0.0, help="C3M 相似度阈值 (默认 0.7)")
    # 质量评估阈值（来自 YAML→C++→CLI；不提供则用 quality_assessment 默认）
    parser.add_argument("--quality-ncc-good", type=float, default=0.0, help="质量阈值: ncc_good (默认 0.3)")
    parser.add_argument("--quality-ncc-acceptable", type=float, default=0.0, help="质量阈值: ncc_acceptable (默认 0.2)")
    parser.add_argument("--quality-rms-good-px", type=float, default=0.0, help="质量阈值: rms_good_px (默认 2.0)")
    parser.add_argument("--quality-rms-acceptable-px", type=float, default=0.0, help="质量阈值: rms_acceptable_px (默认 5.0)")
    parser.add_argument("--quality-inlier-ratio-good", type=float, default=0.0, help="质量阈值: inlier_ratio_good (默认 0.6)")
    parser.add_argument("--quality-inlier-ratio-acceptable", type=float, default=0.0, help="质量阈值: inlier_ratio_acceptable (默认 0.4)")
    args = parser.parse_args()
    # 环境变量可覆盖：UNICALIB_COARSE_REQUIRE_MODEL=true 表示必须用模型
    env_require = os.environ.get("UNICALIB_COARSE_REQUIRE_MODEL", "").strip().lower()
    if env_require in ("1", "true", "yes"):
        args.require_model = True
    elif env_require in ("0", "false", "no"):
        args.require_model = False
    if getattr(args, "no_pnp", False):
        args.use_pnp = False
    if args.device != "auto":
        os.environ["UNICALIB_MIAS_LCEC_DEVICE"] = args.device

    # 应用 C3M 配置参数（命令行 > 环境变量 > 默认值）
    if args.c3m_use_iterative_refine:
        os.environ["UNICALIB_C3M_USE_ITERATIVE_REFINE"] = args.c3m_use_iterative_refine
    if args.c3m_iter_max > 0:
        os.environ["UNICALIB_C3M_ITER_MAX"] = str(args.c3m_iter_max)
    if args.c3m_iter_thresh > 0:
        os.environ["UNICALIB_C3M_ITER_THRESH"] = str(args.c3m_iter_thresh)
    if args.c3m_similarity_threshold > 0:
        os.environ["UNICALIB_C3M_SIMILARITY_THRESHOLD"] = str(args.c3m_similarity_threshold)

    # 质量阈值（可选）：写入环境变量，便于下游统一读取/记录
    if args.quality_ncc_good > 0:
        os.environ["UNICALIB_QUALITY_NCC_GOOD"] = str(args.quality_ncc_good)
    if args.quality_ncc_acceptable > 0:
        os.environ["UNICALIB_QUALITY_NCC_ACCEPTABLE"] = str(args.quality_ncc_acceptable)
    if args.quality_rms_good_px > 0:
        os.environ["UNICALIB_QUALITY_RMS_GOOD_PX"] = str(args.quality_rms_good_px)
    if args.quality_rms_acceptable_px > 0:
        os.environ["UNICALIB_QUALITY_RMS_ACCEPTABLE_PX"] = str(args.quality_rms_acceptable_px)
    if args.quality_inlier_ratio_good > 0:
        os.environ["UNICALIB_QUALITY_INLIER_RATIO_GOOD"] = str(args.quality_inlier_ratio_good)
    if args.quality_inlier_ratio_acceptable > 0:
        os.environ["UNICALIB_QUALITY_INLIER_RATIO_ACCEPTABLE"] = str(args.quality_inlier_ratio_acceptable)

    if args.quiet:
        LOG.setLevel(logging.WARNING)

    t0 = time.perf_counter()
    # 环境信息（便于排查 NumPy/OpenCV 兼容问题）
    try:
        import numpy as np
        LOG.info("环境: numpy=%s", np.__version__)
    except Exception:
        pass

    LOG.info("======== 粗标定开始 ========")
    LOG.info("输入: pcd=%s  image=%s  sensor_config=%s  output_dir=%s",
             args.pcd, args.image, args.sensor_config, args.output_dir)
    if args.lidar_id or args.camera_id:
        LOG.info("传感器 ID: lidar_id=%s  camera_id=%s", args.lidar_id or "(未设)", args.camera_id or "(未设)")

    try:
        LOG.info("[步骤 1/4] 读取 sensor_config...")
        cfg = load_sensor_config(args.sensor_config)
        K, dist = get_camera_k_d(cfg)
    except Exception as e:
        import traceback
        LOG.error("读取 sensor_config 失败: %s", e)
        LOG.error("traceback: %s", traceback.format_exc())
        return 1

    try:
        LOG.info("[步骤 2/4] 读取 PCD...")
        pts = load_pcd_points(args.pcd)
        LOG.info("[步骤 2/4] PCD 读取完成  点数=%d", len(pts))
    except Exception as e:
        import traceback
        LOG.error("读取 PCD 失败: %s", e)
        LOG.error("traceback: %s", traceback.format_exc())
        return 1

    try:
        LOG.info("[步骤 3/4] 读取图像 (优先 Pillow，避免 NumPy 2.x 下 cv2 崩溃)...")
        image = load_image_safe(args.image)
        h, w = image.shape[:2]
        LOG.info("[步骤 3/4] 图像读取完成  尺寸=%dx%d", w, h)
    except Exception as e:
        import traceback
        LOG.error("读取图像失败: %s", e)
        LOG.error("traceback: %s", traceback.format_exc())
        return 1

    R, t = compute_coarse_identity()
    method = "identity"
    # V1: 优先尝试深度模型（若配置了 model_path / mias_repo）
    model_path_opt = (getattr(args, "model_path", "") or os.environ.get("MIAS_LCEC_MODEL", "")).strip()
    mias_repo_opt = (getattr(args, "mias_repo", "") or os.environ.get("MIAS_LCEC_REPO", "")).strip()
    if model_path_opt or mias_repo_opt:
        LOG.info("[粗标定] ===== 尝试深度模型推理 =====")
        LOG.info("[粗标定] 配置: model_path=%s", model_path_opt or "(空)")
        LOG.info("[粗标定] 配置: mias_repo=%s", mias_repo_opt or "(空)")
        LOG.info("[粗标定] require_model=%s", getattr(args, "require_model", False))
        _mias_try_infer = None
        script_dir = Path(__file__).resolve().parent
        if str(script_dir) not in sys.path:
            sys.path.insert(0, str(script_dir))
        try:
            from mias_lcec_infer import try_mias_infer as _mias_try_infer
            LOG.info("[粗标定] mias_lcec_infer 导入成功")
        except ImportError as e:
            LOG.error("[粗标定] mias_lcec_infer 导入失败: %s", e)
            LOG.error("[粗标定] sys.path: %s", sys.path)
        if _mias_try_infer and model_path_opt:
            LOG.info("[粗标定] 开始调用深度模型推理...")
            LOG.info("[粗标定] 输入: pcd=%s image=%s", args.pcd, args.image)
            out = _mias_try_infer(args.pcd, args.image, args.sensor_config, model_path_opt, mias_repo_opt)
            LOG.info("[粗标定] 推理完成，返回值类型: %s", type(out).__name__ if out is not None else "None")
            if out is not None:
                R, t = out[0], out[1]
                method = "mias_lcec"
                LOG.info("[粗标定] 已使用深度模型 (MIAS-LCEC)  model_path=%s  t=[%.4f, %.4f, %.4f]", model_path_opt, t[0], t[1], t[2])
                LOG.info("[粗标定] ★ 结论: 已使用深度模型 (MIAS-LCEC)，初值来自神经网络")
            else:
                if getattr(args, "require_model", False):
                    # 粗标定要求必须使用模型：已配置 model_path 时禁止回退 PnP，直接报错退出
                    LOG.error("[粗标定] 已配置 model_path 且 require_model=true，粗标定必须使用深度模型，但当前无法调用模型推理")
                    LOG.error("[粗标定] 请: 1) 在 Python 3.10 环境下安装 MIAS-LCEC 并确保 LcMatch_CApi 可导入 2) 或使用 MIAS-LCEC 提供的 headless 接口 3) 或传入 --no-require-model/配置 allow_pnp_fallback 以允许回退 PnP")
                    return 1
                # 允许回退：打 WARNING 并继续走 PnP
                LOG.warning("[粗标定] 深度模型未返回结果 (require_model=false)，将回退 PnP 几何初值；请检查: 1) MIAS-LCEC 仓库是否有 infer_unicalib.py 或 LcMatch_CApi 2) Python 版本是否为 3.10 (LcMatch_CApi 需要) 3) model_path 是否正确")
                LOG.warning("[粗标定] ★ 结论: 未使用深度模型，将回退 PnP；原因见上方 [MIAS_LCEC_INFER] 错误日志（如 Read-only file system、ImportError 等）")
        elif not model_path_opt:
            LOG.info("[粗标定] 未配置 model_path，跳过深度模型")
    if method == "identity" and args.use_pnp:
        if not _check_cv2_available():
            LOG.warning("[粗标定] cv2 不可用，不调用 PnP，输出 identity；精标定可能无法收敛，建议修复环境后重试")
        else:
            LOG.info("[粗标定] 使用 PnP/边缘对齐求初值")
            pnp_result = compute_coarse_pnp(pts, image, K, dist)
            if pnp_result is not None:
                R, t = pnp_result
                method = "pnp"
                LOG.info("[粗标定] 方法=PnP 成功  t=[%.4f, %.4f, %.4f]", t[0], t[1], t[2])
            else:
                LOG.warning("[粗标定] PnP 失败，保持恒等外参  t=[0, 0, 0]")
    if method == "identity":
        LOG.info("[粗标定] 方法=identity  输出 t=[0, 0, 0]  (精标定可能无法收敛，建议检查 PnP 或深度模型)")

    # 生成质量报告（统一结构：可复用于 LiDAR-Cam / LiDAR-LiDAR / Cam-Cam）
    quality_report_data = {
        "calibration_type": "lidar_camera_extrinsic",
        "stage": "coarse",
        "reference_sensor": args.lidar_id or "lidar",
        "target_sensor": args.camera_id or "camera",
        "method_used": method,
        "has_valid_extrinsic": method in ("mias_lcec", "pnp"),
        "thresholds": {
            "ncc_good": args.quality_ncc_good if args.quality_ncc_good > 0 else 0.3,
            "ncc_acceptable": args.quality_ncc_acceptable if args.quality_ncc_acceptable > 0 else 0.2,
            "rms_good_px": args.quality_rms_good_px if args.quality_rms_good_px > 0 else 2.0,
            "rms_acceptable_px": args.quality_rms_acceptable_px if args.quality_rms_acceptable_px > 0 else 5.0,
            "inlier_ratio_good": args.quality_inlier_ratio_good if args.quality_inlier_ratio_good > 0 else 0.6,
            "inlier_ratio_acceptable": args.quality_inlier_ratio_acceptable if args.quality_inlier_ratio_acceptable > 0 else 0.4,
        },
    }
    if method == "mias_lcec":
        quality_report_data["ncc"] = -1.0  # 当前粗标定脚本不稳定返回细粒度指标；先写结构，精标定后可更新
        quality_report_data["rms_px"] = -1.0
        quality_report_data["inlier_ratio"] = -1.0
    elif method == "pnp":
        quality_report_data["ncc"] = -1.0
        quality_report_data["rms_px"] = -1.0
        quality_report_data["inlier_ratio"] = -1.0
    else:
        quality_report_data["ncc"] = -1.0
        quality_report_data["rms_px"] = -1.0
        quality_report_data["inlier_ratio"] = -1.0

    quality_report_data["metrics"] = {
        "ncc": quality_report_data.get("ncc", -1.0),
        "rms_px": quality_report_data.get("rms_px", -1.0),
        "inlier_ratio": quality_report_data.get("inlier_ratio", -1.0),
    }

    try:
        LOG.info("[步骤 4/5] 写入 extrinsic_result.yaml...")
        out_path = write_extrinsic_result(
            args.output_dir, R, t,
            lidar_id=args.lidar_id, camera_id=args.camera_id,
        )
    except Exception as e:
        import traceback
        LOG.error("写入 extrinsic_result.yaml 失败: %s", e)
        LOG.error("traceback: %s", traceback.format_exc())
        return 1

    # 尝试导入质量评估模块并生成 quality_report.yaml
    quality_report_path = None
    try:
        LOG.info("[步骤 5/5] 生成 quality_report.yaml...")
        script_dir = Path(__file__).resolve().parent
        if str(script_dir) not in sys.path:
            sys.path.insert(0, str(script_dir))
        from mias_lcec_pytorch.quality_assessment import assess_calibration_quality
        # 评估质量（注意：此处的 NCC/RMS 可能为占位值；先保证结构与阈值链路正确）
        try:
            from mias_lcec_pytorch.quality_assessment import QualityThresholds
            thr = QualityThresholds(
                ncc_good=quality_report_data["thresholds"]["ncc_good"],
                ncc_acceptable=quality_report_data["thresholds"]["ncc_acceptable"],
                rms_good_px=quality_report_data["thresholds"]["rms_good_px"],
                rms_acceptable_px=quality_report_data["thresholds"]["rms_acceptable_px"],
                inlier_ratio_good=quality_report_data["thresholds"]["inlier_ratio_good"],
                inlier_ratio_acceptable=quality_report_data["thresholds"]["inlier_ratio_acceptable"],
            )
            report = assess_calibration_quality(
                ncc=quality_report_data.get("ncc", -1.0),
                rms_px=quality_report_data.get("rms_px", -1.0),
                inlier_ratio=quality_report_data.get("inlier_ratio", -1.0),
                thresholds=thr,
            )
        except Exception:
            report = assess_calibration_quality(
                ncc=quality_report_data.get("ncc", -1.0),
                rms_px=quality_report_data.get("rms_px", -1.0),
                inlier_ratio=quality_report_data.get("inlier_ratio", -1.0),
            )
        quality_report_data.update(report.to_dict())
        quality_report_path = write_quality_report(args.output_dir, quality_report_data)
    except ImportError:
        LOG.debug("[步骤 5/5] 跳过 quality_report.yaml (mias_lcec_pytorch.quality_assessment 不可用)")
    except Exception as e:
        import traceback
        LOG.warning("[步骤 5/5] 生成 quality_report.yaml 失败: %s", e)
        LOG.debug("traceback: %s", traceback.format_exc())

    elapsed = time.perf_counter() - t0
    LOG.info("粗标定完成  方法=%s  耗时=%.2fs  输出=%s", method, elapsed, out_path)
    if method == "mias_lcec":
        LOG.info("[粗标定] ★ 最终: 使用深度模型 (MIAS-LCEC)")
    elif method == "pnp":
        LOG.info("[粗标定] ★ 最终: 使用几何回退 (PnP)，未使用深度模型")
    else:
        LOG.info("[粗标定] ★ 最终: 使用恒等外参 (identity)，未使用深度模型")
    LOG.info("======== 粗标定结束 ========")
    # 保留一行 stdout 便于 C++ 解析：必须包含 method= 以便管线日志明确是否使用深度模型
    print(f"[run_lidar_cam_coarse] OK  method={method}  output={out_path}  elapsed_s={elapsed:.2f}", file=sys.stdout)
    # 未使用深度模型时在 stderr 再打一行诊断摘要，便于从主日志快速定位
    if method != "mias_lcec":
        LOG.warning("[粗标定] 诊断摘要: 未使用深度模型 (method=%s)，详见上方 [MIAS_LCEC_INFER] 错误与修复建议", method)
    return 0


def main_with_traceback() -> int:
    """入口：捕获异常并打完整 traceback，便于 C++ 端日志诊断。"""
    try:
        return main()
    except Exception as e:
        LOG.error("粗标定脚本异常: %s", e)
        import traceback
        LOG.error("完整 traceback:\n%s", traceback.format_exc())
        return 1


if __name__ == "__main__":
    sys.exit(main_with_traceback())
