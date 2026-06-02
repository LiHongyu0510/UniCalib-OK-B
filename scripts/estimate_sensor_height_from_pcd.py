#!/usr/bin/env python3
"""
各雷达使用本传感器 PCD、本传感器坐标系独立拟合地面并计算距地高度（不用安装表）。

- lidar_main:   Z 向上 → 竖直高度 = z - z_ground
- left_front / right_back: Z 向下 → 竖直高度 = z_ground - z
- IMU: 无点云；T_imu_0__lidar_main 约定 p_lidar=T@p_imu，位置=平移列；融合 PCD 拟合地面

外参 calibration_param.yaml 仅用于 IMU 位置及结果表中的相对主雷达差值参考。

详细说明见: docs/SENSOR_HEIGHT_FROM_PCD_CN.md
"""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

try:
    import yaml
except ImportError:
    print("需要 PyYAML: pip install pyyaml", file=sys.stderr)
    sys.exit(1)


@dataclass
class SensorSpec:
    sensor_id: str
    label: str
    z_up: np.ndarray
    x_range: Tuple[float, float]  # 本体系路面 ROI（前方路面所在 X 区间）


SENSORS: Dict[str, SensorSpec] = {
    "lidar_main": SensorSpec(
        "lidar_main", "顶雷达高度", np.array([0.0, 0.0, 1.0]), (3.0, 24.0)
    ),
    "left_front": SensorSpec(
        "left_front", "左前补盲雷达高度", np.array([0.0, 0.0, -1.0]), (-10.0, 4.0)
    ),
    "right_back": SensorSpec(
        "right_back", "右后补盲雷达高度", np.array([0.0, 0.0, -1.0]), (-10.0, 5.0)
    ),
}


def load_pcd_ascii(path: Path) -> np.ndarray:
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    data_i = next(i for i, l in enumerate(lines) if l.strip().upper().startswith("DATA")) + 1
    pts = []
    for line in lines[data_i:]:
        if not line.strip():
            continue
        p = line.split()
        pts.append([float(p[0]), float(p[1]), float(p[2])])
    return np.asarray(pts, dtype=np.float64)


def load_T_4x4(node: dict) -> np.ndarray:
    if node.get("rows") == 3 and node.get("data") and len(node["data"]) == 9:
        R = np.array(node["data"], dtype=np.float64).reshape(3, 3)
        t = np.zeros(3)
        if node.get("translation") and len(node["translation"]) >= 3:
            t = np.array(node["translation"][:3], dtype=np.float64)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t
        return T
    data = node.get("data")
    if data and len(data) == 16:
        return np.array(data, dtype=np.float64).reshape(4, 4)
    raise ValueError("无法解析外参块")


def load_yaml_relaxed(path: Path) -> dict:
    text = path.read_text(encoding="utf-8").replace("!!opencv-matrix", "")
    return yaml.safe_load(text) or {}


def grid_ground_points(
    xyz: np.ndarray,
    x_range: Tuple[float, float],
    y_range: float,
    cell: float,
    z_up: np.ndarray,
    max_slope: float = 0.15,
    max_rough: float = 0.06,
    min_cells: int = 60,
) -> np.ndarray:
    mask = (
        (xyz[:, 0] >= x_range[0])
        & (xyz[:, 0] <= x_range[1])
        & (np.abs(xyz[:, 1]) <= y_range)
    )
    sub = xyz[mask]
    if sub.shape[0] < 200:
        return sub

    # Z 向上：地面为低 Z；Z 向下：地面为高 Z
    if z_up[2] > 0:
        z_cut = np.percentile(sub[:, 2], 18)
        sub = sub[sub[:, 2] <= z_cut + 0.10]
    else:
        z_cut = np.percentile(sub[:, 2], 82)
        sub = sub[sub[:, 2] >= z_cut - 0.10]

    ix = np.floor((sub[:, 0] - x_range[0]) / cell).astype(np.int32)
    iy = np.floor((sub[:, 1] + y_range) / cell).astype(np.int32)
    key = ix * 10000 + iy
    mins: List[np.ndarray] = []
    for k in np.unique(key):
        pts = sub[key == k]
        if pts.shape[0] < 3:
            continue
        if z_up[2] > 0:
            pmin = pts[np.argmin(pts[:, 2])]
        else:
            pmin = pts[np.argmax(pts[:, 2])]
        dxy = np.linalg.norm(pts[:, :2] - pmin[:2], axis=1)
        near = pts[dxy < cell * 1.2]
        if near.shape[0] < 3:
            continue
        dz = near[:, 2].max() - near[:, 2].min()
        if dz > max_rough:
            continue
        if near.shape[0] >= 5:
            order = near[:, 0].argsort()
            sn = near[order]
            dx = sn[-1, 0] - sn[0, 0]
            if dx > 0.05:
                slope = abs(sn[-1, 2] - sn[0, 2]) / dx
                if slope > max_slope:
                    continue
        mins.append(pmin)

    if len(mins) < min_cells:
        if z_up[2] > 0:
            z_cut = np.percentile(sub[:, 2], 25)
            sub2 = sub[sub[:, 2] <= z_cut + 0.05]
            pick = lambda p: p[np.argmin(p[:, 2])]
        else:
            z_cut = np.percentile(sub[:, 2], 75)
            sub2 = sub[sub[:, 2] >= z_cut - 0.05]
            pick = lambda p: p[np.argmax(p[:, 2])]
        ix = np.floor((sub2[:, 0] - x_range[0]) / cell).astype(np.int32)
        iy = np.floor((sub2[:, 1] + y_range) / cell).astype(np.int32)
        key = ix * 10000 + iy
        mins = []
        for k in np.unique(key):
            pts = sub2[key == k]
            mins.append(pick(pts))
    if not mins:
        n_take = min(800, sub.shape[0])
        order = np.argsort(sub[:, 2]) if z_up[2] > 0 else np.argsort(-sub[:, 2])
        return sub[order[:n_take]]
    return np.vstack(mins)


def fit_plane_lsq(points: np.ndarray, z_up: np.ndarray) -> Tuple[np.ndarray, float]:
    if points.shape[0] < 3:
        raise ValueError(f"地面点不足 ({points.shape[0]})，无法拟合平面")
    c = points.mean(axis=0)
    X = points - c
    _, _, vh = np.linalg.svd(X, full_matrices=False)
    n = vh[2]
    if np.dot(n, z_up) < 0:
        n = -n
    n = n / np.linalg.norm(n)
    d = -float(np.dot(n, c))
    return n, d


def fit_ground_robust(
    xyz: np.ndarray,
    x_range: Tuple[float, float],
    y_range: float,
    cell: float,
    z_up: np.ndarray,
) -> Tuple[np.ndarray, float, dict]:
    pts = grid_ground_points(xyz, x_range, y_range, cell, z_up)
    n, d = fit_plane_lsq(pts, z_up)
    mask = (
        (xyz[:, 0] >= x_range[0])
        & (xyz[:, 0] <= x_range[1])
        & (np.abs(xyz[:, 1]) <= y_range)
    )
    roi = xyz[mask]
    if z_up[2] > 0:
        z_cut = np.percentile(roi[:, 2], 20)
        low = roi[roi[:, 2] <= z_cut + 0.06]
    else:
        z_cut = np.percentile(roi[:, 2], 80)
        low = roi[roi[:, 2] >= z_cut - 0.06]
    for _ in range(3):
        dist = np.abs(low @ n + d)
        keep = dist < 0.045
        if keep.sum() < 80:
            keep = dist < np.percentile(dist, 25)
        low = low[keep]
        n, d = fit_plane_lsq(low, z_up)
    dist = np.abs(low @ n + d)
    return n, d, {
        "grid_points": int(pts.shape[0]),
        "refine_points": int(low.shape[0]),
        "rms_mm": float(np.sqrt(np.mean(dist**2)) * 1000),
        "n": n.tolist(),
    }


def z_ground_at_xy(x: float, y: float, n: np.ndarray, d: float) -> float:
    if abs(n[2]) < 1e-8:
        raise ValueError("地平面近似竖直于 XY")
    return -(n[0] * x + n[1] * y + d) / n[2]


def vertical_height(
    origin: np.ndarray, n: np.ndarray, d: float, z_up: np.ndarray
) -> float:
    z_g = z_ground_at_xy(float(origin[0]), float(origin[1]), n, d)
    vec = origin - np.array([0.0, 0.0, z_g])
    return float(np.dot(z_up, vec))


def tilt_from_normal(n: np.ndarray, z_up: np.ndarray) -> Tuple[float, float, float]:
    tilt = float(np.degrees(np.arccos(np.clip(np.dot(z_up, n), -1.0, 1.0))))
    z_axis = z_up / np.linalg.norm(z_up)
    pitch_y = float(np.degrees(np.arctan2(-n[0], n[2] if abs(n[2]) > 1e-6 else 1.0)))
    roll_x = float(np.degrees(np.arctan2(n[1], n[2] if abs(n[2]) > 1e-6 else 1.0)))
    return tilt, pitch_y, roll_x


def load_imu_T_from_calib_param(calib_param: Path) -> np.ndarray:
    """仅从 merged_lidar_main/calibration_param.yaml 读取 T_imu_0__lidar_main。"""
    doc = load_yaml_relaxed(calib_param)
    if "T_imu_0__lidar_main" not in doc:
        raise KeyError("T_imu_0__lidar_main")
    return load_T_4x4(doc["T_imu_0__lidar_main"])


def imu_origin_in_lidar_main(T_lidar_imu: np.ndarray) -> np.ndarray:
    """
    calibration_param.yaml 中 T_imu_0__lidar_main 与 lidar_main__* 同约定：
      p_lidar_main = T @ p_imu  →  IMU 原点在主雷达系 = 平移列 translation。

    勿用 inv(T)[:3,3]（那是 UniCalib 输出 p_imu=T·p_lidar 时的 lidar 原点在 imu 系）。
    """
    return T_lidar_imu[:3, 3].copy()


def estimate_imu_height_from_merged(
    merged_pcd: Path,
    T_imu_lidar: np.ndarray,
    x_range: Tuple[float, float],
    y_range: float,
    cell: float,
) -> Tuple[float, np.ndarray, np.ndarray, float, np.ndarray]:
    """
    IMU 高度：三路雷达已变换到 lidar_main 的融合点云上拟合地面，再算 IMU 原点距地高度。
    返回 (h_imu, p_imu, n, d, xyz)。
    """
    xyz = load_pcd_ascii(merged_pcd)
    z_up = SENSORS["lidar_main"].z_up
    print(f"\n[IMU高度] 融合点云 {merged_pcd.name} ({xyz.shape[0]} 点, lidar_main 系)")
    print(f"  地面 ROI: x∈[{x_range[0]}, {x_range[1]}] m, |y|≤{y_range} m")
    n, d, stats = fit_ground_robust(xyz, x_range, y_range, cell, z_up)
    print(
        f"  地面: 种子 {stats['grid_points']}, 精化 {stats['refine_points']}, "
        f"RMS={stats['rms_mm']:.1f} mm"
    )
    p_imu_in_main = imu_origin_in_lidar_main(T_imu_lidar)
    h = vertical_height(p_imu_in_main, n, d, z_up)
    print(f"  IMU 位置(main) {np.round(p_imu_in_main, 4)}, 距地 {h*1000:.1f} mm")
    return h, p_imu_in_main, n, d, xyz


def detect_struct_origin_main(xyz: np.ndarray, n: np.ndarray, d: float) -> float:
    if abs(n[2]) < 1e-8:
        return float("nan")
    z_g = -(n[0] * xyz[:, 0] + n[1] * xyz[:, 1] + d) / n[2]
    h_vert = xyz[:, 2] - z_g
    body = (
        (xyz[:, 0] >= 0.5)
        & (xyz[:, 0] <= 5.5)
        & (np.abs(xyz[:, 1]) <= 0.6)
        & (h_vert >= 0.08)
        & (h_vert <= 2.2)
    )
    pts, hv = xyz[body], h_vert[body]
    if pts.shape[0] < 40:
        return float("nan")
    x_thr = np.percentile(pts[:, 0], 96)
    fm = pts[:, 0] >= x_thr
    hf = hv[fm]
    if hf.size < 8:
        hf = hv
    return float(np.percentile(hf, 20))


def list_pcds(directory: Path) -> List[Path]:
    files = sorted(directory.glob("*.pcd"))
    if not files:
        raise FileNotFoundError(f"无 PCD: {directory}")
    return files


def pcd_timestamp_ms(path: Path) -> Optional[float]:
    m = re.match(r"^(\d+(?:\.\d+)?)", path.stem)
    return float(m.group(1)) if m else None


def pick_aligned_pcds(dirs: Dict[str, Path], ref: str = "lidar_main") -> Dict[str, Path]:
    lists = {k: list_pcds(v) for k, v in dirs.items()}
    ref_files = lists[ref]
    ref_ts = [pcd_timestamp_ms(p) for p in ref_files]
    if any(t is None for t in ref_ts):
        raise ValueError(f"{ref} 文件名需为时间戳(ms).pcd")
    mid = len(ref_files) // 2
    t0 = ref_ts[mid]
    out = {ref: ref_files[mid]}
    for sid, files in lists.items():
        if sid == ref:
            continue
        ts = [pcd_timestamp_ms(p) for p in files]
        if any(t is None for t in ts):
            raise ValueError(f"{sid} 文件名需为时间戳(ms).pcd")
        j = int(np.argmin([abs(t - t0) for t in ts]))
        out[sid] = files[j]
        print(f"  时间对齐: {ref} {ref_files[mid].name}  <->  {sid} {files[j].name}  (|Δt|={abs(ts[j]-t0):.0f} ms)")
    return out


def estimate_sensor(
    sensor_id: str,
    pcd_path: Path,
    y_range: float,
    cell: float,
    x_range_override: Optional[Tuple[float, float]] = None,
) -> dict:
    spec = SENSORS[sensor_id]
    x_range = x_range_override if x_range_override else spec.x_range
    xyz = load_pcd_ascii(pcd_path)
    print(f"\n[{spec.label}] {pcd_path.name}  ({xyz.shape[0]} 点, 本体系)")
    print(f"  地面 ROI: x∈[{x_range[0]}, {x_range[1]}] m, |y|≤{y_range} m")
    n, d, stats = fit_ground_robust(xyz, x_range, y_range, cell, spec.z_up)
    origin = np.zeros(3)
    h = vertical_height(origin, n, d, spec.z_up)
    tilt, pitch, roll = tilt_from_normal(n, spec.z_up)
    print(
        f"  地面: 种子 {stats['grid_points']}, 精化 {stats['refine_points']}, "
        f"RMS={stats['rms_mm']:.1f} mm"
    )
    print(f"  距地高度 {h*1000:.1f} mm, 倾角(Z与地法向) {tilt:.2f}°")
    return {
        "sensor_id": sensor_id,
        "label": spec.label,
        "pcd": str(pcd_path),
        "height_m": h,
        "tilt_deg": tilt,
        "pitch_deg": pitch,
        "roll_deg": roll,
        "ground_stats": stats,
        "plane_n": n,
        "plane_d": d,
    }


def main() -> int:
    repo = Path(__file__).resolve().parents[1]
    ap = argparse.ArgumentParser(
        description="各雷达用本传感器 PCD 独立估计距地高度"
    )
    ap.add_argument("--pcd-main", type=Path, help="主雷达 PCD")
    ap.add_argument("--pcd-left", type=Path, help="左前补盲 PCD")
    ap.add_argument("--pcd-right", type=Path, help="右后补盲 PCD")
    ap.add_argument(
        "--dir-main",
        type=Path,
        default=repo / "data/lidar_main/top_front_0519",
    )
    ap.add_argument(
        "--dir-left",
        type=Path,
        default=repo / "data/lidar_airy/20260519/left_front",
    )
    ap.add_argument(
        "--dir-right",
        type=Path,
        default=repo / "data/lidar_airy/20260519/right_back",
    )
    ap.add_argument("--frame-index", type=int, default=0, help="各目录排序后第 N 帧")
    ap.add_argument("--align-time", action="store_true", help="按文件名时间戳对齐三雷达")
    ap.add_argument(
        "--calib-param",
        type=Path,
        default=repo / "merged_lidar_main/calibration_param.yaml",
    )
    ap.add_argument(
        "--merged-pcd",
        type=Path,
        default=repo / "merged_lidar_main/merged_idx0.pcd",
        help="三路雷达融合点云 (lidar_main 系)，用于 IMU 地面拟合",
    )
    ap.add_argument(
        "--x-min",
        type=float,
        default=None,
        help="覆盖主雷达地面 ROI 前向下限（补盲仍用各传感器默认 ROI）",
    )
    ap.add_argument("--x-max", type=float, default=None, help="覆盖主雷达地面 ROI 前向上限")
    ap.add_argument("--y-max", type=float, default=4.0)
    ap.add_argument("--grid-cell", type=float, default=0.25)
    args = ap.parse_args()

    main_x_range = (
        (args.x_min if args.x_min is not None else 3.0),
        (args.x_max if args.x_max is not None else 24.0),
    )

    # 解析三路 PCD 路径
    pcds: Dict[str, Path] = {}
    if args.pcd_main:
        pcds["lidar_main"] = args.pcd_main.resolve()
    if args.pcd_left:
        pcds["left_front"] = args.pcd_left.resolve()
    if args.pcd_right:
        pcds["right_back"] = args.pcd_right.resolve()

    dirs = {
        "lidar_main": args.dir_main.resolve(),
        "left_front": args.dir_left.resolve(),
        "right_back": args.dir_right.resolve(),
    }
    if len(pcds) < 3:
        existing = {k: v for k, v in dirs.items() if v.is_dir()}
        if args.align_time:
            if "lidar_main" not in existing:
                print("错误: 时间对齐需要 --dir-main 存在", file=sys.stderr)
                return 1
            print("时间对齐:")
            aligned = pick_aligned_pcds(existing)
            for sid, p in aligned.items():
                if sid not in pcds:
                    pcds[sid] = p
        else:
            print(f"帧选择: frame-index={args.frame_index}")
            for sid, d in dirs.items():
                if sid in pcds:
                    continue
                if not d.is_dir():
                    print(f"警告: 目录不存在 {d}", file=sys.stderr)
                    continue
                files = list_pcds(d)
                idx = args.frame_index
                if idx < 0 or idx >= len(files):
                    print(f"错误: frame-index={idx} 越界 ({len(files)} 帧 @ {sid})", file=sys.stderr)
                    return 1
                pcds[sid] = files[idx]
                print(f"  {sid} -> {files[idx].name}")

    required = ("lidar_main", "left_front", "right_back")
    missing = [s for s in required if s not in pcds]
    if missing:
        print(f"错误: 缺少 PCD: {missing}", file=sys.stderr)
        return 1

    results: List[dict] = []
    for sid in required:
        x_ov = main_x_range if sid == "lidar_main" else None
        results.append(
            estimate_sensor(sid, pcds[sid], args.y_max, args.grid_cell, x_ov)
        )

    h_main = next(r["height_m"] for r in results if r["sensor_id"] == "lidar_main")

    main_r = next(r for r in results if r["sensor_id"] == "lidar_main")
    n_main, d_main = main_r["plane_n"], main_r["plane_d"]

    imu_h = float("nan")
    merged_path = args.merged_pcd.resolve()
    merged_xyz: Optional[np.ndarray] = None
    n_merged: Optional[np.ndarray] = None
    d_merged: Optional[float] = None
    if not args.calib_param.is_file():
        print("警告: 无 calibration_param，跳过 IMU 高度", file=sys.stderr)
    elif not merged_path.is_file():
        print(f"警告: 融合点云不存在 {merged_path}，跳过 IMU 高度", file=sys.stderr)
    else:
        try:
            T_imu = load_imu_T_from_calib_param(args.calib_param.resolve())
        except (KeyError, ValueError) as e:
            print(f"错误: 无法加载 IMU 外参: {e}", file=sys.stderr)
            T_imu = None
        if T_imu is not None:
            imu_h, _, n_merged, d_merged, merged_xyz = estimate_imu_height_from_merged(
                merged_path,
                T_imu,
                main_x_range,
                args.y_max,
                args.grid_cell,
            )

    struct_h = float("nan")
    try:
        if merged_xyz is not None and n_merged is not None and d_merged is not None:
            struct_h = detect_struct_origin_main(merged_xyz, n_merged, d_merged)
        else:
            main_xyz = load_pcd_ascii(pcds["lidar_main"])
            struct_h = detect_struct_origin_main(main_xyz, n_main, d_main)
    except Exception:
        pass

    print("\n" + "=" * 60)
    print("汇总（各雷达用本传感器 PCD + 本体系地面，mm）")
    print("=" * 60)
    for r in results:
        delta = (r["height_m"] - h_main) * 1000
        print(f"  {r['label']:18s}  {r['height_m']*1000:8.1f}   (Δ相对主雷达 {delta:+.0f} mm)")
    if not np.isnan(imu_h):
        print(
            f"  {'IMU高度':18s}  {imu_h*1000:8.1f}   "
            f"(Δ相对主雷达 {(imu_h-h_main)*1000:+.0f} mm, 融合点云地面)"
        )
    if not np.isnan(struct_h):
        # print(f"  {'结构原点高度':18s}  {struct_h*1000:8.1f}   (主雷达点云车头近似)")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())
