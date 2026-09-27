#!/usr/bin/env python3
"""
各雷达在本传感器坐标系下，分别拟合地面/墙面平面，统计点云沿法向的厚度。

厚度定义：拟合平面内点沿法向 signed distance 的分布宽度
  - p95_p5: 95% 分位 − 5% 分位（主指标，mm）
  - std: 标准差（mm）
  - rms: 均方根（mm）

地面 ROI / 拟合方法与 estimate_sensor_height_from_pcd.py 一致。
墙面：ROI 内 RANSAC 拟合近似竖直平面（|n·z_up| ≤ max_tilt），取内点最多的一面。

详细地面约定见 docs/SENSOR_HEIGHT_FROM_PCD_CN.md
"""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

from estimate_sensor_height_from_pcd import (
    SENSORS,
    fit_ground_robust,
    list_pcds,
    load_pcd_ascii,
    pick_aligned_pcds,
)

# 目录名 → sensor_id（vanjee 采集常用）
DIR_TO_SENSOR: Dict[str, str] = {
    "left_front": "left_front",
    "left_front_vanjee": "left_front",
    "right_back": "right_back",
    "right_back_vanjee": "right_back",
    "left_back": "left_back",
    "left_back_vanjee": "left_back",
    "right_front": "right_front",
    "right_front_vanjee": "right_front",
}

DEFAULT_VANJEE_ROOT = (
    Path(__file__).resolve().parents[1] / "data/20260701/record/lidar_airy"
)
DEFAULT_VANJEE_SENSORS = [
    "left_front_vanjee",
    "left_back_vanjee",
    "right_back_vanjee",
    "right_front_vanjee",
]


@dataclass
class ThicknessStats:
    inlier_count: int
    p95_p5_mm: float
    std_mm: float
    rms_mm: float
    p50_mm: float
    max_abs_mm: float


@dataclass
class WallSpec:
    side: str
    y_range: Tuple[float, float]
    x_range: Tuple[float, float]


WALL_ROIS: Dict[str, List[WallSpec]] = {
    "left_front": [
        WallSpec("left", (-10.0, -2.5), (-2.0, 15.0)),
        WallSpec("right", (2.5, 10.0), (-2.0, 15.0)),
    ],
    "right_back": [
        WallSpec("left", (-10.0, -2.5), (-2.0, 15.0)),
        WallSpec("right", (2.5, 10.0), (-2.0, 15.0)),
    ],
    "lidar_main": [
        WallSpec("left", (-10.0, -2.5), (3.0, 24.0)),
        WallSpec("right", (2.5, 10.0), (3.0, 24.0)),
    ],
    "left_back": [
        WallSpec("left", (-10.0, -2.5), (-2.0, 15.0)),
        WallSpec("right", (2.5, 10.0), (-2.0, 15.0)),
    ],
    "right_front": [
        WallSpec("left", (-10.0, -2.5), (-2.0, 15.0)),
        WallSpec("right", (2.5, 10.0), (-2.0, 15.0)),
    ],
}


def thickness_from_distances(dist: np.ndarray) -> ThicknessStats:
    if dist.size == 0:
        return ThicknessStats(0, float("nan"), float("nan"), float("nan"), float("nan"), float("nan"))
    ad = np.abs(dist)
    p5, p50, p95 = np.percentile(ad, [5, 50, 95])
    return ThicknessStats(
        inlier_count=int(dist.size),
        p95_p5_mm=float((p95 - p5) * 1000),
        std_mm=float(ad.std() * 1000),
        rms_mm=float(np.sqrt(np.mean(ad**2)) * 1000),
        p50_mm=float(p50 * 1000),
        max_abs_mm=float(ad.max() * 1000),
    )


def ground_inliers(
    xyz: np.ndarray,
    n: np.ndarray,
    d: float,
    x_range: Tuple[float, float],
    y_range: float,
    band_m: float = 0.08,
) -> np.ndarray:
    mask = (
        (xyz[:, 0] >= x_range[0])
        & (xyz[:, 0] <= x_range[1])
        & (np.abs(xyz[:, 1]) <= y_range)
    )
    roi = xyz[mask]
    dist = roi @ n + d
    return dist[np.abs(dist) <= band_m]


def ransac_vertical_plane(
    xyz: np.ndarray,
    z_up: np.ndarray,
    max_tilt: float = 0.25,
    dist_thresh: float = 0.05,
    iters: int = 2500,
    seed: int = 42,
) -> Optional[Tuple[np.ndarray, float, np.ndarray]]:
    n_pts = xyz.shape[0]
    if n_pts < 80:
        return None
    rng = np.random.default_rng(seed)
    best: Optional[Tuple[int, np.ndarray, float, np.ndarray]] = None
    for _ in range(iters):
        idx = rng.choice(n_pts, 3, replace=False)
        p1, p2, p3 = xyz[idx]
        nn = np.cross(p2 - p1, p3 - p1)
        norm = np.linalg.norm(nn)
        if norm < 1e-8:
            continue
        nn = nn / norm
        if abs(float(np.dot(nn, z_up))) > max_tilt:
            continue
        dd = -float(np.dot(nn, p1))
        dist = xyz @ nn + dd
        inl = np.abs(dist) <= dist_thresh
        cnt = int(inl.sum())
        if cnt < 80:
            continue
        if best is None or cnt > best[0]:
            best = (cnt, nn, dd, dist[inl])
    if best is None:
        return None
    _, nn, dd, dist_inl = best
    return nn, dd, dist_inl


def fit_wall_thickness(
    xyz: np.ndarray,
    z_up: np.ndarray,
    wall_specs: List[WallSpec],
    ground_n: np.ndarray,
    ground_d: float,
    min_ground_sep: float = 0.25,
) -> Optional[dict]:
    best_side: Optional[dict] = None
    for ws in wall_specs:
        mask = (
            (xyz[:, 1] >= ws.y_range[0])
            & (xyz[:, 1] <= ws.y_range[1])
            & (xyz[:, 0] >= ws.x_range[0])
            & (xyz[:, 0] <= ws.x_range[1])
            & (np.abs(xyz @ ground_n + ground_d) >= min_ground_sep)
        )
        sub = xyz[mask]
        fit = ransac_vertical_plane(sub, z_up)
        if fit is None:
            continue
        nn, dd, dist_inl = fit
        stats = thickness_from_distances(dist_inl)
        side = {
            "side": ws.side,
            "y_range": ws.y_range,
            "inliers": stats.inlier_count,
            "plane_n": nn,
            "plane_d": dd,
            "thickness": stats,
        }
        if best_side is None or side["inliers"] > best_side["inliers"]:
            best_side = side
    return best_side


def analyze_sensor(
    sensor_id: str,
    pcd_path: Path,
    y_range: float,
    cell: float,
    ground_band: float,
    x_range_override: Optional[Tuple[float, float]] = None,
) -> dict:
    spec = SENSORS[sensor_id]
    x_range = x_range_override if x_range_override else spec.x_range
    xyz = load_pcd_ascii(pcd_path)
    print(f"\n[{sensor_id}] {pcd_path.name}  ({xyz.shape[0]} 点, 本体系)")

    n, d, gstats = fit_ground_robust(xyz, x_range, y_range, cell, spec.z_up)
    gdist = ground_inliers(xyz, n, d, x_range, y_range, ground_band)
    gthick = thickness_from_distances(gdist)
    print(
        f"  地面 ROI x∈[{x_range[0]}, {x_range[1]}], |y|≤{y_range}, "
        f"内点 {gthick.inlier_count}, 拟合RMS {gstats['rms_mm']:.1f} mm"
    )
    print(
        f"  地面厚度: p95-p5={gthick.p95_p5_mm:.1f} mm, "
        f"std={gthick.std_mm:.1f} mm, rms={gthick.rms_mm:.1f} mm"
    )

    wall = fit_wall_thickness(
        xyz, spec.z_up, WALL_ROIS.get(sensor_id, []), n, d
    )
    if wall is None:
        print("  墙面: 未找到足够竖直平面内点")
        wthick = None
    else:
        wt: ThicknessStats = wall["thickness"]
        print(
            f"  墙面({wall['side']}, y∈{wall['y_range']}): 内点 {wt.inlier_count}, "
            f"n={np.round(wall['plane_n'], 3)}"
        )
        print(
            f"  墙面厚度: p95-p5={wt.p95_p5_mm:.1f} mm, "
            f"std={wt.std_mm:.1f} mm, rms={wt.rms_mm:.1f} mm"
        )
        wthick = wt

    return {
        "sensor_id": sensor_id,
        "pcd": str(pcd_path),
        "ground": gthick,
        "ground_fit_rms_mm": gstats["rms_mm"],
        "wall": wthick,
        "wall_side": wall["side"] if wall else None,
    }


def resolve_sensor_dirs(
    dataset: Path, dir_names: List[str]
) -> Tuple[Dict[str, Path], Dict[str, str]]:
    """返回 {sensor_id: pcd_dir} 与 {sensor_id: 源目录名}。"""
    sensor_dirs: Dict[str, Path] = {}
    labels: Dict[str, str] = {}
    for dirname in dir_names:
        d = dataset / dirname
        if not d.is_dir():
            raise FileNotFoundError(f"目录不存在: {d}")
        sid = DIR_TO_SENSOR.get(dirname, dirname)
        if sid not in SENSORS:
            raise KeyError(f"未知 sensor_id: {sid} (来自目录 {dirname})")
        sensor_dirs[sid] = d
        labels[sid] = dirname
    return sensor_dirs, labels


def aggregate_results(rows: List[dict], labels: Optional[Dict[str, str]] = None) -> None:
    print("\n" + "=" * 72)
    print("汇总 — 点云厚度 (mm)，地面/墙面主指标为 p95−p5")
    print("=" * 72)
    hdr = f"{'传感器':14s}  {'地面p95-p5':>10s}  {'地面std':>8s}  {'墙面p95-p5':>10s}  {'墙面std':>8s}  墙面侧"
    print(hdr)
    print("-" * 72)
    for r in rows:
        g: ThicknessStats = r["ground"]
        w: Optional[ThicknessStats] = r["wall"]
        ws = r["wall_side"] or "-"
        wpp = f"{w.p95_p5_mm:10.1f}" if w else f"{'N/A':>10s}"
        wstd = f"{w.std_mm:8.1f}" if w else f"{'N/A':>8s}"
        sid = r["sensor_id"]
        if labels and sid in labels:
            sid = f"{sid} ({labels[sid]})"
        print(
            f"{sid:28s}  {g.p95_p5_mm:10.1f}  {g.std_mm:8.1f}  "
            f"{wpp}  {wstd}  {ws}"
        )
    print("=" * 72)


def main() -> int:
    ap = argparse.ArgumentParser(description="各雷达地面/墙面点云厚度")
    ap.add_argument(
        "--dataset",
        type=Path,
        default=DEFAULT_VANJEE_ROOT,
        help="含各雷达 PCD 子目录的根路径",
    )
    ap.add_argument(
        "--sensors",
        nargs="+",
        default=DEFAULT_VANJEE_SENSORS,
        help="子目录名（如 left_front_vanjee）或 sensor_id",
    )
    ap.add_argument("--pcd", type=Path, help="单帧 PCD（仅 --sensors 指定一个传感器时）")
    ap.add_argument("--frame-index", type=int, default=0)
    ap.add_argument("--align-time", action="store_true")
    ap.add_argument("--y-max", type=float, default=4.0, help="地面 ROI |y| 上限")
    ap.add_argument("--grid-cell", type=float, default=0.25)
    ap.add_argument("--ground-band", type=float, default=0.08, help="地面内点距平面阈值 (m)")
    ap.add_argument(
        "--multi-frame",
        type=int,
        default=0,
        help=">0 时对前 N 帧取厚度中位数（按 frame-index 起算）",
    )
    args = ap.parse_args()

    dataset = args.dataset.resolve()
    try:
        sensor_dirs, labels = resolve_sensor_dirs(dataset, args.sensors)
    except (FileNotFoundError, KeyError) as e:
        print(f"错误: {e}", file=sys.stderr)
        return 1
    sensor_ids = list(sensor_dirs.keys())
    ref_sid = DIR_TO_SENSOR.get(args.sensors[0], args.sensors[0])
    if ref_sid not in sensor_dirs:
        ref_sid = sensor_ids[0]

    if args.pcd:
        if len(sensor_ids) != 1:
            print("错误: --pcd 仅支持单个传感器", file=sys.stderr)
            return 1
        pcd_map = {sensor_ids[0]: args.pcd.resolve()}
    elif args.align_time and len(sensor_ids) > 1:
        print("时间对齐:")
        pcd_map = pick_aligned_pcds(sensor_dirs, ref=ref_sid)
    else:
        pcd_map = {}
        n_frames = max(1, args.multi_frame)
        for sid, d in sensor_dirs.items():
            files = list_pcds(d)
            indices = range(args.frame_index, min(args.frame_index + n_frames, len(files)))
            pcd_map[sid] = files[args.frame_index]
            if n_frames > 1:
                print(f"  {sid} ({labels[sid]}): 聚合帧 {len(indices)} 帧")

    # 多帧聚合
    if args.multi_frame > 1 and not args.pcd:
        all_rows: Dict[str, List[dict]] = {sid: [] for sid in sensor_ids}
        for sid, d in sensor_dirs.items():
            files = list_pcds(d)
            for fi in range(
                args.frame_index,
                min(args.frame_index + args.multi_frame, len(files)),
            ):
                all_rows[sid].append(
                    analyze_sensor(sid, files[fi], args.y_max, args.grid_cell, args.ground_band)
                )
        rows = []
        for sid in sensor_ids:
            rs = all_rows[sid]
            g_pp = np.median([r["ground"].p95_p5_mm for r in rs])
            g_std = np.median([r["ground"].std_mm for r in rs])
            w_rs = [r for r in rs if r["wall"] is not None]
            if w_rs:
                w_pp = np.median([r["wall"].p95_p5_mm for r in w_rs])
                w_std = np.median([r["wall"].std_mm for r in w_rs])
                w_side = w_rs[0]["wall_side"]
            else:
                w_pp, w_std, w_side = float("nan"), float("nan"), None
            rows.append(
                {
                    "sensor_id": sid,
                    "pcd": f"{args.multi_frame} frames @ {sid}",
                    "ground": ThicknessStats(0, g_pp, g_std, float("nan"), float("nan"), float("nan")),
                    "wall": ThicknessStats(0, w_pp, w_std, float("nan"), float("nan"), float("nan"))
                    if w_rs
                    else None,
                    "wall_side": w_side,
                }
            )
        aggregate_results(rows, labels)
        return 0

    rows = []
    for sid in sensor_ids:
        rows.append(
            analyze_sensor(
                sid, pcd_map[sid], args.y_max, args.grid_cell, args.ground_band
            )
        )
    aggregate_results(rows, labels)
    return 0


if __name__ == "__main__":
    sys.exit(main())
