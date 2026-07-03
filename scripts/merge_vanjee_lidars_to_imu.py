#!/usr/bin/env python3
"""
将万集补盲雷达点云按 imu_in_<sensor> 外参融合到 IMU 坐标系。

外参约定（calibration_param.yaml 中 imu_in_*）:
  p_lidar = T @ p_imu  →  融合时 p_imu = inv(T) @ p_lidar

用法:
  python3 scripts/merge_vanjee_lidars_to_imu.py \\
    --dataset 20260625_3301 \\
    --sensors left_front_vanjee right_front_vanjee
"""

from __future__ import annotations

import argparse
import bisect
import sys
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np

try:
    import yaml
except ImportError:
    print("需要 PyYAML: pip install pyyaml", file=sys.stderr)
    sys.exit(1)

from merge_three_lidars_to_main import (
    load_pcd_ascii,
    load_yaml_relaxed,
    transform_points,
    write_pcd_ascii,
    xyz_intensity_from_array,
)


def load_imu_in_extrinsic(calib: dict, sensor_key: str) -> np.ndarray:
    """读取 imu_in_<sensor_key>_rotation/translation，返回 4x4 (p_lidar = T @ p_imu)。"""
    rot_key = f"imu_in_{sensor_key}_rotation"
    trans_key = f"imu_in_{sensor_key}_translation"
    if rot_key not in calib or trans_key not in calib:
        raise KeyError(f"calibration_param.yaml 缺少 {rot_key} / {trans_key}")
    R = np.array(calib[rot_key]["data"], dtype=np.float64).reshape(3, 3)
    t = np.array(calib[trans_key]["data"], dtype=np.float64).reshape(3)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def load_T_imu_0_extrinsic(calib: dict, sensor_key: str) -> np.ndarray:
    """读取 T_imu_0__<sensor_key>（data + translation），返回 4x4 (p_lidar = T @ p_imu)。"""
    key = f"T_imu_0__{sensor_key}"
    if key not in calib:
        raise KeyError(f"calibration_param.yaml 缺少 {key}")
    block = calib[key]
    R = np.array(block["data"], dtype=np.float64).reshape(3, 3)
    t = np.array(block["translation"], dtype=np.float64).reshape(3)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def T_lidar_to_imu(T_imu_in_lidar: np.ndarray) -> np.ndarray:
    """p_lidar = T @ p_imu  →  p_imu = inv(T) @ p_lidar"""
    return np.linalg.inv(T_imu_in_lidar)


def ts_list(directory: Path) -> List[int]:
    return sorted(int(p.stem) for p in directory.glob("*.pcd"))


def ts_map(directory: Path) -> Dict[int, Path]:
    return {int(p.stem): p for p in directory.glob("*.pcd")}


def nearest_ts(ts: int, arr: List[int]) -> int:
    i = bisect.bisect_left(arr, ts)
    c: List[int] = []
    if i < len(arr):
        c.append(arr[i])
    if i > 0:
        c.append(arr[i - 1])
    return min(c, key=lambda x: abs(x - ts))


def align_pairs(
    ref_name: str,
    ref_dir: Path,
    other_dirs: Dict[str, Path],
    max_dt_ms: int,
) -> List[Tuple[int, Dict[str, Path], int]]:
    ref_map = ts_map(ref_dir)
    ref_ts = sorted(ref_map)
    other_maps = {k: ts_map(v) for k, v in other_dirs.items()}
    other_lists = {k: sorted(m) for k, m in other_maps.items()}

    rows: List[Tuple[int, Dict[str, Path], int]] = []
    for ts in ref_ts:
        paths = {ref_name: ref_map[ts]}
        max_dt = 0
        ok = True
        for sid, ts_arr in other_lists.items():
            nt = nearest_ts(ts, ts_arr)
            dt = abs(nt - ts)
            if dt > max_dt_ms:
                ok = False
                break
            max_dt = max(max_dt, dt)
            paths[sid] = other_maps[sid][nt]
        if ok:
            rows.append((ts, paths, max_dt))
    return rows


def main() -> int:
    repo = Path(__file__).resolve().parents[1]
    ap = argparse.ArgumentParser(description="万集雷达点云融合到 IMU 坐标系")
    ap.add_argument("--calib", type=Path, default=repo / "merged_lidar_main/calibration_param.yaml")
    ap.add_argument("--data-root", type=Path, default=repo / "data/lidar_airy")
    ap.add_argument("--dataset", type=str, default="20260625_3301")
    ap.add_argument(
        "--sensors",
        nargs="+",
        default=["left_front_vanjee", "right_front_vanjee"],
        help="雷达目录名（不含 imu_in_ 后缀，如 left_front / right_front）",
    )
    ap.add_argument(
        "--sensor-dirs",
        nargs="+",
        default=None,
        help="雷达 PCD 目录名（默认与 --sensors 相同）",
    )
    ap.add_argument(
        "--extrinsic-keys",
        nargs="+",
        default=None,
        help="calibration_param 中 imu_in_<key> 后缀（默认 left_front / right_front）",
    )
    ap.add_argument("--ref", type=str, default=None, help="时间对齐基准 sensor_id")
    ap.add_argument(
        "--sensor-path",
        action="append",
        nargs=2,
        metavar=("SENSOR_ID", "PCD_DIR"),
        help="显式指定 sensor_id 与 PCD 目录，可重复；指定后忽略 --dataset/--sensors",
    )
    ap.add_argument(
        "--extrinsic-style",
        action="append",
        nargs=2,
        metavar=("SENSOR_ID", "STYLE"),
        help="外参键风格: imu_in（默认）或 T_imu_0，如 lidar_main T_imu_0",
    )
    ap.add_argument("--max-dt-ms", type=int, default=10)
    ap.add_argument("--output", type=Path, default=None)
    ap.add_argument("--color", action="store_true")
    args = ap.parse_args()

    ext_style_map = {k: v for k, v in (args.extrinsic_style or [])}

    if args.sensor_path:
        sensor_ids = [sid for sid, _ in args.sensor_path]
        dirs = {sid: Path(p).resolve() for sid, p in args.sensor_path}
        ext_keys = args.extrinsic_keys or [
            sid.replace("_vanjee", "") for sid in sensor_ids
        ]
        dataset_label = "custom"
    else:
        sensor_dirs = args.sensor_dirs or args.sensors
        ext_keys = args.extrinsic_keys or [
            s.replace("_vanjee", "") for s in sensor_dirs
        ]
        sensor_ids = sensor_dirs
        dataset_dir = args.data_root / args.dataset
        dirs = {d: dataset_dir / d for d in sensor_dirs}
        dataset_label = str(dataset_dir)

    if len(sensor_ids) != len(ext_keys):
        print("sensor 数量与 --extrinsic-keys 数量须一致", file=sys.stderr)
        return 1

    for sid, p in dirs.items():
        if not p.is_dir():
            print(f"目录不存在 [{sid}]: {p}", file=sys.stderr)
            return 1

    calib = load_yaml_relaxed(args.calib.resolve())
    T_to_imu: Dict[str, np.ndarray] = {}
    for sid, ext_key in zip(sensor_ids, ext_keys):
        style = ext_style_map.get(sid, ext_style_map.get(ext_key, "imu_in"))
        if style == "T_imu_0":
            T_lidar_imu = load_T_imu_0_extrinsic(calib, ext_key)
            label = f"T_imu_0__{ext_key}"
        else:
            T_lidar_imu = load_imu_in_extrinsic(calib, ext_key)
            label = f"imu_in_{ext_key}"
        T_to_imu[ext_key] = T_lidar_to_imu(T_lidar_imu)
        t = T_lidar_imu[:3, 3]
        print(f"外参 {label}: |t|={np.linalg.norm(t):.4f} m  (p_imu = inv(T) @ p_lidar)")

    ref_name = args.ref or sensor_ids[0]
    if ref_name not in dirs:
        print(f"基准 sensor {ref_name} 不在列表中", file=sys.stderr)
        return 1
    ref_dir = dirs[ref_name]
    others = {k: v for k, v in dirs.items() if k != ref_name}

    print(f"数据集: {dataset_label}")
    print(f"基准: {ref_name}, max_dt={args.max_dt_ms} ms")
    pairs = align_pairs(ref_name, ref_dir, others, args.max_dt_ms)
    if not pairs:
        print("无可用对齐帧", file=sys.stderr)
        return 1
    print(f"对齐帧数: {len(pairs)}")

    out_root = args.output or (repo / "merged_lidar_main" / f"{args.dataset}_imu")
    out_root.mkdir(parents=True, exist_ok=True)
    manifest = out_root / "manifest.tsv"

    colors = {
        "lidar_main": (255, 255, 255),
        "left_front": (0, 220, 255),
        "right_front": (255, 140, 0),
        "left_back": (0, 255, 140),
        "right_back": (255, 80, 180),
    }

    with manifest.open("w", encoding="utf-8") as mf:
        hdr = ["ref_ts", "max_dt_ms", "out_pcd"] + [f"src_{d}" for d in sensor_ids]
        mf.write("\t".join(hdr) + "\n")

        for i, (ref_ts, paths, max_dt) in enumerate(pairs):
            merged_xyz: List[np.ndarray] = []
            merged_i: List[np.ndarray] = []
            merged_rgb: List[np.ndarray] = []

            for sid, ext_key in zip(sensor_ids, ext_keys):
                pcd_path = paths[sid]
                data = load_pcd_ascii(pcd_path)
                xyz, intensity = xyz_intensity_from_array(data, [])
                xyz_imu = transform_points(xyz, T_to_imu[ext_key])
                merged_xyz.append(xyz_imu)
                merged_i.append(intensity)
                if args.color:
                    ckey = ext_key
                    rgb = colors.get(ckey, (200, 200, 200))
                    merged_rgb.append(np.tile(rgb, (xyz_imu.shape[0], 1)))

            xyz_all = np.vstack(merged_xyz)
            i_all = np.concatenate(merged_i)
            rgb_all = np.vstack(merged_rgb) if args.color else None

            out_pcd = out_root / f"{ref_ts}.pcd"
            write_pcd_ascii(out_pcd, xyz_all, i_all, rgb_all)

            row = [str(ref_ts), str(max_dt), str(out_pcd.relative_to(out_root))]
            row += [str(paths[d]) for d in sensor_ids]
            mf.write("\t".join(row) + "\n")

            if i == 0 or (i + 1) % 100 == 0 or i + 1 == len(pairs):
                print(f"  [{i + 1}/{len(pairs)}] {out_pcd.name}  {xyz_all.shape[0]} pts  max_dt={max_dt} ms")

    print(f"完成: {len(pairs)} 帧 → {out_root}")
    print(f"对照表: {manifest}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
