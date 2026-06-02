#!/usr/bin/env python3
"""
将 lidar_main、left_front、right_back 三路点云变换到 lidar_main 坐标系并合并输出。

外参语义与 UniCalib 一致：T_target_in_ref，p_ref = T @ p_target。
  - lidar_main_to_left_front.yaml  → T_lf_in_main
  - lidar_main_to_right_back.yaml  → T_rb_in_main

用法示例:
  python3 scripts/merge_three_lidars_to_main.py \\
    --data-dir data \\
    --extrinsic-dir results/lidar_lidar_extrinsic \\
    --frame-index 0

  # 按时间戳对齐（文件名如 1779177240000.pcd，单位 ms）:
  python3 scripts/merge_three_lidars_to_main.py --data-dir data --align-time

  # 从 unicalib_example.yaml 读 data.lidar 路径与 initial_extrinsics:
  python3 scripts/merge_three_lidars_to_main.py \\
    --config calib_unified/config/unicalib_example.yaml \\
    --data-dir data \\
    --use-config-extrinsics
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

try:
    import yaml
except ImportError:
    print("需要 PyYAML: pip install pyyaml", file=sys.stderr)
    sys.exit(1)


def parse_pcd_header(path: Path) -> Tuple[dict, int]:
    fields: List[str] = []
    sizes: List[int] = []
    types: List[str] = []
    counts: List[int] = []
    width = height = points = 0
    data_mode = "ascii"
    header_lines = 0
    with path.open("r", encoding="utf-8", errors="replace") as f:
        for line in f:
            header_lines += 1
            line = line.strip()
            if line.startswith("FIELDS"):
                fields = line.split()[1:]
            elif line.startswith("SIZE"):
                sizes = [int(x) for x in line.split()[1:]]
            elif line.startswith("TYPE"):
                types = line.split()[1:]
            elif line.startswith("COUNT"):
                counts = [int(x) for x in line.split()[1:]]
            elif line.startswith("WIDTH"):
                width = int(line.split()[1])
            elif line.startswith("HEIGHT"):
                height = int(line.split()[1])
            elif line.startswith("POINTS"):
                points = int(line.split()[1])
            elif line.startswith("DATA"):
                data_mode = line.split()[1].lower()
                break
    if data_mode != "ascii":
        raise ValueError(f"{path}: 仅支持 DATA ascii，当前为 {data_mode}")
    return {
        "fields": fields,
        "sizes": sizes,
        "types": types,
        "counts": counts,
        "width": width,
        "height": height,
        "points": points,
    }, header_lines


def load_pcd_ascii(path: Path) -> np.ndarray:
    meta, header_lines = parse_pcd_header(path)
    fields = meta["fields"]
    n = meta["points"]
    if n <= 0:
        return np.zeros((0, len(fields)), dtype=np.float64)
    usecols = list(range(len(fields)))
    data = np.loadtxt(path, dtype=np.float64, comments="#", skiprows=header_lines)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[0] != n and data.shape[0] > 0:
        # 部分文件 POINTS 与行数略有不一致时以实际行为准
        n = data.shape[0]
    return data[:, usecols]


def write_pcd_ascii(path: Path, xyz: np.ndarray, intensity: Optional[np.ndarray] = None,
                    rgb: Optional[np.ndarray] = None) -> None:
    n = xyz.shape[0]
    if intensity is None:
        intensity = np.zeros(n, dtype=np.float64)
    lines = [
        "# .PCD v0.7 - Point Cloud Data file format",
        "VERSION 0.7",
    ]
    if rgb is not None:
        lines += [
            "FIELDS x y z intensity rgb",
            "SIZE 4 4 4 4 4",
            "TYPE F F F F U",
            "COUNT 1 1 1 1 1",
        ]
    else:
        lines += [
            "FIELDS x y z intensity",
            "SIZE 4 4 4 4",
            "TYPE F F F F",
            "COUNT 1 1 1 1",
        ]
    lines += [
        f"WIDTH {n}",
        "HEIGHT 1",
        "VIEWPOINT 0 0 0 1 0 0 0",
        f"POINTS {n}",
        "DATA ascii",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
        if rgb is not None:
            for i in range(n):
                r, g, b = (int(rgb[i, 0]), int(rgb[i, 1]), int(rgb[i, 2]))
                packed = (r << 16) | (g << 8) | b
                f.write(
                    f"{xyz[i, 0]:.6f} {xyz[i, 1]:.6f} {xyz[i, 2]:.6f} "
                    f"{intensity[i]:.0f} {packed}\n"
                )
        else:
            for i in range(n):
                f.write(
                    f"{xyz[i, 0]:.6f} {xyz[i, 1]:.6f} {xyz[i, 2]:.6f} "
                    f"{intensity[i]:.0f}\n"
                )


def transform_points(xyz: np.ndarray, T: np.ndarray) -> np.ndarray:
    if xyz.size == 0:
        return xyz.reshape(0, 3)
    ones = np.ones((xyz.shape[0], 1), dtype=np.float64)
    homo = np.hstack([xyz, ones])
    out = (T @ homo.T).T
    return out[:, :3]


def load_T_from_result_yaml(path: Path) -> np.ndarray:
    with path.open("r", encoding="utf-8") as f:
        doc = yaml.safe_load(f)
    mat = doc.get("transformation_matrix")
    if not mat:
        raise ValueError(f"{path}: 缺少 transformation_matrix")
    return np.array(mat, dtype=np.float64)


def load_T_from_config_block(node: dict) -> np.ndarray:
    data = node.get("data")
    if not data or len(data) != 16:
        raise ValueError("initial_extrinsics 块需要 16 个 data 元素 (4x4 行优先)")
    return np.array(data, dtype=np.float64).reshape(4, 4)


def list_pcds(directory: Path) -> List[Path]:
    files = sorted(directory.glob("*.pcd"))
    if not files:
        raise FileNotFoundError(f"目录无 .pcd: {directory}")
    return files


def pcd_timestamp_ms(path: Path) -> Optional[float]:
    m = re.match(r"^(\d+(?:\.\d+)?)", path.stem)
    return float(m.group(1)) if m else None


def pick_by_index(dirs: Dict[str, Path], index: int) -> Dict[str, Path]:
    lists = {k: list_pcds(v) for k, v in dirs.items()}
    n = min(len(v) for v in lists.values())
    if index < 0 or index >= n:
        raise IndexError(f"frame-index={index} 越界，各目录最少 {n} 帧")
    return {k: lists[k][index] for k in lists}


def pick_by_time(dirs: Dict[str, Path], ref: str = "lidar_main") -> Dict[str, Path]:
    lists = {k: list_pcds(v) for k, v in dirs.items()}
    ref_files = lists[ref]
    ref_ts = [pcd_timestamp_ms(p) for p in ref_files]
    if any(t is None for t in ref_ts):
        raise ValueError(f"{ref} 文件名无法解析时间戳，请用 --frame-index")
    mid = len(ref_files) // 2
    t0 = ref_ts[mid]
    out = {ref: ref_files[mid]}
    for sid, files in lists.items():
        if sid == ref:
            continue
        ts = [pcd_timestamp_ms(p) for p in files]
        if any(t is None for t in ts):
            raise ValueError(f"{sid} 文件名无法解析时间戳")
        j = int(np.argmin([abs(t - t0) for t in ts]))
        out[sid] = files[j]
        print(f"  时间对齐: {ref} t={t0:.0f} ms  <->  {sid} t={ts[j]:.0f} ms  (|Δt|={abs(ts[j]-t0):.0f} ms)")
    return out


def xyz_intensity_from_array(data: np.ndarray, fields: List[str]) -> Tuple[np.ndarray, np.ndarray]:
    # 默认列顺序来自文件头；load 时已按列序
    xi = 0
    yi = 1
    zi = 2
    ii = 3 if data.shape[1] > 3 else None
    xyz = data[:, [xi, yi, zi]]
    if ii is not None:
        intensity = data[:, ii]
    else:
        intensity = np.zeros(xyz.shape[0])
    return xyz, intensity


def load_yaml_relaxed(path: Path) -> dict:
    text = path.read_text(encoding="utf-8")
    text = text.replace("!!opencv-matrix", "")
    text = text.replace("tag:yaml.org,2002:opencv-matrix", "")
    return yaml.safe_load(text) or {}


def read_config_lidar_dirs(config_path: Path, data_dir: Path) -> Dict[str, Path]:
    cfg = load_yaml_relaxed(config_path)
    lidar = cfg.get("data", {}).get("lidar", {})
    out = {}
    for sid in ("lidar_main", "left_front", "right_back"):
        if sid not in lidar:
            raise KeyError(f"config data.lidar 缺少 {sid}")
        p = Path(lidar[sid])
        if not p.is_absolute():
            p = data_dir / p
        out[sid] = p
    return out


def _get_extrinsic_block(init: dict, ref: str, target: str) -> dict:
    for key in (f"{ref}__{target}", f"T_{ref}__{target}"):
        if key in init:
            return init[key]
    raise KeyError(f"initial_extrinsics 缺少 {ref}__{target} 或 T_{ref}__{target}")


def read_config_extrinsics(config_path: Path) -> Dict[str, np.ndarray]:
    cfg = load_yaml_relaxed(config_path)
    init = cfg.get("lidar_lidar", {}).get("initial_extrinsics", {})
    T_lf = load_T_from_config_block(_get_extrinsic_block(init, "lidar_main", "left_front"))
    T_rb = load_T_from_config_block(_get_extrinsic_block(init, "lidar_main", "right_back"))
    return {"left_front": T_lf, "right_back": T_rb}


def main() -> int:
    repo = Path(__file__).resolve().parents[1]
    ap = argparse.ArgumentParser(description="三路 LiDAR 合并到 lidar_main 坐标系")
    ap.add_argument("--data-dir", type=Path, default=repo / "data",
                    help="PCD 根目录 (默认: <repo>/data)")
    ap.add_argument("--config", type=Path, default=repo / "calib_unified/config/unicalib_example.yaml",
                    help="UniCalib 主配置，用于 data.lidar 路径")
    ap.add_argument("--extrinsic-dir", type=Path,
                    default=repo / "results/lidar_lidar_extrinsic",
                    help="外参 yaml 目录 (lidar_main_to_*.yaml)")
    ap.add_argument("--use-config-extrinsics", action="store_true",
                    help="使用 config 中 initial_extrinsics，而非 results 目录")
    ap.add_argument("--frame-index", type=int, default=0,
                    help="各目录排序后第 N 帧 (默认 0)")
    ap.add_argument("--align-time", action="store_true",
                    help="按文件名时间戳(ms) 对齐，取代 frame-index")
    ap.add_argument("--output", type=Path, default=None,
                    help="输出合并 PCD (默认: results/merged_lidar_main/merged_<tag>.pcd)")
    ap.add_argument("--downsample", type=float, default=0.0,
                    help="体素降采样边长(m)，0=不降采样")
    ap.add_argument("--color", action="store_true",
                    help="写入 rgb 字段: main=灰, left_front=青, right_back=橙")
    args = ap.parse_args()

    data_dir = args.data_dir.resolve()
    dirs = read_config_lidar_dirs(args.config.resolve(), data_dir)

    if args.use_config_extrinsics:
        Ts = read_config_extrinsics(args.config.resolve())
        print("外参来源: config initial_extrinsics")
    else:
        ext_dir = args.extrinsic_dir.resolve()
        Ts = {
            "left_front": load_T_from_result_yaml(ext_dir / "lidar_main_to_left_front.yaml"),
            "right_back": load_T_from_result_yaml(ext_dir / "lidar_main_to_right_back.yaml"),
        }
        print(f"外参来源: {ext_dir}")

    if args.align_time:
        print("帧选择: 时间戳对齐")
        frames = pick_by_time(dirs)
        tag = "time_aligned"
    else:
        print(f"帧选择: frame-index={args.frame_index}")
        frames = pick_by_index(dirs, args.frame_index)
        tag = f"idx{args.frame_index}"

    for sid, p in frames.items():
        print(f"  {sid}: {p}")

    colors = {
        "lidar_main": (180, 180, 180),
        "left_front": (0, 220, 255),
        "right_back": (255, 140, 0),
    }
    merged_xyz: List[np.ndarray] = []
    merged_i: List[np.ndarray] = []
    merged_rgb: List[np.ndarray] = []

    for sid, pcd_path in frames.items():
        data = load_pcd_ascii(pcd_path)
        xyz, intensity = xyz_intensity_from_array(data, [])
        if sid == "lidar_main":
            xyz_m = xyz
        else:
            xyz_m = transform_points(xyz, Ts[sid])
            print(f"  变换 {sid} -> lidar_main: {xyz.shape[0]} 点")
        merged_xyz.append(xyz_m)
        merged_i.append(intensity)
        if args.color:
            c = np.tile(colors[sid], (xyz_m.shape[0], 1))
            merged_rgb.append(c)

    xyz_all = np.vstack(merged_xyz)
    i_all = np.concatenate(merged_i)
    rgb_all = np.vstack(merged_rgb) if args.color else None

    if args.downsample > 0:
        try:
            import open3d as o3d
        except ImportError:
            print("降采样需要 open3d: pip install open3d", file=sys.stderr)
            return 1
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz_all)
        pcd = pcd.voxel_down_sample(args.downsample)
        xyz_all = np.asarray(pcd.points)
        print(f"体素降采样 {args.downsample}m -> {xyz_all.shape[0]} 点")

    out = args.output
    if out is None:
        out = repo / "merged_lidar_main" / f"merged_{tag}.pcd"
    out = out.resolve()
    write_pcd_ascii(out, xyz_all, i_all, rgb_all)
    print(f"已写入: {out}  ({xyz_all.shape[0]} 点, 坐标系 lidar_main)")
    print("CloudCompare: 若带 rgb，按 rgb 着色；main=灰 left_front=青 right_back=橙")
    return 0


if __name__ == "__main__":
    sys.exit(main())
