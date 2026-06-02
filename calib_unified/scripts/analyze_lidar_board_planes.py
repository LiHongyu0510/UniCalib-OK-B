#!/usr/bin/env python3
"""Analyze LiDAR planes on vehicle PCDs: old vs new board-detection RANSAC criteria."""

from __future__ import annotations

import argparse
import math
import random
import sys
from pathlib import Path
from typing import Iterable

import numpy as np

BOARD_COLS = 9
BOARD_ROWS = 6
SQUARE_SIZE_M = 0.025
VOXEL_SIZE = 0.02
DIST_THRESH = 0.02
RANSAC_ITERS = 2000
RNG_SEED = 42


def load_pcd_xyz_i(path: Path) -> tuple[np.ndarray, np.ndarray]:
    with path.open("r", encoding="utf-8", errors="replace") as f:
        lines = []
        while True:
            line = f.readline()
            if not line:
                raise ValueError(f"unexpected EOF in header: {path}")
            lines.append(line.strip())
            if line.strip().upper().startswith("DATA"):
                break
        header = "\n".join(lines)
        fields = []
        for h in lines:
            if h.startswith("FIELDS"):
                fields = h.split()[1:]
            if h.startswith("POINTS"):
                n_pts = int(h.split()[1])
        data_mode = lines[-1].split()[-1].lower()
        if data_mode != "ascii":
            raise ValueError(f"only ascii PCD supported: {path} ({data_mode})")
        idx = {name: i for i, name in enumerate(fields)}
        need = {"x", "y", "z"}
        if not need.issubset(idx):
            raise ValueError(f"missing x/y/z in {path}: {fields}")
        i_int = idx.get("intensity", None)
        rows = []
        for _ in range(n_pts):
            parts = f.readline().split()
            if len(parts) < len(fields):
                continue
            x, y, z = float(parts[idx["x"]]), float(parts[idx["y"]]), float(parts[idx["z"]])
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            inten = float(parts[i_int]) if i_int is not None else 0.0
            rows.append((x, y, z, inten))
    arr = np.array(rows, dtype=np.float64)
    return arr[:, :3], arr[:, 3]


def voxel_downsample(xyz: np.ndarray, intensity: np.ndarray, voxel: float) -> tuple[np.ndarray, np.ndarray]:
    keys: dict[tuple[int, int, int], int] = {}
    out_xyz = []
    out_i = []
    for i in range(xyz.shape[0]):
        p = xyz[i]
        key = (
            int(math.floor(p[0] / voxel)),
            int(math.floor(p[1] / voxel)),
            int(math.floor(p[2] / voxel)),
        )
        if key not in keys or intensity[i] > out_i[keys[key]]:
            if key in keys:
                j = keys[key]
            else:
                j = len(out_xyz)
                keys[key] = j
                out_xyz.append(p.copy())
                out_i.append(float(intensity[i]))
            out_xyz[j] = p
            out_i[j] = float(intensity[i])
    return np.asarray(out_xyz), np.asarray(out_i)


def ransac_planes(
    xyz: np.ndarray,
    min_plane_inliers: int,
    min_ratio: float | None,
    iters: int = RANSAC_ITERS,
) -> list[tuple[np.ndarray, float, list[int]]]:
    n = xyz.shape[0]
    if n < 3:
        return []
    rng = random.Random(RNG_SEED)
    candidates: list[tuple[np.ndarray, float, list[int]]] = []
    for _ in range(iters):
        i1, i2, i3 = rng.sample(range(n), 3)
        p1, p2, p3 = xyz[i1], xyz[i2], xyz[i3]
        v1 = p2 - p1
        v2 = p3 - p1
        normal = np.cross(v1, v2)
        norm_len = np.linalg.norm(normal)
        if norm_len < 1e-6:
            continue
        normal = normal / norm_len
        d = -float(np.dot(normal, p1))
        dists = np.abs(xyz @ normal + d)
        inliers = np.where(dists < DIST_THRESH)[0].tolist()
        if len(inliers) < min_plane_inliers:
            continue
        if min_ratio is not None and len(inliers) / n < min_ratio:
            continue
        candidates.append((normal, d, inliers))
    return candidates


def plane_bbox_uv(xyz: np.ndarray, inliers: list[int], normal: np.ndarray) -> tuple[float, float, float, float]:
    pts = xyz[inliers]
    centroid = pts.mean(axis=0)
    normal = normal / (np.linalg.norm(normal) + 1e-12)
    u_axis = np.array([1.0, 0.0, 0.0])
    if abs(np.dot(normal, u_axis)) > 0.9:
        u_axis = np.array([0.0, 1.0, 0.0])
    v_axis = np.cross(normal, u_axis)
    v_axis = v_axis / np.linalg.norm(v_axis)
    u_axis = np.cross(v_axis, normal)
    u_axis = u_axis / np.linalg.norm(u_axis)
    local = pts - centroid
    u = local @ u_axis
    v = local @ v_axis
    return float(u.min()), float(u.max()), float(v.min()), float(v.max())


def select_board_plane(
    candidates: list[tuple[np.ndarray, float, list[int]]],
    xyz: np.ndarray,
    board_cols: int,
    board_rows: int,
    square_size_m: float,
) -> tuple[dict | None, list[dict]]:
    if not candidates:
        return None, []
    exp_w = square_size_m * (board_cols - 1)
    exp_h = square_size_m * (board_rows - 1)
    scored = []
    for normal, d, inliers in candidates:
        u_min, u_max, v_min, v_max = plane_bbox_uv(xyz, inliers, normal)
        w, h = u_max - u_min, v_max - v_min
        w_err = abs(w - exp_w) / exp_w
        h_err = abs(h - exp_h) / exp_h
        size_err = w_err + h_err
        score = len(inliers) * math.exp(-size_err)
        scored.append(
            {
                "inliers": len(inliers),
                "w_m": w,
                "h_m": h,
                "w_err": w_err,
                "h_err": h_err,
                "size_err": size_err,
                "score": score,
            }
        )
    scored.sort(key=lambda x: x["score"], reverse=True)
    best_idx = max(range(len(candidates)), key=lambda i: scored[i]["score"])
    return scored[best_idx], scored[:5]


def analyze_frame(path: Path, board_cols: int, board_rows: int, square_size_m: float) -> dict:
    xyz_raw, inten = load_pcd_xyz_i(path)
    xyz, inten_ds = voxel_downsample(xyz_raw, inten, VOXEL_SIZE)
    n_raw, n_ds = xyz_raw.shape[0], xyz.shape[0]
    min_plane = max(30, (board_cols * board_rows) // 2)

    old_cands = ransac_planes(xyz, min_plane_inliers=100, min_ratio=0.3)
    new_cands = ransac_planes(xyz, min_plane_inliers=min_plane, min_ratio=None)

    # largest plane without ratio (diagnostic)
    all_planes = ransac_planes(xyz, min_plane_inliers=20, min_ratio=None)
    largest = max((len(c[2]) for c in all_planes), default=0)
    largest_ratio = largest / n_ds if n_ds else 0.0

    best_new, top5 = select_board_plane(new_cands, xyz, board_cols, board_rows, square_size_m)

    return {
        "file": path.name,
        "raw_pts": n_raw,
        "ds_pts": n_ds,
        "old_pass": len(old_cands) > 0,
        "old_n_cands": len(old_cands),
        "new_pass": best_new is not None,
        "new_n_cands": len(new_cands),
        "largest_plane_inliers": largest,
        "largest_plane_ratio": largest_ratio,
        "best_new": best_new,
        "top5": top5,
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("pcd_dir", type=Path, help="directory with .pcd files")
    ap.add_argument("-n", type=int, default=8, help="number of frames to sample")
    ap.add_argument("--cols", type=int, default=BOARD_COLS)
    ap.add_argument("--rows", type=int, default=BOARD_ROWS)
    ap.add_argument("--square", type=float, default=SQUARE_SIZE_M)
    args = ap.parse_args()

    files = sorted(args.pcd_dir.glob("*.pcd"))
    if not files:
        print(f"no PCD in {args.pcd_dir}", file=sys.stderr)
        return 1
    step = max(1, len(files) // args.n)
    sample = files[::step][: args.n]

    print(f"Board config: {args.cols}x{args.rows} inner corners, square={args.square} m")
    print(f"Expected plane size: {(args.cols-1)*args.square:.3f} x {(args.rows-1)*args.square:.3f} m")
    print(f"Sample {len(sample)} / {len(files)} from {args.pcd_dir}\n")

    old_ok = new_ok = 0
    for p in sample:
        try:
            r = analyze_frame(p, args.cols, args.rows, args.square)
        except Exception as e:
            print(f"[SKIP] {p.name}: {e}")
            continue
        old_ok += int(r["old_pass"])
        new_ok += int(r["new_pass"])
        print(f"--- {r['file']} ---")
        print(f"  points: raw={r['raw_pts']} downsampled={r['ds_pts']}")
        print(
            f"  largest plane: {r['largest_plane_inliers']} pts "
            f"({100*r['largest_plane_ratio']:.2f}% of cloud)"
        )
        print(
            f"  OLD (>=100 inliers & >=30% ratio): pass={r['old_pass']} "
            f"candidates={r['old_n_cands']}"
        )
        print(
            f"  NEW (>={max(30,(args.cols*args.rows)//2)} inliers, size score): "
            f"pass={r['new_pass']} candidates={r['new_n_cands']}"
        )
        if r["best_new"]:
            b = r["best_new"]
            print(
                f"  best board-like plane: inliers={b['inliers']} "
                f"size={b['w_m']:.3f}x{b['h_m']:.3f}m "
                f"w_err={b['w_err']:.2f} h_err={b['h_err']:.2f} score={b['score']:.1f}"
            )
        elif r["top5"]:
            t = r["top5"][0]
            print(
                f"  top plane (no board-sized match): inliers={t['inliers']} "
                f"size={t['w_m']:.3f}x{t['h_m']:.3f}m"
            )
        print()

    print("=== Summary ===")
    print(f"OLD criteria would pass: {old_ok}/{len(sample)} frames")
    print(f"NEW criteria would pass: {new_ok}/{len(sample)} frames")
    if old_ok == 0 and new_ok > 0:
        print("Conclusion: change is necessary for typical vehicle full-scene clouds.")
    elif old_ok == new_ok:
        print("Conclusion: both pass on sampled frames (board may dominate or data is small).")
    else:
        print("Conclusion: mixed; review per-frame sizes vs expected board.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
