#!/usr/bin/env python3
"""Rename/export LiDAR PCDs for data.lidar file mode (Unix us in filename suffix)."""

from __future__ import annotations

import argparse
import csv
import os
import shutil
import sys
from pathlib import Path


def parse_index(index_path: Path, root_dir: Path) -> list[tuple[int, int, Path]]:
    """Return [(frame_idx, timestamp_us, src_pcd), ...]."""
    rows: list[tuple[int, int, Path]] = []
    with index_path.open() as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(",", 1)
            if len(parts) != 2:
                continue
            ts_us = int(parts[0].strip())
            rel = parts[1].strip()
            src = root_dir / rel
            if not src.is_file():
                # try basename under same dir as index parent
                alt = index_path.parent / Path(rel).name
                if alt.is_file():
                    src = alt
            frame_idx = len(rows)
            rows.append((frame_idx, ts_us, src))
    return rows


def export_pcds(
    rows: list[tuple[int, int, Path]],
    out_dir: Path,
    *,
    use_symlink: bool,
    dry_run: bool,
) -> tuple[int, int]:
    out_dir.mkdir(parents=True, exist_ok=True)
    ok = 0
    missing = 0
    width = max(6, len(str(max((r[0] for r in rows), default=0))))

    for frame_idx, ts_us, src in rows:
        dst_name = f"frame_{frame_idx:0{width}d}_{ts_us}.pcd"
        dst = out_dir / dst_name
        if not src.is_file():
            print(f"MISSING: {src}", file=sys.stderr)
            missing += 1
            continue
        if dry_run:
            print(f"  {src.name} -> {dst_name}")
            ok += 1
            continue
        if dst.exists():
            dst.unlink()
        if use_symlink:
            # 相对路径，避免 Docker 内 /home/... 绝对链接失效
            link_target = os.path.relpath(src.resolve(), out_dir)
            os.symlink(link_target, dst)
        else:
            shutil.copy2(src, dst)
        ok += 1

    return ok, missing


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--index",
        type=Path,
        default=Path("data/lidar_main/lidar_main_pcd/lidar_main_0516_1_index.csv"),
        help="Index CSV: timestamp_us,relative_path",
    )
    ap.add_argument(
        "--root",
        type=Path,
        default=Path("data"),
        help="Root for relative paths in index",
    )
    ap.add_argument(
        "--out",
        type=Path,
        default=Path("data/lidar_main/lidar_main_pcd/lidar_main_0516_1_unix"),
        help="Output directory for renamed PCDs",
    )
    ap.add_argument(
        "--copy",
        action="store_true",
        help="Copy files (default: symlink to save space)",
    )
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    repo = Path(__file__).resolve().parents[1]
    index_path = args.index if args.index.is_absolute() else repo / args.index
    root_dir = args.root if args.root.is_absolute() else repo / args.root
    out_dir = args.out if args.out.is_absolute() else repo / args.out

    if not index_path.is_file():
        print(f"Index not found: {index_path}", file=sys.stderr)
        return 1

    rows = parse_index(index_path, root_dir)
    if not rows:
        print("No rows in index", file=sys.stderr)
        return 1

    print(f"Index: {index_path}")
    print(f"Output: {out_dir} ({'copy' if args.copy else 'symlink'})")
    print(f"Frames: {len(rows)}  ts_us [{rows[0][1]} .. {rows[-1][1]}]")

    ok, missing = export_pcds(
        rows, out_dir, use_symlink=not args.copy, dry_run=args.dry_run
    )
    print(f"Done: {ok} exported, {missing} missing")
    if ok and not args.dry_run:
        # verify first parse matches loader logic
        stem = f"frame_000000_{rows[0][1]}"
        ts_part = stem.rsplit("_", 1)[-1]
        v = float(ts_part)
        ts_s = v / 1e6 if v > 1e15 else v
        print(f"Sample parse: {stem}.pcd -> {ts_s:.3f} s (Unix)")
        print(f"data.lidar config: lidar_main: {out_dir.relative_to(repo) if out_dir.is_relative_to(repo) else out_dir}")
    return 0 if missing == 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
