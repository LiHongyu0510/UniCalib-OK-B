#!/usr/bin/env python3
"""
批量将相机图像从「sensor_YYYY-MM-DD_HH-MM-SS.sss.ext」重命名为 Unix 毫秒时间戳。

适用于 data/Camera/<camera>/<session>/ 这类目录结构，路径与会话名均可自定义。

示例:
  center_front_2026-06-04_16-02-13.026.png  ->  1780560133026.png

用法:
  # 预览：默认处理 data/Camera 下五路相机的 3301、3302_2
  python3 scripts/rename_camera_session_dirs.py --dry-run

  # 原地重命名（默认路径）
  python3 scripts/rename_camera_session_dirs.py --in-place

  # 自定义相机根目录与会话子目录
  python3 scripts/rename_camera_session_dirs.py \\
      --camera-root /path/to/Camera --sessions 3301 3302_2 --in-place

  # 只处理指定相机
  python3 scripts/rename_camera_session_dirs.py \\
      --camera-root data/Camera --cameras center_front surrounding_front \\
      --sessions 3301 --in-place

  # 直接指定若干图像目录（忽略 camera-root / sessions）
  python3 scripts/rename_camera_session_dirs.py \\
      --dirs data/Camera/center_front/3301 data/Camera/center_front/3302_2 \\
      --in-place
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

# 与 rename_camera_images_to_unix_ms.py 同目录
_SCRIPT_DIR = Path(__file__).resolve().parent
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))

from rename_camera_images_to_unix_ms import parse_tz, process_dir  # noqa: E402

DEFAULT_CAMERA_ROOT = Path("data/Camera")
DEFAULT_SESSIONS = ("3301", "3302_2")


def resolve_path(path: Path, repo: Path) -> Path:
    return path if path.is_absolute() else repo / path


def discover_dirs(
    camera_root: Path,
    sessions: list[str],
    cameras: list[str] | None,
) -> list[Path]:
    if not camera_root.is_dir():
        print(f"相机根目录不存在: {camera_root}", file=sys.stderr)
        return []

    if cameras:
        cam_names = cameras
    else:
        cam_names = sorted(p.name for p in camera_root.iterdir() if p.is_dir())

    dirs: list[Path] = []
    for cam in cam_names:
        cam_path = camera_root / cam
        if not cam_path.is_dir():
            print(f"SKIP (camera not found): {cam_path}", file=sys.stderr)
            continue
        for sess in sessions:
            d = cam_path / sess
            if d.is_dir():
                dirs.append(d)
            else:
                print(f"SKIP (session missing): {d}", file=sys.stderr)
    return dirs


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument(
        "--camera-root",
        type=Path,
        default=DEFAULT_CAMERA_ROOT,
        help=f"相机数据根目录（默认: {DEFAULT_CAMERA_ROOT}）",
    )
    ap.add_argument(
        "--sessions",
        nargs="+",
        default=list(DEFAULT_SESSIONS),
        metavar="SESSION",
        help=f"各相机下的会话子目录名（默认: {' '.join(DEFAULT_SESSIONS)}）",
    )
    ap.add_argument(
        "--cameras",
        nargs="*",
        default=None,
        metavar="CAMERA",
        help="仅处理列出的相机子目录；省略则处理 camera-root 下全部子目录",
    )
    ap.add_argument(
        "--dirs",
        nargs="+",
        type=Path,
        metavar="DIR",
        help="直接指定图像目录列表（指定后忽略 --camera-root / --sessions / --cameras）",
    )
    ap.add_argument(
        "--in-place",
        action="store_true",
        help="原地重命名（未指定 --dry-run 时必须加此项）",
    )
    ap.add_argument(
        "--naming",
        choices=("ms", "sensor_ms"),
        default="ms",
        help="ms=1780308293542.png（推荐）；sensor_ms=center_front_1780308293542.png",
    )
    ap.add_argument(
        "--tz",
        default="UTC+8",
        help="文件名中本地时间的时区，默认 UTC+8",
    )
    ap.add_argument("--dry-run", action="store_true", help="只打印计划，不改文件")
    args = ap.parse_args()

    repo = Path(__file__).resolve().parents[1]
    try:
        tz = parse_tz(args.tz)
    except ValueError as e:
        print(e, file=sys.stderr)
        return 1

    if not args.dry_run and not args.in_place:
        print("未指定 --dry-run 时，须加 --in-place", file=sys.stderr)
        return 1

    if args.dirs:
        target_dirs = [resolve_path(d, repo) for d in args.dirs]
    else:
        camera_root = resolve_path(args.camera_root, repo)
        target_dirs = discover_dirs(camera_root, args.sessions, args.cameras)

    if not target_dirs:
        print("未找到可处理的目录", file=sys.stderr)
        return 1

    print(
        f"camera_root={resolve_path(args.camera_root, repo) if not args.dirs else '(dirs mode)'}  "
        f"sessions={args.sessions}  tz={args.tz}  naming={args.naming}  "
        f"dirs={len(target_dirs)}"
    )

    total_ok = total_skip = total_err = 0
    for src_dir in target_dirs:
        # 父目录名即 sensor_id，如 .../center_front/3301 -> center_front
        sensor_id = src_dir.parent.name
        print(f"\n=== {src_dir.relative_to(repo) if src_dir.is_relative_to(repo) else src_dir} "
              f"(sensor={sensor_id}) ===")
        ok, skip, err = process_dir(
            src_dir,
            sensor_id=sensor_id,
            out_dir=None,
            in_place=args.in_place,
            naming=args.naming,
            tz=tz,
            dry_run=args.dry_run,
            write_index=None,
            index_ts_unit="us",
        )
        total_ok += ok
        total_skip += skip
        total_err += err

    print(f"\nDone: renamed={total_ok} skipped={total_skip} errors={total_err}")
    if total_ok and args.naming == "ms":
        print("提示: 重命名后 images_dir 可直接用于 UniCalib；文件名即 Unix 毫秒时间戳。")
    return 0 if total_err == 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
