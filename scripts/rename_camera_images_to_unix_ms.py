#!/usr/bin/env python3
"""
将相机图像从「sensor_YYYY-MM-DD_HH-MM-SS.sss.ext」重命名为 Unix 毫秒时间戳，便于 UniCalib 文件模式解析。

示例:
  center_front_2026-06-01_18-04-53.542.png  ->  1780308293542.png

用法:
  # 预览（不改文件）
  python3 scripts/rename_camera_images_to_unix_ms.py --dir data/Camera/center_front --dry-run

  # 原地重命名
  python3 scripts/rename_camera_images_to_unix_ms.py --dir data/Camera/center_front --in-place

  # 输出到新目录并写 new_format 索引 CSV
  python3 scripts/rename_camera_images_to_unix_ms.py \\
      --dir data/Camera/center_front --sensor-id center_front \\
      --out data/Camera/center_front_ms --write-index camera/center_front_index.csv

  # 批量处理 data/Camera 下各子目录（子目录名即 sensor_id）
  python3 scripts/rename_camera_images_to_unix_ms.py --root data/Camera --in-place
"""

from __future__ import annotations

import argparse
import csv
import re
import shutil
import sys
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Iterator, Optional

IMAGE_EXTS = {".png", ".jpg", ".jpeg", ".bmp", ".webp"}

# center_front_2026-06-01_18-04-53.542  或  2026-06-01_18-04-53.542
FNAME_RE = re.compile(
    r"^(?:(?P<sensor>[\w-]+)_)?"
    r"(?P<date>\d{4}-\d{2}-\d{2})_"
    r"(?P<time>\d{2}-\d{2}-\d{2})"
    r"(?:\.(?P<frac>\d{1,6}))?$",
    re.IGNORECASE,
)


def parse_tz(tz_str: str) -> timezone:
    s = tz_str.strip()
    if s in ("UTC", "utc", "Z"):
        return timezone.utc
    if s.startswith("+") or s.startswith("-"):
        sign = 1 if s[0] == "+" else -1
        parts = s[1:].split(":")
        hours = int(parts[0])
        minutes = int(parts[1]) if len(parts) > 1 else 0
        return timezone(sign * timedelta(hours=hours, minutes=minutes))
    # 常见别名
    aliases = {
        "Asia/Shanghai": timezone(timedelta(hours=8)),
        "CST": timezone(timedelta(hours=8)),
        "UTC+8": timezone(timedelta(hours=8)),
    }
    if s in aliases:
        return aliases[s]
    raise ValueError(f"不支持的时区: {tz_str!r}，请用 UTC+8 / +08:00 / UTC")


def datetime_from_stem(stem: str, tz: timezone) -> Optional[tuple[datetime, Optional[str]]]:
    m = FNAME_RE.match(stem)
    if not m:
        return None
    y, mo, d = map(int, m.group("date").split("-"))
    H, M, S = map(int, m.group("time").split("-"))
    frac = m.group("frac") or "0"
    frac = frac.ljust(6, "0")[:6]
    micro = int(frac)
    dt = datetime(y, mo, d, H, M, S, micro, tzinfo=tz)
    return dt, m.group("sensor")


def iter_images(directory: Path) -> Iterator[Path]:
    for p in sorted(directory.iterdir()):
        if p.is_file() and p.suffix.lower() in IMAGE_EXTS:
            yield p


def target_name(
    src: Path,
    *,
    dt: datetime,
    sensor: Optional[str],
    naming: str,
) -> str:
    ts_ms = int(dt.timestamp() * 1000)
    ext = src.suffix.lower()
    if naming == "ms":
        return f"{ts_ms}{ext}"
    if naming == "sensor_ms":
        prefix = sensor or src.parent.name
        return f"{prefix}_{ts_ms}{ext}"
    raise ValueError(f"unknown naming: {naming}")


def process_dir(
    src_dir: Path,
    *,
    sensor_id: Optional[str],
    out_dir: Optional[Path],
    in_place: bool,
    naming: str,
    tz: timezone,
    dry_run: bool,
    write_index: Optional[Path],
    index_ts_unit: str,
) -> tuple[int, int, int]:
    if not src_dir.is_dir():
        print(f"SKIP (not a dir): {src_dir}", file=sys.stderr)
        return 0, 0, 1

    dst_root = src_dir if in_place else (out_dir or src_dir)
    if not in_place and not dry_run:
        dst_root.mkdir(parents=True, exist_ok=True)

    index_rows: list[tuple[int | str, str]] = []
    ok, skip, err = 0, 0, 0

    for src in iter_images(src_dir):
        parsed = datetime_from_stem(src.stem, tz)
        if parsed is None:
            print(f"SKIP (no datetime in name): {src.name}", file=sys.stderr)
            skip += 1
            continue

        dt, sensor_in_name = parsed
        sensor = sensor_id or sensor_in_name or src_dir.name
        new_name = target_name(src, dt=dt, sensor=sensor, naming=naming)
        dst = dst_root / new_name

        if dst.exists() and dst.resolve() != src.resolve():
            print(f"CONFLICT: {src.name} -> {new_name} (exists)", file=sys.stderr)
            err += 1
            continue

        ts_ms = int(dt.timestamp() * 1000)
        rel_for_index = new_name
        if write_index and not in_place and out_dir:
            rel_for_index = str(Path(out_dir.name) / new_name) if out_dir != src_dir else new_name

        if dry_run:
            print(f"  {src.name}  ->  {new_name}  ({ts_ms} ms, sensor={sensor})")
        else:
            if in_place:
                if src.name != new_name:
                    src.rename(dst)
            else:
                shutil.copy2(src, dst)
            print(f"  {src.name} -> {new_name}")

        if write_index:
            if index_ts_unit == "us":
                index_rows.append((ts_ms * 1000, rel_for_index))
            elif index_ts_unit == "ms":
                index_rows.append((ts_ms, rel_for_index))
            elif index_ts_unit == "s":
                index_rows.append((ts_ms / 1000.0, rel_for_index))
            else:
                raise ValueError(index_ts_unit)

        ok += 1

    if write_index and index_rows and not dry_run:
        write_index.parent.mkdir(parents=True, exist_ok=True)
        with write_index.open("w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            for ts, rel in index_rows:
                w.writerow([ts, rel])
        print(f"Wrote index ({len(index_rows)} rows): {write_index}")

    return ok, skip, err


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--dir", type=Path, help="单个相机图像目录")
    g.add_argument(
        "--root",
        type=Path,
        help="多相机根目录；每个子目录视为一个 sensor（子目录名= sensor_id）",
    )
    ap.add_argument("--sensor-id", type=str, default=None, help="单目录模式下的 sensor_id（写索引时用）")
    ap.add_argument(
        "--out",
        type=Path,
        default=None,
        help="输出目录（与 --in-place 互斥；默认与 --dir 相同则原地）",
    )
    ap.add_argument(
        "--in-place",
        action="store_true",
        help="原地重命名（默认若未指定 --out 且非 dry-run 则要求显式 --in-place 或 --out）",
    )
    ap.add_argument(
        "--naming",
        choices=("ms", "sensor_ms"),
        default="ms",
        help="ms=1780308293542.png（推荐，UniCalib stod 可直接解析）；sensor_ms=center_front_1780308293542.png",
    )
    ap.add_argument(
        "--tz",
        default="UTC+8",
        help="文件名中本地时间的时区，默认 UTC+8（国内采集）",
    )
    ap.add_argument(
        "--write-index",
        type=Path,
        default=None,
        help="写出 new_format 相机索引 CSV（timestamp,path）；path 为相对 new_format.root_dir 的路径",
    )
    ap.add_argument(
        "--index-ts-unit",
        choices=("us", "ms", "s"),
        default="us",
        help="索引 CSV 第一列时间戳单位（与 unicalib new_format.timestamp_unit 一致，默认 us）",
    )
    ap.add_argument("--dry-run", action="store_true", help="只打印计划，不改文件")
    args = ap.parse_args()

    repo = Path(__file__).resolve().parents[1]
    try:
        tz = parse_tz(args.tz)
    except ValueError as e:
        print(e, file=sys.stderr)
        return 1

    if args.in_place and args.out:
        print("请只指定 --in-place 或 --out 之一", file=sys.stderr)
        return 1
    if not args.dry_run and not args.in_place and not args.out:
        print("未指定 --dry-run 时，须加 --in-place 或 --out <目录>", file=sys.stderr)
        return 1

    total_ok = total_skip = total_err = 0

    if args.dir:
        src_dir = args.dir if args.dir.is_absolute() else repo / args.dir
        out_dir = None
        if args.out:
            out_dir = args.out if args.out.is_absolute() else repo / args.out
        index_path = None
        if args.write_index:
            index_path = args.write_index if args.write_index.is_absolute() else repo / args.write_index
        print(f"Dir: {src_dir}  tz={args.tz}  naming={args.naming}")
        ok, skip, err = process_dir(
            src_dir,
            sensor_id=args.sensor_id,
            out_dir=out_dir,
            in_place=args.in_place,
            naming=args.naming,
            tz=tz,
            dry_run=args.dry_run,
            write_index=index_path,
            index_ts_unit=args.index_ts_unit,
        )
        total_ok += ok
        total_skip += skip
        total_err += err
    else:
        root = args.root if args.root.is_absolute() else repo / args.root
        if not root.is_dir():
            print(f"Root not found: {root}", file=sys.stderr)
            return 1
        for sub in sorted(p for p in root.iterdir() if p.is_dir()):
            out_sub = None
            if args.out:
                base_out = args.out if args.out.is_absolute() else repo / args.out
                out_sub = base_out / sub.name
            index_path = None
            if args.write_index:
                base_idx = args.write_index if args.write_index.is_absolute() else repo / args.write_index
                index_path = base_idx.parent / f"{sub.name}_index.csv"
            print(f"\n=== {sub.name} ===")
            ok, skip, err = process_dir(
                sub,
                sensor_id=sub.name,
                out_dir=out_sub,
                in_place=args.in_place,
                naming=args.naming,
                tz=tz,
                dry_run=args.dry_run,
                write_index=index_path,
                index_ts_unit=args.index_ts_unit,
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
