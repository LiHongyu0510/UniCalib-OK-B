#!/usr/bin/env python3
"""
Export Bynav CORRIMU text log -> UniCalib IMU CSV (Unix s, gx,gy,gz,ax,ay,az).

Matches calib_unified/src/io/oem7_imu_reader.cpp:
  - spdlog lines: accel:[...] gyro:[...] + header: timestamp (physical units)
  - legacy 11-field: MMDD TIME ts ts frame + 6 values
      * 默认 X2D 文本 log: ax,ay,az,gx,gy,gz -> CSV gx,gy,gz,ax,ay,az
      * 少数 OEM7 语义: roll,pitch,yaw,lon,lat,vert（按数据探测）
"""

from __future__ import annotations

import argparse
import re
import statistics as st
import sys
from pathlib import Path

GPS_EPOCH_UNIX = 315964800.0
DEFAULT_LEAP = 18
TIMESTAMP_LOOKS_UNIX = 1.65e9


class BynavTimestampNormalizer:
    def __init__(self) -> None:
        self.carry_sec = 0
        self.prev_frac = -1.0
        self.initialized = False

    def normalize(self, ts: float) -> float:
        ibase = int(ts)
        frac = ts - ibase
        if not self.initialized:
            self.carry_sec = ibase
            self.prev_frac = frac
            self.initialized = True
            return float(self.carry_sec) + frac
        if frac < self.prev_frac - 0.05:
            self.carry_sec += 1
        elif ibase > self.carry_sec:
            self.carry_sec = ibase
        self.prev_frac = frac
        return float(self.carry_sec) + frac


def gpst_to_unix(ts: float, leap_sec: float) -> float:
    if ts >= TIMESTAMP_LOOKS_UNIX:
        return ts
    return GPS_EPOCH_UNIX + ts - leap_sec


def parse_bracket_triple(line: str, key: str) -> list[float] | None:
    needle = f"{key}:["
    pos = line.find(needle)
    if pos < 0:
        return None
    inner = line[pos + len(needle) : line.find("]", pos)]
    parts = [p.strip() for p in inner.split(",") if p.strip()]
    if len(parts) < 3:
        return None
    try:
        return [float(parts[0]), float(parts[1]), float(parts[2])]
    except ValueError:
        return None


def parse_header_timestamp(line: str) -> float | None:
    m = re.search(r"header:([0-9]+(?:\.[0-9]+)?)", line)
    if not m:
        return None
    return float(m.group(1))


def detect_eleven_field_layout(v: list[float]) -> str:
    """Mirror oem7_imu_reader: |az|>3 -> ax,ay,az,gx,gy,gz; |vert|>3 -> OEM7 rates."""
    if len(v) < 6:
        return "accel_gyro"
    if abs(v[2]) > 3.0 and abs(v[3]) < 3.0:
        return "accel_gyro"
    if abs(v[5]) > 3.0 and abs(v[2]) < 3.0:
        return "corr_semantic"
    return "accel_gyro"


def map_eleven_field(v: list[float], layout: str) -> tuple[list[float], list[float]]:
    if layout == "accel_gyro":
        accel = v[0:3]
        gyro = v[3:6]
    else:
        gyro = v[0:3]
        accel = v[3:6]
    return gyro, accel


def parse_spdlog_line(line: str, ts_norm: BynavTimestampNormalizer) -> tuple[float, list[float], list[float]] | None:
    if "accel:[" not in line or "gyro:[" not in line:
        return None
    accel = parse_bracket_triple(line, "accel")
    gyro = parse_bracket_triple(line, "gyro")
    if accel is None or gyro is None:
        return None
    ts = parse_header_timestamp(line)
    if ts is None:
        parts = line.split()
        if len(parts) < 3:
            return None
        try:
            ts = float(parts[2])
        except ValueError:
            return None
    ts = ts_norm.normalize(ts)
    return ts, gyro, accel


def parse_eleven_field_line(
    line: str, layout: str, ts_norm: BynavTimestampNormalizer
) -> tuple[float, list[float], list[float]] | None:
    parts = line.split()
    if len(parts) < 11:
        return None
    try:
        ts = ts_norm.normalize(float(parts[2]))
        v = [float(parts[5 + i]) for i in range(6)]
    except ValueError:
        return None
    gyro, accel = map_eleven_field(v, layout)
    return ts, gyro, accel


def export_log(in_path: Path, out_path: Path, leap_sec: float, force_layout: str | None) -> tuple[int, str]:
    ts_norm = BynavTimestampNormalizer()
    eleven_layout: str | None = force_layout
    rows: list[str] = [
        "# timestamp_s,gx,gy,gz,ax,ay,az "
        "(CORRIMU log -> UniCalib; gyro rad/s or CORRIMU rates, accel m/s^2)\n"
    ]
    n_spd = n_11 = 0

    with in_path.open() as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue

            parsed = parse_spdlog_line(line, ts_norm)
            if parsed is not None:
                ts, gyro, accel = parsed
                n_spd += 1
            else:
                if eleven_layout is None:
                    parts = line.split()
                    if len(parts) >= 11:
                        try:
                            probe = [float(parts[5 + i]) for i in range(6)]
                            eleven_layout = detect_eleven_field_layout(probe)
                        except ValueError:
                            eleven_layout = "accel_gyro"
                    else:
                        continue
                parsed = parse_eleven_field_line(line, eleven_layout, ts_norm)
                if parsed is None:
                    continue
                ts, gyro, accel = parsed
                n_11 += 1

            ts = gpst_to_unix(ts, leap_sec)
            rows.append(
                f"{ts:.9f},{gyro[0]:.9f},{gyro[1]:.9f},{gyro[2]:.9f},"
                f"{accel[0]:.9f},{accel[1]:.9f},{accel[2]:.9f}\n"
            )

    if len(rows) <= 1:
        return 0, eleven_layout or "unknown"

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text("".join(rows))
    mode = f"spdlog={n_spd} eleven_field={n_11} layout={eleven_layout or 'spdlog'}"
    return len(rows) - 1, mode


def sanity(path: Path) -> None:
    gyros, accels, ts_list = [], [], []
    for line in path.read_text().splitlines():
        if not line or line.startswith("#"):
            continue
        p = line.split(",")
        ts_list.append(float(p[0]))
        gyros.append((float(p[1]) ** 2 + float(p[2]) ** 2 + float(p[3]) ** 2) ** 0.5)
        accels.append((float(p[4]) ** 2 + float(p[5]) ** 2 + float(p[6]) ** 2) ** 0.5)
    print(f"  rows={len(ts_list)}  t=[{ts_list[0]:.3f}, {ts_list[-1]:.3f}]")
    print(f"  |gyro| med={st.median(gyros):.4f}  |accel| med={st.median(accels):.4f} m/s^2")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--in",
        dest="in_path",
        type=Path,
        default=Path(
            "data/imu/imu_in/bynav_X2D_driver_0516_1/bynav_X2D_driver/log/"
            "CORRIMU_data_2026-05-16_19-49-03.log"
        ),
    )
    ap.add_argument(
        "--out",
        dest="out_path",
        type=Path,
        default=Path("data/imu/imu/imu_1/cs/imu_corrimu_unix.csv"),
    )
    ap.add_argument("--leap-sec", type=float, default=float(DEFAULT_LEAP))
    ap.add_argument(
        "--force-eleven-layout",
        choices=("accel_gyro", "corr_semantic", "auto"),
        default="auto",
        help="11-field: accel_gyro=ax,ay,az,gx,gy,gz (X2D 默认); corr_semantic=OEM7 rates",
    )
    args = ap.parse_args()
    repo = Path(__file__).resolve().parents[1]
    in_path = args.in_path if args.in_path.is_absolute() else repo / args.in_path
    out_path = args.out_path if args.out_path.is_absolute() else repo / args.out_path

    if not in_path.is_file():
        print(f"Input not found: {in_path}", file=sys.stderr)
        return 1

    force = None if args.force_eleven_layout == "auto" else args.force_eleven_layout
    n, mode = export_log(in_path, out_path, args.leap_sec, force)
    if n == 0:
        print("No rows exported", file=sys.stderr)
        return 1
    print(f"Wrote {n} rows -> {out_path}")
    print(f"  parse: {mode}")
    sanity(out_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
