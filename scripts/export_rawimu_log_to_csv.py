#!/usr/bin/env python3
"""
Export Bynav RAWIMU text log -> UniCalib IMU CSV (Unix s, gyro rad/s, accel m/s^2).

Axis order matches oem7_imu_reader.cpp RawImu branch: ax ay az gx gy gz.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


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


def parse_rawimu_line(line: str, ts_norm: BynavTimestampNormalizer) -> tuple[float, list[float], list[float]] | None:
    line = line.strip()
    if not line or line.startswith("#"):
        return None
    parts = line.split()
    if len(parts) < 11:
        return None
    try:
        ts = ts_norm.normalize(float(parts[2]))
        # MMDD TIME ts ts frame ax ay az gx gy gz
        ax, ay, az = float(parts[5]), float(parts[6]), float(parts[7])
        gx, gy, gz = float(parts[8]), float(parts[9]), float(parts[10])
    except ValueError:
        return None
    return ts, [gx, gy, gz], [ax, ay, az]


def export_log(in_path: Path, out_path: Path, gps_to_unix: float | None) -> int:
    ts_norm = BynavTimestampNormalizer()
    rows: list[str] = [
        "# timestamp_s,gx,gy,gz,ax,ay,az (from RAWIMU log, RawImu axis order)\n"
    ]
    n = 0
    with in_path.open() as f:
        for line in f:
            parsed = parse_rawimu_line(line, ts_norm)
            if parsed is None:
                continue
            ts, gyro, accel = parsed
            if ts < 1.65e9 and gps_to_unix is not None:
                ts += gps_to_unix
            rows.append(
                f"{ts:.9f},{gyro[0]:.9f},{gyro[1]:.9f},{gyro[2]:.9f},"
                f"{accel[0]:.9f},{accel[1]:.9f},{accel[2]:.9f}\n"
            )
            n += 1
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text("".join(rows))
    return n


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--in",
        dest="in_path",
        type=Path,
        default=Path(
            "data/imu/imu_in/bynav_X2D_driver_0516_1/bynav_X2D_driver/log/"
            "RAWIMU_data_2026-05-16_19-49-03.log"
        ),
    )
    ap.add_argument(
        "--out",
        dest="out_path",
        type=Path,
        default=Path("data/imu/imu/imu_1/imu_raw_unix.csv"),
    )
    ap.add_argument(
        "--gps-to-unix",
        type=float,
        default=None,
        help="Only if timestamps look like GPST; default auto (skip if already Unix)",
    )
    args = ap.parse_args()
    repo = Path(__file__).resolve().parents[1]
    in_path = args.in_path if args.in_path.is_absolute() else repo / args.in_path
    out_path = args.out_path if args.out_path.is_absolute() else repo / args.out_path

    if not in_path.is_file():
        print(f"Input not found: {in_path}", file=sys.stderr)
        return 1

    n = export_log(in_path, out_path, args.gps_to_unix)
    print(f"Wrote {n} rows -> {out_path}")

    # quick sanity
    import statistics as st

    lines = [l for l in out_path.read_text().splitlines() if l and not l.startswith("#")]
    gyros = []
    accels = []
    ts_list = []
    for l in lines[:500]:
        p = l.split(",")
        ts_list.append(float(p[0]))
        gyros.append((float(p[1]) ** 2 + float(p[2]) ** 2 + float(p[3]) ** 2) ** 0.5)
        accels.append((float(p[4]) ** 2 + float(p[5]) ** 2 + float(p[6]) ** 2) ** 0.5)
    print(f"  t0={ts_list[0]:.3f}  |gyro| med={st.median(gyros):.4f} rad/s  |accel| med={st.median(accels):.4f} m/s^2")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
