#!/usr/bin/env python3
"""Resample IMU CSV to uniform rate (default 100 Hz) for LiDAR 10Hz alignment."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


def load_csv(path: Path) -> list[tuple[float, list[float], list[float]]]:
    rows = []
    for line in path.read_text().splitlines():
        if not line.strip() or line.startswith("#"):
            continue
        p = line.split(",")
        if len(p) < 7:
            continue
        rows.append((float(p[0]), [float(p[i]) for i in range(1, 4)], [float(p[i]) for i in range(4, 7)]))
    rows.sort(key=lambda r: r[0])
    return rows


def lerp(a: float, b: float, t: float, t0: float, t1: float) -> float:
    if t1 <= t0:
        return a
    alpha = (t - t0) / (t1 - t0)
    return (1.0 - alpha) * a + alpha * b


def resample(rows, rate_hz: float) -> list[str]:
    if len(rows) < 2:
        return []
    dt = 1.0 / rate_hz
    t0, t1 = rows[0][0], rows[-1][0]
    out = [f"# timestamp_s,gx,gy,gz,ax,ay,az (resampled {rate_hz:.0f} Hz)\n"]
    j = 0
    t = t0
    while t <= t1 + 1e-9:
        while j + 1 < len(rows) and rows[j + 1][0] < t:
            j += 1
        if j + 1 >= len(rows):
            break
        g0, a0 = rows[j][1], rows[j][2]
        g1, a1 = rows[j + 1][1], rows[j + 1][2]
        t_a, t_b = rows[j][0], rows[j + 1][0]
        gx = lerp(g0[0], g1[0], t, t_a, t_b)
        gy = lerp(g0[1], g1[1], t, t_a, t_b)
        gz = lerp(g0[2], g1[2], t, t_a, t_b)
        ax = lerp(a0[0], a1[0], t, t_a, t_b)
        ay = lerp(a0[1], a1[1], t, t_a, t_b)
        az = lerp(a0[2], a1[2], t, t_a, t_b)
        out.append(f"{t:.9f},{gx:.9f},{gy:.9f},{gz:.9f},{ax:.9f},{ay:.9f},{az:.9f}\n")
        t += dt
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--in", dest="in_path", type=Path,
                    default=Path("data/imu/imu/imu_1/imu_raw_unix.csv"))
    ap.add_argument("--out", dest="out_path", type=Path,
                    default=Path("data/imu/imu/imu_1/imu_raw_unix_100hz.csv"))
    ap.add_argument("--rate", type=float, default=100.0)
    args = ap.parse_args()
    repo = Path(__file__).resolve().parents[1]
    in_path = args.in_path if args.in_path.is_absolute() else repo / args.in_path
    out_path = args.out_path if args.out_path.is_absolute() else repo / args.out_path
    rows = load_csv(in_path)
    lines = resample(rows, args.rate)
    out_path.write_text("".join(lines))
    print(f"Wrote {len(lines)-1} rows @ {args.rate} Hz -> {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
