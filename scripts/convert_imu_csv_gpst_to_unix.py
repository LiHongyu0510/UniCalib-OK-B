#!/usr/bin/env python3
"""Convert IMU CSV timestamps GPST -> Unix for data.lidar file-mode alignment.

Applies BynavTimestampNormalizer before offset (X2D log: .95 -> .105 without integer carry).
"""

from __future__ import annotations

import argparse
from pathlib import Path

DEFAULT_LEAP = 18
GPS_EPOCH_UNIX = 315964800.0
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


def convert(in_path: Path, out_path: Path, leap_sec: float) -> int:
    offset = GPS_EPOCH_UNIX - leap_sec
    ts_norm = BynavTimestampNormalizer()
    n = 0
    lines = [
        "# timestamp_s,gx,gy,gz,ax,ay,az "
        f"(GPST normalized +{offset:.0f} -> Unix)\n"
    ]
    with in_path.open() as f:
        for line in f:
            raw = line.strip()
            if not raw or raw.startswith("#"):
                continue
            if "timestamp" in raw.lower() and "gx" in raw.lower():
                continue
            parts = raw.replace(",", " ").split()
            if len(parts) < 7:
                continue
            t = float(parts[0])
            if t < TIMESTAMP_LOOKS_UNIX:
                t = ts_norm.normalize(t) + offset
            lines.append(
                f"{t:.9f},{parts[1]},{parts[2]},{parts[3]},"
                f"{parts[4]},{parts[5]},{parts[6]}\n"
            )
            n += 1
    out_path.write_text("".join(lines))
    return n


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--in", dest="in_path", type=Path, required=True)
    ap.add_argument("--out", dest="out_path", type=Path, required=True)
    ap.add_argument("--leap-sec", type=float, default=DEFAULT_LEAP)
    args = ap.parse_args()
    n = convert(args.in_path, args.out_path, args.leap_sec)
    print(f"Wrote {n} rows -> {args.out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
