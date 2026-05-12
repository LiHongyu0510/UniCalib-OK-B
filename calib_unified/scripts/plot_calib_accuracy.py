#!/usr/bin/env python3
"""
UniCalib — 标定精度曲线绘制

读取五种标定产生的 CSV（calib_accuracy_*.csv），绘制精度随运行次数的曲线，便于直观查看趋势。

用法:
  python3 plot_calib_accuracy.py --dir ./calib_output
  python3 plot_calib_accuracy.py --dir ./calib_output/joint --out ./plots
  python3 plot_calib_accuracy.py --dir ./calib_output --show

依赖: pandas, matplotlib
  pip install pandas matplotlib
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

try:
    import pandas as pd
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.dates as mdates
except ImportError as e:
    print("请安装依赖: pip install pandas matplotlib", file=sys.stderr)
    raise SystemExit(1) from e

# 五种标定 CSV 文件名与显示名
CALIB_FILES = [
    ("calib_accuracy_cam_intrinsic.csv", "相机内参"),
    ("calib_accuracy_imu_intrinsic.csv", "IMU内参"),
    ("calib_accuracy_lidar_cam_extrin.csv", "LiDAR-相机外参"),
    ("calib_accuracy_cam_cam_extrin.csv", "相机-相机外参"),
    ("calib_accuracy_imu_lidar_extrin.csv", "IMU-LiDAR外参"),
]


def load_csv(path: Path) -> pd.DataFrame | None:
    if not path.exists():
        return None
    try:
        df = pd.read_csv(path)
        if df.empty:
            return None
        return df
    except Exception as e:
        print(f"  [WARN] 读取失败 {path}: {e}", file=sys.stderr)
        return None


def plot_one(df: pd.DataFrame, title: str, ax: plt.Axes, task_key: str) -> None:
    """在 ax 上绘制单种标定的精度曲线。"""
    n = len(df)
    x = list(range(1, n + 1))

    # 成功/失败
    success = df.get("success")
    if success is not None:
        success = success.map(lambda v: str(v).strip().lower() in ("true", "1", "yes"))

    if task_key == "imu_intrinsic":
        # IMU 内参：只绘制噪声/零偏不稳（无量纲或单位各异，同图多曲线）
        has_any = False
        for col, lbl in [
            ("noise_gyro", "陀螺噪声"),
            ("bias_instab_gyro", "陀螺零偏不稳"),
            ("noise_accel", "加速度计噪声"),
            ("bias_instab_accel", "加速度计零偏不稳"),
        ]:
            if col not in df.columns:
                continue
            vals = pd.to_numeric(df[col], errors="coerce")
            if vals.notna().any():
                ax.plot(x, vals, "o-", label=lbl, markersize=4)
                has_any = True
        if has_any:
            ax.set_ylabel("噪声/零偏不稳")
            ax.legend(loc="upper right", fontsize=7)
            ax.tick_params(axis="y", labelsize=7)
        else:
            ax.text(0.5, 0.5, "无数值列", ha="center", va="center", transform=ax.transAxes)
    else:
        # 外参/相机内参：residual_rms 主曲线
        rms = df.get("residual_rms")
        if rms is not None:
            rms = pd.to_numeric(rms, errors="coerce")
            valid = rms.notna() & (rms >= 0)
            if valid.any():
                ax.plot(
                    [x[i] for i in range(n) if valid.iloc[i]],
                    rms[valid].tolist(),
                    "o-",
                    color="C0",
                    label="residual_rms",
                    markersize=6,
                )
                if success is not None:
                    fail = ~success & valid
                    if fail.any():
                        ax.scatter(
                            [x[i] for i in range(n) if fail.iloc[i]],
                            rms[fail].tolist(),
                            color="red",
                            s=80,
                            marker="x",
                            zorder=5,
                            label="失败",
                        )
                ax.set_ylabel("residual_rms (px 或无量纲)")
                ax.legend(loc="upper right", fontsize=8)
            else:
                ax.text(0.5, 0.5, "无有效 residual_rms", ha="center", va="center", transform=ax.transAxes)
        else:
            ax.text(0.5, 0.5, "无 residual_rms 列", ha="center", va="center", transform=ax.transAxes)

    ax.set_title(title, fontsize=10)
    ax.set_xlabel("运行次数")
    ax.grid(True, alpha=0.3)
    ax.set_xlim(0.5, n + 0.5)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="读取 calib_accuracy_*.csv 并绘制五种标定精度曲线",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--dir",
        type=str,
        default="./calib_output",
        help="包含 calib_accuracy_*.csv 的目录（默认: ./calib_output）",
    )
    parser.add_argument(
        "--out",
        type=str,
        default=None,
        help="输出图像目录或路径（默认: 与 --dir 相同，文件名 calib_accuracy_plots.png）",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="完成后弹出显示窗口",
    )
    parser.add_argument(
        "--format",
        type=str,
        choices=["png", "pdf", "svg"],
        default="png",
        help="输出图像格式（默认: png）",
    )
    args = parser.parse_args()

    base = Path(args.dir).resolve()
    if not base.exists():
        print(f"目录不存在: {base}", file=sys.stderr)
        sys.exit(1)

    out_path = args.out
    if out_path is None:
        out_path = base / "calib_accuracy_plots.png"
    else:
        out_path = Path(out_path).resolve()
        if out_path.suffix == "" or out_path.suffix.lower() not in (".png", ".pdf", ".svg"):
            out_path = out_path / "calib_accuracy_plots.png"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    if out_path.suffix.lower() != f".{args.format}":
        out_path = out_path.with_suffix(f".{args.format}")

    # 收集有数据的 CSV
    plotted = 0
    fig, axes = plt.subplots(2, 3, figsize=(12, 8))
    axes_flat = axes.ravel()

    for idx, (filename, title) in enumerate(CALIB_FILES):
        if idx >= len(axes_flat):
            break
        path = base / filename
        df = load_csv(path)
        ax = axes_flat[idx]
        if df is not None:
            task_key = filename.replace("calib_accuracy_", "").replace(".csv", "")
            plot_one(df, title, ax, task_key)
            plotted += 1
        else:
            ax.set_title(title, fontsize=10)
            ax.text(0.5, 0.5, "无数据", ha="center", va="center", transform=ax.transAxes)
            ax.set_xlabel("运行次数")

    # 隐藏多余子图
    for j in range(plotted, len(axes_flat)):
        axes_flat[j].set_visible(False)

    plt.suptitle("UniCalib 标定精度记录", fontsize=12)
    plt.tight_layout()
    plt.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"已保存: {out_path}")
    if args.show:
        plt.show()
    plt.close()


if __name__ == "__main__":
    main()
