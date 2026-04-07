#!/usr/bin/env python3
"""
UniCalib 一键运行脚本
用法:
  python scripts/run_calibration.py --config config/unicalib_config.yaml \
      --data /path/to/data_or_rosbag --output ./results

  # 分阶段运行
  python scripts/run_calibration.py --config ... --data ... --stage intrinsic
  python scripts/run_calibration.py --config ... --data ... --stage coarse
  python scripts/run_calibration.py --config ... --data ... --stage fine
  python scripts/run_calibration.py --config ... --data ... --stage validate
"""
import argparse
import logging
import sys
from pathlib import Path

# 将 UniCalib 根目录加入 Python 路径
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))


def setup_logging(log_level: str):
    level = getattr(logging, log_level.upper(), logging.INFO)
    logging.basicConfig(
        level=level,
        format="[%(asctime)s] [%(levelname)s] %(name)s: %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )


def main():
    parser = argparse.ArgumentParser(
        description="UniCalib — 统一多传感器自动标定系统",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    parser.add_argument("--config", required=True,
                        help="配置文件路径 (YAML)")
    parser.add_argument("--data", required=True,
                        help="数据目录或 rosbag 路径")
    parser.add_argument("--output", default="./calib_results",
                        help="结果输出目录 (默认: ./calib_results)")
    parser.add_argument("--stage", default="all",
                        choices=["all", "intrinsic", "coarse", "fine", "validate"],
                        help="执行阶段 (默认: all)")
    parser.add_argument("--log-level", default="INFO",
                        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
                        help="日志级别")
    args = parser.parse_args()

    setup_logging(args.log_level)
    logger = logging.getLogger("unicalib")

    # 验证路径
    if not Path(args.config).exists():
        logger.error(f"Config file not found: {args.config}")
        sys.exit(1)
    if not Path(args.data).exists():
        logger.error(f"Data path not found: {args.data}")
        sys.exit(1)

    # 将 output_dir 注入 config (覆盖文件中的设置)
    import yaml
    with open(args.config) as f:
        config = yaml.safe_load(f)
    config.setdefault("system", {})["output_dir"] = args.output
    import tempfile, os
    tmp_cfg = tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False)
    yaml.dump(config, tmp_cfg)
    tmp_cfg.close()

    try:
        from unicalib.core.system import UniCalibSystem
        system = UniCalibSystem(tmp_cfg.name)

        if args.stage == "all":
            results = system.run_full_pipeline(args.data)
        else:
            results = system.run_stage(args.stage, args.data)

        logger.info(f"\n{'='*60}")
        logger.info(f"  完成！结果已保存至: {args.output}")
        logger.info(f"{'='*60}")

    finally:
        os.unlink(tmp_cfg.name)


if __name__ == "__main__":
    main()
