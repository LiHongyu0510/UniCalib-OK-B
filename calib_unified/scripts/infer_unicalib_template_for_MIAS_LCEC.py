#!/usr/bin/env python3
"""
UniCalib 粗标定 — MIAS-LCEC headless 接口

用法：将此文件复制到 MIAS-LCEC 的 bin/python/ 目录下并重命名为 infer_unicalib.py，
     则 UniCalib 粗标定管线（mias_lcec_infer.try_mias_infer）会自动发现并调用 estimate_extrinsic。
  - 需在 Python 3.10 环境下运行（LcMatch_CApi.so 为 cpython-310）。
  - 若 MIAS-LCEC 提供 headless 接口（estimate_extrinsic / run_headless 等），将在此处调用。

接口约定：
  estimate_extrinsic(pcd_path, image_path, sensor_config_path, model_path)
    -> (R 3x3, t 3,) 或 dict(R=..., t=...) 或 None
"""

from __future__ import annotations

import logging
import os
import sys
from typing import Optional, Tuple, Union

import numpy as np

_LOG = logging.getLogger("infer_unicalib")


def estimate_extrinsic(
    pcd_path: str,
    image_path: str,
    sensor_config_path: str,
    model_path: str,
) -> Optional[Union[Tuple[np.ndarray, np.ndarray], dict]]:
    """
    使用 MIAS-LCEC / Overlap Transformer 模型估计 LiDAR→Camera 外参 (R, t)。
    返回 (R 3x3, t 3,) 或 {"R": R, "t": t}，失败返回 None。
    """
    if not os.path.isfile(pcd_path) or not os.path.isfile(image_path):
        _LOG.debug("estimate_extrinsic: 输入文件不存在 pcd=%s image=%s", pcd_path, image_path)
        return None
    if not os.path.isfile(model_path):
        _LOG.debug("estimate_extrinsic: 模型文件不存在 model_path=%s", model_path)
        return None

    # 尝试从同目录 LcMatch_CApi（.so）调用 headless 接口
    infer_fn = None
    try:
        mod = __import__("LcMatch_CApi")
        for name in ("estimate_extrinsic", "run_headless", "calibrate", "run", "infer"):
            if hasattr(mod, name):
                attr = getattr(mod, name)
                if callable(attr):
                    infer_fn = attr
                    _LOG.info("[infer_unicalib] 使用 LcMatch_CApi.%s", name)
                    break
        if infer_fn is None and hasattr(mod, "LcMatchCApi"):
            api = getattr(mod, "LcMatchCApi")
            if callable(api):
                try:
                    code = getattr(api, "__code__", None)
                    if code is not None and getattr(code, "co_argcount", 0) >= 1:
                        infer_fn = api
                        _LOG.info("[infer_unicalib] 使用 LcMatch_CApi.LcMatchCApi(带参)")
                except TypeError:
                    pass
    except ImportError as e:
        _LOG.debug("[infer_unicalib] LcMatch_CApi 不可导入 (需 Python 3.10 及 .so): %s", e)
        return None

    if infer_fn is None:
        _LOG.debug("[infer_unicalib] LcMatch_CApi 中未发现 headless 可调用接口")
        return None

    try:
        out = infer_fn(
            pcd_path=pcd_path,
            image_path=image_path,
            sensor_config_path=sensor_config_path,
            model_path=model_path,
        )
    except TypeError:
        try:
            out = infer_fn(pcd_path, image_path, sensor_config_path, model_path)
        except Exception as e:
            _LOG.warning("[infer_unicalib] 调用失败: %s", e)
            return None
    except Exception as e:
        _LOG.warning("[infer_unicalib] 调用失败: %s", e)
        return None

    if out is None:
        return None
    if isinstance(out, (tuple, list)) and len(out) >= 2:
        R, t = np.asarray(out[0], dtype=np.float64), np.asarray(out[1], dtype=np.float64).ravel()
    elif isinstance(out, dict) and "R" in out and "t" in out:
        R = np.asarray(out["R"], dtype=np.float64)
        t = np.asarray(out["t"], dtype=np.float64).ravel()
    else:
        _LOG.warning("[infer_unicalib] 返回格式不支持: %s", type(out))
        return None
    if R.shape != (3, 3) or t.shape != (3,):
        _LOG.warning("[infer_unicalib] R/t 形状错误 R=%s t=%s", R.shape, t.shape)
        return None
    return (R, t)
