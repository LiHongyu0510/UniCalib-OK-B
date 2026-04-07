#!/usr/bin/env python3
"""
UniCalib — MIAS-LCEC / Overlap Transformer 推理封装（可选）

当配置了 model_path 且 MIAS-LCEC 仓库可用时，尝试用深度模型估计 LiDAR-Camera 外参；
否则返回 None，由 run_lidar_cam_coarse.py 回退到 PnP/几何初值。

接口:
  try_mias_infer(pcd_path, image_path, sensor_config_path, model_path, mias_repo_dir)
    -> (R 3x3, t 3,) 或 None
"""

from __future__ import annotations

import logging
import os
import sys
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np


def _setup_mias_logging(verbose: bool = True) -> logging.Logger:
    """
    为 mias_lcec_infer 模块配置日志输出到 stderr，确保深度模型推理失败原因能被 C++ 端捕获。
    """
    log = logging.getLogger("mias_lcec_infer")
    if log.handlers:  # 避免重复添加 handler
        return log
    log.setLevel(logging.DEBUG if verbose else logging.INFO)
    h = logging.StreamHandler(sys.stderr)
    h.setFormatter(logging.Formatter(
        "[MIAS_LCEC_INFER %(asctime)s.%(msecs)03d] [%(levelname)s] %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    ))
    log.addHandler(h)
    return log


_LOG = _setup_mias_logging()


def _infer_mias_lcec_root_from_model_path(model_path: str) -> Optional[str]:
    """从 model_path 推导 MIAS-LCEC 仓库根目录（如 .../MIAS-LCEC/model/xxx.pth.tar -> .../MIAS-LCEC）。"""
    if not model_path or not os.path.isfile(model_path):
        return None
    path = os.path.abspath(model_path)
    # 约定: 模型在 MIAS-LCEC/model/ 下
    if "MIAS-LCEC" in path and path.rstrip("/").endswith(".pth.tar"):
        parent = os.path.dirname(path)
        if os.path.basename(parent) == "model":
            root = os.path.dirname(parent)
            if os.path.isdir(root):
                return root
    if "model" in path.split(os.sep):
        parent = os.path.dirname(path)
        if os.path.basename(parent) == "model":
            root = os.path.dirname(parent)
            if os.path.isdir(root):
                return root
    return None


def try_mias_infer(
    pcd_path: str,
    image_path: str,
    sensor_config_path: str,
    model_path: str,
    mias_repo_dir: str = "",
) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """
    尝试用 MIAS-LCEC / Overlap Transformer 估计 T_cam_lidar (R, t)。
    任一环节失败则返回 None，调用方应回退 PnP。
    当 mias_repo_dir 为 calib_unified 时，会从 model_path 推导 MIAS-LCEC 根目录并尝试从该处导入推理模块。
    """
    if not model_path or not os.path.isfile(model_path):
        _LOG.warning("[MIAS-LCEC 推理] 未使用深度模型: 模型文件不存在或未配置 model_path=%s", model_path or "(空)")
        return None

    # 候选仓库列表：1) 配置的 mias_repo_dir  2) 从 model_path 推导的 MIAS-LCEC 根目录
    repos_to_try: List[Tuple[str, str]] = []
    repo = (mias_repo_dir or os.environ.get("MIAS_LCEC_REPO", "")).strip()
    if repo and os.path.isdir(repo):
        repos_to_try.append((repo, "配置的 mias_repo"))
    mias_root = _infer_mias_lcec_root_from_model_path(model_path)
    if mias_root and (mias_root not in [r[0] for r in repos_to_try]):
        repos_to_try.append((mias_root, "从 model_path 推导的 MIAS-LCEC 根目录"))

    if not repos_to_try:
        _LOG.warning("[MIAS-LCEC 推理] 未使用深度模型: 未配置或不存在 MIAS-LCEC 仓库 (mias_repo=%s)", repo or "(空)")
        return None

    # 修复只读文件系统：LcMatch_CApi 的 envpath.so 会访问 <某路径>/ResImg。尝试将 EXE_ABS_PATH 指向可写目录。
    # 注意：实测 LcMatch_CApi 的 .so 在初始化时从自身路径计算 EXE_ABS_PATH，不读取 os.environ，故设置环境变量可能无效。
    work_dir = os.path.dirname(os.path.abspath(sensor_config_path))
    resimg_dir = os.path.join(work_dir, "ResImg")
    try:
        os.makedirs(resimg_dir, exist_ok=True)
        os.environ["EXE_ABS_PATH"] = work_dir
        _LOG.info("[MIAS-LCEC 推理] 设置 EXE_ABS_PATH=%s 并创建 ResImg=%s (若 .so 不读此变量仍会报错)", work_dir, resimg_dir)
    except OSError as e:
        _LOG.warning("[MIAS-LCEC 推理] 无法创建可写 ResImg 目录 %s: %s", resimg_dir, e)
    _LOG.info("[MIAS-LCEC 推理] 即将导入 c3m/LcMatch_CApi；若报 Read-only file system: .../MIAS-LCEC/bin/ResImg/ 说明 .so 未使用 EXE_ABS_PATH，须将 MIAS-LCEC 目录挂载为可写")

    last_error_msg: Optional[str] = None
    for repo, repo_src in repos_to_try:
        try:
            added_paths: list[str] = []
            sys.path.insert(0, repo)
            added_paths.append(repo)
            if os.path.isdir(os.path.join(repo, "bin")):
                bin_path = os.path.join(repo, "bin")
                if bin_path not in sys.path:
                    sys.path.insert(0, bin_path)
                    added_paths.append(bin_path)
            bin_python = os.path.join(repo, "bin", "python")
            if os.path.isdir(bin_python) and bin_python not in sys.path:
                sys.path.insert(0, bin_python)
                added_paths.append(bin_python)
            try:
                infer_fn = None
                # 1) 标准模块名：infer_unicalib / infer / c3m / mias_infer
                for mod_name in ("infer_unicalib", "infer", "c3m", "mias_infer"):
                    try:
                        mod = __import__(mod_name)
                        if hasattr(mod, "estimate_extrinsic"):
                            infer_fn = getattr(mod, "estimate_extrinsic")
                            break
                        if hasattr(mod, "infer"):
                            infer_fn = getattr(mod, "infer")
                            break
                    except ImportError:
                        continue
                # 2) 直接尝试 MIAS-LCEC 的 LcMatch_CApi（.so 可能暴露 estimate_extrinsic / run_headless 等）
                if infer_fn is None and os.path.isdir(bin_python):
                    try:
                        mod = __import__("LcMatch_CApi")
                        for name in ("estimate_extrinsic", "run_headless", "calibrate", "run", "infer"):
                            if hasattr(mod, name):
                                attr = getattr(mod, name)
                                if callable(attr):
                                    infer_fn = attr
                                    _LOG.info("[MIAS-LCEC 推理] 从 LcMatch_CApi 使用可调用: %s", name)
                                    break
                        if infer_fn is None and hasattr(mod, "LcMatchCApi"):
                            api = getattr(mod, "LcMatchCApi")
                            if callable(api) and hasattr(api, "__code__") and api.__code__.co_argcount >= 1:
                                infer_fn = api
                                _LOG.info("[MIAS-LCEC 推理] 从 LcMatch_CApi 使用 LcMatchCApi(带参)")
                    except ImportError as e:
                        _LOG.error("[MIAS-LCEC 推理] LcMatch_CApi 不可导入 (可能需 Python 3.10): %s", e)
                        _LOG.error("[MIAS-LCEC 推理] 当前 sys.path: %s", "\n".join(sys.path))
                if infer_fn is None:
                    _LOG.warning("[MIAS-LCEC 推理] %s 中未找到 estimate_extrinsic/infer 入口，尝试下一候选 (repo=%s)", repo_src, repo)
                    continue
                _LOG.info("[MIAS-LCEC 推理] 使用深度模型 仓库=%s (%s) 模型=%s", repo, repo_src, model_path)
                try:
                    out = infer_fn(
                        pcd_path=pcd_path,
                        image_path=image_path,
                        sensor_config_path=sensor_config_path,
                        model_path=model_path,
                    )
                except TypeError:
                    out = infer_fn(pcd_path, image_path, sensor_config_path, model_path)
                if out is None:
                    continue
                R, t = out if isinstance(out, (tuple, list)) else (out["R"], out["t"])
                R = np.asarray(R, dtype=np.float64)
                t = np.asarray(t, dtype=np.float64).ravel()
                if R.shape != (3, 3) or t.shape != (3,):
                    continue
                return (R, t)
            finally:
                for p in added_paths:
                    if p in sys.path:
                        sys.path.remove(p)
        except Exception as e:
            import traceback
            last_error_msg = "{}: {}".format(type(e).__name__, e)
            _LOG.error("[MIAS-LCEC 推理] 推理异常 (仓库=%s): %s\n%s", repo_src, e, traceback.format_exc())
            if "Read-only file system" in str(e) or "Errno 30" in str(e):
                _LOG.error("[MIAS-LCEC 推理] 只读文件系统: LcMatch_CApi 的 envpath.so 从自身路径计算 ResImg，未读取 EXE_ABS_PATH 环境变量。唯一解决办法: 将 MIAS-LCEC 目录（或至少 <MIAS-LCEC>/bin）在容器/宿主机挂载为可写，使 <MIAS-LCEC>/bin/ResImg/ 可创建或可写")
            continue

    _LOG.info("[MIAS-LCEC 推理] 未使用深度模型: 所有候选仓库中均未找到可调用的 estimate_extrinsic/infer 入口；若已配置 model_path 则粗标定将失败（禁止 PnP 回退）")
    if last_error_msg:
        _LOG.error("[MIAS-LCEC 推理] ★ 深度模型未使用 — 原因摘要: %s", last_error_msg)
        if "Read-only" in last_error_msg or "Errno 30" in last_error_msg:
            _LOG.error("[MIAS-LCEC 推理] 修复建议: 在 Docker/运行环境中将 MIAS-LCEC 目录挂载为可写，例如 -v /path/to/MIAS-LCEC:/root/calib_ws/MIAS-LCEC (不要加 :ro)，确保 <MIAS-LCEC>/bin/ResImg/ 可由进程创建或写入")
        # 输出一行便于 C++ 解析的标记，用于粗标定诊断
        try:
            print("COARSE_REASON={}".format(last_error_msg.replace("\n", " ").strip()), file=sys.stdout)
        except Exception:
            pass

    # 纯 PyTorch 推理（不依赖 MIAS-LCEC .so / Python 3.10）
    # 支持 .pth.tar（OverlapTransformer）、.pth、.pt（如 mobile_sam.pt）以走 SAM-only 或全量 C3M
    if model_path.endswith(".pth.tar") or model_path.endswith(".pth") or model_path.endswith(".pt"):
        _LOG.info("[MIAS-LCEC 推理] .so 方案不可用，尝试纯 PyTorch 实现 (model_path=%s)", model_path)
        out = _try_self_contained_pytorch_infer(
            pcd_path, image_path, sensor_config_path, model_path
        )
        if out is not None:
            _LOG.info("[MIAS-LCEC 推理] ★ 纯 PyTorch 推理成功，返回 (R, t)")
            return out
        _LOG.warning("[MIAS-LCEC 推理] 纯 PyTorch 推理未返回结果，见上方 [纯 PyTorch] 步骤日志定位原因")

    return None


def _try_self_contained_pytorch_infer(
    pcd_path: str,
    image_path: str,
    sensor_config_path: str,
    model_path: str,
) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """
    纯 PyTorch 实现（无 .so 依赖，无 Python 3.10 限制）。
    
    使用 mias_lcec_pytorch 模块实现 LiDAR-Camera 外参估计:
    - BEV 投影
    - Overlap Transformer 特征提取
    - MobileSAM 分割
    - C3M Mask Matching
    - PnP 求解
    """
    # 优先尝试纯 PyTorch 实现
    out = _try_pure_pytorch_infer(pcd_path, image_path, sensor_config_path, model_path)
    if out is not None:
        return out
    
    # 回退：旧的占位实现（仅加载 checkpoint 检查）
    try:
        import torch
    except ImportError:
        _LOG.debug("[MIAS-LCEC 推理] 自包含推理: torch 不可用，跳过")
        return None
    if not os.path.isfile(pcd_path) or not os.path.isfile(image_path) or not os.path.isfile(model_path):
        return None
    try:
        torch.load(model_path, map_location="cpu", weights_only=False)
    except Exception as e:
        _LOG.debug("[MIAS-LCEC 推理] 自包含推理: 加载 checkpoint 失败 %s", e)
        return None
    _LOG.debug("[MIAS-LCEC 推理] 自包含推理: checkpoint 可加载，但纯 PyTorch 模块不可用，返回 None")
    return None


def _resolve_device_for_mias() -> str:
    """
    解析 MIAS 粗标定使用的计算设备。
    - 环境变量 UNICALIB_MIAS_LCEC_DEVICE / MIAS_LCEC_DEVICE 为 cpu 时强制 CPU。
    - 为 cuda 或 auto 时：若本机 GPU 架构不被当前 PyTorch 支持（如 RTX 50 系 sm_120 与旧版 PyTorch），
      会探测一次实际 CUDA 运行，失败则回退到 CPU，避免 "no kernel image is available for execution on the device" 崩溃。
    """
    try:
        import torch
    except ImportError:
        return "cpu"

    env_device = (os.environ.get("UNICALIB_MIAS_LCEC_DEVICE") or os.environ.get("MIAS_LCEC_DEVICE") or "").strip().lower()
    if env_device == "cpu":
        return "cpu"
    if not torch.cuda.is_available():
        return "cpu"

    # 用户显式要求 cuda 或 auto：探测当前 PyTorch 是否能在本机 GPU 上运行
    try:
        # 轻量探测：在 GPU 上分配并做一次运算，触发 kernel 编译/执行
        t = torch.zeros(1, device="cuda")
        _ = t + 1
        return "cuda"
    except RuntimeError as e:
        err = str(e).lower()
        if "no kernel image" in err or "not compatible" in err or "sm_" in err or "cuda capability" in err:
            _LOG.warning(
                "[纯 PyTorch] 当前 PyTorch 不支持本机 GPU 架构，已回退到 CPU（原因: %s）。"
                " 若需使用 GPU，请安装支持本机 CUDA 架构的 PyTorch（如 RTX 50 系需 sm_120 支持）。",
                e,
            )
        else:
            _LOG.warning("[纯 PyTorch] CUDA 探测失败，回退到 CPU: %s", e)
        return "cpu"


def _try_pure_pytorch_infer(
    pcd_path: str,
    image_path: str,
    sensor_config_path: str,
    model_path: str,
) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """
    纯 PyTorch 实现：调用 mias_lcec_pytorch 模块。
    """
    _LOG.info("[纯 PyTorch] ========== 开始纯 PyTorch 粗标定 ==========")
    _LOG.info("[纯 PyTorch] 输入: pcd=%s image=%s sensor_config=%s model_path=%s",
              pcd_path, image_path, sensor_config_path, model_path)

    try:
        import yaml
    except ImportError as e:
        _LOG.warning("[纯 PyTorch] 步骤[1] 失败 — yaml 不可用: %s", e)
        return None

    # 读取相机内参
    _LOG.debug("[纯 PyTorch] 步骤[2] 读取 sensor_config...")
    try:
        with open(sensor_config_path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)
    except Exception as e:
        _LOG.warning("[纯 PyTorch] 步骤[2] 失败 — 读取 sensor_config 异常: %s", e)
        return None

    if "camera" not in cfg:
        _LOG.warning("[纯 PyTorch] 步骤[2] 失败 — sensor_config 缺少 camera 字段")
        return None

    cam = cfg["camera"]
    fx = float(cam.get("fx", 0))
    fy = float(cam.get("fy", 0))
    cx = float(cam.get("cx", 0))
    cy = float(cam.get("cy", 0))
    _LOG.info("[纯 PyTorch] 步骤[2] 完成 — 内参 fx=%.2f fy=%.2f cx=%.2f cy=%.2f", fx, fy, cx, cy)

    if fx <= 0 or fy <= 0:
        _LOG.warning("[纯 PyTorch] 步骤[2] 失败 — 相机内参无效 fx=%s fy=%s", fx, fy)
        return None

    K = np.array([
        [fx, 0, cx],
        [0, fy, cy],
        [0, 0, 1],
    ], dtype=np.float64)

    dist = cam.get("dist_coeffs")
    if dist is not None:
        dist = np.asarray(dist, dtype=np.float64)
    else:
        dist = np.zeros(5, dtype=np.float64)

    # 导入纯 PyTorch 模块
    _LOG.debug("[纯 PyTorch] 步骤[3] 导入 mias_lcec_pytorch 模块...")
    try:
        script_dir = Path(__file__).resolve().parent
        # 插入 scripts 目录以便 import mias_lcec_pytorch
        if str(script_dir) not in sys.path:
            sys.path.insert(0, str(script_dir))
        from mias_lcec_pytorch.coarse_calib import MIASLCECCoarseCalib
        _LOG.info("[纯 PyTorch] 步骤[3] 完成 — 模块导入成功")
    except ImportError as e:
        _LOG.warning("[纯 PyTorch] 步骤[3] 失败 — 模块导入异常: %s (请确认 scripts 下存在 mias_lcec_pytorch)", e)
        return None

    # 判断是否仅用 mobile_sam.pt（不加载 pretrained_overlap_transformer.pth.tar）
    model_basename = os.path.basename(model_path)
    use_sam_only = model_basename == "mobile_sam.pt" or model_path.endswith("/mobile_sam.pt")

    # 查找 MobileSAM 权重
    _LOG.debug("[纯 PyTorch] 步骤[4] 解析 MobileSAM 权重路径...")
    mobilesam_ckpt = os.environ.get("MOBILESAM_CHECKPOINT", "").strip()
    if use_sam_only and os.path.isfile(model_path):
        mobilesam_ckpt = os.path.abspath(model_path)
        _LOG.info("[纯 PyTorch] 步骤[4] 仅 SAM 模式 — 使用 model_path 作为 MobileSAM 权重: %s", mobilesam_ckpt)
    elif not mobilesam_ckpt:
        model_dir = os.path.dirname(model_path)
        candidates = [
            os.path.join(model_dir, "mobile_sam.pt"),
            "/root/calib_ws/MIAS-LCEC/model/mobile_sam.pt",
            "/root/calib_ws/MIAS-LCEC/bin/MobileSAM/weights/mobile_sam.pt",
            "/root/calib_ws/MIAS-LCEC/bin/weights/mobile_sam.pt",
            os.path.join(model_dir, "..", "bin", "MobileSAM", "weights", "mobile_sam.pt"),
            os.path.join(model_dir, "..", "..", "bin", "MobileSAM", "weights", "mobile_sam.pt"),
        ]
        for c in candidates:
            if c and os.path.isfile(c):
                mobilesam_ckpt = c
                break
    if mobilesam_ckpt:
        _LOG.info("[纯 PyTorch] 步骤[4] 完成 — MobileSAM 权重: %s", mobilesam_ckpt)
    else:
        _LOG.info("[纯 PyTorch] 步骤[4] 完成 — 未找到 MobileSAM 权重，将使用简化分割")

    # 检测设备：环境变量 UNICALIB_MIAS_LCEC_DEVICE 或 MIAS_LCEC_DEVICE 可强制为 cpu/cuda
    # 若选 cuda 但当前 PyTorch 不支持本机 GPU 架构（如 RTX 50 系 sm_120），会回退到 CPU 避免 "no kernel image" 崩溃
    device = _resolve_device_for_mias()
    _LOG.info("[纯 PyTorch] 步骤[5] 计算设备: %s", device)

    # 创建粗标定器（仅 SAM 模式时 overlap_ckpt 为空，不加载 .pth.tar）
    _LOG.debug("[纯 PyTorch] 步骤[6] 创建 MIASLCECCoarseCalib (use_sam_only=%s)...", use_sam_only)
    try:
        calib = MIASLCECCoarseCalib(
            overlap_ckpt="" if use_sam_only else model_path,
            mobilesam_ckpt=mobilesam_ckpt if mobilesam_ckpt else None,
            device=device,
            use_sam_only=use_sam_only,
        )
        _LOG.info("[纯 PyTorch] 步骤[6] 完成 — 粗标定器创建成功")
    except Exception as e:
        import traceback
        _LOG.error("[纯 PyTorch] 步骤[6] 失败 — 创建粗标定器异常: %s", e)
        _LOG.debug("[纯 PyTorch] traceback:\n%s", traceback.format_exc())
        return None

    # 执行推理
    _LOG.info("[纯 PyTorch] 步骤[7] 执行 estimate_extrinsic...")
    try:
        result = calib.estimate_extrinsic(
            pcd_path=pcd_path,
            image_path=image_path,
            camera_k=K,
            camera_dist=dist,
        )
    except Exception as e:
        import traceback
        _LOG.error("[纯 PyTorch] 步骤[7] 失败 — 推理异常: %s", e)
        _LOG.error("[纯 PyTorch] traceback:\n%s", traceback.format_exc())
        return None

    if result is None:
        _LOG.warning("[纯 PyTorch] 步骤[7] 完成 — 推理返回 None（可能为 BEV/OT/SAM/C3M/PnP 任一环节失败，见上方日志）")
        return None

    R, t = result
    _LOG.info("[纯 PyTorch] ========== 纯 PyTorch 粗标定成功 ==========")
    _LOG.info("[纯 PyTorch] 输出 R.shape=%s t=[%.4f, %.4f, %.4f]", R.shape, t[0], t[1], t[2])
    return (R, t)


def try_load_pth_and_placeholder_infer(
    pcd_path: str,
    image_path: str,
    model_path: str,
) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """
    占位：仅加载 .pth/.pth.tar 检查可用性，不执行真实位姿预测。
    真实推理需 MIAS-LCEC 仓库中完整网络定义与数据预处理。
    """
    try:
        import torch
    except ImportError:
        return None
    if not os.path.isfile(model_path):
        return None
    try:
        ckpt = torch.load(model_path, map_location="cpu", weights_only=False)
        if isinstance(ckpt, dict) and "state_dict" in ckpt:
            state = ckpt["state_dict"]
        else:
            state = ckpt
        # 无网络定义时无法前向，直接返回 None
        if not isinstance(state, dict):
            return None
        _LOG.debug("已加载 checkpoint 键数: %d", len(state))
    except Exception as e:
        _LOG.debug("加载 checkpoint 失败: %s", e)
        return None
    return None
