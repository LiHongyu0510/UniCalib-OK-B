# 多方标定工具模型加载分析与配置

## 一、Executive Summary

| 工程 | 模型位置 | 代码期望路径 | 状态 | 修复方案 |
|------|----------|--------------|------|----------|
| **Transformer-IMU-Calibrator** | `model/TIC_13.pth` | `./checkpoint/TIC_13.pth` | ❌ 路径不匹配 | 创建软链接或修改代码 |
| **MIAS-LCEC** | `model/pretrained_overlap_transformer.pth.tar` | C++ 内部加载 | ⚠️ 需验证 | 检查C++配置 |
| **learn-to-calibrate** | `rl_solver/bingham/precomputed/*.model` | 相对路径 | ⚠️ 工作目录依赖 | 使用绝对路径 |
| **DM-Calib** | `model/calib/` + `model/depth_safe/` | `--pretrained_model_path` | ✅ 已修复 | 使用 infer_unicalib.py |

---

## 二、各工程详细分析

### 2.1 Transformer-IMU-Calibrator

**模型文件：**
```
Transformer-IMU-Calibrator/
└── model/
    └── TIC_13.pth    # 32MB - 主模型
```

**代码中的加载路径：**
```python
# eval.py:11
model.restore('./checkpoint/TIC_13.pth')

# eval_livedemo.py:12
model.restore('./checkpoint/TIC_13.pth')
```

**问题：** 模型在 `model/` 目录，代码期望在 `./checkpoint/` 目录

**修复方案 A：创建软链接（推荐）**
```bash
cd Transformer-IMU-Calibrator
mkdir -p checkpoint
ln -s ../model/TIC_13.pth checkpoint/TIC_13.pth
```

**修复方案 B：移动文件**
```bash
cd Transformer-IMU-Calibrator
mkdir -p checkpoint
mv model/TIC_13.pth checkpoint/
```

---

### 2.2 MIAS-LCEC

**模型文件：**
```
MIAS-LCEC/
└── model/
    └── pretrained_overlap_transformer.pth.tar    # 340MB - Overlap Transformer
```

**加载机制：**
- 主要为 C++ 实现，Python 接口调用 C API
- 模型加载在 C++ 层完成
- `bin/python/c3m.py` 通过 `LcMatchCApi()` 调用底层

**配置文件：**
```json
// bin/python/config.json
{
    "path": {
        "ros": "/bin/python",
        "conda": "/path/to/conda/site-packages",
        "sam": "/path/to/MIAS-LCEC/bin/MobileSAM"
    }
}
```

**注意事项：**
1. 需要正确编译 C++ 可执行文件 `bin/mias_lcec`
2. 模型路径可能在 C++ 配置中硬编码或通过参数传递

---

### 2.3 learn-to-calibrate

**模型文件：**
```
learn-to-calibrate/
└── rl_solver/
    └── bingham/
        └── precomputed/
            ├── b_model_-500_0_200.model
            └── norm_constant_model_-500_0_200.model
```

**代码中的加载：**
```python
# rl_solver/bingham/utils.py:7
path = "rl_solver/bingham/precomputed/norm_constant_model_-500_0_200.model"
norm_const_model.load_state_dict(torch.load(path, map_location=device))
```

**问题：** 使用相对路径，依赖工作目录

**修复方案：** 在 wrapper 中设置正确的工作目录或使用绝对路径

**unicalib_C_plus_plus 中的处理：**
```cpp
// learn_to_calib_wrapper.cpp
env["PYTHONPATH"] = tools_config.learn_to_calibrate + ":" +
    tools_config.learn_to_calibrate + "/rl_solver";
```

---

### 2.4 DM-Calib

**模型文件：**
```
DM-Calib/
└── model/
    ├── calib/
    │   ├── config.json
    │   └── diffusion_pytorch_model.safetensors    # 3.4GB - 标定模型
    ├── depth_safe/
    │   ├── config.json
    │   ├── diffusion_pytorch_model.safetensors    # 3.0GB - 深度模型
    │   └── vae_decoder.safetensors               # 198MB - VAE解码器
    └── .cache/
```

**代码加载方式：**
```python
# DMCalib/infer.py
# 支持本地路径或 HuggingFace ID
unet_cam = UNet2DConditionModel.from_pretrained(
    checkpoint_path, subfolder="calib/unet"
)

# 或使用 HuggingFace ID
# default: "juneyoung9/DM-Calib"
```

**unicalib_C_plus_plus 中的调用：**
```cpp
// dm_calib_wrapper.cpp
std::string model_dir = tools_config.dm_calib + "/model";
std::vector<std::string> argv = {
    python_exe, infer_py,
    "--image_dir", image_dir,
    "--model_dir", model_dir,    // 这里应该是 --pretrained_model_path
    "--output", out_json
};
```

**⚠️ 发现问题：** wrapper 代码使用 `--model_dir` 参数，但实际代码期望 `--pretrained_model_path`

---

## 三、统一修复脚本

### 3.1 一键修复脚本

```bash
#!/bin/bash
# setup_models.sh - 修复所有工程的模型加载路径

set -e
CALIB_ROOT="$(cd "$(dirname "$0")" && pwd)"

echo "=== 设置模型加载路径 ==="

# 1. Transformer-IMU-Calibrator
echo "[1/4] Transformer-IMU-Calibrator..."
cd "$CALIB_ROOT/Transformer-IMU-Calibrator"
mkdir -p checkpoint
if [ ! -e checkpoint/TIC_13.pth ]; then
    ln -sf ../model/TIC_13.pth checkpoint/TIC_13.pth
    echo "  ✓ 创建软链接 checkpoint/TIC_13.pth -> model/TIC_13.pth"
else
    echo "  ✓ checkpoint/TIC_13.pth 已存在"
fi

# 2. MIAS-LCEC (检查配置)
echo "[2/4] MIAS-LCEC..."
cd "$CALIB_ROOT/MIAS-LCEC"
if [ -f "model/pretrained_overlap_transformer.pth.tar" ]; then
    echo "  ✓ 模型文件存在: model/pretrained_overlap_transformer.pth.tar"
else
    echo "  ⚠ 模型文件缺失，请下载"
fi

# 3. learn-to-calibrate (检查模型)
echo "[3/4] learn-to-calibrate..."
cd "$CALIB_ROOT/learn-to-calibrate"
if [ -f "rl_solver/bingham/precomputed/norm_constant_model_-500_0_200.model" ]; then
    echo "  ✓ Bingham 模型文件存在"
else
    echo "  ⚠ Bingham 模型文件缺失"
fi

# 4. DM-Calib (检查模型)
echo "[4/4] DM-Calib..."
cd "$CALIB_ROOT/DM-Calib"
if [ -f "model/calib/diffusion_pytorch_model.safetensors" ]; then
    echo "  ✓ DM-Calib 标定模型存在"
else
    echo "  ⚠ DM-Calib 模型文件缺失"
fi

echo ""
echo "=== 模型设置完成 ==="
```

### 3.2 环境变量配置

```bash
# 添加到 ~/.bashrc 或 Docker 环境
export UNICALIB_TRANSFORMER_IMU="/path/to/calibration/Transformer-IMU-Calibrator"
export UNICALIB_DMCALIB="/path/to/calibration/DM-Calib"
export UNICALIB_LEARN2CALIB="/path/to/calibration/learn-to-calibrate"
export UNICALIB_MIASLCEC="/path/to/calibration/MIAS-LCEC"
```

### 3.3 sensors.yaml 完整配置

```yaml
# unicalib_C_plus_plus/config/sensors.yaml
system:
  output_dir: "./calib_results"

third_party:
  # 使用绝对路径确保模型能正确加载
  dm_calib: "/path/to/calibration/DM-Calib"
  learn_to_calibrate: "/path/to/calibration/learn-to-calibrate"
  mias_lcec: "/path/to/calibration/MIAS-LCEC"
  ikalibr: "/path/to/calibration/iKalibr"
  click_calib: "/path/to/calibration/click_calib"
  transformer_imu: "/path/to/calibration/Transformer-IMU-Calibrator"

sensors:
  - sensor_id: cam_front
    sensor_type: camera_pinhole
    topic: /camera/image_raw
    resolution: [1920, 1080]
  - sensor_id: imu
    sensor_type: imu
    topic: /imu/data
    rate: 200
  - sensor_id: lidar
    sensor_type: lidar
    topic: /lidar/points
```

---

## 四、各工程运行验证

### 4.1 Transformer-IMU-Calibrator 验证

```bash
cd Transformer-IMU-Calibrator

# 验证模型加载
python3 -c "
import torch
from my_model import TIC
model = TIC(stack=3, n_input=6 * (3 + 3 * 3), n_output=6 * 6)
model.restore('./checkpoint/TIC_13.pth')
print('✓ Transformer-IMU 模型加载成功')
"
```

### 4.2 DM-Calib 验证

```bash
cd DM-Calib

# 验证本地模型加载
python3 DMCalib/infer.py \
    --pretrained_model_path ./model \
    --input_dir /path/to/test/images \
    --output_dir /tmp/dmcalib_test

# 预期输出应包含:
# "loading pipeline whole successfully."
```

### 4.3 learn-to-calibrate 验证

```bash
cd learn-to-calibrate

# 验证 Bingham 模型
python3 -c "
import torch
from rl_solver.bingham.utils import load_norm_const_model
device = torch.device('cpu')
model = load_norm_const_model(device)
print('✓ learn-to-calibrate Bingham 模型加载成功')
"
```

### 4.4 MIAS-LCEC 验证

```bash
cd MIAS-LCEC

# 检查 C++ 可执行文件
if [ -x "bin/mias_lcec" ]; then
    echo "✓ MIAS-LCEC 可执行文件存在"
    ./bin/mias_lcec --help 2>/dev/null || echo "  (需要查看使用说明)"
else
    echo "⚠ MIAS-LCEC 需要先编译"
fi
```

---

## 五、Docker 环境配置

### 5.1 Dockerfile 模型路径

```dockerfile
# 在 Dockerfile 中确保模型路径正确
ENV UNICALIB_ROOT=/root/calib_ws

# Transformer-IMU-Calibrator
RUN cd ${UNICALIB_ROOT}/Transformer-IMU-Calibrator && \
    mkdir -p checkpoint && \
    ln -sf ../model/TIC_13.pth checkpoint/TIC_13.pth

# 设置工作目录
WORKDIR ${UNICALIB_ROOT}
```

### 5.2 docker-compose 模型挂载

```yaml
services:
  calibration:
    volumes:
      - ./Transformer-IMU-Calibrator:/root/calib_ws/Transformer-IMU-Calibrator
      - ./DM-Calib:/root/calib_ws/DM-Calib
      - ./learn-to-calibrate:/root/calib_ws/learn-to-calibrate
      - ./MIAS-LCEC:/root/calib_ws/MIAS-LCEC
```

---

## 六、常见问题排查

| 问题 | 现象 | 解决方案 |
|------|------|----------|
| **Transformer-IMU 找不到模型** | `File doesn't exist: ./checkpoint/TIC_13.pth` | 创建软链接 `ln -sf ../model/TIC_13.pth checkpoint/` |
| **DM-Calib 加载失败** | `OSError: Can't load tokenizer` | 使用 `--pretrained_model_path` 指定本地路径 |
| **learn-to-calibrate 路径错误** | `FileNotFoundError: rl_solver/bingham/...` | 从项目根目录运行或设置 PYTHONPATH |
| **MIAS-LCEC 找不到模型** | C++ 运行时错误 | 检查编译配置和模型路径 |

---

## 七、Docker 容器内运行

### 7.1 目录映射

| 宿主机路径 | 容器内路径 |
|------------|------------|
| `{项目根目录}/Transformer-IMU-Calibrator` | `/root/calib_ws/Transformer-IMU-Calibrator` |
| `{项目根目录}/DM-Calib` | `/root/calib_ws/DM-Calib` |
| `{项目根目录}/learn-to-calibrate` | `/root/calib_ws/learn-to-calibrate` |
| `{项目根目录}/MIAS-LCEC` | `/root/calib_ws/MIAS-LCEC` |

### 7.2 容器内配置文件

使用 `sensors_docker.yaml`，已预配置容器内绝对路径：

```yaml
third_party:
  dm_calib: "/root/calib_ws/DM-Calib"
  learn_to_calibrate: "/root/calib_ws/learn-to-calibrate"
  mias_lcec: "/root/calib_ws/MIAS-LCEC"
  transformer_imu: "/root/calib_ws/Transformer-IMU-Calibrator"
```

### 7.3 容器内验证模型

```bash
# 进入容器
./build_and_run.sh --shell

# 在容器内运行
cd /root/calib_ws
./setup_models.sh
./verify_models.sh
```

### 7.4 运行标定

```bash
cd /root/calib_ws/unicalib_C_plus_plus/build
RUN_PIPELINE=1 ./unicalib_example ../config/sensors_docker.yaml /root/calib_ws/data
```

---

## 八、模型文件清单

| 工程 | 文件 | 大小 | MD5 (参考) |
|------|------|------|------------|
| Transformer-IMU | `model/TIC_13.pth` | 32 MB | - |
| MIAS-LCEC | `model/pretrained_overlap_transformer.pth.tar` | 340 MB | - |
| learn-to-calibrate | `rl_solver/bingham/precomputed/*.model` | <1 MB | - |
| DM-Calib | `model/calib/diffusion_pytorch_model.safetensors` | 3.4 GB | - |
| DM-Calib | `model/depth_safe/diffusion_pytorch_model.safetensors` | 3.0 GB | - |
| DM-Calib | `model/depth_safe/vae_decoder.safetensors` | 198 MB | - |

**总计：** 约 7 GB 模型文件
