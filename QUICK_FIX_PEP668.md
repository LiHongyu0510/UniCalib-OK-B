# PEP 668 错误快速修复指南

## 问题现象

```bash
error: externally-managed-environment

× This environment is externally managed
╰─> To install Python packages system-wide, try apt install
    python3-xyz, where xyz is the package you are trying to
    install.
```

**原因**：Python 3.12+ 引入了 PEP 668 保护机制，禁止直接在系统 Python 环境中使用 `pip install`。

---

## 解决方案（3种方式）

### 方案1：Docker 容器内编译（推荐）⭐

这是项目的标准工作流，环境已配置好，无限制。

```bash
# 步骤1：进入 Docker 容器
./build_and_run.sh --shell

# 步骤2：在容器内执行编译
cd /workspace/UniCalib
./build_external_tools.sh

# 步骤3：编译完成后退出容器
exit
```

**优点**：
- ✅ Docker 镜像 `calib_env:humble` 已配置好 Python 环境
- ✅ 不受宿主机 PEP 668 限制
- ✅ 环境隔离，可重现
- ✅ 符合项目设计初衷

---

### 方案2：使用改进的脚本（支持虚拟环境）

已修改 `build_external_tools.sh`，自动检测环境并创建虚拟环境。

```bash
# 直接运行即可（无需手动创建虚拟环境）
./build_external_tools.sh
```

**工作原理**：
- 检测是否在 Docker 容器内
  - 如果是容器：直接使用系统 pip
  - 如果是宿主机：为每个工具创建独立的 `.venv` 虚拟环境
- 虚拟环境路径记录在各工具目录的 `.venv_path` 文件中

**优点**：
- ✅ 自动化，无需手动干预
- ✅ 每个工具环境独立，避免依赖冲突
- ✅ 符合 Python 最佳实践

---

### 方案3：临时强制安装（不推荐）

仅用于临时调试，不建议用于生产环境。

```bash
# 注意：这会破坏系统 Python 环境保护
pip3 install --break-system-packages -r requirements.txt
```

**缺点**：
- ❌ 破坏系统 Python 保护机制
- ❌ 可能影响系统其他 Python 应用
- ❌ 不可逆风险

---

## 验证修复

```bash
# 检查是否成功安装依赖
./build_external_tools.sh --check-only

# 或查看虚拟环境是否创建
ls -la DM-Calib/.venv/
ls -la learn-to-calibrate/.venv/
```

---

## 后续使用

### 在虚拟环境中运行工具

如果使用了方案2（虚拟环境），需要先激活虚拟环境：

```bash
# DM-Calib 示例
cd DM-Calib
source .venv/bin/activate
python DMCalib/tools/infer.py --help

# learn-to-calibrate 示例
cd learn-to-calibrate
source .venv/bin/activate
python demo/calib.sh
```

### 一键激活脚本（可选）

如果经常需要运行不同工具，可以创建快捷脚本：

```bash
# ~/.bashrc 或 ~/.zshrc 添加
alias calib-dm='source ~/Documents/github/UniCalib/DM-Calib/.venv/bin/activate'
alias calib-l2c='source ~/Documents/github/UniCalib/learn-to-calibrate/.venv/bin/activate'
alias calib-mias='source ~/Documents/github/UniCalib/MIAS-LCEC/.venv/bin/activate'
alias calib-click='source ~/Documents/github/UniCalib/click_calib/.venv/bin/activate'
```

---

## 常见问题

### Q1: 为什么推荐 Docker 方式？

A: 项目设计之初就是基于 Docker 的，镜像已预配置好所有依赖（包括 ROS2、CUDA、PyTorch 等），环境一致性强，便于团队协作。

### Q2: 虚拟环境占用多少空间？

A: 每个工具的虚拟环境约 500MB - 2GB（取决于依赖包）。如果空间紧张，可以使用 Docker 方式。

### Q3: 如何清理虚拟环境？

```bash
# 删除所有工具的虚拟环境
rm -rf DM-Calib/.venv
rm -rf learn-to-calibrate/.venv
rm -rf MIAS-LCEC/.venv
rm -rf click_calib/.venv
```

### Q4: iKalibr 需要虚拟环境吗？

A: iKalibr 是 ROS2 + C++ 项目，使用 colcon 编译，不涉及 Python pip 安装，无需虚拟环境。

---

## 相关文档

- [外部工具编译指南](BUILD_EXTERNAL_TOOLS.md)
- [构建和运行指南](BUILD_AND_RUN_GUIDE.md)
- [主 README](README.md)
