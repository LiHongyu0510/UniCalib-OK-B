# 🚀 快速执行 - MIAS-LCEC GLM 修复

## ✅ 已完成的修复

1. ✅ 下载 GLM 0.9.9.8 到 `docker/deps/glm/`
2. ✅ 修复 `build_external_tools.sh` 编译路径
3. ✅ 添加本地 GLM 依赖支持（通过 `GLM_ROOT_DIR`）
4. ✅ 挂载 `docker/deps` 目录到容器
5. ✅ Dockerfile 添加 GLM apt 回退

---

## 🎯 立即执行

### 验证 GLM 已下载

```bash
ls -la /home/wqs/Documents/github/UniCalib/docker/deps/glm/glm/glm.hpp
```

### 执行编译

```bash
cd /home/wqs/Documents/github/UniCalib
bash build_and_run.sh
```

### 预期输出（MIAS-LCEC 部分）

```
>>> 编译 MIAS-LCEC (C++ + Python)
[INFO] 使用本地 GLM: /root/calib_ws/docker/deps/glm
[INFO] 编译 C++ 扩展...
-- Found GLM: /root/calib_ws/docker/deps/glm/glm/glm.hpp
[PASS] MIAS-LCEC 编译完成
```

---

## 📁 核心修改

### 1. GLM 位置
```
docker/deps/glm/
└── glm/
    └── glm.hpp  ← Header-only 库
```

### 2. 环境变量
```bash
export GLM_ROOT_DIR="/root/calib_ws/docker/deps/glm"
```

### 3. Docker 挂载
```bash
-v "$(pwd)/docker/deps:/root/thirdparty/deps:ro"
```

---

## 🐛 问题排查

### GLM 未找到
```bash
# 检查文件
ls -la docker/deps/glm/glm/glm.hpp

# 重新下载
cd docker/deps && bash download_glm.sh
```

### 容器内无法访问
```bash
# 验证挂载
docker run --rm -v $(pwd):/root/calib_ws calib_env:humble \
    ls -la /root/calib_ws/docker/deps/glm/glm/glm.hpp
```

---

## 📚 详细文档

- `FIX_COMPLETE.md` - 完整修复说明
- `docker/DEPS_README.md` - 依赖管理文档
- `docker/download_glm.sh` - GLM 下载脚本

---

**现在可以执行 `bash build_and_run.sh` 了！** 🎉
