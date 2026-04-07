#!/bin/bash
set -e
# 环境变量：强制 LiteLLM 使用本地成本表，解决 GitHub 连接超时
export LITELLM_LOCAL_MODEL_COST_MAP=True

# 启动 Cecli (它会自动读取项目根目录下的 .aider.conf.yml)
all_proxy="" http_proxy="" https_proxy="" cecli
