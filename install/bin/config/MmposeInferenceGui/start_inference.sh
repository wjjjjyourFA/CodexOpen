#!/usr/bin/env bash

set -e

# 1. 检查是否传入了参数（必须传入1个参数）
if [ $# -ne 1 ]; then
  echo "用法错误！正确用法：./start_inference.sh <num>"
  echo "示例：./start_inference.sh 1 2 3 4 5"
  exit 1  # 退出脚本，返回错误码
fi

# 2. 将传入的第一个参数赋值给变量 video_number，让语义更清晰
NUM="$1"
echo "=============================="
echo "FSZN analysis start"
echo "num = ${NUM}"
echo "=============================="

# # ---- 1. 初始化 conda ----
# # CONDA_BASE="$HOME/miniconda3"
# CONDA_BASE="$HOME/anaconda3"
# if [ ! -f "$CONDA_BASE/etc/profile.d/conda.sh" ]; then
#   echo "ERROR: conda.sh not found at $CONDA_BASE"
#   exit 1
# fi
# source "$CONDA_BASE/etc/profile.d/conda.sh"
# conda activate openmmlab
# echo "Activated conda env: $(conda info --envs | grep '*')"

# # ---- 2. 执行推理 ----
# MMPOSE_ROOT="/home/jojo/OpenMMLab/mmpose-1.3.2"
# DEMO_DIR="$MMPOSE_ROOT/demo"

# CONFIG_ROOT="/media/jojo/AQiDePan/CodexOpen/install/bin/config/MmposeInferenceGui"

# cd "$DEMO_DIR"
# python ./inferencer_demo_fszn00.py \
#   --config "$CONFIG_ROOT/mmpose_$NUM.ini"

echo "=============================="
echo "FSZN analysis finished"
echo "=============================="