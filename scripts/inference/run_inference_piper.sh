#!/bin/bash
# Inference script for Piper robot diffusion model
# Piper 机械臂扩散模型推理脚本
#
# 使用方法:
#   1. 从验证集随机选择起点/终点:  bash run_inference_piper.sh
#   2. 使用自定义起点/终点:        bash run_inference_piper.sh custom
#
# 自定义起点/终点时，修改下面的 CUSTOM_Q_START 和 CUSTOM_Q_GOAL 变量
# 格式: 逗号分隔的6个关节角度值（弧度）

# ============ 自定义起点和终点 (可修改) ============
CUSTOM_Q_START="-2.233940,2.373187,-2.514964,0.943502,1.213379,0.733293"       # 起点关节角度
CUSTOM_Q_GOAL="0.742593,1.9,-0.925766,-1.096702,-1.074476,0.775625"         # 终点关节角度
# ==================================================

PROJECT_ROOT="/home/hong/Projects/MotionPlanningDiffusion/mpd-splines-public"

cd $PROJECT_ROOT

# Activate conda environment
source ~/miniconda3/etc/profile.d/conda.sh
conda activate mpd-splines-public

# Set environment variables
source set_env_variables.sh

# Add isaacgym to PYTHONPATH
export PYTHONPATH="${PROJECT_ROOT}/deps/isaacgym/python:${PYTHONPATH}"

# 检查是否使用自定义起点/终点
if [ "$1" == "custom" ]; then
    echo "使用自定义起点和终点:"
    echo "  起点: $CUSTOM_Q_START"
    echo "  终点: $CUSTOM_Q_GOAL"
    SELECTION_MODE="custom"
else
    SELECTION_MODE="validation"
fi

# Run inference
cd scripts/inference
if [ "$1" == "custom" ]; then
    python3 inference.py \
        --cfg_inference_path="./cfgs/config_EnvWarehouse-RobotPiper_00.yaml" \
        --selection_start_goal="$SELECTION_MODE" \
        --custom_q_start="$CUSTOM_Q_START" \
        --custom_q_goal="$CUSTOM_Q_GOAL" \
        --n_start_goal_states=1 \
        --render_joint_space_time_iters=True \
        --render_env_robot_trajectories=False \
        --run_evaluation_issac_gym=True \
        --render_isaacgym_viewer=True \
        --render_isaacgym_movie=True \
        --device="cuda:0" \
        --seed=2 \
        --results_dir="logs_inference_piper"
else
    python3 inference.py \
        --cfg_inference_path="./cfgs/config_EnvWarehouse-RobotPiper_00.yaml" \
        --selection_start_goal="$SELECTION_MODE" \
        --n_start_goal_states=1 \
        --render_joint_space_time_iters=True \
        --render_env_robot_trajectories=False \
        --run_evaluation_issac_gym=True \
        --render_isaacgym_viewer=True \
        --render_isaacgym_movie=True \
        --device="cuda:0" \
        --seed=5 \
        --results_dir="logs_inference_piper"
fi

echo "Inference completed! Results saved to logs_inference_piper/"
