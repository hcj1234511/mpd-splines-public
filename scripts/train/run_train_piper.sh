#!/bin/bash
# Training script for Piper robot diffusion model

PROJECT_ROOT="/home/hong/Projects/MotionPlanningDiffusion/mpd-splines-public"

cd $PROJECT_ROOT

# Activate conda environment
source ~/miniconda3/etc/profile.d/conda.sh
conda activate mpd-splines-public

# Set environment variables
source set_env_variables.sh

# Add isaacgym to PYTHONPATH
export PYTHONPATH="${PROJECT_ROOT}/deps/isaacgym/python:${PYTHONPATH}"

# Run training
cd scripts/train
python3 train_piper.py \
    --dataset_dir "${PROJECT_ROOT}/data_trajectories_hcj/piper_100000_obstacle-free" \
    --dataset_file_merged "dataset_merged_processed.hdf5" \
    --num_T_pts 256 \
    --batch_size 32 \
    --num_train_steps 100000 \
    --device "cuda:0" \
    --seed 1 \
    --results_dir "logs_piper" \
    --wandb_mode "disabled"

echo "Training completed!"
