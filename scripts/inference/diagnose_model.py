#!/usr/bin/env python
"""
诊断脚本：区分问题是出在推理参数还是训练模型
"""
import isaacgym  # Must import first!

import os
import sys
import torch
import numpy as np
import matplotlib.pyplot as plt
import h5py

# 添加项目路径
sys.path.insert(0, '/home/hong/Projects/MotionPlanningDiffusion/mpd-splines-public')

from mpd.utils.loaders import load_params_from_yaml
from torch_robotics.torch_utils.torch_utils import to_numpy


def check_training_data_smoothness(dataset_path):
    """检查训练数据的平滑度"""
    print("\n" + "="*80)
    print("1. 检查训练数据平滑度")
    print("="*80)
    
    with h5py.File(dataset_path, 'r') as f:
        sol_path = f['sol_path'][:]  # [N, T, D]
        print(f"数据形状: {sol_path.shape}")
        
        # 计算速度和加速度
        velocities = np.diff(sol_path, axis=1)
        accelerations = np.diff(velocities, axis=1)
        
        # 计算统计信息
        vel_std = np.std(velocities, axis=1).mean(axis=0)
        acc_std = np.std(accelerations, axis=1).mean(axis=0)
        
        print(f"\n每个关节的速度标准差（平均）: {vel_std}")
        print(f"每个关节的加速度标准差（平均）: {acc_std}")
        
        # 检查是否有异常跳跃
        vel_max = np.abs(velocities).max(axis=(0, 1))
        acc_max = np.abs(accelerations).max(axis=(0, 1))
        
        print(f"\n每个关节的最大速度: {vel_max}")
        print(f"每个关节的最大加速度: {acc_max}")
        
        # 可视化几条轨迹
        fig, axes = plt.subplots(3, 2, figsize=(14, 10))
        
        for i in range(min(5, sol_path.shape[0])):
            for j in range(min(6, sol_path.shape[2])):
                row = j // 2
                col = j % 2
                if row < 3:
                    axes[row, col].plot(sol_path[i, :, j], alpha=0.5, linewidth=0.8)
                    axes[row, col].set_title(f'Joint {j+1}')
                    axes[row, col].set_xlabel('Time step')
                    axes[row, col].set_ylabel('Position (rad)')
        
        plt.suptitle('Training Data: Sample Trajectories (5 samples)')
        plt.tight_layout()
        plt.savefig('diagnose_training_data.png', dpi=150)
        print(f"\n训练数据可视化已保存到: diagnose_training_data.png")
        
        # 判断数据质量
        if acc_std.mean() > 0.1:
            print("\n⚠️  警告: 训练数据加速度变化较大，可能存在不平滑的轨迹")
        else:
            print("\n✅ 训练数据看起来比较平滑")
            
        return sol_path


def check_model_reconstruction(model_dir, dataset_path, device='cuda:0'):
    """检查模型对训练数据的重建能力"""
    print("\n" + "="*80)
    print("2. 检查模型重建能力")
    print("="*80)
    
    # 加载模型参数
    args_path = os.path.join(model_dir, 'args.yaml')
    args = load_params_from_yaml(args_path)
    
    print(f"模型配置:")
    print(f"  - n_diffusion_steps: {args.get('n_diffusion_steps', 'N/A')}")
    print(f"  - variance_schedule: {args.get('variance_schedule', 'N/A')}")
    print(f"  - bspline_num_control_points: {args.get('bspline_num_control_points_desired', 'N/A')}")
    print(f"  - num_train_steps: {args.get('num_train_steps', 'N/A')}")
    print(f"  - batch_size: {args.get('batch_size', 'N/A')}")
    print(f"  - lr: {args.get('lr', 'N/A')}")
    
    # 检查训练loss
    # 如果有wandb日志或其他记录，可以在这里加载
    
    return args


def analyze_inference_results(results_path):
    """分析推理结果"""
    print("\n" + "="*80)
    print("3. 分析推理结果")
    print("="*80)
    
    if not os.path.exists(results_path):
        print(f"结果文件不存在: {results_path}")
        return None
        
    results = torch.load(results_path, map_location='cpu')
    
    print(f"推理结果包含的键: {list(results.keys())}")
    
    # 检查生成的轨迹
    if hasattr(results, 'q_trajs_pos_iter_0') and results.q_trajs_pos_iter_0 is not None:
        q_trajs = to_numpy(results.q_trajs_pos_iter_0)
        print(f"\n生成轨迹形状: {q_trajs.shape}")  # [batch, T, D]
        
        # 计算速度和加速度
        velocities = np.diff(q_trajs, axis=1)
        accelerations = np.diff(velocities, axis=1)
        
        vel_std_per_traj = np.std(velocities, axis=1)  # [batch, D]
        acc_std_per_traj = np.std(accelerations, axis=1)  # [batch, D]
        
        print(f"\n所有生成轨迹的速度标准差:")
        print(f"  平均: {vel_std_per_traj.mean(axis=0)}")
        print(f"  最小: {vel_std_per_traj.min(axis=0)}")
        print(f"  最大: {vel_std_per_traj.max(axis=0)}")
        
        print(f"\n所有生成轨迹的加速度标准差:")
        print(f"  平均: {acc_std_per_traj.mean(axis=0)}")
        print(f"  最小: {acc_std_per_traj.min(axis=0)}")
        print(f"  最大: {acc_std_per_traj.max(axis=0)}")
        
        # 可视化生成的轨迹
        fig, axes = plt.subplots(3, 2, figsize=(14, 10))
        
        n_samples = min(10, q_trajs.shape[0])
        for i in range(n_samples):
            for j in range(min(6, q_trajs.shape[2])):
                row = j // 2
                col = j % 2
                if row < 3:
                    axes[row, col].plot(q_trajs[i, :, j], alpha=0.5, linewidth=0.8)
                    axes[row, col].set_title(f'Joint {j+1}')
                    axes[row, col].set_xlabel('Time step')
                    axes[row, col].set_ylabel('Position (rad)')
        
        plt.suptitle(f'Generated Trajectories ({n_samples} samples)')
        plt.tight_layout()
        plt.savefig('diagnose_generated_trajectories.png', dpi=150)
        print(f"\n生成轨迹可视化已保存到: diagnose_generated_trajectories.png")
        
        # 检查最佳轨迹
        if hasattr(results, 'q_trajs_pos_best') and results.q_trajs_pos_best is not None:
            q_best = to_numpy(results.q_trajs_pos_best)
            print(f"\n最佳轨迹形状: {q_best.shape}")
            
            vel_best = np.diff(q_best, axis=0)
            acc_best = np.diff(vel_best, axis=0)
            
            print(f"最佳轨迹速度标准差: {np.std(vel_best, axis=0)}")
            print(f"最佳轨迹加速度标准差: {np.std(acc_best, axis=0)}")
            
            # 检查是否有突变
            vel_jumps = np.abs(vel_best).max(axis=0)
            acc_jumps = np.abs(acc_best).max(axis=0)
            
            print(f"\n最佳轨迹最大速度变化: {vel_jumps}")
            print(f"最佳轨迹最大加速度变化: {acc_jumps}")
            
            if acc_jumps.max() > 0.5:
                print("\n⚠️  警告: 最佳轨迹存在较大的加速度突变，这可能导致'晃悠'")
        
        return q_trajs
    
    return None


def compare_with_training_data(training_data, generated_data):
    """对比训练数据和生成数据的分布"""
    print("\n" + "="*80)
    print("4. 对比训练数据和生成数据")
    print("="*80)
    
    if training_data is None or generated_data is None:
        print("缺少数据，无法对比")
        return
    
    # 计算速度分布
    train_vel = np.diff(training_data, axis=1).flatten()
    gen_vel = np.diff(generated_data, axis=1).flatten()
    
    train_acc = np.diff(np.diff(training_data, axis=1), axis=1).flatten()
    gen_acc = np.diff(np.diff(generated_data, axis=1), axis=1).flatten()
    
    fig, axes = plt.subplots(1, 2, figsize=(12, 4))
    
    axes[0].hist(train_vel, bins=100, alpha=0.5, label='Training', density=True)
    axes[0].hist(gen_vel, bins=100, alpha=0.5, label='Generated', density=True)
    axes[0].set_title('Velocity Distribution')
    axes[0].legend()
    axes[0].set_xlabel('Velocity')
    
    axes[1].hist(train_acc, bins=100, alpha=0.5, label='Training', density=True)
    axes[1].hist(gen_acc, bins=100, alpha=0.5, label='Generated', density=True)
    axes[1].set_title('Acceleration Distribution')
    axes[1].legend()
    axes[1].set_xlabel('Acceleration')
    
    plt.tight_layout()
    plt.savefig('diagnose_distribution_comparison.png', dpi=150)
    print(f"\n分布对比已保存到: diagnose_distribution_comparison.png")
    
    # 计算分布差异
    train_vel_std = np.std(train_vel)
    gen_vel_std = np.std(gen_vel)
    train_acc_std = np.std(train_acc)
    gen_acc_std = np.std(gen_acc)
    
    print(f"\n速度标准差: 训练={train_vel_std:.4f}, 生成={gen_vel_std:.4f}, 比值={gen_vel_std/train_vel_std:.2f}")
    print(f"加速度标准差: 训练={train_acc_std:.4f}, 生成={gen_acc_std:.4f}, 比值={gen_acc_std/train_acc_std:.2f}")
    
    if gen_acc_std / train_acc_std > 2.0:
        print("\n⚠️  生成轨迹的加速度变化比训练数据大很多！")
        print("   可能原因:")
        print("   1. 模型训练不充分")
        print("   2. 推理参数导致轨迹不平滑")
    elif gen_acc_std / train_acc_std > 1.5:
        print("\n⚠️  生成轨迹比训练数据略粗糙")
        print("   建议调整推理参数")
    else:
        print("\n✅ 生成轨迹和训练数据分布相似")
        print("   如果仍有'晃悠'，可能是执行速度过快")


def print_recommendations():
    """打印建议"""
    print("\n" + "="*80)
    print("5. 诊断建议")
    print("="*80)
    
    print("""
根据诊断结果，问题可能来自以下方面：

【如果训练数据不平滑】
  → 需要重新处理训练数据，添加平滑滤波
  → 或者在数据采集时使用更平滑的轨迹

【如果模型重建能力差】
  → 增加训练步数（当前100000，可以尝试200000）
  → 调整学习率
  → 检查模型架构是否合适

【如果生成轨迹比训练数据粗糙】
  → 调整推理参数：
    - 增加 ddim_sampling_timesteps（如 50）
    - 降低 ddim_eta（如 0.0）
    - 增加 n_guide_steps
    - 增加速度/加速度惩罚权重

【如果轨迹本身平滑但执行时晃悠】
  → 增加 trajectory_duration（如 20-30秒）
  → 检查 Isaac Gym 的控制器参数
  → 添加轨迹后处理（低通滤波）
""")


if __name__ == "__main__":
    # 配置路径
    dataset_path = "/home/hong/Projects/MotionPlanningDiffusion/mpd-splines-public/data_trajectories_hcj/piper_100000/dataset_merged_processed.hdf5"
    model_dir = "/home/hong/Projects/MotionPlanningDiffusion/mpd-splines-public/scripts/train/logs_piper/1"
    results_path = "/home/hong/Projects/MotionPlanningDiffusion/mpd-splines-public/scripts/inference/logs_inference_piper/1/results_single_plan-000.pt"
    
    print("="*80)
    print("Piper 机械臂扩散模型诊断工具")
    print("="*80)
    
    # 1. 检查训练数据
    training_data = check_training_data_smoothness(dataset_path)
    
    # 2. 检查模型配置
    model_args = check_model_reconstruction(model_dir, dataset_path)
    
    # 3. 分析推理结果
    generated_data = analyze_inference_results(results_path)
    
    # 4. 对比分布
    if training_data is not None and generated_data is not None:
        compare_with_training_data(training_data, generated_data)
    
    # 5. 打印建议
    print_recommendations()
    
    print("\n" + "="*80)
    print("诊断完成！请查看生成的图片文件进行进一步分析。")
    print("="*80)
