#!/usr/bin/env python3
"""
测试 Piper 机械臂 IK 精度
随机生成100个关节配置，计算FK，然后反求IK，统计关节空间误差

使用方法：
    cd /home/hong/Projects/MotionPlanningDiffusion/mpd-splines-public
    source set_env_variables.sh
    conda activate mpd-splines-public
    python scripts/Hcj/test_piper_ik_accuracy.py
"""

import sys
import os

# 添加项目路径
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, PROJECT_ROOT)
sys.path.insert(0, os.path.join(PROJECT_ROOT, "mpd", "torch_robotics"))

import torch
import numpy as np
import matplotlib.pyplot as plt
from torch_robotics.torch_kinematics_tree.geometrics.utils import link_pos_from_link_tensor, link_rot_from_link_tensor
from torch_robotics.torch_kinematics_tree.models.robot_tree import DifferentiableTree
from torch_robotics.torch_kinematics_tree.utils.files import get_robot_path
from torch_robotics.torch_utils.seed import fix_random_seed
from torch_robotics.torch_utils.torch_timer import TimerCUDA
from torch_robotics.torch_utils.torch_utils import to_numpy


class DifferentiablePiper(DifferentiableTree):
    """Piper 机械臂的可微分运动学模型"""
    def __init__(self, link_list=None, device="cpu"):
        robot_file = get_robot_path() / "piper_description" / "urdf" / "piper_description_fixed.urdf"
        self.model_path = robot_file.as_posix()
        self.name = "differentiable_piper"
        super().__init__(self.model_path, self.name, link_list=link_list, device=device)


def rotation_matrix_to_quaternion(R):
    """旋转矩阵转四元数 (x, y, z, w 格式)"""
    trace = np.trace(R)
    
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (R[2, 1] - R[1, 2]) * s
        qy = (R[0, 2] - R[2, 0]) * s
        qz = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    
    # 归一化
    norm = np.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
    return qx/norm, qy/norm, qz/norm, qw/norm


def test_ik_accuracy(robot, device, tensor_args, num_tests=100, ee_link="gripper_base"):
    """
    测试IK精度
    
    流程：
    1. 随机生成关节配置 q_original
    2. 计算FK得到 H_target
    3. 使用IK求解得到 q_ik
    4. 统计关节空间误差 ||q_ik - q_original||
    """
    print("="*70)
    print(f"Piper 机械臂 IK 精度测试 ({num_tests} 次)")
    print("="*70)
    
    # 获取关节限制
    lower, upper, _, _ = robot.get_joint_limit_array()
    lower_t = torch.tensor(lower, **tensor_args)
    upper_t = torch.tensor(upper, **tensor_args)
    
    print(f"\n关节限制:")
    for i in range(6):
        print(f"  关节 {i+1}: [{lower[i]:.3f}, {upper[i]:.3f}] rad "
              f"([{np.rad2deg(lower[i]):.1f}°, {np.rad2deg(upper[i]):.1f}°])")
    
    # 统计数据
    results = {
        'success': [],
        'joint_errors': [],  # 关节空间误差 (L2范数)
        'joint_errors_per_joint': [],  # 每个关节的误差
        'position_errors': [],  # 位置误差
        'rotation_errors': [],  # 旋转误差
        'q_original': [],
        'q_ik': [],
        'iteration_times': [],
    }
    
    print(f"\n开始测试...")
    print("="*70)
    
    for test_idx in range(num_tests):
        # 1. 随机生成关节配置（在限制范围内）
        q_original = lower_t + torch.rand(1, robot._n_dofs, **tensor_args) * (upper_t - lower_t)
        
        # 2. 计算FK
        H_target = robot.compute_forward_kinematics_all_links(q_original, link_list=[ee_link])
        pos_target = link_pos_from_link_tensor(H_target).squeeze()
        rot_target = link_rot_from_link_tensor(H_target).squeeze()
        
        # 3. 使用智能初始化求解IK（从原始解附近开始）
        # 添加噪声以测试IK的鲁棒性
        q0 = q_original.clone()
        q0[0, :6] += torch.randn(6, **tensor_args) * 0.1
        q0 = torch.clamp(q0, lower_t, upper_t)
        
        with TimerCUDA() as t:
            q_ik, idx_valid = robot.inverse_kinematics(
                H_target,
                link_name=ee_link,
                q0=q0,
                q0_noise=0.0,
                batch_size=1,
                max_iters=500,
                lr=1e-1,
                se3_eps=1e-2,
                eps_joint_lim=torch.pi / 64,
                print_freq=-1,
                debug=False,
            )
        
        # 4. 统计结果
        if idx_valid.nelement() > 0:
            # 成功
            q_solution = q_ik[idx_valid[0]]
            
            # 关节空间误差
            joint_error_per_joint = torch.abs(q_solution[:6] - q_original[0, :6])
            joint_error = torch.linalg.norm(joint_error_per_joint).item()
            
            # 验证FK
            H_check = robot.compute_forward_kinematics_all_links(q_solution.unsqueeze(0), link_list=[ee_link])
            pos_check = link_pos_from_link_tensor(H_check).squeeze()
            rot_check = link_rot_from_link_tensor(H_check).squeeze()
            
            position_error = torch.linalg.norm(pos_check - pos_target).item()
            
            # 旋转误差（四元数距离）
            rot_target_np = to_numpy(rot_target)
            rot_check_np = to_numpy(rot_check)
            quat_target = rotation_matrix_to_quaternion(rot_target_np)
            quat_check = rotation_matrix_to_quaternion(rot_check_np)
            rotation_error = np.linalg.norm(np.array(quat_check) - np.array(quat_target))
            
            results['success'].append(True)
            results['joint_errors'].append(joint_error)
            results['joint_errors_per_joint'].append(to_numpy(joint_error_per_joint))
            results['position_errors'].append(position_error)
            results['rotation_errors'].append(rotation_error)
            results['q_original'].append(to_numpy(q_original[0, :6]))
            results['q_ik'].append(to_numpy(q_solution[:6]))
            results['iteration_times'].append(t.elapsed)
            
            if (test_idx + 1) % 10 == 0:
                print(f"测试 {test_idx+1:3d}/{num_tests}: ✓ 成功 | "
                      f"关节误差={joint_error:.4f} rad | "
                      f"位置误差={position_error:.6f} m | "
                      f"时间={t.elapsed:.3f}s")
        else:
            # 失败
            results['success'].append(False)
            results['joint_errors'].append(np.nan)
            results['joint_errors_per_joint'].append(np.full(6, np.nan))
            results['position_errors'].append(np.nan)
            results['rotation_errors'].append(np.nan)
            results['q_original'].append(to_numpy(q_original[0, :6]))
            results['q_ik'].append(np.full(6, np.nan))
            results['iteration_times'].append(t.elapsed)
            
            if (test_idx + 1) % 10 == 0:
                print(f"测试 {test_idx+1:3d}/{num_tests}: ✗ 失败 | 时间={t.elapsed:.3f}s")
    
    return results


def analyze_and_plot_results(results, num_tests):
    """分析并可视化结果"""
    print("\n" + "="*70)
    print("统计结果")
    print("="*70)
    
    # 成功率
    success_count = sum(results['success'])
    success_rate = success_count / num_tests * 100
    
    print(f"\n成功率: {success_count}/{num_tests} ({success_rate:.1f}%)")
    
    if success_count == 0:
        print("\n没有成功的测试，无法进行统计分析")
        return
    
    # 关节空间误差统计
    joint_errors = np.array([e for e, s in zip(results['joint_errors'], results['success']) if s])
    position_errors = np.array([e for e, s in zip(results['position_errors'], results['success']) if s])
    rotation_errors = np.array([e for e, s in zip(results['rotation_errors'], results['success']) if s])
    iteration_times = np.array([t for t, s in zip(results['iteration_times'], results['success']) if s])
    
    print(f"\n关节空间误差 (L2范数):")
    print(f"  平均值: {joint_errors.mean():.6f} rad ({np.rad2deg(joint_errors.mean()):.3f}°)")
    print(f"  中位数: {np.median(joint_errors):.6f} rad ({np.rad2deg(np.median(joint_errors)):.3f}°)")
    print(f"  标准差: {joint_errors.std():.6f} rad ({np.rad2deg(joint_errors.std()):.3f}°)")
    print(f"  最小值: {joint_errors.min():.6f} rad ({np.rad2deg(joint_errors.min()):.3f}°)")
    print(f"  最大值: {joint_errors.max():.6f} rad ({np.rad2deg(joint_errors.max()):.3f}°)")
    
    # 每个关节的误差统计
    joint_errors_per_joint = np.array([e for e, s in zip(results['joint_errors_per_joint'], results['success']) if s])
    print(f"\n每个关节的平均误差:")
    for i in range(6):
        joint_i_errors = joint_errors_per_joint[:, i]
        print(f"  关节 {i+1}: {joint_i_errors.mean():.6f} rad ({np.rad2deg(joint_i_errors.mean()):.3f}°)")
    
    print(f"\n位置误差:")
    print(f"  平均值: {position_errors.mean():.6f} m ({position_errors.mean()*1000:.3f} mm)")
    print(f"  中位数: {np.median(position_errors):.6f} m ({np.median(position_errors)*1000:.3f} mm)")
    print(f"  最大值: {position_errors.max():.6f} m ({position_errors.max()*1000:.3f} mm)")
    
    print(f"\n旋转误差 (四元数距离):")
    print(f"  平均值: {rotation_errors.mean():.6f}")
    print(f"  中位数: {np.median(rotation_errors):.6f}")
    print(f"  最大值: {rotation_errors.max():.6f}")
    
    print(f"\n求解时间:")
    print(f"  平均值: {iteration_times.mean():.3f} s")
    print(f"  中位数: {np.median(iteration_times):.3f} s")
    print(f"  最大值: {iteration_times.max():.3f} s")
    
    # 绘图
    fig, axes = plt.subplots(2, 3, figsize=(16, 10))
    
    # 1. 关节空间误差分布
    ax = axes[0, 0]
    ax.hist(joint_errors, bins=30, edgecolor='black', alpha=0.7)
    ax.axvline(joint_errors.mean(), color='r', linestyle='--', linewidth=2, label=f'平均值: {joint_errors.mean():.4f}')
    ax.axvline(np.median(joint_errors), color='g', linestyle='--', linewidth=2, label=f'中位数: {np.median(joint_errors):.4f}')
    ax.set_xlabel('关节空间误差 (rad)')
    ax.set_ylabel('频数')
    ax.set_title('关节空间误差分布 (L2范数)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # 2. 每个关节的误差箱线图
    ax = axes[0, 1]
    ax.boxplot([joint_errors_per_joint[:, i] for i in range(6)], 
                labels=[f'J{i+1}' for i in range(6)])
    ax.set_ylabel('关节误差 (rad)')
    ax.set_title('每个关节的误差分布')
    ax.grid(True, alpha=0.3)
    
    # 3. 位置误差分布
    ax = axes[0, 2]
    ax.hist(position_errors * 1000, bins=30, edgecolor='black', alpha=0.7, color='orange')
    ax.axvline(position_errors.mean() * 1000, color='r', linestyle='--', linewidth=2, 
               label=f'平均值: {position_errors.mean()*1000:.3f}')
    ax.set_xlabel('位置误差 (mm)')
    ax.set_ylabel('频数')
    ax.set_title('末端执行器位置误差分布')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # 4. 关节误差 vs 位置误差
    ax = axes[1, 0]
    ax.scatter(joint_errors, position_errors * 1000, alpha=0.6)
    ax.set_xlabel('关节空间误差 (rad)')
    ax.set_ylabel('位置误差 (mm)')
    ax.set_title('关节误差 vs 位置误差')
    ax.grid(True, alpha=0.3)
    
    # 5. 旋转误差分布
    ax = axes[1, 1]
    ax.hist(rotation_errors, bins=30, edgecolor='black', alpha=0.7, color='green')
    ax.axvline(rotation_errors.mean(), color='r', linestyle='--', linewidth=2,
               label=f'平均值: {rotation_errors.mean():.4f}')
    ax.set_xlabel('旋转误差 (四元数距离)')
    ax.set_ylabel('频数')
    ax.set_title('末端执行器旋转误差分布')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # 6. 求解时间分布
    ax = axes[1, 2]
    ax.hist(iteration_times, bins=30, edgecolor='black', alpha=0.7, color='purple')
    ax.axvline(iteration_times.mean(), color='r', linestyle='--', linewidth=2,
               label=f'平均值: {iteration_times.mean():.3f}s')
    ax.set_xlabel('求解时间 (s)')
    ax.set_ylabel('频数')
    ax.set_title('IK求解时间分布')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    save_path = "scripts/Hcj/piper_ik_accuracy_results.png"
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    print(f"\n结果图已保存至: {save_path}")
    
    return fig


def main():
    print("="*70)
    print("Piper 机械臂 IK 精度测试")
    print("="*70)
    
    # 配置
    seed = 42
    fix_random_seed(seed)
    
    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"\n使用设备: {device}")
    tensor_args = dict(device=device, dtype=torch.float32)
    
    # 加载 Piper 机械臂模型
    print("\n加载 Piper 机械臂模型...")
    robot = DifferentiablePiper(device=device)
    print(f"模型路径: {robot.model_path}")
    
    # 测试IK精度
    num_tests = 100
    results = test_ik_accuracy(robot, device, tensor_args, num_tests=num_tests)
    
    # 分析并可视化结果
    analyze_and_plot_results(results, num_tests)
    
    print("\n" + "="*70)
    print("测试完成!")
    print("="*70)


if __name__ == "__main__":
    main()
