#!/usr/bin/env python3
"""
可视化IK求解过程 - 展示优化迭代过程
"""

import sys
import os
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, PROJECT_ROOT)
sys.path.insert(0, os.path.join(PROJECT_ROOT, "mpd", "torch_robotics"))

import torch
import numpy as np
import matplotlib.pyplot as plt
from torch_robotics.torch_kinematics_tree.geometrics.utils import link_pos_from_link_tensor, link_rot_from_link_tensor, SE3_distance
from torch_robotics.torch_kinematics_tree.models.robot_tree import DifferentiableTree
from torch_robotics.torch_kinematics_tree.utils.files import get_robot_path
from torch_robotics.torch_utils.torch_utils import to_numpy


class DifferentiablePiper(DifferentiableTree):
    def __init__(self, link_list=None, device="cpu"):
        robot_file = get_robot_path() / "piper_description" / "urdf" / "piper_description_fixed.urdf"
        self.model_path = robot_file.as_posix()
        self.name = "differentiable_piper"
        super().__init__(self.model_path, self.name, link_list=link_list, device=device)


def manual_ik_with_logging(robot, H_target, q0, ee_link, max_iters=200, lr=1e-1, device="cpu"):
    """
    手动实现IK求解，记录每次迭代的状态
    """
    tensor_args = dict(device=device, dtype=torch.float32)
    
    # 获取关节限制
    lower, upper, _, _ = robot.get_joint_limit_array()
    lower = torch.tensor(lower, **tensor_args)
    upper = torch.tensor(upper, **tensor_args)
    
    # 初始化
    q = q0.clone().requires_grad_(True)
    optimizer = torch.optim.Adam([q], lr=lr)
    
    # 记录历史
    history = {
        'iteration': [],
        'loss_total': [],
        'loss_se3': [],
        'loss_joint_limits': [],
        'position_error': [],
        'rotation_error': [],
        'q_values': [],
    }
    
    print("="*70)
    print("IK优化过程可视化")
    print("="*70)
    print(f"初始关节角度: {to_numpy(q0[0, :6])}")
    
    for i in range(max_iters):
        optimizer.zero_grad()
        
        # 计算FK
        H_current = robot.compute_forward_kinematics_all_links(q, link_list=[ee_link])
        
        # SE3误差
        err_se3 = SE3_distance(H_current.squeeze(1), H_target, w_pos=1.0, w_rot=1.0)
        
        # 关节限制误差
        lower_mask = torch.where(q < lower, 1, 0)
        err_joint_limit_lower = ((lower - q).pow(2) * lower_mask).sum(-1)
        upper_mask = torch.where(q > upper, 1, 0)
        err_joint_limit_upper = ((upper - q).pow(2) * upper_mask).sum(-1)
        err_joint_limits = err_joint_limit_lower + err_joint_limit_upper
        
        # 总损失
        w_se3 = 1.0
        w_joint_limits = 300.0
        loss = w_se3 * err_se3 + w_joint_limits * err_joint_limits
        
        # 记录
        history['iteration'].append(i)
        history['loss_total'].append(loss.item())
        history['loss_se3'].append(err_se3.item())
        history['loss_joint_limits'].append(err_joint_limits.item())
        
        # 计算位置和旋转误差
        pos_current = link_pos_from_link_tensor(H_current).squeeze()
        pos_target = link_pos_from_link_tensor(H_target).squeeze()
        pos_error = torch.linalg.norm(pos_current - pos_target).item()
        
        history['position_error'].append(pos_error)
        history['rotation_error'].append(err_se3.item() - pos_error)  # 近似
        history['q_values'].append(to_numpy(q[0, :6]).copy())
        
        # 打印进度
        if i % 20 == 0 or i < 5:
            print(f"\n迭代 {i:3d}: loss={loss.item():.4f}, "
                  f"se3_err={err_se3.item():.4f}, "
                  f"pos_err={pos_error:.6f}m")
        
        # 检查收敛
        if err_se3.item() < 0.01 and err_joint_limits.item() < 0.01:
            print(f"\n✓ 收敛！迭代次数: {i}")
            break
        
        # 优化步骤
        loss.backward()
        optimizer.step()
    
    print(f"\n最终关节角度: {to_numpy(q[0, :6])}")
    print(f"最终位置误差: {pos_error:.6f} m")
    
    return q, history


def plot_ik_history(history, save_path="scripts/Hcj/ik_optimization_process.png"):
    """绘制IK优化过程"""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # 1. 总损失
    ax = axes[0, 0]
    ax.plot(history['iteration'], history['loss_total'], 'b-', linewidth=2)
    ax.set_xlabel('迭代次数')
    ax.set_ylabel('总损失')
    ax.set_title('总损失函数变化')
    ax.grid(True, alpha=0.3)
    ax.set_yscale('log')
    
    # 2. SE3误差和关节限制误差
    ax = axes[0, 1]
    ax.plot(history['iteration'], history['loss_se3'], 'r-', label='SE3误差', linewidth=2)
    ax.plot(history['iteration'], history['loss_joint_limits'], 'g-', label='关节限制误差', linewidth=2)
    ax.set_xlabel('迭代次数')
    ax.set_ylabel('误差')
    ax.set_title('损失函数分解')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_yscale('log')
    
    # 3. 位置和旋转误差
    ax = axes[1, 0]
    ax.plot(history['iteration'], history['position_error'], 'b-', label='位置误差 (m)', linewidth=2)
    ax.axhline(y=0.01, color='r', linestyle='--', label='收敛阈值', alpha=0.5)
    ax.set_xlabel('迭代次数')
    ax.set_ylabel('位置误差 (m)')
    ax.set_title('末端执行器位置误差')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # 4. 关节角度变化
    ax = axes[1, 1]
    q_values = np.array(history['q_values'])
    for i in range(6):
        ax.plot(history['iteration'], q_values[:, i], label=f'关节{i+1}', linewidth=2)
    ax.set_xlabel('迭代次数')
    ax.set_ylabel('关节角度 (rad)')
    ax.set_title('关节角度变化')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    print(f"\n优化过程图已保存至: {save_path}")
    
    return fig


def main():
    device = "cuda" if torch.cuda.is_available() else "cpu"
    tensor_args = dict(device=device, dtype=torch.float32)
    
    robot = DifferentiablePiper(device=device)
    ee_link = "gripper_base"
    
    # 目标：已知解 [0, 2, -2, 0, 0, 0]
    q_target = torch.zeros(1, robot._n_dofs, **tensor_args)
    target_joints = [0, 2, -2, 0, 0, 0, 0, 0]
    for i, angle in enumerate(target_joints):
        q_target[0, i] = angle
    
    # 计算目标位姿
    H_target = robot.compute_forward_kinematics_all_links(q_target, link_list=[ee_link])
    
    print("\n目标配置:")
    print(f"  关节角度: {target_joints[:6]}")
    print(f"  末端位置: {to_numpy(link_pos_from_link_tensor(H_target).squeeze())}")
    
    # 测试1: 从随机初始值开始
    print("\n" + "="*70)
    print("测试1: 从随机初始值开始（容易失败）")
    print("="*70)
    
    lower, upper, _, _ = robot.get_joint_limit_array()
    lower_t = torch.tensor(lower, **tensor_args)
    upper_t = torch.tensor(upper, **tensor_args)
    q0_random = lower_t + torch.rand(1, robot._n_dofs, **tensor_args) * (upper_t - lower_t)
    
    q_result1, history1 = manual_ik_with_logging(
        robot, H_target, q0_random, ee_link, 
        max_iters=200, lr=1e-1, device=device
    )
    
    # 测试2: 从接近目标的初始值开始
    print("\n" + "="*70)
    print("测试2: 从接近目标的初始值开始（容易成功）")
    print("="*70)
    
    q0_near = q_target.clone()
    q0_near[0, :6] += torch.randn(6, **tensor_args) * 0.1
    
    q_result2, history2 = manual_ik_with_logging(
        robot, H_target, q0_near, ee_link,
        max_iters=200, lr=1e-1, device=device
    )
    
    # 绘制对比图
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    
    ax = axes[0]
    ax.plot(history1['iteration'], history1['position_error'], 'r-', linewidth=2, label='随机初始化')
    ax.axhline(y=0.01, color='g', linestyle='--', label='收敛阈值', alpha=0.5)
    ax.set_xlabel('迭代次数')
    ax.set_ylabel('位置误差 (m)')
    ax.set_title('随机初始化 - 难以收敛')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_yscale('log')
    
    ax = axes[1]
    ax.plot(history2['iteration'], history2['position_error'], 'b-', linewidth=2, label='智能初始化')
    ax.axhline(y=0.01, color='g', linestyle='--', label='收敛阈值', alpha=0.5)
    ax.set_xlabel('迭代次数')
    ax.set_ylabel('位置误差 (m)')
    ax.set_title('智能初始化 - 快速收敛')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_yscale('log')
    
    plt.tight_layout()
    plt.savefig("scripts/Hcj/ik_comparison.png", dpi=150, bbox_inches='tight')
    print(f"\n对比图已保存至: scripts/Hcj/ik_comparison.png")
    
    # 绘制详细的优化过程（测试2）
    plot_ik_history(history2)
    
    print("\n" + "="*70)
    print("总结")
    print("="*70)
    print("IK求解方法: 基于优化的数值方法（Adam + 梯度下降）")
    print("损失函数: SE3位姿误差 + 关节限制惩罚")
    print("优化变量: 所有8个DOF（包括夹爪关节）")
    print("\n关键发现:")
    print("  1. 随机初始化容易陷入局部最优")
    print("  2. 智能初始化（从已知解附近开始）可以快速收敛")
    print("  3. 夹爪关节参与优化，但对gripper_base位姿影响很小")


if __name__ == "__main__":
    main()
