#!/usr/bin/env python3
"""
简化的IK测试 - 诊断为什么IK求解失败
"""

import sys
import os
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, PROJECT_ROOT)
sys.path.insert(0, os.path.join(PROJECT_ROOT, "mpd", "torch_robotics"))

import torch
import numpy as np
from torch_robotics.torch_kinematics_tree.geometrics.utils import link_pos_from_link_tensor, link_rot_from_link_tensor
from torch_robotics.torch_kinematics_tree.models.robot_tree import DifferentiableTree
from torch_robotics.torch_kinematics_tree.utils.files import get_robot_path
from torch_robotics.torch_utils.torch_utils import to_numpy


class DifferentiablePiper(DifferentiableTree):
    def __init__(self, link_list=None, device="cpu"):
        robot_file = get_robot_path() / "piper_description" / "urdf" / "piper_description_fixed.urdf"
        self.model_path = robot_file.as_posix()
        self.name = "differentiable_piper"
        super().__init__(self.model_path, self.name, link_list=link_list, device=device)


def main():
    device = "cuda" if torch.cuda.is_available() else "cpu"
    tensor_args = dict(device=device, dtype=torch.float32)
    
    robot = DifferentiablePiper(device=device)
    
    print("="*70)
    print("简化IK诊断测试")
    print("="*70)
    
    # 已知的正确解
    q_correct = torch.zeros(1, robot._n_dofs, **tensor_args)
    correct_joints = [0, 2, -2, 0, 0, 0, 0, 0]
    for i, angle in enumerate(correct_joints):
        q_correct[0, i] = angle
    
    # 计算FK
    ee_link = "gripper_base"
    H_target = robot.compute_forward_kinematics_all_links(q_correct, link_list=[ee_link])
    
    print(f"\n已知正确解: {to_numpy(q_correct[0, :6])}")
    print(f"对应的末端位置: {to_numpy(link_pos_from_link_tensor(H_target).squeeze())}")
    
    # 测试：从接近正确解的初始值开始
    print(f"\n测试1: 从接近正确解的初始值开始")
    print("="*60)
    
    for noise_level in [0.01, 0.05, 0.1, 0.2]:
        q_init = q_correct.clone()
        # 只对前6个关节添加噪声
        q_init[0, :6] += torch.randn(6, **tensor_args) * noise_level
        
        print(f"\n噪声水平: {noise_level}")
        print(f"初始值: {to_numpy(q_init[0, :6])}")
        
        q_ik, idx_valid = robot.inverse_kinematics(
            H_target,
            link_name=ee_link,
            q0=q_init,
            q0_noise=0.0,  # 不添加额外噪声
            batch_size=1,
            max_iters=500,
            lr=1e-1,
            se3_eps=1e-2,
            eps_joint_lim=torch.pi / 64,
            print_freq=-1,
            debug=False,
        )
        
        if idx_valid.nelement() > 0:
            print(f"✓ 成功! 解: {to_numpy(q_ik[idx_valid[0], :6])}")
        else:
            print(f"✗ 失败")
    
    # 测试：检查关节限制
    print(f"\n\n测试2: 检查关节限制")
    print("="*60)
    lower, upper, _, _ = robot.get_joint_limit_array()
    
    for i in range(6):
        angle = correct_joints[i]
        in_limit = lower[i] <= angle <= upper[i]
        margin_lower = angle - lower[i]
        margin_upper = upper[i] - angle
        
        print(f"关节 {i}: {angle:.3f} rad")
        print(f"  限制: [{lower[i]:.3f}, {upper[i]:.3f}]")
        print(f"  下限余量: {margin_lower:.3f}, 上限余量: {margin_upper:.3f}")
        print(f"  状态: {'✓ 在限制内' if in_limit else '✗ 超出限制'}")
        
        if margin_lower < 0.1 or margin_upper < 0.1:
            print(f"  ⚠ 警告: 接近关节限制!")
    
    print(f"\n\n结论:")
    print("="*60)
    print("IK求解失败的可能原因:")
    print("1. 优化器的初始化策略不佳")
    print("2. 关节限制的软约束处理可能有问题")
    print("3. 该IK实现可能对某些配置不够鲁棒")
    print("\n建议:")
    print("- 对于已知的FK解，可以直接使用，不需要通过IK求解")
    print("- 如果必须使用IK，考虑使用其他IK库（如TRAC-IK）")


if __name__ == "__main__":
    main()
