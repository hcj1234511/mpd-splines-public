#!/usr/bin/env python3
"""
测试 Panda 机械臂的 FK 和 IK
使用项目中的 robot_tree.py 实现

使用方法：
    cd /home/hong/Projects/MotionPlanningDiffusion/mpd-splines-public
    source set_env_variables.sh
    conda activate mpd-splines-public
    python scripts/Hcj/test_panda_ik.py
"""

import sys
import torch

# 添加项目路径
sys.path.insert(0, '/home/hong/Projects/MotionPlanningDiffusion/mpd-splines-public')

from torch_robotics.robots import RobotPanda
from torch_robotics.torch_kinematics_tree.geometrics.utils import (
    link_pos_from_link_tensor,
    link_rot_from_link_tensor,
)
from torch_robotics.torch_utils.torch_utils import to_numpy


def main():
    print("=" * 70)
    print("Panda 机械臂 FK/IK 测试")
    print("=" * 70)
    
    # 设置设备
    device = "cuda" if torch.cuda.is_available() else "cpu"
    tensor_args = dict(device=device, dtype=torch.float32)
    print(f"使用设备: {device}")
    
    # 加载机器人
    print("\n1. 加载 RobotPanda...")
    robot = RobotPanda(tensor_args=tensor_args)
    diff_robot = robot.diff_panda  # 可微分运动学模型
    
    print(f"   ✅ 加载成功！")
    print(f"   - 总关节数量 (q_dim): {robot.q_dim}")
    print(f"   - 可微分模型关节数 (_n_dofs): {diff_robot._n_dofs}")
    print(f"   - 末端执行器 link: {robot.link_name_ee}")
    print(f"   - 模型路径: {diff_robot.model_path}")
    
    # 打印关节限制
    print("\n2. 关节限制:")
    print(f"   - 下限: {to_numpy(robot.q_pos_min)}")
    print(f"   - 上限: {to_numpy(robot.q_pos_max)}")
    
    # ============================================
    # 3. 测试 FK - 关节全为0
    # ============================================
    print("\n" + "=" * 70)
    print("3. 测试正运动学 (FK) - 关节全为0")
    print("=" * 70)
    
    # 设置关节角度为0
    q_zero = torch.zeros(1, diff_robot._n_dofs, **tensor_args)
    print(f"   输入关节角度: {to_numpy(q_zero[0])}")
    
    # 计算 FK
    ee_link = robot.link_name_ee  # "panda_hand"
    H_fk = diff_robot.compute_forward_kinematics_all_links(q_zero, link_list=[ee_link])
    
    # 提取位置和旋转
    pos_fk = link_pos_from_link_tensor(H_fk).squeeze()
    rot_fk = link_rot_from_link_tensor(H_fk).squeeze()
    
    print(f"\n   末端位置: {to_numpy(pos_fk)}")
    print(f"   末端旋转矩阵:\n{to_numpy(rot_fk)}")

    # ============================================
    # 4. 测试 IK - 用 FK 结果作为目标
    # ============================================
    print("\n" + "=" * 70)
    print("4. 测试逆运动学 (IK) - 用 FK 结果作为目标")
    print("=" * 70)
    
    # 目标位姿就是 FK 计算的结果
    H_target = H_fk.clone().squeeze(1)  # [batch, 4, 4]
    print(f"   目标位置: {to_numpy(pos_fk)}")
    
    # 从零位附近开始求解 IK
    q_init = torch.zeros(1, diff_robot._n_dofs, **tensor_args)
    print(f"   初始关节角度: {to_numpy(q_init[0])}")
    
    # 求解 IK
    print("\n   正在求解 IK...")
    q_ik, idx_valid = diff_robot.inverse_kinematics(
        H_target,
        link_name=ee_link,
        batch_size=1,
        q0=q_init,
        q0_noise=0.01,
        max_iters=1000,
        lr=0.05,
        se3_eps=0.1,
        print_freq=200,
    )
    
    print(f"\n   IK 求解结果: {to_numpy(q_ik[0])}")
    print(f"   有效解索引: {idx_valid}")
    print(f"   求解成功: {idx_valid.nelement() > 0}")
    
    # ============================================
    # 5. 验证 IK 结果
    # ============================================
    print("\n" + "=" * 70)
    print("5. 验证 IK 结果")
    print("=" * 70)
    
    # 用 IK 结果计算 FK
    H_verify = diff_robot.compute_forward_kinematics_all_links(q_ik, link_list=[ee_link])
    pos_verify = link_pos_from_link_tensor(H_verify).squeeze()
    rot_verify = link_rot_from_link_tensor(H_verify).squeeze()
    
    print(f"   FK(IK结果) 位置: {to_numpy(pos_verify)}")
    print(f"   目标位置:        {to_numpy(pos_fk)}")
    
    # 计算误差
    pos_error = torch.linalg.norm(pos_verify - pos_fk).item()
    rot_error = torch.linalg.norm(rot_verify - rot_fk).item()
    
    print(f"\n   位置误差: {pos_error:.6e}")
    print(f"   旋转误差: {rot_error:.6e}")
    
    # 关节空间误差
    joint_error = torch.linalg.norm(q_ik[0] - q_zero[0]).item()
    print(f"   关节误差 (与零位): {joint_error:.6f} rad")
    
    # ============================================
    # 6. 测试另一个配置
    # ============================================
    print("\n" + "=" * 70)
    print("6. 测试另一个关节配置")
    print("=" * 70)
    
    # 设置一个非零关节配置 (Panda 有 7 个关节)
    q_test = torch.zeros(1, diff_robot._n_dofs, **tensor_args)
    q_test[0, :7] = torch.tensor([0.5, -0.3, 0.4, -1.5, 0.3, 1.2, 0.1], **tensor_args)
    print(f"   测试关节角度: {to_numpy(q_test[0, :7])}")
    
    # FK
    H_test = diff_robot.compute_forward_kinematics_all_links(q_test, link_list=[ee_link])
    pos_test = link_pos_from_link_tensor(H_test).squeeze()
    print(f"   FK 末端位置: {to_numpy(pos_test)}")
    
    # IK (从零位开始)
    q_init2 = torch.zeros(1, diff_robot._n_dofs, **tensor_args)
    print(f"\n   从零位开始求解 IK...")
    
    q_ik2, idx_valid2 = diff_robot.inverse_kinematics(
        H_test.squeeze(1),
        link_name=ee_link,
        batch_size=1,
        q0=q_init2,
        q0_noise=0.01,
        max_iters=1000,
        lr=0.05,
        se3_eps=0.1,
        print_freq=200,
    )
    
    print(f"\n   IK 求解结果: {to_numpy(q_ik2[0, :7])}")
    print(f"   原始关节角度: {to_numpy(q_test[0, :7])}")
    print(f"   求解成功: {idx_valid2.nelement() > 0}")
    
    # 验证
    H_verify2 = diff_robot.compute_forward_kinematics_all_links(q_ik2, link_list=[ee_link])
    pos_verify2 = link_pos_from_link_tensor(H_verify2).squeeze()
    pos_error2 = torch.linalg.norm(pos_verify2 - pos_test).item()
    joint_error2 = torch.linalg.norm(q_ik2[0, :7] - q_test[0, :7]).item()
    
    print(f"\n   位置误差: {pos_error2:.6e}")
    print(f"   关节误差: {joint_error2:.6f} rad")
    
    print("\n" + "=" * 70)
    print("测试完成！")
    print("=" * 70)


if __name__ == "__main__":
    main()
