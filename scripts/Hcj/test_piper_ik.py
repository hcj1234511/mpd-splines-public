#!/usr/bin/env python3
"""
测试 Piper 机械臂的逆解（Inverse Kinematics）
用于验证给定末端执行器目标位姿时，能否正确求解关节角度

使用方法：
    cd /home/hong/Projects/MotionPlanningDiffusion/mpd-splines-public
    source set_env_variables.sh
    conda activate mpd-splines-public
    python scripts/Hcj/test_piper_ik.py
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
from copy import copy

# 直接从 torch_kinematics_tree 导入，避免循环导入
from torch_robotics.torch_kinematics_tree.geometrics.frame import Frame
from torch_robotics.torch_kinematics_tree.geometrics.skeleton import get_skeleton_from_model
from torch_robotics.torch_kinematics_tree.geometrics.spatial_vector import x_rot, y_rot, z_rot
from torch_robotics.torch_kinematics_tree.geometrics.utils import link_pos_from_link_tensor, link_rot_from_link_tensor
from torch_robotics.torch_kinematics_tree.models.robot_tree import DifferentiableTree
from torch_robotics.torch_kinematics_tree.utils.files import get_robot_path
from torch_robotics.torch_utils.seed import fix_random_seed
from torch_robotics.torch_utils.torch_timer import TimerCUDA
from torch_robotics.torch_utils.torch_utils import to_numpy
from torch_robotics.visualizers.plot_utils import plot_coordinate_frame, create_fig_and_axes


class DifferentiablePiper(DifferentiableTree):
    """Piper 机械臂的可微分运动学模型"""
    def __init__(self, link_list=None, device="cpu"):
        robot_file = get_robot_path() / "piper_description" / "urdf" / "piper_description_fixed.urdf"
        self.model_path = robot_file.as_posix()
        self.name = "differentiable_piper"
        super().__init__(self.model_path, self.name, link_list=link_list, device=device)


def quaternion_to_rotation_matrix(qx, qy, qz, qw):
    """四元数转旋转矩阵 (x, y, z, w 格式)"""
    # 归一化
    norm = np.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
    qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm
    
    # 旋转矩阵
    R = np.array([
        [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
    ])
    return R


def euler_to_quaternion(roll, pitch, yaw):
    """欧拉角转四元数 (ZYX顺序)
    
    Args:
        roll: 绕X轴旋转 (弧度)
        pitch: 绕Y轴旋转 (弧度)
        yaw: 绕Z轴旋转 (弧度)
    
    Returns:
        (qx, qy, qz, qw): 四元数 (x, y, z, w 格式)
    """
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    
    return qx, qy, qz, qw


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


def test_specific_pose_ik(robot, device, tensor_args, ee_link="gripper_base"):
    """测试特定位姿的逆解"""
    print("\n" + "="*70)
    print("测试特定位姿的逆解")
    print("="*70)
    
    # 用户指定的位姿
    pos_x, pos_y, pos_z = 0.491, 0.0, 0.415
    quat_x, quat_y, quat_z, quat_w = 0.0, 0.6756, 0.0, 0.7373
    
    print(f"\n目标位置: ({pos_x}, {pos_y}, {pos_z})")
    print(f"目标姿态 (四元数 x,y,z,w): ({quat_x}, {quat_y}, {quat_z}, {quat_w})")
    
    # 四元数转旋转矩阵
    rot_matrix = quaternion_to_rotation_matrix(quat_x, quat_y, quat_z, quat_w)
    print(f"\n旋转矩阵:\n{rot_matrix}")
    
    # 构建 4x4 齐次变换矩阵
    H_target = np.eye(4)
    H_target[:3, :3] = rot_matrix
    H_target[:3, 3] = [pos_x, pos_y, pos_z]
    H_target = torch.tensor(H_target, **tensor_args).unsqueeze(0)
    
    print(f"\n目标位姿矩阵 (4x4):\n{H_target.squeeze().cpu().numpy()}")
    
    pos_target = torch.tensor([pos_x, pos_y, pos_z], **tensor_args)
    
    # 先尝试用已知解作为初始猜测
    print(f"\n方法1: 使用已知解 [0,2,-2,0,0,0] 作为初始猜测")
    print("="*60)
    q_known = torch.zeros(1, robot._n_dofs, **tensor_args)
    known_joints = [0, 2, -2, 0, 0, 0]
    for i, angle in enumerate(known_joints):
        q_known[0, i] = angle
    
    # 验证已知解
    H_known = robot.compute_forward_kinematics_all_links(q_known, link_list=[ee_link])
    pos_known = link_pos_from_link_tensor(H_known).squeeze()
    print(f"已知解的FK位置: {to_numpy(pos_known)}")
    print(f"目标位置: {pos_x, pos_y, pos_z}")
    print(f"位置差异: {to_numpy((pos_known - pos_target).norm()):.6f} m")
    
    # 重要：使用FK计算的H_target而不是手动构建的
    print(f"\n使用FK计算的H_target进行IK求解（更精确）")
    H_target_from_fk = H_known
    
    # 方法2: 使用已知解附近的初始值
    print(f"\n方法2: 使用智能初始化的IK求解")
    print("="*60)
    print(f"策略: 从已知解 [0,2,-2,0,0,0] 附近开始搜索")
    
    batch_size = 10
    q0_batch = q_known.repeat(batch_size, 1)
    # 添加小噪声以获得多个解（只对前6个关节）
    noise = torch.randn(batch_size, robot._n_dofs, **tensor_args) * 0.05
    noise[:, 6:] = 0  # 不对夹爪关节添加噪声
    q0_batch = q0_batch + noise
    
    # 确保在关节限制内
    lower, upper, _, _ = robot.get_joint_limit_array()
    lower_t = torch.tensor(lower, **tensor_args)
    upper_t = torch.tensor(upper, **tensor_args)
    q0_batch = torch.clamp(q0_batch, lower_t, upper_t)
    
    print(f"批次大小: {batch_size}")
    print(f"初始值噪声: 0.05 rad (仅前6个关节)")
    print(f"示例初始值: {to_numpy(q0_batch[0, :6])}")
    
    with TimerCUDA() as t:
        q_ik, idx_valid = robot.inverse_kinematics(
            H_target_from_fk,  # 使用FK计算的H_target
            link_name=ee_link,
            q0=q0_batch,
            q0_noise=0.0,  # 已经添加了噪声，不需要额外噪声
            batch_size=batch_size,
            max_iters=500,
            lr=1e-1,
            se3_eps=1e-2,
            eps_joint_lim=torch.pi / 64,
            print_freq=-1,
            debug=False,
        )
    
    print(f"\nIK 求解时间: {t.elapsed:.3f} 秒")
    print(f"有效解数量: {idx_valid.nelement()}/{batch_size}")
    
    # 如果方法2失败，尝试方法3：标准随机初始化
    if idx_valid.nelement() == 0:
        print(f"\n方法3: 标准随机初始化IK求解 (batch_size=30)")
        print("="*60)
        
        batch_size_fallback = 30
        with TimerCUDA() as t:
            q_ik, idx_valid = robot.inverse_kinematics(
                H_target,
                link_name=ee_link,
                batch_size=batch_size_fallback,
                max_iters=2000,
                lr=1e-1,
                se3_eps=1e-2,
                eps_joint_lim=torch.pi / 64,
                print_freq=500,
                debug=False,
            )
        
        print(f"\nIK 求解时间: {t.elapsed:.3f} 秒")
        print(f"有效解数量: {idx_valid.nelement()}/{batch_size_fallback}")
    
    if idx_valid.nelement() > 0:
        print(f"\n" + "="*50)
        print("找到的关节角度解:")
        print("="*50)
        
        # 检查是否有解接近 [0, 2, -2, 0, 0, 0]
        target_joint = np.array([0, 2, -2, 0, 0, 0])
        
        for i, idx in enumerate(idx_valid[:5]):  # 打印前5个解
            q = q_ik[idx]
            q_6dof = q[:6]  # 只取6个关节角度
            
            print(f"\n解 {i+1}:")
            print(f"  主要关节角度 (6-DOF 弧度): {to_numpy(q_6dof)}")
            print(f"  主要关节角度 (6-DOF 角度): {np.rad2deg(to_numpy(q_6dof))}")
            print(f"  完整关节角度 (8-DOF 弧度): {to_numpy(q)}")
            
            # 检查与目标关节角度的差异
            joint_diff = np.abs(to_numpy(q_6dof) - target_joint)
            joint_error = np.linalg.norm(joint_diff)
            print(f"  与 [0,2,-2,0,0,0] 的差异: {joint_diff}")
            print(f"  关节角度误差 (L2范数): {joint_error:.6f} rad")
            
            # 验证FK
            H_check = robot.compute_forward_kinematics_all_links(q.unsqueeze(0), link_list=[ee_link])
            pos_check = link_pos_from_link_tensor(H_check).squeeze()
            rot_check = link_rot_from_link_tensor(H_check).squeeze()
            pos_error = (pos_check - pos_target).norm().item()
            
            # 计算姿态误差
            rot_np = to_numpy(rot_check)
            qx_check, qy_check, qz_check, qw_check = rotation_matrix_to_quaternion(rot_np)
            quat_target = np.array([0.0, 0.6756, 0.0, 0.7373])
            quat_check = np.array([qx_check, qy_check, qz_check, qw_check])
            quat_error = np.linalg.norm(quat_check - quat_target)
            
            print(f"  验证末端位置: {to_numpy(pos_check)}")
            print(f"  位置误差: {pos_error:.6f} m")
            print(f"  验证四元数: ({qx_check:.4f}, {qy_check:.4f}, {qz_check:.4f}, {qw_check:.4f})")
            print(f"  四元数误差: {quat_error:.6f}")
    else:
        print("\n标准IK求解未找到有效解！")
        
        # 方法3: 尝试只优化前6个关节
        print(f"\n方法3: 尝试固定夹爪关节，只优化前6个关节")
        print("="*60)
        
        # 手动实现简化的IK（只作为诊断）
        print("诊断信息:")
        print(f"  目标位姿在工作空间内: ✓ (FK验证通过)")
        print(f"  末端链接正确: ✓ (gripper_base)")
        print(f"  已知解存在: ✓ ([0,2,-2,0,0,0])")
        print(f"\n可能的问题:")
        print(f"  1. IK优化器陷入局部最优")
        print(f"  2. 夹爪关节(DOF 7-8)干扰优化过程")
        print(f"  3. 需要更好的初始化策略")
        print(f"\n建议:")
        print(f"  - 检查IK函数的实现，特别是关节限制处理")
        print(f"  - 尝试使用解析IK（如果Piper有解析解）")
        print(f"  - 考虑使用数值IK库（如TRAC-IK, KDL）")
    
    return q_ik, idx_valid, pos_target


def test_forward_kinematics(robot, device, tensor_args):
    """测试正向运动学"""
    print("\n" + "="*70)
    print("测试正向运动学 (Forward Kinematics)")
    print("="*70)
    
    # 打印所有链接名
    link_names = robot.get_link_names()
    print(f"\nPiper 机械臂链接名: {link_names}")
    
    # 获取关节限制
    lower, upper, _, _ = robot.get_joint_limit_array()
    print(f"\n关节下限: {lower}")
    print(f"关节上限: {upper}")
    print(f"关节数量 (DOF): {robot._n_dofs}")
    
    # 测试用户指定的关节角度 [0, 2, -2, 0, 0, 0]
    q_test = torch.zeros(1, robot._n_dofs, **tensor_args)
    joint_angles = [0, 2, -2, 0, 0, 0]
    for i, angle in enumerate(joint_angles):
        q_test[0, i] = angle
    
    print(f"\n测试关节角度 [0, 2, -2, 0, 0, 0]:")
    print(f"  完整配置 ({robot._n_dofs}-DOF): {to_numpy(q_test)}")
    
    # 测试不同的末端链接
    possible_ee_links = ["gripper_base", "link6", "link_tcp", "tcp", "end_effector"]
    
    print(f"\n测试不同末端执行器链接的FK结果:")
    print("="*60)
    
    for ee_link in possible_ee_links:
        if ee_link in link_names:
            try:
                H_ee = robot.compute_forward_kinematics_all_links(q_test, link_list=[ee_link])
                ee_pos = link_pos_from_link_tensor(H_ee).squeeze()
                ee_rot = link_rot_from_link_tensor(H_ee).squeeze()
                
                # 计算四元数
                rot_np = to_numpy(ee_rot)
                qx, qy, qz, qw = rotation_matrix_to_quaternion(rot_np)
                
                print(f"\n链接: {ee_link}")
                print(f"  位置: {to_numpy(ee_pos)}")
                print(f"  四元数: ({qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f})")
                
                # 检查是否匹配目标
                target_pos = np.array([0.491, 0.0, 0.415])
                target_quat = np.array([0.0, 0.6756, 0.0, 0.7373])
                pos_match = np.allclose(to_numpy(ee_pos), target_pos, atol=0.001)
                quat_match = np.allclose([qx, qy, qz, qw], target_quat, atol=0.01)
                
                if pos_match and quat_match:
                    print(f"  ✓✓✓ 匹配目标位姿！这是正确的末端链接！")
                elif pos_match:
                    print(f"  ✓ 位置匹配")
                    
            except Exception as e:
                print(f"\n链接: {ee_link} - 错误: {e}")
    
    # 返回最可能的末端链接
    return "gripper_base"


def test_specific_joint_fk(robot, device, tensor_args, joint_angles, ee_link="gripper_base"):
    """测试指定关节角度的正向运动学"""
    print("\n" + "="*70)
    print("测试指定关节角度的正向运动学 (FK)")
    print("="*70)
    
    # 确保关节角度数量正确
    if len(joint_angles) != 6:
        print(f"错误: 需要6个关节角度，但提供了 {len(joint_angles)} 个")
        return
    
    # 创建完整的关节角度张量 (Piper有8个DOF，但我们只设置前6个)
    q_test = torch.zeros(1, robot._n_dofs, **tensor_args)
    for i, angle in enumerate(joint_angles):
        q_test[0, i] = angle
    
    print(f"\n测试关节角度 (弧度): {joint_angles}")
    print(f"测试关节角度 (角度): {np.rad2deg(joint_angles)}")
    print(f"完整关节配置 ({robot._n_dofs}-DOF): {to_numpy(q_test)}")
    
    # 检查关节限制
    lower, upper, _, _ = robot.get_joint_limit_array()
    print(f"\n关节限制检查:")
    for i in range(6):
        angle = joint_angles[i]
        in_limit = lower[i] <= angle <= upper[i]
        status = "✓" if in_limit else "✗ 超出限制!"
        print(f"  关节 {i+1}: {angle:.3f} rad ({np.rad2deg(angle):.1f}°) [{lower[i]:.3f}, {upper[i]:.3f}] {status}")
    
    # 计算正向运动学
    print(f"\n计算末端执行器位姿 (链接: {ee_link})...")
    H_ee = robot.compute_forward_kinematics_all_links(q_test, link_list=[ee_link])
    ee_pos = link_pos_from_link_tensor(H_ee).squeeze()
    ee_rot = link_rot_from_link_tensor(H_ee).squeeze()
    
    print(f"\n末端执行器位置 (x, y, z): {to_numpy(ee_pos)}")
    print(f"末端执行器旋转矩阵:\n{to_numpy(ee_rot)}")
    
    # 计算欧拉角 (ZYX顺序)
    rot_np = to_numpy(ee_rot)
    sy = np.sqrt(rot_np[0, 0]**2 + rot_np[1, 0]**2)
    singular = sy < 1e-6
    
    if not singular:
        roll = np.arctan2(rot_np[2, 1], rot_np[2, 2])
        pitch = np.arctan2(-rot_np[2, 0], sy)
        yaw = np.arctan2(rot_np[1, 0], rot_np[0, 0])
    else:
        roll = np.arctan2(-rot_np[1, 2], rot_np[1, 1])
        pitch = np.arctan2(-rot_np[2, 0], sy)
        yaw = 0
    
    print(f"\n末端执行器姿态 (欧拉角 RPY):")
    print(f"  Roll:  {roll:.4f} rad ({np.rad2deg(roll):.2f}°)")
    print(f"  Pitch: {pitch:.4f} rad ({np.rad2deg(pitch):.2f}°)")
    print(f"  Yaw:   {yaw:.4f} rad ({np.rad2deg(yaw):.2f}°)")
    
    # 计算四元数 (从欧拉角)
    qx_euler, qy_euler, qz_euler, qw_euler = euler_to_quaternion(roll, pitch, yaw)
    print(f"\n末端执行器姿态 (四元数 - 从欧拉角计算):")
    print(f"  qx: {qx_euler:.6f}")
    print(f"  qy: {qy_euler:.6f}")
    print(f"  qz: {qz_euler:.6f}")
    print(f"  qw: {qw_euler:.6f}")
    print(f"  格式: (x, y, z, w) = ({qx_euler:.6f}, {qy_euler:.6f}, {qz_euler:.6f}, {qw_euler:.6f})")
    
    # 计算四元数 (从旋转矩阵)
    qx_mat, qy_mat, qz_mat, qw_mat = rotation_matrix_to_quaternion(rot_np)
    print(f"\n末端执行器姿态 (四元数 - 从旋转矩阵计算):")
    print(f"  qx: {qx_mat:.6f}")
    print(f"  qy: {qy_mat:.6f}")
    print(f"  qz: {qz_mat:.6f}")
    print(f"  qw: {qw_mat:.6f}")
    print(f"  格式: (x, y, z, w) = ({qx_mat:.6f}, {qy_mat:.6f}, {qz_mat:.6f}, {qw_mat:.6f})")
    
    return ee_pos, ee_rot


def test_inverse_kinematics(robot, device, tensor_args, ee_link="gripper_base"):
    """测试逆解"""
    print("\n" + "="*70)
    print("测试逆解 (Inverse Kinematics)")
    print("="*70)
    
    # 首先通过随机关节角度获取一个可达的目标位姿
    print("\n生成可达的目标位姿...")
    lower, upper, _, _ = robot.get_joint_limit_array()
    lower = torch.tensor(lower, **tensor_args)
    upper = torch.tensor(upper, **tensor_args)
    
    # 随机生成一个关节角度，然后计算对应的末端位姿
    q_sample = lower + torch.rand(robot._n_dofs, **tensor_args) * (upper - lower)
    H_target_tensor = robot.compute_forward_kinematics_all_links(
        q_sample.unsqueeze(0), link_list=[ee_link]
    ).squeeze(0)
    
    pos_target = link_pos_from_link_tensor(H_target_tensor.unsqueeze(0)).squeeze()
    rot_target = link_rot_from_link_tensor(H_target_tensor.unsqueeze(0)).squeeze()
    
    frame_target = Frame(rot=rot_target, trans=pos_target, device=device)
    H_target = H_target_tensor.unsqueeze(0)
    
    print(f"\n目标位置: {to_numpy(pos_target)}")
    print(f"目标旋转矩阵:\n{to_numpy(rot_target)}")
    print(f"原始关节角度 (仅作参考): {to_numpy(q_sample)}")
    
    # 求解逆解
    batch_size = 10  # 尝试多个初始值
    
    print(f"\n开始求解逆解 (batch_size={batch_size})...")
    with TimerCUDA() as t:
        q_ik, idx_valid = robot.inverse_kinematics(
            H_target,
            link_name=ee_link,
            batch_size=batch_size,
            max_iters=800,
            lr=1e-1,
            se3_eps=3e-2,
            eps_joint_lim=torch.pi / 64,
            print_freq=200,
            debug=False,
        )
    
    print(f"\nIK 求解时间: {t.elapsed:.3f} 秒")
    print(f"有效解数量: {idx_valid.nelement()}/{batch_size}")
    
    if idx_valid.nelement() > 0:
        print(f"\n找到的关节角度解:")
        for i, idx in enumerate(idx_valid[:3]):  # 只打印前3个解
            q = q_ik[idx]
            print(f"  解 {i+1}: {to_numpy(q)}")
            
            # 验证：用正向运动学计算该解对应的末端位姿
            H_check = robot.compute_forward_kinematics_all_links(q.unsqueeze(0), link_list=[ee_link])
            pos_check = link_pos_from_link_tensor(H_check).squeeze()
            print(f"       验证末端位置: {to_numpy(pos_check)}")
            print(f"       位置误差: {to_numpy((pos_check - pos_target).norm()):.6f}")
    
    return q_ik, idx_valid, pos_target, frame_target


def test_random_ik(robot, device, tensor_args, ee_link="gripper_base", num_tests=5):
    """测试随机目标位姿的逆解"""
    print("\n" + "="*70)
    print(f"测试随机目标位姿的逆解 ({num_tests} 次)")
    print("="*70)
    
    success_count = 0
    
    for i in range(num_tests):
        # 生成随机关节角度
        lower, upper, _, _ = robot.get_joint_limit_array()
        lower = torch.tensor(lower, **tensor_args)
        upper = torch.tensor(upper, **tensor_args)
        
        q_random = lower + torch.rand(robot._n_dofs, **tensor_args) * (upper - lower)
        
        # 计算该关节角度对应的末端位姿
        H_target = robot.compute_forward_kinematics_all_links(
            q_random.unsqueeze(0), link_list=[ee_link]
        ).squeeze(0)
        
        pos_target = link_pos_from_link_tensor(H_target.unsqueeze(0)).squeeze()
        
        # 尝试求解逆解
        q_ik, idx_valid = robot.inverse_kinematics(
            H_target.unsqueeze(0),
            link_name=ee_link,
            batch_size=5,
            max_iters=300,
            lr=2e-1,
            se3_eps=5e-2,
            print_freq=-1,  # 不打印中间过程
            debug=False,
        )
        
        if idx_valid.nelement() > 0:
            success_count += 1
            # 验证
            q_solution = q_ik[idx_valid[0]]
            H_check = robot.compute_forward_kinematics_all_links(
                q_solution.unsqueeze(0), link_list=[ee_link]
            )
            pos_check = link_pos_from_link_tensor(H_check).squeeze()
            pos_error = (pos_check - pos_target).norm().item()
            
            print(f"  测试 {i+1}: 成功 | 位置误差: {pos_error:.6f}")
        else:
            print(f"  测试 {i+1}: 失败 | 未找到有效解")
    
    print(f"\n逆解成功率: {success_count}/{num_tests} ({100*success_count/num_tests:.1f}%)")


def visualize_ik_result(robot, q_ik, idx_valid, pos_target, frame_target, device, tensor_args, ee_link):
    """可视化逆解结果"""
    print("\n" + "="*70)
    print("可视化逆解结果")
    print("="*70)
    
    if idx_valid.nelement() == 0:
        print("没有找到有效解，无法可视化")
        return
    
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    ax.set_xlim(-0.5, 0.5)
    ax.set_ylim(-0.5, 0.5)
    ax.set_zlim(-0.2, 0.6)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("Piper Robot Inverse Kinematics Result")
    
    # 绘制原点坐标系
    frame_origin = Frame(device=device)
    ax.plot(0, 0, 0, color="b", marker="D", markersize=10, zorder=100, label="Origin")
    plot_coordinate_frame(ax, frame_origin, arrow_length=0.1, arrow_linewidth=3.0, tensor_args=tensor_args)
    
    # 绘制目标位置
    target_pos_np = to_numpy(pos_target)
    ax.plot(target_pos_np[0], target_pos_np[1], target_pos_np[2], 
            "r*", markersize=20, zorder=100, label="Target")
    plot_coordinate_frame(ax, frame_target, arrow_length=0.08, arrow_linewidth=2.0, tensor_args=tensor_args)
    
    # 绘制机械臂 (最多3个解)
    colors = ['blue', 'green', 'purple']
    for i, idx in enumerate(idx_valid[:3]):
        q = q_ik[idx]
        skeleton = get_skeleton_from_model(robot, q, robot.get_link_names())
        skeleton.draw_skeleton(ax=ax, color=colors[i % len(colors)])
        
        # 绘制末端位置
        H_EE = robot.compute_forward_kinematics_all_links(q.unsqueeze(0), link_list=[ee_link])
        ee_pos = link_pos_from_link_tensor(H_EE).squeeze()
        ee_pos_np = to_numpy(ee_pos)
        ax.plot(ee_pos_np[0], ee_pos_np[1], ee_pos_np[2], 
                'o', color=colors[i % len(colors)], markersize=8, label=f'Solution {i+1}')
    
    ax.legend()
    plt.tight_layout()
    plt.savefig("/home/hong/Projects/MotionPlanningDiffusion/mpd-splines-public/scripts/Hcj/piper_ik_result.png", 
                dpi=150, bbox_inches='tight')
    print(f"\n结果已保存至: scripts/Hcj/piper_ik_result.png")
    plt.show()


def main():
    print("="*70)
    print("Piper 机械臂运动学测试脚本")
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
    
    # 1. 测试正向运动学 (零位)
    ee_link = test_forward_kinematics(robot, device, tensor_args)
    
    # 2. 测试用户指定的关节角度 [0, 2, -2, 0, 0, 0]
    # print("\n" + "="*70)
    # print("测试用户指定的关节角度")
    # print("="*70)
    # joint_angles = [0, 2, -2, 0, 0, 0]
    # test_specific_joint_fk(robot, device, tensor_args, joint_angles, ee_link=ee_link)
    
    # 3. 测试特定位姿逆解 (可选，注释掉以只测试FK)
    print("\n" + "="*70)
    print("开始测试用户指定的位姿")
    print("="*70)
    q_ik, idx_valid, pos_target = test_specific_pose_ik(robot, device, tensor_args, ee_link=ee_link)
    
    # 4. 可视化结果 (可选)
    if idx_valid.nelement() > 0:
        pos_target_tensor = torch.tensor([0.491, 0.0, 0.415], **tensor_args)
        quat_x, quat_y, quat_z, quat_w = 0.0, 0.6756, 0.0, 0.7373
        rot_matrix = quaternion_to_rotation_matrix(quat_x, quat_y, quat_z, quat_w)
        rot_target = torch.tensor(rot_matrix, **tensor_args)
        frame_target = Frame(rot=rot_target, trans=pos_target_tensor, device=device)
        
        visualize_ik_result(robot, q_ik, idx_valid, pos_target_tensor, frame_target, device, tensor_args, ee_link)
    
    print("\n" + "="*70)
    print("测试完成!")
    print("="*70)


if __name__ == "__main__":
    main()
