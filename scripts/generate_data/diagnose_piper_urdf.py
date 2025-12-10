#!/usr/bin/env python3
"""
详细诊断 Piper URDF 在 PyBullet 中的问题
"""
import sys
import numpy as np
import torch
import pybullet as p
from pybullet_utils import bullet_client

sys.path.insert(0, '/home/hong/Projects/MotionPlanningDiffusion/mpd-splines-public')

from torch_robotics.robots import RobotPiper

def main():
    print("=" * 80)
    print("诊断 Piper URDF 文件")
    print("=" * 80)
    
    tensor_args = {"device": "cpu", "dtype": torch.float32}
    
    # 加载机器人
    print("\n1. 加载 RobotPiper...")
    robot_tr = RobotPiper(gripper=True, tensor_args=tensor_args)
    print(f"   ✅ 加载成功！")
    print(f"   - q_dim: {robot_tr.q_dim}")
    print(f"   - arm_q_dim: {robot_tr.arm_q_dim}")
    print(f"   - gripper_q_dim: {robot_tr.gripper_q_dim}")
    
    # 打印 torch_robotics 中的关节限制
    print("\n2. Torch Robotics 中的关节限制:")
    print(f"   q_pos_min: {robot_tr.q_pos_min}")
    print(f"   q_pos_max: {robot_tr.q_pos_max}")
    
    # 创建 PyBullet 客户端
    print("\n3. 加载到 PyBullet...")
    pybullet_client = bullet_client.BulletClient(connection_mode=p.DIRECT, options="")
    pybullet_client.setGravity(0, 0, 0.0)
    
    urdf_file = robot_tr.robot_urdf_file
    print(f"   URDF 文件: {urdf_file}")
    
    robot_id = pybullet_client.loadURDF(
        urdf_file,
        (0, 0, 0),
        useFixedBase=True,
        flags=p.URDF_USE_SELF_COLLISION,
    )
    
    num_joints = pybullet_client.getNumJoints(robot_id)
    print(f"   ✅ 加载成功！关节数: {num_joints}")
    
    # 打印 PyBullet 中的关节信息
    print("\n4. PyBullet 中的关节信息:")
    movable_joints = []
    for i in range(num_joints):
        joint_info = pybullet_client.getJointInfo(robot_id, i)
        joint_name = joint_info[1].decode('utf-8')
        joint_type = joint_info[2]
        lower = joint_info[8]
        upper = joint_info[9]
        
        type_names = {
            p.JOINT_REVOLUTE: "REVOLUTE",
            p.JOINT_PRISMATIC: "PRISMATIC",
            p.JOINT_FIXED: "FIXED",
        }
        type_name = type_names.get(joint_type, f"UNKNOWN({joint_type})")
        
        print(f"   Joint {i:2d}: {joint_name:20s} | Type: {type_name:10s} | Limits: [{lower:7.3f}, {upper:7.3f}]")
        
        if joint_type != p.JOINT_FIXED:
            movable_joints.append(i)
    
    print(f"\n   可移动关节索引: {movable_joints}")
    print(f"   可移动关节数量: {len(movable_joints)}")
    
    # 测试不同的关节位置
    print("\n5. 测试不同的关节配置:")
    
    # 测试1: 零位姿
    print("\n   测试 1: 零位姿")
    q_zero = [0.0] * len(movable_joints)
    for idx, joint_idx in enumerate(movable_joints):
        pybullet_client.resetJointState(robot_id, joint_idx, q_zero[idx])
    
    contacts = pybullet_client.getContactPoints(robot_id, robot_id)
    print(f"   - 自碰撞点数: {len(contacts)}")
    if len(contacts) > 0:
        print("   ⚠️  存在自碰撞！")
        for contact in contacts[:3]:  # 只显示前3个
            link_a = contact[3]
            link_b = contact[4]
            distance = contact[8]
            print(f"      Link {link_a} <-> Link {link_b}, distance: {distance:.4f}")
    else:
        print("   ✅ 无自碰撞")
    
    # 测试2: 基于限制中点的位置
    print("\n   测试 2: 关节限制中点")
    q_mid = []
    for joint_idx in movable_joints:
        joint_info = pybullet_client.getJointInfo(robot_id, joint_idx)
        lower = joint_info[8]
        upper = joint_info[9]
        mid = (lower + upper) / 2.0
        q_mid.append(mid)
    
    print(f"   - 中点位置: {np.array(q_mid)}")
    for idx, joint_idx in enumerate(movable_joints):
        pybullet_client.resetJointState(robot_id, joint_idx, q_mid[idx])
    
    contacts = pybullet_client.getContactPoints(robot_id, robot_id)
    print(f"   - 自碰撞点数: {len(contacts)}")
    if len(contacts) > 0:
        print("   ⚠️  存在自碰撞！")
        for contact in contacts[:3]:
            link_a = contact[3]
            link_b = contact[4]
            distance = contact[8]
            print(f"      Link {link_a} <-> Link {link_b}, distance: {distance:.4f}")
    else:
        print("   ✅ 无自碰撞")
    
    # 测试3: 手动设置的安全位置
    print("\n   测试 3: 手动安全位置")
    # 基于 Piper 的工作空间设置
    q_safe = [0.0, 1.57, -1.5, 0.0, 0.0, 0.0, 0.0, 0.0][:len(movable_joints)]
    print(f"   - 安全位置: {np.array(q_safe)}")
    
    for idx, joint_idx in enumerate(movable_joints):
        pybullet_client.resetJointState(robot_id, joint_idx, q_safe[idx])
    
    contacts = pybullet_client.getContactPoints(robot_id, robot_id)
    print(f"   - 自碰撞点数: {len(contacts)}")
    if len(contacts) > 0:
        print("   ⚠️  存在自碰撞！")
        for contact in contacts[:5]:
            link_a = contact[3]
            link_b = contact[4]
            distance = contact[8]
            link_a_name = pybullet_client.getJointInfo(robot_id, link_a)[12].decode('utf-8') if link_a >= 0 else "base"
            link_b_name = pybullet_client.getJointInfo(robot_id, link_b)[12].decode('utf-8') if link_b >= 0 else "base"
            print(f"      {link_a_name} <-> {link_b_name}, distance: {distance:.4f}")
    else:
        print("   ✅ 无自碰撞")
    
    print("\n" + "=" * 80)
    print("诊断完成！")
    print("=" * 80)
    
    pybullet_client.disconnect()

if __name__ == "__main__":
    main()
