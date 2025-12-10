#!/usr/bin/env python3
"""
测试 Piper 机器人在 PyBullet + OMPL 中的运动规划
"""
import sys
import os
import numpy as np
import torch
import pybullet as p
from pybullet_utils import bullet_client

# 添加路径
sys.path.insert(0, '/home/hong/Projects/MotionPlanningDiffusion/mpd-splines-public')

from torch_robotics.robots import RobotPiper
from torch_robotics.environments import EnvWarehouse
from torch_robotics.torch_utils.torch_utils import DEFAULT_TENSOR_ARGS
from pb_ompl.pb_ompl import PbOMPLRobot, PbOMPL

def main():
    print("=" * 80)
    print("测试 Piper 机器人在 PyBullet + OMPL 环境中")
    print("=" * 80)
    
    tensor_args = {"device": "cpu", "dtype": torch.float32}  # 使用 torch.float32 而不是字符串
    
    # 1. 加载机器人
    print("\n1. 加载 RobotPiper...")
    robot_tr = RobotPiper(gripper=True, tensor_args=tensor_args)
    print(f"   ✅ 机器人加载成功！DOF: {robot_tr.q_dim}")
    
    # 2. 加载环境
    print("\n2. 加载 EnvWarehouse...")
    env_tr = EnvWarehouse(
        precompute_sdf_obj_fixed=False,
        precompute_sdf_obj_extra=False,
        tensor_args=tensor_args
    )
    print("   ✅ 环境加载成功！")
    
    # 3. 创建 PyBullet 客户端
    print("\n3. 初始化 PyBullet...")
    pybullet_client = bullet_client.BulletClient(connection_mode=p.GUI, options="")
    pybullet_client.setGravity(0, 0, 0.0)
    pybullet_client.setTimeStep(1.0 / 240.0)
    print("   ✅ PyBullet 初始化成功！")
    
    # 4. 加载机器人到 PyBullet
    print("\n4. 加载机器人到 PyBullet...")
    urdf_file = robot_tr.robot_urdf_file
    print(f"   URDF 文件: {urdf_file}")
    
    robot_id = pybullet_client.loadURDF(
        urdf_file,
        (0, 0, 0),
        useFixedBase=True,
        flags=p.URDF_USE_SELF_COLLISION,
    )
    
    robot_pb = PbOMPLRobot(
        pybullet_client,
        robot_id,
        urdf_path=urdf_file,
        link_name_ee=robot_tr.link_name_ee,
    )
    print(f"   ✅ 机器人加载到 PyBullet 成功！Robot ID: {robot_id}")
    print(f"   - 末端执行器链接: {robot_tr.link_name_ee}")
    
    num_joints = pybullet_client.getNumJoints(robot_id)
    print(f"   - 关节数: {num_joints}")
    
    # 打印每个关节的信息
    print("\n   关节信息：")
    for i in range(num_joints):
        joint_info = pybullet_client.getJointInfo(robot_id, i)
        joint_name = joint_info[1].decode('utf-8')
        joint_type = joint_info[2]
        if joint_type != p.JOINT_FIXED:
            lower = joint_info[8]
            upper = joint_info[9]
            print(f"   - Joint {i}: {joint_name}, type={joint_type}, limits=[{lower:.3f}, {upper:.3f}]")
    
    # 5. 创建 OMPL 接口
    print("\n5. 创建 OMPL 接口...")
    obstacles = []
    pbompl_interface = PbOMPL(
        pybullet_client,
        robot_pb,
        obstacles,
        min_distance_robot_env=0.0  # 先设置为 0 测试
    )
    pbompl_interface.set_planner("RRTConnect")
    print("   ✅ OMPL 接口创建成功！")
    
    # 6. 测试零位姿是否有效
    print("\n6. 测试零位姿是否有效...")
    q_zero = np.zeros(robot_tr.q_dim)
    is_valid = pbompl_interface.is_state_valid(q_zero)
    print(f"   - 零位姿有效性: {is_valid}")
    if not is_valid:
        print("   ⚠️  零位姿无效！可能存在自碰撞或超出关节限制")
    
    # 7. 测试随机采样无碰撞状态
    print("\n7. 测试随机采样无碰撞状态...")
    try:
        q_sample = pbompl_interface.get_state_not_in_collision(debug=True)
        print(f"   ✅ 采样成功！")
        print(f"   - 关节位置: {q_sample}")
        
        # 设置机器人到这个位置
        for i, q in enumerate(q_sample):
            pybullet_client.resetJointState(robot_id, i, q)
        print("   - 机器人已移动到采样位置")
        
    except Exception as e:
        print(f"   ❌ 采样失败: {e}")
        import traceback
        traceback.print_exc()
        
        # 尝试手动设置一个安全的位置
        print("\n   尝试手动设置安全位置...")
        q_safe = np.array([0.0, 1.57, -1.5, 0.0, 0.0, 0.0, 0.0, 0.0])  # 基于关节限制的中间值
        is_safe_valid = pbompl_interface.is_state_valid(q_safe)
        print(f"   - 安全位置有效性: {is_safe_valid}")
        if is_safe_valid:
            for i, q in enumerate(q_safe):
                pybullet_client.resetJointState(robot_id, i, q)
            print(f"   - 安全位置: {q_safe}")
    
    print("\n" + "=" * 80)
    print("测试完成！按 Ctrl+C 退出...")
    print("=" * 80)
    
    # 保持 GUI 打开
    try:
        while True:
            p.stepSimulation()
    except KeyboardInterrupt:
        print("\n退出...")
        pybullet_client.disconnect()

if __name__ == "__main__":
    main()
