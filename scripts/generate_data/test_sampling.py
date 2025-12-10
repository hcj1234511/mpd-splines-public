#!/usr/bin/env python3
"""
直接测试 get_state_not_in_collision 函数
"""
import sys
import numpy as np
import torch
import pybullet as p
from pybullet_utils import bullet_client

sys.path.insert(0, '/home/hong/Projects/MotionPlanningDiffusion/mpd-splines-public')

from torch_robotics.robots import RobotPiper
from torch_robotics.environments import EnvSpheres3D
from pb_ompl.pb_ompl import PbOMPLRobot, PbOMPL

def main():
    print("=" * 80)
    print("直接测试 get_state_not_in_collision")
    print("=" * 80)
    
    tensor_args = {"device": "cpu", "dtype": torch.float32}
    
    # 加载机器人和环境
    robot_tr = RobotPiper(gripper=True, tensor_args=tensor_args)
    env_tr = EnvSpheres3D(
        precompute_sdf_obj_fixed=False,
        precompute_sdf_obj_extra=False,
        tensor_args=tensor_args
    )
    
    # 创建 PyBullet
    pybullet_client = bullet_client.BulletClient(connection_mode=p.DIRECT, options="")
    pybullet_client.setGravity(0, 0, 0.0)
    
    robot_id = pybullet_client.loadURDF(
        robot_tr.robot_urdf_file,
        (0, 0, 0),
        useFixedBase=True,
        flags=p.URDF_USE_SELF_COLLISION,
    )
    
    robot_pb = PbOMPLRobot(
        pybullet_client,
        robot_id,
        urdf_path=robot_tr.robot_urdf_file,
        link_name_ee=robot_tr.link_name_ee,
    )
    
    # 创建 OMPL
    obstacles = []
    pbompl = PbOMPL(pybullet_client, robot_pb, obstacles, min_distance_robot_env=0.0)
    pbompl.set_planner("RRTConnect")
    pbompl.setup_collision_detection(robot_pb, obstacles, self_collisions=False, allow_collision_links=[])
    
    print(f"\n关节限制:")
    for i, (low, high) in enumerate(robot_pb.joint_bounds):
        print(f"  Joint {i}: [{low:7.3f}, {high:7.3f}]")
    
    # 手动测试采样
    print(f"\n手动测试 10 次随机采样:")
    for i in range(10):
        q_sample = robot_pb.get_random_joint_position()
        is_valid = pbompl.is_state_valid(q_sample, check_bounds=True)
        print(f"  尝试 {i+1}: valid={is_valid}, q={q_sample}")
        
        if not is_valid:
            # 检查为什么无效
            # 边界检查
            out_of_bounds = False
            for j, (q, low, high) in enumerate(zip(q_sample, robot_pb.joint_bounds_low_np, robot_pb.joint_bounds_high_np)):
                if q < low or q > high:
                    print(f"    Joint {j}: q={q:.6f} 超出 [{low:.6f}, {high:.6f}]")
                    out_of_bounds = True
            
            if not out_of_bounds:
                print(f"    边界检查通过，但状态仍然无效 - 可能是碰撞")
    
    # 测试 get_state_not_in_collision
    print(f"\n测试 get_state_not_in_collision (max_tries=10):")
    try:
        q_valid = pbompl.get_state_not_in_collision(max_tries=10, raise_error=False, debug=True)
        if q_valid is not None:
            print(f"✅ 成功: {q_valid}")
        else:
            print(f"❌ 失败: 返回 None")
    except Exception as e:
        print(f"❌ 异常: {e}")
    
    pybullet_client.disconnect()
    print("\n" + "=" * 80)

if __name__ == "__main__":
    main()
