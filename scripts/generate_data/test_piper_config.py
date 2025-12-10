#!/usr/bin/env python3
"""
诊断 Piper 机器人配置问题的脚本
"""
import torch
from torch_robotics.robots import RobotPiper
from torch_robotics.environments import EnvWarehouse
from torch_robotics.torch_utils.torch_utils import DEFAULT_TENSOR_ARGS

def main():
    print("=" * 80)
    print("Piper 机器人配置诊断")
    print("=" * 80)
    
    tensor_args = {"device": "cpu", "dtype": torch.float32}  # 使用 torch.float32 而不是字符串
    
    try:
        # 1. 加载 Piper 机器人
        print("\n1. 正在加载 RobotPiper...")
        robot = RobotPiper(tensor_args=tensor_args, use_self_collision_storm=False)
        print("   ✅ RobotPiper 加载成功！")
        
        # 2. 检查关节自由度
        print(f"\n2. 机器人关节信息：")
        print(f"   - 手臂关节数: {robot.arm_q_dim}")
        print(f"   - 夹爪关节数: {robot.gripper_q_dim}")
        print(f"   - 总关节数: {robot.q_dim}")
        print(f"   - 末端执行器链接: {robot.link_name_ee}")
        
        # 3. 检查关节限制
        print(f"\n3. 关节限制：")
        print(f"   - q_pos_min: {robot.q_pos_min}")
        print(f"   - q_pos_max: {robot.q_pos_max}")
        
        # 4. 测试正向运动学
        print(f"\n4. 测试正向运动学...")
        q_test = torch.zeros(robot.q_dim, **tensor_args)  # 使用完整的 q_dim（包括夹爪）
        try:
            ee_pose = robot.fk_map_collision(q_test.unsqueeze(0))
            print(f"   ✅ FK 测试成功！")
            print(f"   - 零位姿末端位置: {ee_pose[0, :3]}")
        except Exception as e:
            print(f"   ❌ FK 测试失败: {e}")
            
        # 5. 测试逆向运动学
        print(f"\n5. 测试逆向运动学...")
        try:
            if hasattr(robot, 'inverse_kinematics_single'):
                target_pose = torch.tensor([0.3, 0.0, 0.3, 1.0, 0.0, 0.0, 0.0], **tensor_args).unsqueeze(0)
                q_ik = robot.inverse_kinematics_single(target_pose, num_retries=10)
                if q_ik is not None:
                    print(f"   ✅ IK 测试成功！")
                    print(f"   - IK 结果: {q_ik}")
                else:
                    print(f"   ⚠️  IK 未找到解（这可能是正常的，取决于目标位姿）")
            else:
                print(f"   ⚠️  RobotPiper 没有 inverse_kinematics_single 方法")
                print(f"   （这是正常的，IK 功能可能通过其他方式实现）")
        except Exception as e:
            print(f"   ❌ IK 测试失败: {e}")
            
        # 6. 加载环境
        print(f"\n6. 正在加载环境...")
        try:
            env = EnvWarehouse(
                precompute_sdf_obj_fixed=False,
                precompute_sdf_obj_extra=False,
                tensor_args=tensor_args
            )
            print(f"   ✅ EnvWarehouse 加载成功！")
        except Exception as e:
            print(f"   ❌ 环境加载失败: {e}")
            
        # 7. 检查 URDF 文件
        print(f"\n7. URDF 文件信息：")
        print(f"   - URDF 路径: {robot.robot_urdf_file}")
        import os
        if os.path.exists(robot.robot_urdf_file):
            print(f"   ✅ URDF 文件存在")
        else:
            print(f"   ❌ URDF 文件不存在！")
            
        # 8. 检查碰撞球配置
        print(f"\n8. 碰撞球配置：")
        if hasattr(robot, 'link_collision_spheres_names'):
            print(f"   - 碰撞球链接数: {len(robot.link_collision_spheres_names)}")
            print(f"   - 碰撞球总数: {len(robot.link_collision_spheres_radii)}")
            print(f"   ✅ 碰撞球配置存在")
        else:
            print(f"   ⚠️  未找到碰撞球配置")
            
        print("\n" + "=" * 80)
        print("诊断完成！")
        print("=" * 80)
        
    except Exception as e:
        print(f"\n❌ 发生错误: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
