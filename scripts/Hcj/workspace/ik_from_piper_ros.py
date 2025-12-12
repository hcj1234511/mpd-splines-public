#!/usr/bin/env python3
"""
Piper 机械臂 IK 求解器
基于 Pinocchio 的阻尼最小二乘法逆运动学求解

使用方法：
    cd /home/hong/Projects/MotionPlanningDiffusion/mpd-splines-public/scripts/Hcj/workspace
    python ik_from_piper_ros.py
"""

import numpy as np
import pinocchio as pin
import os


class PiperIK:
    """Piper 机械臂逆运动学求解器 (阻尼最小二乘法)"""

    def __init__(self, urdf_path=None):
        """
        初始化 IK 求解器

        Args:
            urdf_path: URDF 文件路径，None 则使用默认路径
        """
        np.set_printoptions(precision=5, suppress=True, linewidth=200)

        # 设置 URDF 路径
        if urdf_path is None:
            current_dir = os.path.dirname(os.path.abspath(__file__))
            urdf_path = os.path.join(
                current_dir, "..", "piper_description", "urdf", "piper_description.urdf"
            )

        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f"URDF file not found: {urdf_path}")

        # 加载完整机器人模型
        self.robot = pin.RobotWrapper.BuildFromURDF(urdf_path)

        # 锁定夹爪关节，创建 6-DOF 简化模型
        self.joints_to_lock = ["joint7", "joint8"]
        self.reduced_robot = self.robot.buildReducedRobot(
            list_of_joints_to_lock=self.joints_to_lock,
            reference_configuration=np.array([0] * self.robot.model.nq),
        )

        self.model = self.reduced_robot.model
        self.data = self.reduced_robot.data

        # 使用 link6 作为末端执行器
        self.ee_frame_name = "link6"
        self.ee_frame_id = self.model.getFrameId(self.ee_frame_name)

        # 存储上一次的解，用于热启动
        self.last_solution = np.zeros(self.model.nq)

        print(f"PiperIK initialized successfully!")
        print(f"  - URDF: {urdf_path}")
        print(f"  - DOF: {self.model.nq}")
        print(f"  - EE frame: {self.ee_frame_name} (id={self.ee_frame_id})")
        print(f"  - Joint limits:")
        print(f"    Lower: {self.model.lowerPositionLimit}")
        print(f"    Upper: {self.model.upperPositionLimit}")

    def forward_kinematics(self, q):
        """
        计算正运动学

        Args:
            q: 关节角度 (6,) numpy array

        Returns:
            SE3: 末端执行器位姿
        """
        q = np.asarray(q).flatten()
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[self.ee_frame_id].copy()

    def inverse_kinematics(
        self,
        target_pose,
        q_init=None,
        eps=1e-6,
        max_iter=1000,
        dt=1.0,
        damp=1e-6,
    ):
        """
        使用阻尼最小二乘法计算逆运动学

        Args:
            target_pose: 目标位姿，可以是:
                - pin.SE3 对象
                - 4x4 齐次变换矩阵 (numpy array)
                - dict: {'position': [x,y,z], 'quaternion': [x,y,z,w]} 或
                        {'position': [x,y,z], 'rotation': 3x3 matrix}
            q_init: 初始关节角度，None 则使用上一次的解
            eps: 收敛阈值
            max_iter: 最大迭代次数
            dt: 步长
            damp: 阻尼系数

        Returns:
            tuple: (q_solution, success, error_norm)
        """
        # 转换目标位姿为 SE3
        if isinstance(target_pose, pin.SE3):
            oMdes = pin.SE3(target_pose.rotation.copy(), target_pose.translation.copy())
        elif isinstance(target_pose, np.ndarray) and target_pose.shape == (4, 4):
            oMdes = pin.SE3(target_pose[:3, :3].copy(), target_pose[:3, 3].copy())
        elif isinstance(target_pose, dict):
            pos = np.array(target_pose["position"])
            if "quaternion" in target_pose:
                quat = target_pose["quaternion"]  # [x, y, z, w]
                rot = pin.Quaternion(quat[3], quat[0], quat[1], quat[2]).toRotationMatrix()
            elif "rotation" in target_pose:
                rot = np.array(target_pose["rotation"])
            else:
                rot = np.eye(3)
            oMdes = pin.SE3(rot, pos)
        else:
            raise ValueError(f"Unsupported target_pose type: {type(target_pose)}")

        # 设置初始值
        if q_init is None:
            q = self.last_solution.copy()
        else:
            q = np.asarray(q_init).flatten().copy()


        # 迭代求解
        for i in range(max_iter):
            # 计算当前 FK
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)

            # 当前位姿
            oMf = self.data.oMf[self.ee_frame_id]

            # 计算位姿误差 (LOCAL 坐标系)
            iMd = oMf.actInv(oMdes)
            error = pin.log6(iMd).vector

            # 检查收敛
            error_norm = np.linalg.norm(error)
            if error_norm < eps:
                self.last_solution = q.copy()
                return q, True, error_norm

            # 计算雅可比矩阵 (LOCAL 坐标系)
            J = pin.computeFrameJacobian(
                self.model, self.data, q, self.ee_frame_id, pin.ReferenceFrame.LOCAL
            )

            # 阻尼最小二乘求解
            JJT = J @ J.T + damp * np.eye(6)
            v = np.linalg.solve(JJT, error)
            dq = J.T @ v

            # 更新关节角度
            q = pin.integrate(self.model, q, dt * dq)

            # 限制关节角度在范围内
            if np.isfinite(self.model.lowerPositionLimit).all():
                q = np.clip(q, self.model.lowerPositionLimit, self.model.upperPositionLimit)

        # 未收敛
        self.last_solution = q.copy()
        return q, False, error_norm

    def solve_ik_from_xyz_rpy(self, x, y, z, roll, pitch, yaw, q_init=None):
        """
        从位置和欧拉角求解 IK

        Args:
            x, y, z: 目标位置 (米)
            roll, pitch, yaw: 目标姿态 (弧度)
            q_init: 初始关节角度

        Returns:
            tuple: (q_solution, success, error_norm)
        """
        rot = pin.rpy.rpyToMatrix(roll, pitch, yaw)
        target_pose = pin.SE3(rot, np.array([x, y, z]))
        return self.inverse_kinematics(target_pose, q_init)



# ============================================
# 测试代码
# ============================================
if __name__ == "__main__":
    print("=" * 70)
    print("Piper IK 求解器测试 (阻尼最小二乘法)")
    print("=" * 70)

    # 创建 IK 求解器
    ik_solver = PiperIK()

    # ============================================
    # 测试 1: FK -> IK 验证
    # ============================================
    print("\n" + "=" * 70)
    print("测试 1: 关节全为 0 的 FK，然后反求 IK")
    print("=" * 70)

    # FK: 关节全为 0
    q_zero = np.zeros(6)
    pose_fk = ik_solver.forward_kinematics(q_zero)
    print(f"输入关节角度: {q_zero}")
    print(f"FK 末端位置: {pose_fk.translation}")
    print(f"FK 末端旋转矩阵:\n{pose_fk.rotation}")

    # IK: 用 FK 结果作为目标
    print("\n从随机初始位置求解 IK...")
    q_init = np.random.uniform(-0.5, 0.5, 6)
    print(f"初始关节角度: {q_init}")

    q_ik, success, error = ik_solver.inverse_kinematics(pose_fk, q_init)
    print(f"\nIK 求解结果: {q_ik}")
    print(f"求解成功: {success}")
    print(f"误差: {error:.6e}")

    # 验证
    pose_verify = ik_solver.forward_kinematics(q_ik)
    pos_error = np.linalg.norm(pose_verify.translation - pose_fk.translation)
    print(f"\n验证 - 位置误差: {pos_error:.6e} m")
    print(f"验证 - 关节误差: {np.linalg.norm(q_ik - q_zero):.6f} rad")

    # ============================================
    # 测试 2: 非零关节配置 (在关节限制范围内)
    # ============================================
    print("\n" + "=" * 70)
    print("测试 2: 非零关节配置 (在关节限制范围内)")
    print("=" * 70)

    # 注意: joint3 限制是 [-2.967, 0]，必须是负值
    q_test = np.array([0.5, 0.5, -0.5, -0.2, 0.3, 0.1])
    print(f"测试关节角度: {q_test}")

    pose_test = ik_solver.forward_kinematics(q_test)
    print(f"FK 末端位置: {pose_test.translation}")

    # 从零位开始求解 IK
    print("\n从零位开始求解 IK...")
    q_ik2, success2, error2 = ik_solver.inverse_kinematics(pose_test, np.zeros(6))
    print(f"IK 求解结果: {q_ik2}")
    print(f"原始关节角度: {q_test}")
    print(f"求解成功: {success2}")
    print(f"误差: {error2:.6e}")

    # 验证
    pose_verify2 = ik_solver.forward_kinematics(q_ik2)
    pos_error2 = np.linalg.norm(pose_verify2.translation - pose_test.translation)
    joint_error2 = np.linalg.norm(q_ik2 - q_test)
    print(f"\n验证 - 位置误差: {pos_error2:.6e} m")
    print(f"验证 - 关节误差: {joint_error2:.6f} rad (IK 多解，关节误差可能较大)")

    # ============================================
    # 测试 3: 使用 xyz + rpy 接口 (用 FK 结果作为目标)
    # ============================================
    print("\n" + "=" * 70)
    print("测试 3: 使用 xyz + rpy 接口")
    print("=" * 70)

    # 先用一个合法配置计算 FK，得到可达的目标位姿
    q_valid = np.array([0.3, 0.0, 0.0, 0.0, 0.0, 0.2])
    pose_valid = ik_solver.forward_kinematics(q_valid)
    x, y, z = pose_valid.translation
    # 从旋转矩阵提取 rpy
    rpy = pin.rpy.matrixToRpy(pose_valid.rotation)
    roll, pitch, yaw = rpy
    print(f"目标位置: [{x:.4f}, {y:.4f}, {z:.4f}]")
    print(f"目标姿态 (rpy): [{roll:.4f}, {pitch:.4f}, {yaw:.4f}]")

    q_ik3, success3, error3 = ik_solver.solve_ik_from_xyz_rpy(x, y, z, roll, pitch, yaw, q_init=np.zeros(6))
    print(f"\nIK 求解结果: {q_ik3}")
    print(f"求解成功: {success3}")
    print(f"误差: {error3:.6e}")

    if success3:
        pose_verify3 = ik_solver.forward_kinematics(q_ik3)
        print(f"验证 - FK 位置: {pose_verify3.translation}")

    print("\n" + "=" * 70)
    print("测试完成！")
    print("=" * 70)
