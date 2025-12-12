import pinocchio as pin
import numpy as np
import os

# ============================================
# 1. 加载机器人模型
# ============================================
urdf_path = os.path.join("..", "piper_description", "urdf", "piper_description.urdf")
model = pin.buildModelFromUrdf(urdf_path)
data = model.createData()


# ============================================
# 2. 正运动学 (FK) 函数
# ============================================
def forward_kinematics(model, data, q, frame_name=None):
    """
    计算正运动学
    Args:
        model: pinocchio模型
        data: pinocchio数据
        q: 关节角度 (numpy array)
        frame_name: 目标frame名称，None则返回最后一个frame
    Returns:
        oMf: SE3变换 (包含位置和旋转)
    """
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)

    if frame_name is None:
        # 返回最后一个frame
        return data.oMf[-1]
    else:
        if model.existFrame(frame_name):
            frame_id = model.getFrameId(frame_name)
            return data.oMf[frame_id]
        else:
            raise ValueError(f"Frame '{frame_name}' not found in model")


# ============================================
# 3. 逆运动学 (IK) 函数
# ============================================
def inverse_kinematics(
    model,
    data,
    target_pose,
    frame_name=None,
    q_init=None,
    eps=1e-4,
    max_iter=1000,
    dt=1.0,
    damp=1e-6,
):
    """
    使用阻尼最小二乘法计算逆运动学
    Args:
        model: pinocchio模型
        data: pinocchio数据
        target_pose: 目标位姿 (pin.SE3)
        frame_name: 目标frame名称
        q_init: 初始关节角度，None则使用零位
        eps: 收敛阈值
        max_iter: 最大迭代次数
        dt: 步长
        damp: 阻尼系数
    Returns:
        q: 求解的关节角度
        success: 是否收敛
    """
    # 复制目标位姿，避免被外部修改
    oMdes = pin.SE3(target_pose.rotation.copy(), target_pose.translation.copy())

    # 获取frame_id
    if frame_name is None:
        frame_id = model.nframes - 1
    else:
        if model.existFrame(frame_name):
            frame_id = model.getFrameId(frame_name)
        else:
            raise ValueError(f"Frame '{frame_name}' not found in model")

    # 初始化关节角度
    if q_init is None:
        q = np.zeros(model.nq)
    else:
        q = q_init.copy()

    # 迭代求解
    for i in range(max_iter):
        # 计算当前FK
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)

        # 当前位姿
        oMf = data.oMf[frame_id]

        # 计算位姿误差 (世界坐标系)
        # error = oMdes * oMf^(-1) 的 log
        iMd = oMf.actInv(oMdes)  # oMf^(-1) * oMdes
        error = pin.log6(iMd).vector

        # 检查收敛
        error_norm = np.linalg.norm(error)
        if error_norm < eps:
            print(f"IK converged in {i+1} iterations, error: {error_norm:.6e}")
            return q, True

        # 计算雅可比矩阵 (LOCAL坐标系，与误差一致)
        J = pin.computeFrameJacobian(model, data, q, frame_id, pin.ReferenceFrame.LOCAL)

        # 阻尼最小二乘求解: dq = J^T * (J * J^T + damp * I)^-1 * error
        JJT = J @ J.T + damp * np.eye(6)
        v = np.linalg.solve(JJT, error)
        dq = J.T @ v

        # 更新关节角度
        q = pin.integrate(model, q, dt * dq)

        # 限制关节角度在范围内 (如果有限制)
        if np.isfinite(model.lowerPositionLimit).all():
            q = np.clip(q, model.lowerPositionLimit, model.upperPositionLimit)

    print(f"IK did not converge after {max_iter} iterations, final error: {error_norm:.6e}")
    return q, False


# ============================================
# 4. 测试代码
# ============================================
if __name__ == "__main__":
    print("=" * 60)
    print("机器人模型信息")
    print("=" * 60)
    print(f"机器人名称: {model.name}")
    print(f"关节数量 (nq): {model.nq}")
    print(f"速度维度 (nv): {model.nv}")
    print(f"Frame数量: {model.nframes}")
    # print("\n所有Frames:")
    # for i, frame in enumerate(model.frames):
    #     print(f"  {i}: {frame.name}")

    # ---- 测试FK ----
    print("\n" + "=" * 60)
    print("测试正运动学 (FK)")
    print("=" * 60)
    q_test = np.zeros(model.nq)  # 所有关节为0
    q_test[:6] = np.array([0.5, -0.3, 0.4, -0.2, 0.3, 0.1])  # 设置前6个关节
    print(f"输入关节角度: {q_test}")

    pose_fk = forward_kinematics(model, data, q_test, "link6")
    print(f"\n末端位置: {pose_fk.translation}")
    print(f"末端旋转矩阵:\n{pose_fk.rotation}")

    # ---- 测试IK ----
    print("\n" + "=" * 60)
    print("测试逆运动学 (IK)")
    print("=" * 60)

    # 使用FK结果作为IK目标，验证IK正确性
    # 重要：复制 SE3 对象，避免被后续计算覆盖
    target_pose = pin.SE3(pose_fk.rotation.copy(), pose_fk.translation.copy())
    target_position = target_pose.translation.copy()
    target_rotation = target_pose.rotation.copy()
    print(f"目标位置: {target_position}")

    # 从随机初始位置求解
    q_init = np.random.uniform(-0.5, 0.5, model.nq)
    print(f"初始关节角度: {q_init}")

    q_ik, success = inverse_kinematics(model, data, target_pose, q_init=q_init, frame_name="link6")
    print(f"\nIK求解结果: {q_ik}")
    print(f"求解成功: {success}")

    # 验证IK结果
    pose_verify = forward_kinematics(model, data, q_ik, frame_name="link6")
    print(f"\n验证 - FK(IK结果)位置: {pose_verify.translation}")
    print(f"目标位置 (保存的副本): {target_position}")
    print(f"位置误差: {np.linalg.norm(pose_verify.translation - target_position):.6e}")
    print(f"姿态误差: {np.linalg.norm(pose_verify.rotation - target_rotation):.6e}")

