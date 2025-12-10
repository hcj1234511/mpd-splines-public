# import torch
# import numpy as np
# import matplotlib.pyplot as plt

# # 1. 读取单次规划结果
# data = torch.load("logs/author_EnvWarehouse-RobotPanda/2/results_single_plan-000.pt", map_location="cpu")

# # 2. 拿到 best 轨迹（注意：看你是 dict 还是 DotMap）
# try:
#     q_best = data["q_trajs_pos_best"]
# except TypeError:
#     q_best = data.q_trajs_pos_best

# # 3. 转成 numpy，形状 [T, 7]
# q_best = q_best.cpu().numpy()
# print("q_best shape:", q_best.shape)  # 应该是 (T, 7)

# T, dof = q_best.shape
# assert dof == 7, f"期望 7 维关节，但得到 {dof}"

# # 时间步索引
# t = np.arange(T)

# # 4. 画 7 个子图（一个关节一张图）
# fig, axes = plt.subplots(7, 1, figsize=(8, 12), sharex=True)

# joint_names = [f"Joint {i+1}" for i in range(7)]

# for j in range(7):
#     ax = axes[j]
#     ax.plot(t, q_best[:, j])
#     ax.set_ylabel(joint_names[j])
#     ax.grid(True, linestyle="--", alpha=0.3)

# axes[-1].set_xlabel("Time step")

# plt.tight_layout()
# plt.savefig("logs/2/best_trajectory.png")
# plt.show()



# # 绘制2D轨迹 + 最佳速度/加速度
# import torch
# import matplotlib.pyplot as plt

# path = "logs/author_EnvNarrowPassageDense2D/2/results_single_plan-000.pt"
# data = torch.load(path, map_location="cpu")

# # 位置轨迹
# q_valid = data["q_trajs_pos_valid"]      # [N_valid, T, D]
# q_best  = data["q_trajs_pos_best"]       # [T, D]

# # 最佳速度、加速度
# t      = data["timesteps"]              # [T]
# v_best = data["q_trajs_vel_best"]       # [T, D]
# a_best = data["q_trajs_acc_best"]       # [T, D]

# # 转成 numpy
# q_valid_np = q_valid.numpy()
# q_best_np  = q_best.numpy()
# t_np       = t.numpy()
# v_best_np  = v_best.numpy()
# a_best_np  = a_best.numpy()

# D = q_best_np.shape[1]   # 维度，2D pointmass 就是 2

# # ------------------ 1. 工作空间中的所有轨迹 + 最优轨迹 ------------------
# plt.figure(figsize=(6, 6))
# for traj in q_valid_np:
#     plt.plot(traj[:, 0], traj[:, 1], linewidth=0.5, alpha=0.3)

# plt.plot(q_best_np[:, 0], q_best_np[:, 1], linewidth=2.0)

# plt.xlabel("x")
# plt.ylabel("y")
# plt.title("author:All valid trajectories (thin) and best trajectory (bold)")
# plt.axis("equal")
# plt.grid(True)
# plt.savefig("logs/author_EnvNarrowPassageDense2D/2/all_valid_trajectories_and_best_trajectory.png")
# plt.show()

# # 维度名字映射：0 -> x, 1 -> y
# dim_names = ["x", "y"]

# # ------------------ 2. 最优轨迹的速度（每个维度一条曲线） ------------------
# plt.figure(figsize=(7, 4))
# for d in range(D):
#     name = dim_names[d] if d < len(dim_names) else f"dim {d}"
#     plt.plot(t_np, v_best_np[:, d], label=name)

# plt.xlabel("time")
# plt.ylabel("velocity")
# plt.title("author:Best trajectory velocity")
# plt.grid(True)
# plt.legend()
# plt.tight_layout()
# plt.savefig("logs/author_EnvNarrowPassageDense2D/2/best_trajectory_velocity.png")
# plt.show()

# # ------------------ 3. 最优轨迹的加速度（每个维度一条曲线） ------------------
# plt.figure(figsize=(7, 4))
# for d in range(D):
#     name = dim_names[d] if d < len(dim_names) else f"dim {d}"
#     plt.plot(t_np, a_best_np[:, d], label=name)

# plt.xlabel("time")
# plt.ylabel("acceleration")
# plt.title("author:Best trajectory acceleration")
# plt.grid(True)
# plt.legend()
# plt.tight_layout()
# plt.savefig("logs/author_EnvNarrowPassageDense2D/2/best_trajectory_acceleration.png")
# plt.show()



