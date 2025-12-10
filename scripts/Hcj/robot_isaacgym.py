from isaacgym import gymapi
import math
import os
import numpy as np


# 1. 获取 gym handle
gym = gymapi.acquire_gym()

# 2. 创建仿真器（先用 CPU，确认没问题再开 GPU）
sim_params = gymapi.SimParams()
sim_params.dt = 1.0 / 60.0
sim_params.substeps = 2
sim_params.up_axis = gymapi.UP_AXIS_Z
sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.81)   # 保留重力

sim_params.physx.use_gpu = True
sim_params.use_gpu_pipeline = False

sim = gym.create_sim(0, 0, gymapi.SIM_PHYSX, sim_params)
if sim is None:
    raise Exception("Failed to create sim")

# 3. 加地面
plane_params = gymapi.PlaneParams()
plane_params.normal = gymapi.Vec3(0, 0, 1)
gym.add_ground(sim, plane_params)

# 4. 创建 viewer
viewer = gym.create_viewer(sim, gymapi.CameraProperties())
if viewer is None:
    raise Exception("Failed to create viewer")

# 5. 加载机械臂 asset（URDF）
asset_root = "/home/hong/Projects/MotionPlanningDiffusion/mpd-splines-public/scripts"
asset_file = "Hcj/piper_description/urdf/piper_description.urdf"

asset_options = gymapi.AssetOptions()
asset_options.fix_base_link = True      # 基座固定在世界
asset_options.disable_gravity = False   # 链接仍受重力影响

piper_asset = gym.load_asset(sim, asset_root, asset_file, asset_options)

# *** 新增：查询 DOF 数量 ***
num_dofs = gym.get_asset_dof_count(piper_asset)
print("Piper DOF 数量:", num_dofs)


# 6. 创建 env，并在其中放一个机械臂 actor
envs = []
piper_handles = []

env_lower = gymapi.Vec3(-1.0, -1.0, 0.0)
env_upper = gymapi.Vec3(1.0, 1.0, 1.0)

num_envs = 1
for i in range(num_envs):
    env = gym.create_env(sim, env_lower, env_upper, num_envs)

    # 初始位姿
    pose = gymapi.Transform()
    pose.p = gymapi.Vec3(0.0, 0.0, 0.0)
    pose.r = gymapi.Quat.from_euler_zyx(0.0, 0.0, 0.0)

    piper_handle = gym.create_actor(env, piper_asset, pose, "piper", i, 1)

    
    # *** 新增：设置关节控制模式为位置控制，并加刚度 / 阻尼 ***
    dof_props = gym.get_actor_dof_properties(env, piper_handle)

    for j in range(num_dofs):
        # 有些 DOF 可能是固定的，可以加判断：如果 type 是 NONE 就跳过
        # if dof_props['dofType'][j] == gymapi.DOF_NONE: continue

        dof_props["driveMode"][j] = gymapi.DOF_MODE_POS   # 位置控制
        dof_props["stiffness"][j] = 400.0                 # 越大越“硬”
        dof_props["damping"][j] = 80.0                    # 阻尼，防抖

    gym.set_actor_dof_properties(env, piper_handle, dof_props)

    # *** 新增：给一个初始目标关节角（这里全部设为 0） ***
    initial_q = np.zeros(num_dofs, dtype=np.float32)
    gym.set_actor_dof_position_targets(env, piper_handle, initial_q)

    envs.append(env)
    piper_handles.append(piper_handle)

# 7. 把摄像机对着机械臂
cam_pos = gymapi.Vec3(1.5, 1.5, 1.0)
cam_target = gymapi.Vec3(0.0, 0.0, 0.5)
gym.viewer_camera_look_at(viewer, envs[0], cam_pos, cam_target)

# 8. 主循环
while not gym.query_viewer_has_closed(viewer):
    # 如果你暂时不想让它动，下面这句可以不更新，只保持 initial_q
    # gym.set_actor_dof_position_targets(envs[0], piper_handles[0], initial_q)

    gym.simulate(sim)
    gym.fetch_results(sim, True)

    gym.step_graphics(sim)
    gym.draw_viewer(viewer, sim, True)

    gym.sync_frame_time(sim)

gym.destroy_viewer(viewer)
gym.destroy_sim(sim)
