"""
在 Isaac Gym 中可视化 Piper 机械臂轨迹
"""
import sys
from pathlib import Path

# 添加项目根目录到 Python 路径
project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))

# IMPORTANT: Isaac Gym 必须在 torch 之前导入！
import isaacgym  # noqa: F401

import torch
import h5py

from experiment_launcher.utils import fix_random_seed
from torch_robotics.torch_utils.torch_utils import get_torch_device
from torch_robotics.environments import EnvWarehouse, EnvSpheres3D
from torch_robotics.robots import RobotPiper
from torch_robotics.torch_kinematics_tree.utils.files import get_robot_path
from mpd.parametric_trajectory.trajectory_bspline import ParametricTrajectoryBspline
from mpd.torch_robotics.torch_robotics.isaac_gym_envs.motion_planning_envs import (
    MotionPlanningIsaacGymEnv,
    MotionPlanningControllerIsaacGym,
)


seed = 1
fix_random_seed(seed)

device = get_torch_device()
tensor_args = {"device": "cpu", "dtype": torch.float32}

# ---------------------------- Environment, Robot, PlanningTask ---------------------------------
# env = EnvSpheres3D(tensor_args=tensor_args)

# 注意：rotation_z_axis_deg 必须与数据生成时一致！
# 数据生成时使用默认值 0，所以这里也要用 0
env = EnvWarehouse(rotation_z_axis_deg=0, tensor_args=tensor_args)

# gripper=False: 6 DOF arm, gripper_collision=True: include fixed gripper for collision
robot = RobotPiper(gripper=False, gripper_collision=True, tensor_args=tensor_args)

parametric_trajectory = ParametricTrajectoryBspline(
    n_control_points=18,
    degree=5,
    num_T_pts=128,
    zero_vel_at_start_and_goal=True,
    zero_acc_at_start_and_goal=True,
    remove_outer_control_points=False,
    keep_last_control_point=False,
    trajectory_duration=5.0,
    tensor_args=tensor_args,
)

# -------------------------------- Physics --------------------------------
draw_collision_spheres = False
# robot_asset_file = robot.robot_urdf_collision_spheres_file if draw_collision_spheres else robot.robot_urdf_file
num_envs = 5
motion_planning_isaac_env = MotionPlanningIsaacGymEnv(
    env,
    robot,
    asset_root=get_robot_path().as_posix(),
    # robot_asset_file=robot_asset_file.replace(get_robot_path().as_posix() + "/", ""),
    # 使用与数据生成相同的 URDF（6 DOF + 固定夹爪用于碰撞检测）
    robot_asset_file="piper_description/urdf/piper_description_gripper_fixed.urdf",
    # robot_asset_file="piper_description/urdf/piper_description_no_gripper.urdf",
    controller_type="position",
    num_envs=num_envs,
    all_robots_in_one_env=True,
    show_viewer=True,
    sync_viewer_with_real_time=False,
    viewer_time_between_steps=0.01,
    render_camera_global=True,
    render_camera_global_append_to_recorder=False,
    color_robots=False,
    collor_robots_in_collision=True,
    draw_goal_configuration=False,
    draw_contact_forces=False,
    draw_end_effector_path=True,    # 显示末端轨迹
    draw_end_effector_frame=False,   # 显示末端坐标系
    draw_ee_pose_goal=False,
)

# motion_planning_isaac_env.ee_pose_goal = torch.tensor([0.5, 0, 0.5, 0, 0, 0, 1], **tensor_args)

motion_planning_controller = MotionPlanningControllerIsaacGym(motion_planning_isaac_env)

# while not motion_planning_isaac_env.gym.query_viewer_has_closed(motion_planning_isaac_env.viewer):
# # 如果你暂时不想让它动，下面这句可以不更新，只保持 initial_q
# # gym.set_actor_dof_position_targets(envs[0], piper_handles[0], initial_q)
#     motion_planning_isaac_env.gym.simulate(motion_planning_isaac_env.sim)
#     motion_planning_isaac_env.gym.fetch_results(motion_planning_isaac_env.sim, True)

#     motion_planning_isaac_env.gym.step_graphics(motion_planning_isaac_env.sim)
#     motion_planning_isaac_env.gym.draw_viewer(motion_planning_isaac_env.viewer, motion_planning_isaac_env.sim, True)

#     motion_planning_isaac_env.gym.sync_frame_time(motion_planning_isaac_env.sim)

# motion_planning_isaac_env.gym.destroy_viewer(motion_planning_isaac_env.viewer)
# motion_planning_isaac_env.gym.destroy_sim(motion_planning_isaac_env.sim)

# Set all joint positions to 0
initial_joint_pos = torch.zeros(robot.arm_q_dim, **tensor_args)
trajectories_joint_pos = initial_joint_pos.repeat(100, num_envs, 1)

# file_path = "joint_states_2025-11-09_11-51-44.csv"
# df = pd.read_csv(file_path)
# trajectories_np = df.iloc[1:429, 1:7].to_numpy() #shape (428, 6)

# 从训练数据中读取一条执行
root_dir = Path('/home/hong/Projects/MotionPlanningDiffusion/mpd-splines-public')

path = root_dir / 'scripts' / 'Hcj' / 'test_data' / 'dataset.hdf5'
with h5py.File(path, 'r') as f:
    sol_path = f['sol_path']
    trajectories_np = sol_path[8]
    print('---------------------------',trajectories_np)
# trajectories_np = trajectories_np[:, :6]
# print('---------------------------',trajectories_np.shape)

# # 执行推理轨迹
# path = "/home/hong/Projects/MotionPlanningDiffusion/mpd-splines-public/scripts/inference/logs_inference_piper/1/results_single_plan-000.pt"
# data = torch.load(path, map_location="cpu")  # 在 mpd 环境里 dotmap 已经有了
# # 最优关节轨迹（位置）
# trajectories_np = data["q_trajs_pos_best"]   # Tensor [T, D]，例如 [256, 6]

# 3. 转成 torch 张量，并 reshape 成 (428, 1, 6)
trajectories_tensor = torch.tensor(trajectories_np, dtype=torch.float32)  # (428, 6)
trajectories_tensor = trajectories_tensor.unsqueeze(1)                    # (428, 1, 6)
print(trajectories_tensor[-1][0])

# 执行轨迹并获取统计信息
statistics = motion_planning_controller.execute_trajectories(
    trajectories_tensor,
    q_pos_starts=trajectories_tensor[0],
    q_pos_goal=trajectories_tensor[-1][0],
    n_pre_steps=60,
    n_post_steps=200,
    make_video=False,
    video_duration=5.0,
    make_gif=False,
    stop_robot_if_in_contact=True,
)

# 打印执行结果
print("\n" + "=" * 60)
print("轨迹执行结果统计")
print("=" * 60)
print(f"总轨迹数: {statistics.n_trajectories_collision + statistics.n_trajectories_free}")
print(f"碰撞轨迹数: {statistics.n_trajectories_collision}")
print(f"无碰撞轨迹数: {statistics.n_trajectories_free}")
print(f"无碰撞比例: {statistics.n_trajectories_free_fraction * 100:.1f}%")

if statistics.n_trajectories_collision > 0:
    print("\n⚠️  轨迹因碰撞而停止!")
else:
    print("\n✅ 轨迹成功到达目标点!")
print("=" * 60)
