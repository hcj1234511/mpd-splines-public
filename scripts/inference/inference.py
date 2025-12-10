from mpd.utils.patches import numpy_monkey_patch
numpy_monkey_patch()

import time
from functools import partial

import isaacgym


from dotmap import DotMap

import gc
import os
from pprint import pprint

import numpy as np
import torch
from einops._torch_specific import allow_ops_in_compiled_graph  # requires einops>=0.6.1

from experiment_launcher import single_experiment_yaml, run_experiment
from mpd.inference.inference import EvaluationSamplesGenerator, GenerativeOptimizationPlanner, render_results
from mpd.metrics.metrics import PlanningMetricsCalculator
from mpd.utils.loaders import get_planning_task_and_dataset, load_params_from_yaml, save_to_yaml
from torch_robotics.isaac_gym_envs.motion_planning_envs import (
    MotionPlanningIsaacGymEnv,
    MotionPlanningControllerIsaacGym,
)
from torch_robotics.robots import RobotPanda
from torch_robotics.torch_kinematics_tree.utils.files import get_robot_path
from torch_robotics.torch_utils.seed import fix_random_seed
from torch_robotics.torch_utils.torch_utils import get_torch_device, to_torch, to_numpy

allow_ops_in_compiled_graph()


def pad_q_for_robot(q, robot):
    """Pad joint positions to match robot DOF for FK/Isaac Gym.
    E.g., pad 6-DOF Piper data to 8-DOF for FK with gripper joints."""
    robot_dof = len(robot.q_pos_min)
    data_dim = q.shape[-1]
    if data_dim < robot_dof:
        padding = torch.zeros(*q.shape[:-1], robot_dof - data_dim,
                              device=q.device, dtype=q.dtype)
        q = torch.cat([q, padding], dim=-1)
    return q


def pad_q_to_8dof(q):
    """Pad joint positions to 8-DOF for Isaac Gym (Piper robot).
    Always pads to 8 dimensions regardless of robot definition."""
    target_dof = 8
    data_dim = q.shape[-1]
    if data_dim < target_dof:
        padding = torch.zeros(*q.shape[:-1], target_dof - data_dim,
                              device=q.device, dtype=q.dtype)
        q = torch.cat([q, padding], dim=-1)
    return q


@single_experiment_yaml
def experiment(
    ########################################################################
    # Configuration path defining the model and the inference parameters
    # cfg_inference_path: str = './cfgs/config_EnvNarrowPassageDense2D-RobotPointMass2D_00.yaml',
    # cfg_inference_path: str = './cfgs/config_EnvPlanar2Link-RobotPlanar2Link_00.yaml',
    # cfg_inference_path: str = './cfgs/config_EnvPlanar4Link-RobotPlanar4Link_00.yaml',
    # cfg_inference_path: str = './cfgs/config_EnvSimple2D-RobotPointMass2D_00.yaml',
    # cfg_inference_path: str = './cfgs/config_EnvSpheres3D-RobotPanda_00.yaml',
    cfg_inference_path: str = "./cfgs/config_EnvWarehouse-RobotPanda-config_file_v01_00.yaml",
    ########################################################################
    # Select the start and goal from the training or validation/test set.
    selection_start_goal: str = "validation",  # training, validation/test, custom
    ########################################################################
    # number of start and goal states to evaluate
    n_start_goal_states: int = 1,
    ########################################################################
    # Custom start and goal joint positions (comma-separated, e.g., "0,1.57,-1.5,0,0,0")
    # Only used when selection_start_goal="custom"
    custom_q_start: str = "",  # e.g., "0,1.57,-1.5,0,0,0"
    custom_q_goal: str = "",   # e.g., "0.5,1.0,-1.0,0.3,0.2,0"
    ########################################################################
    save_results_single_plan_low_mem: bool = False,
    ########################################################################
    # Visualization options
    render_joint_space_time_iters: bool = True,
    render_joint_space_env_iters: bool = False,
    render_env_robot_opt_iters: bool = False,
    render_env_robot_trajectories: bool = False,
    render_pybullet: bool = False,
    draw_collision_spheres: bool = False,
    run_evaluation_issac_gym: bool = True,  #isaacgym可视化
    render_isaacgym_viewer: bool = True,
    render_isaacgym_movie: bool = True,
    ########################################################################
    device: str = "cuda:0",  # cpu, cuda
    debug: bool = False,
    ########################################################################
    # MANDATORY
    # seed: int = int(time.time()),
    seed: int = 1,
    results_dir: str = "logs",
    ########################################################################
    **kwargs,
):
    # Set random seed for reproducibility
    fix_random_seed(seed)

    device = get_torch_device(device)
    tensor_args = {"device": device, "dtype": torch.float32}

    # Save and load the inference configuration
    args_inference = DotMap(load_params_from_yaml(cfg_inference_path))

    if "cvae" in args_inference.planner_alg:
        if args_inference.model_selection == "bspline":
            args_inference.model_dir = args_inference.model_dir_cvae_bspline
        elif args_inference.model_selection == "waypoints":
            args_inference.model_dir = args_inference.model_dir_cvae_waypoints
        else:
            raise NotImplementedError
    else:
        if args_inference.model_selection == "bspline":
            args_inference.model_dir = args_inference.model_dir_ddpm_bspline
        elif args_inference.model_selection == "waypoints":
            args_inference.model_dir = args_inference.model_dir_ddpm_waypoints
        else:
            raise NotImplementedError

    args_inference.model_dir = os.path.expandvars(args_inference.model_dir)

    save_to_yaml(args_inference.toDict(), os.path.join(results_dir, "args_inference.yaml"))

    print(f"\n-------------------------------------------------------------------------------------------------")
    print(f"cfg_inference_path:\n{cfg_inference_path}")
    print(f"Model:\n{args_inference.model_dir}")
    print(f"--------------------------------------------------------------------------------------------------")

    ################################################################################################################
    # Load dataset, environment, robot and planning task.
    # Override training parameters.
    args_train = DotMap(load_params_from_yaml(os.path.join(args_inference.model_dir, "args.yaml")))
    
    # Check if train_subset_indices.pt exists, if not, set load_indices=False
    train_indices_path = os.path.join(args_inference.model_dir, "train_subset_indices.pt")
    load_indices = os.path.exists(train_indices_path)
    if not load_indices:
        print(f"Warning: {train_indices_path} not found. Regenerating train/val split.")
    
    args_train.update(
        **args_inference,
        gripper=True,
        reload_data=False,
        results_dir=results_dir,
        load_indices=load_indices,
        tensor_args=tensor_args,
    )
    planning_task, train_subset, _, val_subset, _ = get_planning_task_and_dataset(**args_train)

    ################################################################################################################
    # Generator of evaluation samples
    evaluation_samples_generator = EvaluationSamplesGenerator(
        planning_task,
        train_subset,
        val_subset,
        selection_start_goal=selection_start_goal,
        planner="RRTConnect",
        tensor_args=tensor_args,
        debug=debug,
        render_pybullet=render_pybullet,
        **args_inference,
    )

    ################################################################################################################
    # Load the generative model planner
    generative_optimization_planner = GenerativeOptimizationPlanner(
        planning_task,
        train_subset.dataset,
        args_train,
        args_inference,
        tensor_args,
        sampling_based_planner_fn=partial(
            evaluation_samples_generator.generate_data_ompl_worker.run,
            planner_allowed_time=10.0,
            interpolate_num=args_inference.num_T_pts,
            simplify_path=True,
        ),
        debug=debug,
    )

    ################################################################################################################
    # IsaacGym environment and motion planning controller
    motion_planning_isaac_env = None
    if run_evaluation_issac_gym:
        robot_asset_file = planning_task.robot.robot_urdf_file
        if draw_collision_spheres:
            robot_asset_file = planning_task.robot.robot_urdf_collision_spheres_file
        motion_planning_isaac_env = MotionPlanningIsaacGymEnv(
            planning_task.env,
            planning_task.robot,
            asset_root=get_robot_path().as_posix(),
            robot_asset_file=robot_asset_file.replace(get_robot_path().as_posix() + "/", ""),
            num_envs=args_inference.n_trajectory_samples,
            # all_robots_in_one_env=True if n_start_goal_states == 1 else False,
            all_robots_in_one_env=True,
            render_isaacgym_viewer=render_isaacgym_viewer,
            render_camera_global=render_isaacgym_movie,
            render_camera_global_append_to_recorder=render_isaacgym_movie,
            sync_viewer_with_real_time=False,
            show_viewer=render_isaacgym_viewer,
            camera_global_from_top=True if planning_task.env.dim == 2 else False,
            add_ground_plane=False,
            viewer_time_between_steps=torch.diff(planning_task.parametric_trajectory.get_timesteps()[:2]).item(),
            draw_goal_configuration=True if not train_subset.dataset.context_ee_goal_pose else False,
            draw_ee_pose_goal=True if train_subset.dataset.context_ee_goal_pose else False,
            color_robots=False,
            draw_contact_forces=False,
            draw_end_effector_frame=False,
            draw_end_effector_path=True,
        )

        motion_planning_controller_isaac_gym = MotionPlanningControllerIsaacGym(motion_planning_isaac_env)

    ################################################################################################################
    # Metrics calculator
    planning_metrics_calculator = PlanningMetricsCalculator(planning_task)

    dt_trajectory = float(torch.diff(planning_task.parametric_trajectory.get_timesteps()[:2]).item())
    ################################################################################################################
    # Plan for several start and goal states sequentially
    use_custom_start_goal = selection_start_goal == "custom" and custom_q_start and custom_q_goal
    
    if use_custom_start_goal:
        # Parse custom start and goal positions
        # Clean up the string: remove quotes and extra spaces
        clean_q_start = custom_q_start.strip().strip('"').strip("'")
        clean_q_goal = custom_q_goal.strip().strip('"').strip("'")
        q_pos_start_custom = torch.tensor([float(x.strip()) for x in clean_q_start.split(',')], **tensor_args)
        q_pos_goal_custom = torch.tensor([float(x.strip()) for x in clean_q_goal.split(',')], **tensor_args)
        print(f"\n使用自定义起点和终点:")
        print(f"  起点 (q_start): {q_pos_start_custom.tolist()}")
        print(f"  终点 (q_goal):  {q_pos_goal_custom.tolist()}")
        n_start_goal_states = 1  # Only one planning when using custom
        idx_sample_l = [0]
    elif selection_start_goal == "training":
        idx_sample_l = np.random.choice(np.arange(len(train_subset)), n_start_goal_states)
    else:
        idx_sample_l = np.random.choice(np.arange(len(val_subset)), n_start_goal_states)
    
    for idx_sg, idx_sample in enumerate(idx_sample_l):
        print(f"\n-------------------------------------------------------------------------------------------------")
        print(f"----------------PLANNING {idx_sg+1}/{n_start_goal_states}------------------")
        print(f"--------------------------------------------------------------------------------------------------")

        results_single_plan = DotMap(t_generator=0.0, t_guide=0.0)

        if use_custom_start_goal:
            q_pos_start = q_pos_start_custom
            q_pos_goal = q_pos_goal_custom
            # Compute ee_pose_goal from q_pos_goal using FK
            q_goal_padded = pad_q_to_8dof(q_pos_goal.unsqueeze(0))
            ee_pose_goal = planning_task.robot.get_EE_pose(q_goal_padded).squeeze(0)
        else:
            q_pos_start, q_pos_goal, ee_pose_goal = evaluation_samples_generator.get_data_sample(idx_sg)

        print("\n----------------START AND GOAL states----------------")
        print(f"q_pos_start: {q_pos_start}")
        print(f"q_pos_goal: {q_pos_goal}")
        print(f"ee_pose_goal: {ee_pose_goal}")

        if debug:
            evaluation_samples_generator.add_start_goal_marker(q_pos_start, q_pos_goal)

        ############################################################################################################
        # Run motion planning inference
        print(f"\n----------------PLAN TRAJECTORIES----------------")
        print(f"Starting inference...")
        results_single_plan = generative_optimization_planner.plan_trajectory(
            q_pos_start, q_pos_goal, ee_pose_goal, results_ns=results_single_plan, debug=debug
        )
        print(f"...inference finished.")

        ############################################################################################################
        # Show in pybullet the best trajectory
        if render_pybullet and results_single_plan.q_trajs_pos_best is not None:
            time.sleep(3)
            ########################
            # Visualize in Pybullet
            q_pos_path = to_numpy(results_single_plan.q_trajs_pos_best)
            # add panda grippers to the path
            if (
                isinstance(planning_task.robot, RobotPanda)
                and q_pos_path.shape[1] == 7
                and evaluation_samples_generator.generate_data_ompl_worker.pbompl_interface.robot.num_dim == 9
            ):
                q_pos_path = np.concatenate((q_pos_path, np.zeros((q_pos_path.shape[0], 2))), axis=-1)
                evaluation_samples_generator.generate_data_ompl_worker.pbompl_interface.execute(
                q_pos_path,
                sleep_time=dt_trajectory,   # 用我们算好的 dt
                )

        ############################################################################################################
        # Evaluate and show in IsaacGym
        isaacgym_statistics = None
        if run_evaluation_issac_gym and results_single_plan.q_trajs_pos_valid is not None:
            ########################
            # Pad q_pos_goal to 8-DOF for FK (Piper: 6 -> 8)
            q_pos_goal_tensor = to_torch(q_pos_goal, device) if not isinstance(q_pos_goal, torch.Tensor) else q_pos_goal.to(device)
            q_pos_goal_padded = pad_q_to_8dof(q_pos_goal_tensor)
            motion_planning_isaac_env.ee_pose_goal = planning_task.robot.get_EE_pose(
                q_pos_goal_padded.unsqueeze(0), flatten_pos_quat=True, quat_xyzw=True
            ).squeeze(0)

            # # Execute all valid trajectories
            # if results_single_plan.q_trajs_pos_valid.shape[0] > 0:
            #     q_trajs_pos = results_single_plan.q_trajs_pos_valid.movedim(1, 0)  # horizon, batch, D
            #     isaacgym_statistics = motion_planning_controller_isaac_gym.execute_trajectories(
            #         q_trajs_pos,
            #         q_pos_starts=q_trajs_pos[0],
            #         q_pos_goal=q_trajs_pos[-1][0],  # add steps for better visualization
            #         n_pre_steps=5 if render_isaacgym_viewer or render_isaacgym_movie else 0,
            #         n_post_steps=5 if render_isaacgym_viewer or render_isaacgym_movie else 0,
            #         stop_robot_if_in_contact=False,
            #         make_video=render_isaacgym_movie,
            #         video_duration=args_inference.trajectory_duration,
            #         video_path=os.path.join(results_dir, f"isaacgym-{idx_sg:03d}.mp4"),
            #         make_gif=False,
            #     )
            # results_single_plan.isaacgym_statistics = isaacgym_statistics
            # 只执行最优轨迹 q_trajs_pos_best
            q_best = results_single_plan.q_trajs_pos_best  # 通常形状 [T, 6] for Piper
            
            # Check if q_best is None (no valid trajectory found)
            if q_best is None:
                print("Warning: No best trajectory found (q_trajs_pos_best is None). Skipping Isaac Gym visualization.")
                isaacgym_statistics = None
            else:
                # Pad trajectory to 8-DOF for Isaac Gym (Piper robot: 6 -> 8, last 2 for gripper)
                q_best_padded = pad_q_to_8dof(q_best)
                print(f"Padded trajectory shape: {q_best.shape} -> {q_best_padded.shape}")

                # MotionPlanningControllerIsaacGym 期望的是 [T, batch, D]
                if q_best_padded.dim() == 2:
                    # [T, D] -> [T, 1, D]
                    q_trajs_pos = q_best_padded.unsqueeze(1)
                elif q_best_padded.dim() == 3:
                    # 如果已经是 [1, T, D] 之类，统一成 [T, 1, D]
                    q_trajs_pos = q_best_padded.movedim(1, 0)[:, :1, :]
                else:
                    raise ValueError(f"Unexpected shape for q_trajs_pos_best: {q_best.shape}")

                isaacgym_statistics = motion_planning_controller_isaac_gym.execute_trajectories(
                    q_trajs_pos,
                    q_pos_starts=q_trajs_pos[0],        # [batch, D]，这里 batch=1
                    q_pos_goal=q_trajs_pos[-1][0],      # 这一条轨迹的终点
                    n_pre_steps=5 if render_isaacgym_viewer or render_isaacgym_movie else 0,
                    n_post_steps=5 if render_isaacgym_viewer or render_isaacgym_movie else 0,
                    stop_robot_if_in_contact=False,
                    make_video=render_isaacgym_movie,
                    video_duration=args_inference.trajectory_duration,
                    video_path=os.path.join(results_dir, f"isaacgym-{idx_sg:03d}.mp4"),
                    make_gif=False,
                )

            results_single_plan.isaacgym_statistics = isaacgym_statistics

        ############################################################################################################
        # Compute motion planning metrics
        print(f"\n----------------METRICS----------------")
        results_single_plan.metrics = planning_metrics_calculator.compute_metrics(results_single_plan)

        print(f"t_inference_total: {results_single_plan.t_inference_total:.3f} sec")
        print(f"t_generator: {results_single_plan.t_generator:.3f} sec")
        print(f"t_guide: {results_single_plan.t_guide:.3f} sec")

        print(f"isaacgym_statistics:")
        pprint(results_single_plan.isaacgym_statistics)

        print(f"metrics:")
        pprint(results_single_plan.metrics)

        # Save data
        results_single_plan_to_save = results_single_plan
        if save_results_single_plan_low_mem:
            results_single_plan_to_save = DotMap(
                t_generator=results_single_plan.t_generator,
                t_guide=results_single_plan.t_guide,
                t_inference_total=results_single_plan.t_inference_total,
                q_pos_start=q_pos_start,
                q_pos_goal=q_pos_goal,
                ee_pose_goal=ee_pose_goal,
                control_points_iters=results_single_plan.control_points_iters,
                metrics=results_single_plan.metrics,
                isaacgym_statistics=results_single_plan.isaacgym_statistics,
            )
        torch.save(
            results_single_plan_to_save,
            os.path.join(results_dir, f"results_single_plan-{idx_sg:03d}.pt"),
            _use_new_zipfile_serialization=True,
        )

        ############################################################################################################
        # Render sampling results
        render_results(
            args_inference,
            planning_task,
            q_pos_start,
            q_pos_goal,
            results_single_plan,
            idx_sg,
            results_dir,
            render_joint_space_time_iters=render_joint_space_time_iters,
            render_joint_space_env_iters=render_joint_space_env_iters,
            render_planning_env_robot_opt_iters=render_env_robot_opt_iters,
            render_planning_env_robot_trajectories=render_env_robot_trajectories,
            debug=debug,
        )

        ############################################################################################################
        # empty memory
        del results_single_plan
        gc.collect()
        torch.cuda.empty_cache()

    ################################################################################################################
    # clean up
    evaluation_samples_generator.generate_data_ompl_worker.terminate()
    if motion_planning_isaac_env is not None:
        motion_planning_isaac_env.clean_up()
        del motion_planning_isaac_env
        del motion_planning_controller_isaac_gym
    gc.collect()
    torch.cuda.empty_cache()


if __name__ == "__main__":
    run_experiment(experiment)
