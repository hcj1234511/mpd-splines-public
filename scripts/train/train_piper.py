#!/usr/bin/env python
"""
Training script for Piper Robot with diffusion model
专用于 Piper 机械臂的扩散模型训练脚本
"""
import isaacgym  # Must import first

import os
import sys
import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import torch
import time
from torch.utils.data import DataLoader

from experiment_launcher import single_experiment_yaml, run_experiment
from mpd import trainer
from mpd.trainer.trainer import get_num_epochs
from mpd.models import UNET_DIM_MULTS, TemporalTransformer, TemporalUnet
from mpd.models.diffusion_models.context_models import ContextModelQs, ContextModelEEPoseGoal, ContextModelCombined
from mpd.datasets.trajectories_dataset_bspline import adjust_bspline_number_control_points
from mpd.utils.loaders import get_model, get_loss, get_summary, load_params_from_yaml
from torch_robotics import environments, robots
from torch_robotics.tasks.tasks import PlanningTask
from mpd.parametric_trajectory.trajectory_bspline import ParametricTrajectoryBspline
from torch_robotics.torch_utils.seed import fix_random_seed
from torch_robotics.torch_utils.torch_utils import get_torch_device
from mpd.datasets.trajectories_dataset_bspline import TrajectoryDatasetBspline
import math
from sklearn.model_selection import train_test_split
import numpy as np

os.environ["HDF5_USE_FILE_LOCKING"] = "FALSE"

os.environ["WANDB_API_KEY"] = "999"
WANDB_MODE = "disabled"
WANDB_ENTITY = "mpd-splines"
DEBUG = False


@single_experiment_yaml
def experiment(
    ########################################################################
    # Dataset - Piper specific
    dataset_dir: str = "/home/hong/Projects/MotionPlanningDiffusion/mpd-splines-public/data_trajectories_hcj/piper_100000",
    dataset_file_merged: str = "dataset_merged_processed.hdf5",
    reload_data: bool = False,
    preload_data_to_device: bool = True,
    n_task_samples: int = -1,  # -1 for all
    ########################################################################
    # Parametric trajectory
    parametric_trajectory_class: str = "ParametricTrajectoryBspline",
    bspline_degree: int = 5,
    bspline_num_control_points_desired: int = 22,  # adjusted such that trainable control points are a multiple of 8
    num_T_pts: int = 256,  # number of time steps for trajectory interpolation (Piper data uses 256)
    ########################################################################
    # Context model
    context_qs: bool = True,
    context_qs_n_layers: int = 2,
    context_q_out_dim: int = 128,
    context_qs_act: str = "relu",
    # End-effector pose conditioned model
    context_ee_goal_pose: bool = False,
    context_ee_goal_pose_n_layers: int = 2,
    context_ee_goal_pose_out_dim: int = 128,
    context_ee_goal_pose_act: str = "relu",
    # Combined context model
    context_combined_out_dim: int = 128,
    ########################################################################
    # Generative prior model
    generative_model_class: str = "GaussianDiffusionModel",
    # Diffusion Model
    variance_schedule: str = "cosine",
    n_diffusion_steps: int = 100,
    predict_epsilon: bool = True,
    conditioning_type: str = "default",
    # Unet
    unet_input_dim: int = 32,
    unet_dim_mults_option: int = 1,
    # CVAE
    cvae_latent_dim: int = 32,
    loss_cvae_kl_weight: float = 1e-1,
    ########################################################################
    # Training parameters
    batch_size: int = 128,
    lr: float = 3e-4,
    clip_grad: bool = False,
    num_train_steps: int = 100_000,
    use_ema: bool = True,
    use_amp: bool = False,
    # Summary parameters
    steps_til_summary: int = 50 if DEBUG else 1000000,  # Skip summary for now (collision detection issue with reduced DOF)
    summary_class: str = "SummaryTrajectoryGeneration",
    steps_til_ckpt: int = 5000 if DEBUG else 20000,
    ########################################################################
    device: str = "cuda:0",
    debug: bool = DEBUG,
    ########################################################################
    # MANDATORY
    seed: int = 1,
    results_dir: str = "logs_piper",
    ########################################################################
    # WandB
    wandb_mode: str = "disabled" if DEBUG else WANDB_MODE,
    wandb_entity: str = WANDB_ENTITY,
    wandb_project: str = "train_piper_diffusion",
    **kwargs,
):
    print()
    print("-" * 100)
    print(f"Training Piper Robot with Diffusion Model")
    print(f"Dataset: {dataset_dir}")
    print(f"Parametric Trajectory: {parametric_trajectory_class}")
    print("-" * 100)
    print()

    # Set random seed for reproducibility
    fix_random_seed(seed)

    device = get_torch_device(device=device)
    tensor_args = {"device": device, "dtype": torch.float32}

    ########################################################################
    # Planning task and dataset
    print("\n--------------- Loading data")
    
    # Load args from the dataset directory
    base_dir = dataset_dir
    dataset_args = load_params_from_yaml(os.path.join(base_dir, "args.yaml"))
    
    # Environment
    env_class = getattr(environments, dataset_args["env_id"])
    env = env_class(**kwargs, tensor_args=tensor_args)

    # Robot
    robot_class = getattr(robots, dataset_args["robot_id"])
    robot = robot_class(**kwargs, tensor_args=tensor_args)

    # Task
    dataset_args["obstacle_cutoff_margin"] = dataset_args.get("min_distance_robot_env", 0.0)
    dataset_args["margin_for_dense_collision_checking"] = 0.0

    # Create a parametric trajectory
    bspline_n_control_points, bspline_n_removed_control_points = adjust_bspline_number_control_points(
        bspline_num_control_points_desired,
        context_qs,
        context_ee_goal_pose,
        True,  # zero_vel_at_start_and_goal
        True,  # zero_acc_at_start_and_goal
    )
    
    print(f"--------------- Parametric trajectory -- {parametric_trajectory_class}")
    print(
        f"Number of B-spline control points.\n"
        f"\tdesired          : {bspline_num_control_points_desired}\n"
        f"\tadjusted         : {bspline_n_control_points}\n"
        f"\tlearnable + fixed: {bspline_n_control_points - bspline_n_removed_control_points} + {bspline_n_removed_control_points}\n"
    )

    parametric_trajectory = ParametricTrajectoryBspline(
        n_control_points=bspline_n_control_points,
        degree=bspline_degree,
        zero_vel_at_start_and_goal=True,
        zero_acc_at_start_and_goal=True,
        num_T_pts=num_T_pts,
        remove_outer_control_points=context_qs,
        keep_last_control_point=context_ee_goal_pose,
        trajectory_duration=5.0,
        phase_time_class="PhaseTimeLinear",
        phase_time_args={},
        tensor_args=tensor_args,
    )

    planning_task = PlanningTask(
        env=env,
        robot=robot,
        parametric_trajectory=parametric_trajectory,
        **dataset_args,
        tensor_args=tensor_args,
    )

    # Load dataset
    full_dataset = TrajectoryDatasetBspline(
        planning_task=planning_task,
        base_dir=base_dir,
        dataset_file_merged=dataset_file_merged,
        preload_data_to_device=preload_data_to_device,
        context_qs=context_qs,
        context_ee_goal_pose=context_ee_goal_pose,
        tensor_args=tensor_args,
        **kwargs,
    )
    print(full_dataset)

    # Split into train and validation
    task_ids = list(full_dataset.map_task_id_to_control_points_id.keys())
    val_set_size = math.ceil(len(task_ids) * 0.025)
    train_subset_task_indices, val_subset_task_indices = train_test_split(task_ids, test_size=val_set_size)
    train_subset_indices = np.concatenate(
        [
            full_dataset.map_task_id_to_control_points_id[train_subset_task_idx]
            for train_subset_task_idx in train_subset_task_indices
        ]
    )
    val_subset_indices = np.concatenate(
        [
            full_dataset.map_task_id_to_control_points_id[val_subset_task_idx]
            for val_subset_task_idx in val_subset_task_indices
        ]
    )
    
    train_subset = torch.utils.data.Subset(full_dataset, train_subset_indices)
    val_subset = torch.utils.data.Subset(full_dataset, val_subset_indices)

    print(f"train_subset size: {len(train_subset.indices)}")
    print(f"val_subset size  : {len(val_subset.indices)}")
    
    train_dataloader = DataLoader(
        train_subset, batch_size=batch_size, shuffle=True, drop_last=False
    )
    val_dataloader = DataLoader(val_subset, batch_size=batch_size, shuffle=True, drop_last=False)

    ########################################################################
    # Model
    context_model_qs = None
    if context_qs:
        context_model_qs = ContextModelQs(
            in_dim=full_dataset.context_q_dim,
            out_dim=context_q_out_dim,
            n_layers=context_qs_n_layers,
            act=context_qs_act,
        )

    context_model_ee_pose_goal = None
    if context_ee_goal_pose:
        context_model_ee_pose_goal = ContextModelEEPoseGoal(
            out_dim=context_ee_goal_pose_out_dim,
            n_layers=context_ee_goal_pose_n_layers,
            act=context_ee_goal_pose_act,
        )

    context_model = None
    if not (context_model_qs is None and context_model_ee_pose_goal is None):
        context_model = ContextModelCombined(
            context_model_qs=context_model_qs,
            context_model_ee_pose_goal=context_model_ee_pose_goal,
            out_dim=context_combined_out_dim,
        )

    diffusion_configs = dict(
        variance_schedule=variance_schedule,
        n_diffusion_steps=n_diffusion_steps,
        predict_epsilon=predict_epsilon,
    )

    cvae_configs = dict(
        cvae_latent_dim=cvae_latent_dim,
    )

    unet_configs = dict(
        state_dim=full_dataset.state_dim,
        n_support_points=full_dataset.n_learnable_control_points,
        unet_input_dim=unet_input_dim,
        dim_mults=UNET_DIM_MULTS[unet_dim_mults_option],
        conditioning_type=conditioning_type if context_model is not None else "None",
        conditioning_embed_dim=context_model.out_dim if context_model is not None else None,
    )

    print(f"\n--------------- Model Configuration")
    print(f"state_dim (joints): {full_dataset.state_dim}")
    print(f"n_support_points (control points): {full_dataset.n_learnable_control_points}")
    print(f"Input shape: [batch, {full_dataset.n_learnable_control_points}, {full_dataset.state_dim}]")

    model = get_model(
        model_class=generative_model_class,
        denoise_fn=TemporalUnet(**unet_configs),
        context_model=context_model,
        tensor_args=tensor_args,
        **cvae_configs,
        **diffusion_configs,
        **unet_configs,
    )

    ########################################################################
    # Loss
    if generative_model_class == "GaussianDiffusionModel":
        loss_class = "GaussianDiffusionLoss"
    elif generative_model_class == "CVAEModel":
        loss_class = "CVAELoss"
    else:
        raise ValueError(f"Unknown generative_model_class: {generative_model_class}")

    loss_fn = val_loss_fn = get_loss(loss_class=loss_class, loss_cvae_kl_weight=loss_cvae_kl_weight)

    ########################################################################
    # Summary - Skip for now due to collision detection issues with reduced DOF
    summary_fn = None  # get_summary(summary_class=summary_class, debug=debug)

    ########################################################################
    # Train
    print(f"\n--------------- Training")
    epochs = get_num_epochs(num_train_steps, batch_size, len(train_subset))
    trainer.train(
        model=model,
        train_dataloader=train_dataloader,
        train_subset=train_subset,
        val_dataloader=val_dataloader,
        val_subset=val_subset,
        planning_task=planning_task,
        epochs=epochs,
        num_train_steps=num_train_steps,
        loss_fn=loss_fn,
        val_loss_fn=val_loss_fn,
        summary_fn=summary_fn,
        lr=lr,
        steps_til_summary=steps_til_summary,
        steps_til_checkpoint=steps_til_ckpt,
        model_dir=results_dir,
        clip_grad=clip_grad,
        use_ema=use_ema,
        use_amp=use_amp,
        tensor_args=tensor_args,
    )

    print("\n" + "=" * 100)
    print("Training completed!")
    print(f"Results saved to: {results_dir}")
    print("=" * 100)


if __name__ == "__main__":
    run_experiment(experiment)
