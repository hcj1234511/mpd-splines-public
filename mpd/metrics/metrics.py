import torch
from dotmap import DotMap

from torch_robotics.torch_utils.torch_utils import to_numpy
from torch_robotics.trajectory.metrics import (
    compute_trajectory_diversity,
    compute_path_length,
    compute_smoothness,
    compute_ee_pose_errors,
)


def pad_q_pos_for_fk(q_pos, robot):
    """Pad joint positions to match robot DOF for FK computation.
    E.g., pad 6-DOF Piper data to 8-DOF for FK with gripper joints."""
    robot_dof = len(robot.q_pos_min)
    data_dim = q_pos.shape[-1]
    if data_dim < robot_dof:
        padding = torch.zeros(*q_pos.shape[:-1], robot_dof - data_dim,
                              device=q_pos.device, dtype=q_pos.dtype)
        q_pos = torch.cat([q_pos, padding], dim=-1)
    return q_pos


class PlanningMetricsCalculator:

    def __init__(self, planning_task, **kwargs):
        self.planning_task = planning_task

    def compute_metrics(self, results_single_plan: DotMap, **kwargs):
        # All computations are done in valid trajectories (collision-free and within joint limits)
        metrics = DotMap()

        q_trajs_iter_0 = torch.cat(
            [
                results_single_plan.q_trajs_pos_iter_0,
                results_single_plan.q_trajs_vel_iter_0,
                results_single_plan.q_trajs_acc_iter_0,
            ],
            dim=-1,
        )
        q_trajs_pos_iter_0 = results_single_plan.q_trajs_pos_iter_0

        # Check if data dimension matches robot DOF (for collision detection)
        data_dim = q_trajs_pos_iter_0.shape[-1]
        robot_dof = len(self.planning_task.robot.q_pos_min)
        skip_collision_detection = data_dim < robot_dof
        if skip_collision_detection:
            print(f"Note: Skipping collision-based validation (data dim {data_dim} < robot DOF {robot_dof})")

        q_trajs_valid = None
        q_trajs_pos_valid = None
        if results_single_plan.q_trajs_pos_valid is not None:
            q_trajs_valid = torch.cat(
                [
                    results_single_plan.q_trajs_pos_valid,
                    results_single_plan.q_trajs_vel_valid,
                    results_single_plan.q_trajs_acc_valid,
                ],
                dim=-1,
            )
            q_trajs_pos_valid = results_single_plan.q_trajs_pos_valid

        q_trajs_best = None
        q_trajs_pos_best = None
        if results_single_plan.q_trajs_pos_best is not None:
            q_trajs_best = torch.cat(
                [
                    results_single_plan.q_trajs_pos_best,
                    results_single_plan.q_trajs_vel_best,
                    results_single_plan.q_trajs_acc_best,
                ],
                dim=-1,
            )
            q_trajs_pos_best = results_single_plan.q_trajs_pos_best

        # success rate on all trajectories
        if skip_collision_detection:
            # Skip collision-based metrics, assume all trajectories are valid
            metrics.trajs_all.success = 1
            metrics.trajs_all.success_no_joint_limits_vel_acc = 1
            metrics.trajs_all.fraction_valid = 1.0
            metrics.trajs_all.fraction_valid_no_joint_limits_vel_acc = 1.0
            metrics.trajs_all.collision_intensity = 0.0
            # Treat all trajectories as valid for further processing
            if q_trajs_pos_valid is None:
                q_trajs_pos_valid = q_trajs_pos_iter_0
                q_trajs_valid = q_trajs_iter_0
            if q_trajs_pos_best is None:
                # Select first trajectory as best
                q_trajs_pos_best = q_trajs_pos_iter_0[0]
                q_trajs_best = q_trajs_iter_0[0]
        else:
            metrics.trajs_all.success = self.planning_task.compute_success_valid_trajs(
                q_trajs_iter_0, filter_joint_limits_vel_acc=True
            )
            metrics.trajs_all.success_no_joint_limits_vel_acc = self.planning_task.compute_success_valid_trajs(
                q_trajs_iter_0, filter_joint_limits_vel_acc=False
            )
            # validity metrics on all trajectories
            metrics.trajs_all.fraction_valid = self.planning_task.compute_fraction_valid_trajs(
                q_trajs_iter_0, filter_joint_limits_vel_acc=True
            )
            metrics.trajs_all.fraction_valid_no_joint_limits_vel_acc = self.planning_task.compute_fraction_valid_trajs(
                q_trajs_iter_0, filter_joint_limits_vel_acc=False
            )
            # collision intensity of unvalid trajectories
            metrics.trajs_all.collision_intensity = to_numpy(
                self.planning_task.compute_collision_intensity_trajs(q_trajs_iter_0, filter_joint_limits_vel_acc=False)
            ).squeeze()

        # EE pose errors on all trajectories
        ee_pose_goal = results_single_plan.ee_pose_goal
        # Pad to robot DOF for FK (e.g., 6 -> 8 for Piper)
        q_for_fk = pad_q_pos_for_fk(q_trajs_pos_iter_0[..., -1, :], self.planning_task.robot)
        ee_pose_goal_achieved = self.planning_task.robot.get_EE_pose(q_for_fk)
        error_ee_pose_goal_position, error_ee_pose_goal_orientation = compute_ee_pose_errors(
            ee_pose_goal, ee_pose_goal_achieved
        )
        ee_pose_goal_error_position_norm = torch.linalg.norm(error_ee_pose_goal_position, dim=-1)
        ee_pose_goal_error_orientation_norm = torch.rad2deg(torch.linalg.norm(error_ee_pose_goal_orientation, dim=-1))
        metrics.trajs_all.ee_pose_goal_error_position_norm_mean = to_numpy(
            ee_pose_goal_error_position_norm.mean()
        ).squeeze()
        metrics.trajs_all.ee_pose_goal_error_position_norm_std = to_numpy(
            ee_pose_goal_error_position_norm.std()
        ).squeeze()
        metrics.trajs_all.ee_pose_goal_error_orientation_norm_mean = to_numpy(
            ee_pose_goal_error_orientation_norm.mean()
        ).squeeze()
        metrics.trajs_all.ee_pose_goal_error_orientation_norm_std = to_numpy(
            ee_pose_goal_error_orientation_norm.std()
        ).squeeze()

        # EE pose errors on valid trajectories
        if q_trajs_pos_valid is not None:
            q_for_fk = pad_q_pos_for_fk(q_trajs_pos_valid[..., -1, :], self.planning_task.robot)
            ee_pose_goal_achieved = self.planning_task.robot.get_EE_pose(q_for_fk)
            error_ee_pose_goal_position, error_ee_pose_goal_orientation = compute_ee_pose_errors(
                ee_pose_goal, ee_pose_goal_achieved
            )
            ee_pose_goal_error_position_norm = torch.linalg.norm(error_ee_pose_goal_position, dim=-1)
            ee_pose_goal_error_orientation_norm = torch.rad2deg(
                torch.linalg.norm(error_ee_pose_goal_orientation, dim=-1)
            )
            metrics.trajs_valid.ee_pose_goal_error_position_norm_mean = to_numpy(
                ee_pose_goal_error_position_norm.mean()
            ).squeeze()
            metrics.trajs_valid.ee_pose_goal_error_position_norm_std = to_numpy(
                ee_pose_goal_error_position_norm.std()
            ).squeeze()
            metrics.trajs_valid.ee_pose_goal_error_orientation_norm_mean = to_numpy(
                ee_pose_goal_error_orientation_norm.mean()
            ).squeeze()
            metrics.trajs_valid.ee_pose_goal_error_orientation_norm_std = to_numpy(
                ee_pose_goal_error_orientation_norm.std()
            ).squeeze()

        # EE pose errors on best trajectory
        if q_trajs_pos_best is not None:
            q_for_fk = pad_q_pos_for_fk(q_trajs_pos_best[..., -1, :][None, ...], self.planning_task.robot)
            ee_pose_goal_achieved = self.planning_task.robot.get_EE_pose(q_for_fk)
            error_ee_pose_goal_position, error_ee_pose_goal_orientation = compute_ee_pose_errors(
                ee_pose_goal, ee_pose_goal_achieved
            )
            ee_pose_goal_error_position_norm = torch.linalg.norm(error_ee_pose_goal_position, dim=-1)
            ee_pose_goal_error_orientation_norm = torch.rad2deg(
                torch.linalg.norm(error_ee_pose_goal_orientation, dim=-1)
            )
            metrics.trajs_best.ee_pose_goal_error_position_norm = to_numpy(ee_pose_goal_error_position_norm).squeeze()
            metrics.trajs_best.ee_pose_goal_error_orientation_norm = to_numpy(
                ee_pose_goal_error_orientation_norm
            ).squeeze()

        # Path length on valid trajectories
        if q_trajs_valid is not None:
            batch_path_length = compute_path_length(q_trajs_valid, self.planning_task.robot)
            metrics.trajs_valid.path_length_mean = to_numpy(batch_path_length.mean()).squeeze()
            metrics.trajs_valid.path_length_std = to_numpy(batch_path_length.std()).squeeze()
        # Path length on best trajectory
        if q_trajs_best is not None:
            metrics.trajs_best.path_length = to_numpy(
                compute_path_length(q_trajs_best[None, ...], self.planning_task.robot)
            ).squeeze()

        # Smoothness on valid trajectories
        if q_trajs_valid is not None:
            batch_smoothness = compute_smoothness(q_trajs_valid, self.planning_task.robot)
            metrics.trajs_valid.smoothness_mean = to_numpy(batch_smoothness.mean()).squeeze()
            metrics.trajs_valid.smoothness_std = to_numpy(batch_smoothness.std()).squeeze()
        # Smoothness on best trajectory
        if q_trajs_best is not None:
            metrics.trajs_best.smoothness = to_numpy(
                compute_smoothness(q_trajs_best[None, ...], self.planning_task.robot)
            ).squeeze()

        # Variance of waypoints on valid trajectories
        if q_trajs_valid.shape[0] > 1:
            metrics.trajs_valid.diversity = to_numpy(
                compute_trajectory_diversity(q_trajs_valid, self.planning_task.robot)
            ).squeeze()
        else:
            metrics.trajs_valid.diversity = None

        return metrics
