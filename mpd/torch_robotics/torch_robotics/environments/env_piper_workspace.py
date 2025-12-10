"""
专门为 Piper 机械臂设计的环境
在 Piper 的工作空间内添加障碍物，使 RRTConnect 生成非线性轨迹
"""
from copy import copy

import numpy as np
import torch
from matplotlib import pyplot as plt

from torch_robotics.environments.env_base import EnvBase
from torch_robotics.environments.primitives import ObjectField, MultiBoxField
from torch_robotics.torch_utils.torch_utils import DEFAULT_TENSOR_ARGS, to_torch
from torch_robotics.visualizers.plot_utils import create_fig_and_axes


class EnvPiperWorkspace(EnvBase):
    """
    专门为 Piper 机械臂设计的环境
    
    Piper 工作空间范围 (10%-90%):
      X: -0.23 ~ 0.31 m
      Y: -0.35 ~ 0.35 m
      Z: -0.05 ~ 0.66 m
    
    障碍物设计：在采样区域中间放置障碍物，迫使轨迹绕行
    """

    def __init__(self, tensor_args=DEFAULT_TENSOR_ARGS, **kwargs):
        
        # 障碍物1：中央竖立的圆柱形障碍物（用方块近似）
        # 位于 Piper 工作空间中心，强制轨迹必须绕过
        center_obstacle = MultiBoxField(
            np.array([
                [0.22, 0.0, 0.30],   # 正前方中央
            ]),
            np.array([
                [0.06, 0.08, 0.40],  # 细长的柱子
            ]),
            tensor_args=tensor_args,
        )
        center_obj_field = ObjectField([center_obstacle], "center_pillar")
        
        # 障碍物2：左侧挡板
        left_obstacle = MultiBoxField(
            np.array([
                [0.18, 0.20, 0.35],  # 左前方
            ]),
            np.array([
                [0.10, 0.04, 0.30],  # 横向挡板
            ]),
            tensor_args=tensor_args,
        )
        left_obj_field = ObjectField([left_obstacle], "left_barrier")
        
        # 障碍物3：右侧挡板
        right_obstacle = MultiBoxField(
            np.array([
                [0.18, -0.20, 0.35],  # 右前方
            ]),
            np.array([
                [0.10, 0.04, 0.30],  # 横向挡板
            ]),
            tensor_args=tensor_args,
        )
        right_obj_field = ObjectField([right_obstacle], "right_barrier")
        
        # 障碍物4：底部平台（可选，模拟桌面）
        bottom_platform = MultiBoxField(
            np.array([
                [0.22, 0.0, 0.08],  # 底部
            ]),
            np.array([
                [0.30, 0.40, 0.04],  # 薄平台
            ]),
            tensor_args=tensor_args,
        )
        bottom_obj_field = ObjectField([bottom_platform], "bottom_platform")
        
        obj_list = [center_obj_field, left_obj_field, right_obj_field, bottom_obj_field]

        super().__init__(
            limits=torch.tensor([[-0.5, -0.5, -0.2], [0.5, 0.5, 0.8]], **tensor_args),
            obj_fixed_list=obj_list,
            tensor_args=tensor_args,
            **kwargs,
        )

    def get_gpmp2_params(self, robot=None):
        """GPMP2 规划器参数"""
        params = dict(
            n_support_points=64,
            n_interpolated_points=None,
            dt=0.04,
            opt_iters=100,
            temperature=1.0,
            step_size=0.05,
        )
        return params

    def get_rrt_connect_params(self, robot=None):
        """RRTConnect 规划器参数"""
        params = dict(
            n_iters=10000,
            step_size=torch.pi / 80,
            n_radius=torch.pi / 4,
            n_pre_samples=50000,
            max_time=15
        )
        return params


class EnvPiperWorkspaceV2(EnvBase):
    """
    Piper 工作空间 V2 版本 - 更多障碍物，强制更复杂的轨迹
    """

    def __init__(self, tensor_args=DEFAULT_TENSOR_ARGS, **kwargs):
        
        # 多个小障碍物散布在工作空间中
        obstacles = MultiBoxField(
            np.array([
                [0.20, 0.0, 0.25],    # 中央低处
                [0.25, 0.0, 0.45],    # 中央高处
                [0.18, 0.12, 0.30],   # 左侧
                [0.18, -0.12, 0.30],  # 右侧
                [0.22, 0.0, 0.08],    # 底部平台
            ]),
            np.array([
                [0.05, 0.06, 0.15],   # 小方块
                [0.05, 0.06, 0.15],   # 小方块
                [0.06, 0.04, 0.20],   # 薄挡板
                [0.06, 0.04, 0.20],   # 薄挡板
                [0.25, 0.35, 0.03],   # 底部薄平台
            ]),
            tensor_args=tensor_args,
        )
        obstacle_field = ObjectField([obstacles], "piper_obstacles")
        
        obj_list = [obstacle_field]

        super().__init__(
            limits=torch.tensor([[-0.5, -0.5, -0.2], [0.5, 0.5, 0.8]], **tensor_args),
            obj_fixed_list=obj_list,
            tensor_args=tensor_args,
            **kwargs,
        )

    def get_gpmp2_params(self, robot=None):
        params = dict(
            n_support_points=64,
            n_interpolated_points=None,
            dt=0.04,
            opt_iters=100,
            temperature=1.0,
            step_size=0.05,
        )
        return params

    def get_rrt_connect_params(self, robot=None):
        params = dict(
            n_iters=10000,
            step_size=torch.pi / 80,
            n_radius=torch.pi / 4,
            n_pre_samples=50000,
            max_time=15
        )
        return params


if __name__ == "__main__":
    # 测试环境
    print("Testing EnvPiperWorkspace...")
    env = EnvPiperWorkspace(tensor_args=DEFAULT_TENSOR_ARGS)
    fig, ax = create_fig_and_axes(env.dim)
    env.render(ax)
    plt.title("EnvPiperWorkspace - 专为 Piper 设计的障碍物环境")
    plt.show()

    print("\nTesting EnvPiperWorkspaceV2...")
    env2 = EnvPiperWorkspaceV2(tensor_args=DEFAULT_TENSOR_ARGS)
    fig, ax = create_fig_and_axes(env2.dim)
    env2.render(ax)
    plt.title("EnvPiperWorkspaceV2 - 更多障碍物")
    plt.show()
