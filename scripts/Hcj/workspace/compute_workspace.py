"""
计算机械臂工作空间的脚本
通过蒙特卡洛采样方法，在关节空间中随机采样，计算末端执行器的位置分布
"""

import sys
import os
import numpy as np
import torch
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from pathlib import Path

# 添加项目路径
project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))
sys.path.insert(0, str(project_root / "mpd"))

from torch_robotics.torch_kinematics_tree.models.robot_tree import DifferentiableTree


def compute_workspace(urdf_path, ee_link="link6", num_samples=100000, device="cpu"):
    """计算机械臂工作空间"""
    print(f"加载 URDF: {urdf_path}")
    print(f"末端执行器: {ee_link}, 采样数量: {num_samples}, 设备: {device}")
    
    robot = DifferentiableTree(model_path=urdf_path, device=device)
    lower, upper, _, _ = robot.get_joint_limit_array()
    n_dofs = robot._n_dofs
    
    print(f"\n机械臂自由度: {n_dofs}")
    print(f"关节限位 (rad):")
    for i in range(n_dofs):
        print(f"  Joint {i+1}: [{lower[i]:.4f}, {upper[i]:.4f}] (范围: {np.rad2deg(upper[i]-lower[i]):.1f}°)")
    
    link_names = robot.get_link_names()
    print(f"\n链接名称: {link_names}")
    
    if ee_link not in link_names:
        ee_link = link_names[-1]
        print(f"使用末端执行器: {ee_link}")
    
    print(f"\n开始采样...")
    lower_t = torch.tensor(lower, dtype=torch.float32, device=device)
    upper_t = torch.tensor(upper, dtype=torch.float32, device=device)
    
    batch_size = 10000
    all_positions = []
    
    for i in range(0, num_samples, batch_size):
        current_batch = min(batch_size, num_samples - i)
        q_random = torch.rand(current_batch, n_dofs, device=device)
        q_random = lower_t + q_random * (upper_t - lower_t)
        
        with torch.no_grad():
            H = robot.compute_forward_kinematics_all_links(q_random, link_list=[ee_link])
            positions = H[:, 0, :3, 3]
            all_positions.append(positions.cpu().numpy())
        
        if (i + current_batch) % 20000 == 0:
            print(f"  已采样: {i + current_batch}/{num_samples}")
    
    ee_positions = np.concatenate(all_positions, axis=0)
    
    print(f"\n工作空间统计:")
    print(f"  X: [{ee_positions[:,0].min():.4f}, {ee_positions[:,0].max():.4f}] m")
    print(f"  Y: [{ee_positions[:,1].min():.4f}, {ee_positions[:,1].max():.4f}] m")
    print(f"  Z: [{ee_positions[:,2].min():.4f}, {ee_positions[:,2].max():.4f}] m")
    
    distances = np.linalg.norm(ee_positions, axis=1)
    print(f"  距离原点: [{distances.min():.4f}, {distances.max():.4f}] m")
    
    return ee_positions, (lower, upper)



def visualize_workspace_3d(ee_positions, save_path=None, title="Robot Workspace"):
    """3D 可视化工作空间点云"""
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    max_display = 20000
    if len(ee_positions) > max_display:
        idx = np.random.choice(len(ee_positions), max_display, replace=False)
        display_positions = ee_positions[idx]
    else:
        display_positions = ee_positions
    
    colors = display_positions[:, 2]
    scatter = ax.scatter(
        display_positions[:, 0], display_positions[:, 1], display_positions[:, 2],
        c=colors, cmap='viridis', s=1, alpha=0.5
    )
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title(title)
    
    max_range = np.array([
        ee_positions[:,0].max() - ee_positions[:,0].min(),
        ee_positions[:,1].max() - ee_positions[:,1].min(),
        ee_positions[:,2].max() - ee_positions[:,2].min()
    ]).max() / 2.0
    
    mid_x = (ee_positions[:,0].max() + ee_positions[:,0].min()) / 2
    mid_y = (ee_positions[:,1].max() + ee_positions[:,1].min()) / 2
    mid_z = (ee_positions[:,2].max() + ee_positions[:,2].min()) / 2
    
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)
    
    plt.colorbar(scatter, label='Z (m)', shrink=0.6)
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"3D 图像已保存: {save_path}")
    # plt.show()


def visualize_workspace_projections(ee_positions, save_path=None):
    """可视化工作空间在三个平面上的投影"""
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    
    axes[0].scatter(ee_positions[:,0], ee_positions[:,1], s=0.5, alpha=0.3)
    axes[0].set_xlabel('X (m)')
    axes[0].set_ylabel('Y (m)')
    axes[0].set_title('Top View (XY)')
    axes[0].set_aspect('equal')
    axes[0].grid(True, alpha=0.3)
    
    axes[1].scatter(ee_positions[:,0], ee_positions[:,2], s=0.5, alpha=0.3)
    axes[1].set_xlabel('X (m)')
    axes[1].set_ylabel('Z (m)')
    axes[1].set_title('Side View (XZ)')
    axes[1].set_aspect('equal')
    axes[1].grid(True, alpha=0.3)
    
    axes[2].scatter(ee_positions[:,1], ee_positions[:,2], s=0.5, alpha=0.3)
    axes[2].set_xlabel('Y (m)')
    axes[2].set_ylabel('Z (m)')
    axes[2].set_title('Front View (YZ)')
    axes[2].set_aspect('equal')
    axes[2].grid(True, alpha=0.3)
    
    plt.suptitle('Workspace Projections', fontsize=14)
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"投影图像已保存: {save_path}")
    # plt.show()


def visualize_workspace_slices(ee_positions, save_path=None, num_slices=6):
    """可视化工作空间的水平切片"""
    z_min, z_max = ee_positions[:,2].min(), ee_positions[:,2].max()
    z_levels = np.linspace(z_min, z_max, num_slices + 2)[1:-1]
    
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))
    axes = axes.flatten()
    slice_thickness = (z_max - z_min) / (num_slices * 2)
    
    for i, z in enumerate(z_levels):
        ax = axes[i]
        mask = np.abs(ee_positions[:,2] - z) < slice_thickness
        slice_points = ee_positions[mask]
        
        if len(slice_points) > 0:
            ax.scatter(slice_points[:,0], slice_points[:,1], s=1, alpha=0.5)
            ax.set_title(f'Z = {z:.3f} m ({len(slice_points)} pts)')
        else:
            ax.set_title(f'Z = {z:.3f} m (no points)')
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
    
    plt.suptitle('Workspace Horizontal Slices', fontsize=14)
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"切片图像已保存: {save_path}")
    # plt.show()


def compute_workspace_volume(ee_positions, resolution=0.02):
    """计算工作空间体积"""
    mins = ee_positions.min(axis=0)
    maxs = ee_positions.max(axis=0)
    grid_size = np.ceil((maxs - mins) / resolution).astype(int) + 1
    voxel_grid = np.zeros(grid_size, dtype=np.int32)
    
    indices = ((ee_positions - mins) / resolution).astype(int)
    indices = np.clip(indices, 0, grid_size - 1)
    
    for idx in indices:
        voxel_grid[idx[0], idx[1], idx[2]] += 1
    
    occupied = np.sum(voxel_grid > 0)
    volume = occupied * (resolution ** 3)
    
    print(f"\n工作空间体积分析:")
    print(f"  体素分辨率: {resolution} m")
    print(f"  占用体素: {occupied}")
    print(f"  工作空间体积: {volume:.4f} m³")
    
    return volume, voxel_grid


class WorkspaceChecker:
    """
    工作空间检查器 - 判断目标点是否在机械臂工作空间内
    
    使用方法:
        # 方法1: 从 URDF 创建（会自动采样计算工作空间）
        checker = WorkspaceChecker.from_urdf(urdf_path, num_samples=100000)
        
        # 方法2: 从已保存的点云加载
        checker = WorkspaceChecker.from_pointcloud("workspace_points.npy")
        
        # 检查单个点
        is_reachable = checker.is_in_workspace([0.3, 0.1, 0.2])
        
        # 检查多个点
        points = [[0.3, 0.1, 0.2], [0.5, 0.0, 0.3]]
        results = checker.is_in_workspace_batch(points)
    """
    
    def __init__(self, ee_positions, resolution=0.02):
        """
        初始化工作空间检查器
        
        Args:
            ee_positions: 末端执行器位置点云 [N, 3]
            resolution: 体素分辨率 (m)，越小越精确但内存占用越大
        """
        self.resolution = resolution
        self.ee_positions = ee_positions
        
        # 计算边界
        self.mins = ee_positions.min(axis=0)
        self.maxs = ee_positions.max(axis=0)
        
        # 创建体素网格
        self.grid_size = np.ceil((self.maxs - self.mins) / resolution).astype(int) + 1
        self.voxel_grid = np.zeros(self.grid_size, dtype=bool)
        
        # 填充体素
        indices = ((ee_positions - self.mins) / resolution).astype(int)
        indices = np.clip(indices, 0, self.grid_size - 1)
        for idx in indices:
            self.voxel_grid[idx[0], idx[1], idx[2]] = True
        
        # 统计信息
        self.occupied_voxels = np.sum(self.voxel_grid)
        self.volume = self.occupied_voxels * (resolution ** 3)
        
        print(f"WorkspaceChecker 初始化完成:")
        print(f"  体素分辨率: {resolution} m")
        print(f"  网格大小: {self.grid_size}")
        print(f"  工作空间体积: {self.volume:.4f} m³")
    
    @classmethod
    def from_urdf(cls, urdf_path, ee_link="link6", num_samples=100000, 
                  resolution=0.02, device="cpu"):
        """
        从 URDF 文件创建工作空间检查器
        
        Args:
            urdf_path: URDF 文件路径
            ee_link: 末端执行器链接名
            num_samples: 采样数量
            resolution: 体素分辨率
            device: 计算设备
        """
        ee_positions, _ = compute_workspace(urdf_path, ee_link, num_samples, device)
        return cls(ee_positions, resolution)
    
    @classmethod
    def from_pointcloud(cls, pointcloud_path, resolution=0.02):
        """
        从已保存的点云文件加载
        
        Args:
            pointcloud_path: 点云文件路径 (.npy)
            resolution: 体素分辨率
        """
        ee_positions = np.load(pointcloud_path)
        print(f"从 {pointcloud_path} 加载了 {len(ee_positions)} 个点")
        return cls(ee_positions, resolution)
    
    def is_in_workspace(self, point):
        """
        检查单个点是否在工作空间内
        
        Args:
            point: 3D 坐标 [x, y, z]，可以是 list, tuple, np.array 或 torch.Tensor
        
        Returns:
            bool: True 表示在工作空间内
        """
        point = np.asarray(point).flatten()[:3]
        
        # 快速边界检查
        if np.any(point < self.mins) or np.any(point > self.maxs):
            return False
        
        # 体素检查
        idx = ((point - self.mins) / self.resolution).astype(int)
        idx = np.clip(idx, 0, self.grid_size - 1)
        
        return bool(self.voxel_grid[idx[0], idx[1], idx[2]])
    
    def is_in_workspace_batch(self, points):
        """
        批量检查多个点是否在工作空间内
        
        Args:
            points: 多个 3D 坐标 [N, 3]
        
        Returns:
            np.array: 布尔数组，True 表示在工作空间内
        """
        points = np.asarray(points)
        if points.ndim == 1:
            points = points.reshape(1, -1)
        
        results = np.zeros(len(points), dtype=bool)
        
        for i, point in enumerate(points):
            results[i] = self.is_in_workspace(point)
        
        return results
    
    def distance_to_workspace(self, point):
        """
        计算点到工作空间的近似距离
        
        Args:
            point: 3D 坐标 [x, y, z]
        
        Returns:
            float: 距离（如果在工作空间内返回 0，否则返回到最近点的距离）
        """
        point = np.asarray(point).flatten()[:3]
        
        if self.is_in_workspace(point):
            return 0.0
        
        # 计算到所有采样点的距离，返回最小值
        distances = np.linalg.norm(self.ee_positions - point, axis=1)
        return float(distances.min())
    
    def get_nearest_reachable_point(self, point):
        """
        获取工作空间内距离目标点最近的可达点
        
        Args:
            point: 3D 坐标 [x, y, z]
        
        Returns:
            np.array: 最近的可达点坐标
        """
        point = np.asarray(point).flatten()[:3]
        distances = np.linalg.norm(self.ee_positions - point, axis=1)
        nearest_idx = np.argmin(distances)
        return self.ee_positions[nearest_idx].copy()
    
    def get_workspace_bounds(self):
        """获取工作空间的边界框"""
        return {
            'x_min': float(self.mins[0]), 'x_max': float(self.maxs[0]),
            'y_min': float(self.mins[1]), 'y_max': float(self.maxs[1]),
            'z_min': float(self.mins[2]), 'z_max': float(self.maxs[2]),
        }
    
    def save(self, path):
        """保存工作空间数据"""
        np.savez(path, 
                 ee_positions=self.ee_positions,
                 resolution=self.resolution)
        print(f"工作空间数据已保存: {path}")
    
    @classmethod
    def load(cls, path):
        """加载工作空间数据"""
        data = np.load(path)
        return cls(data['ee_positions'], float(data['resolution']))


def main():
    urdf_path = "/home/hong/Projects/MotionPlanningDiffusion/mpd-splines-public/scripts/Hcj/piper_description/urdf/piper_description.urdf"
    output_dir = Path(__file__).parent / "workspace_analysis"
    output_dir.mkdir(exist_ok=True)
    
    device = "cuda" if torch.cuda.is_available() else "cpu"
    
    # 创建工作空间检查器
    checker = WorkspaceChecker.from_urdf(
        urdf_path=urdf_path,
        ee_link="link6",
        num_samples=100000,
        resolution=0.02,
        device=device
    )
    
    # 保存工作空间数据
    checker.save(output_dir / "workspace_data.npz")
    np.save(output_dir / "workspace_points.npy", checker.ee_positions)
    
    # 演示：检查目标点是否在工作空间内
    print("\n" + "="*50)
    print("工作空间检查示例:")
    print("="*50)
    
    test_points = [
        [0.3, 0.0, 0.2],   # 可能在工作空间内
        [0.0, 0.0, 0.5],   # 可能在工作空间内
        [1.0, 1.0, 1.0],   # 可能在工作空间外
        [0.0, 0.0, 0.0],   # 原点附近
    ]
    
    for point in test_points:
        is_reachable = checker.is_in_workspace(point)
        distance = checker.distance_to_workspace(point)
        status = "✓ 可达" if is_reachable else "✗ 不可达"
        print(f"  点 {point}: {status}, 距离工作空间: {distance:.4f} m")
    
    # 获取工作空间边界
    bounds = checker.get_workspace_bounds()
    print(f"\n工作空间边界:")
    print(f"  X: [{bounds['x_min']:.4f}, {bounds['x_max']:.4f}] m")
    print(f"  Y: [{bounds['y_min']:.4f}, {bounds['y_max']:.4f}] m")
    print(f"  Z: [{bounds['z_min']:.4f}, {bounds['z_max']:.4f}] m")
    
    # 生成可视化
    print("\n生成可视化...")
    visualize_workspace_3d(checker.ee_positions, save_path=output_dir / "workspace_3d.png", title="Piper Robot Workspace")
    visualize_workspace_projections(checker.ee_positions, save_path=output_dir / "workspace_projections.png")
    visualize_workspace_slices(checker.ee_positions, save_path=output_dir / "workspace_slices.png")
    
    print(f"\n完成！结果保存在: {output_dir}")
    
    return checker


if __name__ == "__main__":
    main()
