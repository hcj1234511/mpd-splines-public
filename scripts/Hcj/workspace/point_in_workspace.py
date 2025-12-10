from compute_workspace import WorkspaceChecker

# 方法1: 从 URDF 创建（首次使用，会采样计算工作空间）
checker = WorkspaceChecker.from_urdf(
    urdf_path="/home/hong/Projects/MotionPlanningDiffusion/mpd-splines-public/scripts/Hcj/piper_description/urdf/piper_description.urdf",
    ee_link="link6",
    num_samples=100000,
    device="cuda"
)

# # 方法2: 从已保存的数据加载（后续使用，速度快）
# checker = WorkspaceChecker.load("workspace_analysis/workspace_data.npz")

# 检查单个点是否可达
point = [0.300000,0.200000,0.143000]
is_reachable = checker.is_in_workspace(point)
print(f"点 {point} 可达: {is_reachable}")

# 批量检查多个点
points = [[0.3, 0.1, 0.2], [0.5, 0.0, 0.3], [1.0, 1.0, 1.0]]
results = checker.is_in_workspace_batch(points)
print(f"可达性: {results}")  # [True, True, False]

# 计算到工作空间的距离
distance = checker.distance_to_workspace([1.0, 1.0, 1.0])
print(f"距离工作空间: {distance:.4f} m")

# 获取最近的可达点
nearest = checker.get_nearest_reachable_point([1.0, 1.0, 1.0])
print(f"最近可达点: {nearest}")

# 获取工作空间边界
bounds = checker.get_workspace_bounds()
