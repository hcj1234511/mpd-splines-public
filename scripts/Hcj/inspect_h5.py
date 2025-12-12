import h5py
from pathlib import Path

root_dir = Path('/home/hong/Projects/MotionPlanningDiffusion/mpd-splines-public')

# path = root_dir / 'scripts' / 'generate_data' / 'data' / 'env-robot' / '1764753835' / 'dataset.hdf5'
# path = root_dir / 'data_trajectories_hcj' / 'piper_100000_obstacle-free' / 'dataset_merged_processed.hdf5' 
# path = root_dir / 'data_trajectories' / 'EnvWarehouse-RobotPanda-config_file_v01-joint_joint-one-RRTConnect' / 'dataset_merged_doubled.hdf5'
# path = root_dir / 'data_trajectories_hcj' / 'dataset.hdf5'
path = root_dir / 'scripts' / 'Hcj' / 'test_data' / 'dataset.hdf5'

# print("path =", path)
# print("exists:", path.exists())

with h5py.File(path, 'r') as f:
    print('keys are:', list(f.keys()))
    # for i in f.keys():
    #     dset = f[i]
    #     print(i, 'shape: ',dset.shape, 'dtype: ', dset.dtype)
    all_states_valid_after_bspline_fit = f['all_states_valid_after_bspline_fit']
    sol_path = f['sol_path']
    sol_path_after_bspline_fit = f['sol_path_after_bspline_fit']
    task_id = f["task_id"]
    success = f["success"]
    # traj = sol_path[9] 
    print(sol_path.shape)
    for i in range(len(success)):
        print(f"success{i}:{success[i]}")
# import matplotlib.pyplot as plt   
# import numpy as np

#        # (256,7)
# # 如果是 torch.Tensor 就 traj = sol_path[0].cpu().numpy()

# T = traj.shape[0]
# x = np.arange(T)

# fig, axes = plt.subplots(6, 1, figsize=(8, 12), sharex=True)

# for j in range(6):
#     ax = axes[j]
#     ax.plot(x, traj[:, j])
#     ax.set_ylabel(f'joint {j+1}')
#     ax.grid(True)

# axes[-1].set_xlabel("Timestep")
# fig.suptitle("Joint trajectories (sample 0)", y=0.92)
# plt.tight_layout()
# plt.show()


#     # for i in range(10000):
#     #     # print(sol_path_after_bspline_fit[i])
#     #     print(sol_path[i].shape) 
    

