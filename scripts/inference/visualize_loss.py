import numpy as np
import torch

# 读取
train_losses = np.load("../train/logs/1/checkpoints/train_losses.npy", allow_pickle=True)
val_losses = np.load("../train/logs/1/checkpoints/val_losses.npy", allow_pickle=True)

print("train_losses.shape:", train_losses.shape)
print("val_losses.shape:", val_losses.shape)
print(train_losses)
# for name, values in train_losses:
#     print("=== TRAIN:", name, "===")
#     # values 可能是 torch.Tensor 或 list
#     if isinstance(values, torch.Tensor):
#         v = values.cpu().numpy()
#     else:
#         v = np.array(values)

#     print("  steps:", len(v))
#     print("  first 5:", v[:5])
#     print("  last 5 :", v[-5:])

# for name, values in val_losses:
#     print("=== VAL:", name, "===")
#     if isinstance(values, torch.Tensor):
#         v = values.cpu().numpy()
#     else:
#         v = np.array(values)

#     print("  steps:", len(v))
#     print("  first 5:", v[:5])
#     print("  last 5 :", v[-5:])
