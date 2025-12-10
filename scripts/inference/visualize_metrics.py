# import torch

# data = torch.load("logs/3/results_single_plan-000.pt", map_location="cpu")

# print("T (total):", data["t_inference_total"])
# print("S (success):", data["metrics"]["trajs_all"]["success"])
# print("collision_intensity (I):", data["metrics"]["trajs_all"]["collision_intensity"])
# print("PL best:", data["metrics"]["trajs_best"]["path_length"])
# print("VAR (diversity):", data["metrics"]["trajs_valid"]["diversity"])
# print("n_free / n_total:",
#       data["isaacgym_statistics"]["n_trajectories_free"],
#       "/",
#       data["isaacgym_statistics"]["n_trajectories_free"] + data["isaacgym_statistics"]["n_trajectories_collision"])

import torch
import math
import numpy as np
from pathlib import Path

# ===== 显式写死路径和文件名模式 =====
# ROOT_DIR = Path("/home/hong/文档/实验记录/推理/obstacle-free_pipper_100000")   # 改成你的目录
ROOT_DIR = Path("./logs_inference_piper/2") 
PREFIX = "results_single_plan"
START_IDX = 0
END_IDX = 9
# ==================================


def to_scalar(x):
    """尽量把各种类型转成一个 float 标量:
    - int/float
    - 0维 tensor
    - 任意维 tensor / list / ndarray -> 取均值
    """
    import numbers

    if x is None:
        return None

    # 纯数字
    if isinstance(x, numbers.Number):
        return float(x)

    # torch.Tensor
    if isinstance(x, torch.Tensor):
        if x.numel() == 0:
            return None
        return float(x.float().mean().item())

    # numpy 数组
    if isinstance(x, np.ndarray):
        if x.size == 0:
            return None
        return float(x.astype(np.float32).mean())

    # list / tuple
    if isinstance(x, (list, tuple)):
        if len(x) == 0:
            return None
        # 尝试用 numpy 转一下
        try:
            arr = np.array(x, dtype=np.float32)
            return float(arr.mean())
        except Exception:
            return None

    return None


def main():
    sum_T = sum_S = sum_I = sum_PL = sum_VAR = 0.0
    cnt_T = cnt_S = cnt_I = cnt_PL = cnt_VAR = 0

    total_free = 0
    total_coll = 0

    first = True

    for i in range(START_IDX, END_IDX + 1):
        fname = f"{PREFIX}-{i:03d}.pt"
        fpath = ROOT_DIR / fname

        if not fpath.exists():
            print(f"[skip] {fpath} 不存在")
            continue

        print(f"[load] {fpath}")
        data = torch.load(fpath, map_location="cpu")

        # ---------- 时间 T ----------
        T = to_scalar(data.get("t_inference_total", None))
        if T is not None and not math.isnan(T):
            sum_T += T
            cnt_T += 1

        metrics = data.get("metrics", {})
        trajs_all = metrics.get("trajs_all", {})
        trajs_best = metrics.get("trajs_best", {})
        trajs_valid = metrics.get("trajs_valid", {})

        # ---------- 成功率 S ----------
        S = to_scalar(trajs_all.get("success", None))
        if S is not None and not math.isnan(S):
            sum_S += S
            cnt_S += 1

        # ---------- 碰撞强度 I ----------
        I = to_scalar(trajs_all.get("collision_intensity", None))
        if I is not None and not math.isnan(I):
            sum_I += I
            cnt_I += 1

        # ---------- 最优路径长度 PL ----------
        PL = to_scalar(trajs_best.get("path_length", None))
        if PL is not None and not math.isnan(PL):
            sum_PL += PL
            cnt_PL += 1

        # ---------- 多样性 VAR ----------
        VAR = to_scalar(trajs_valid.get("diversity", None))
        if VAR is not None and not math.isnan(VAR):
            sum_VAR += VAR
            cnt_VAR += 1

        # ---------- isaacgym 成功条数 ----------
        isaac_stats = data.get("isaacgym_statistics", {})
        n_free = to_scalar(isaac_stats.get("n_trajectories_free", 0)) or 0.0
        n_coll = to_scalar(isaac_stats.get("n_trajectories_collision", 0)) or 0.0

        n_free = int(n_free)
        n_coll = int(n_coll)

        total_free += n_free
        total_coll += n_coll

        if first:
            first = False
            print("  示例指标（已经做了均值压缩）:")
            print(f"    T   = {T}")
            print(f"    S   = {S}")
            print(f"    I   = {I}")
            print(f"    PL  = {PL}")
            print(f"    VAR = {VAR}")
            print(f"    n_free / n_total = {n_free} / {n_free + n_coll}")

    print("\n================== 汇总结果 ==================")

    if cnt_T > 0:
        print(f"T  (time)              avg = {sum_T / cnt_T:.6f}  (N={cnt_T})")
    if cnt_S > 0:
        avg_S = sum_S / cnt_S
        print(f"S  (success rate)      avg = {avg_S:.6f}  ({avg_S*100:.2f} %)  (N={cnt_S})")
    if cnt_I > 0:
        print(f"I  (collision ratio)   avg = {sum_I / cnt_I:.6f}  (N={cnt_I})")
    else:
        print("I  (collision ratio)   无有效样本")
    if cnt_PL > 0:
        print(f"PL (best path length)  avg = {sum_PL / cnt_PL:.6f}  (N={cnt_PL})")
    else:
        print("PL (best path length)  无有效样本")
    if cnt_VAR > 0:
        print(f"VAR (diversity)        avg = {sum_VAR / cnt_VAR:.6f}  (N={cnt_VAR})")
    else:
        print("VAR (diversity)        无有效样本")

    n_total = total_free + total_coll
    if n_total > 0:
        global_sr = total_free / n_total
        print(f"\nGlobal success (from n_free/n_total): "
              f"{total_free} / {n_total} = {global_sr:.6f}  ({global_sr*100:.2f} %)")


if __name__ == "__main__":
    main()

