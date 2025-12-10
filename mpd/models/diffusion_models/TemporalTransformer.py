import einops
import torch
import torch.nn as nn

from mpd.models.layers.layers import (
    TimeEncoder,
    MLP,
)
from torch_robotics.torch_utils.torch_timer import TimerCUDA

import numpy as np

UNET_DIM_MULTS = {
    0: (1, 2, 4),
    1: (1, 2, 4, 8),
    2: (1, 2),
}


class TransformerBlock(nn.Module):
    """
    标准 Transformer Encoder Block:
    - 多头自注意力
    - 前馈网络
    - Pre-LN + 残差
    """

    def __init__(self, dim, num_heads=4, dim_ff=None, dropout=0.0):
        super().__init__()
        if dim_ff is None:
            dim_ff = dim * 4

        self.norm1 = nn.LayerNorm(dim)
        self.attn = nn.MultiheadAttention(
            embed_dim=dim,
            num_heads=num_heads,
            batch_first=True,  # 输入 [B, L, D]
        )
        self.dropout1 = nn.Dropout(dropout)

        self.norm2 = nn.LayerNorm(dim)
        self.ff = nn.Sequential(
            nn.Linear(dim, dim_ff),
            nn.GELU(),
            nn.Dropout(dropout),
            nn.Linear(dim_ff, dim),
        )
        self.dropout2 = nn.Dropout(dropout)

    def forward(self, x):
        # x: [B, L, D]
        # 自注意力
        x_norm = self.norm1(x)
        attn_out, _ = self.attn(x_norm, x_norm, x_norm)
        x = x + self.dropout1(attn_out)

        # 前馈
        x_norm = self.norm2(x)
        ff_out = self.ff(x_norm)
        x = x + self.dropout2(ff_out)

        return x


class TemporalTransformer(nn.Module):
    """
    用 Transformer 取代 U-Net 的扩散去噪网络。

    输入:
        x: [B, horizon, state_dim]
        time: [B] 或 [B,] 浮点时间步
        context: [B, conditioning_embed_dim] (取决于 conditioning_type)

    conditioning_type:
        - None / "None": 只用时间嵌入
        - "default": 时间嵌入 + context 拼起来，作为全局条件加到 token 上
        - "concatenate": 在输入 token 维度上拼 context，再投影到模型维度
        （"attention" 这里暂时不实现，如果你确实需要可以再扩展）
    """

    def __init__(
        self,
        n_support_points=None,       # 保留接口，实际上就是 horizon
        state_dim=None,
        unet_input_dim=32,
        dim_mults=(1, 2, 4, 8),
        time_emb_dim=32,
        self_attention=True,         # 保留接口，不用它关掉 attention
        conditioning_embed_dim=4,
        conditioning_type=None,
        attention_num_heads=4,
        attention_dim_head=32,       # 这里不用，保留接口
        num_layers=None,             # 可选：不传则根据 dim_mults 推一个
        dropout=0.0,
        **kwargs,
    ):
        super().__init__()

        assert state_dim is not None, "state_dim must be provided"
        self.n_support_points = n_support_points
        self.state_dim = state_dim
        # 处理 conditioning_type，跟原版保持一致
        if conditioning_type is None or conditioning_type == "None":
            conditioning_type = None
        elif conditioning_type in ["concatenate", "attention", "default"]:
            # attention 在这里先不实现
            if conditioning_type == "attention":
                raise NotImplementedError("conditioning_type='attention' 暂时未在 TemporalTransformer 中实现")
        else:
            raise NotImplementedError(f"Unknown conditioning_type: {conditioning_type}")
        self.conditioning_type = conditioning_type
        self.conditioning_embed_dim = conditioning_embed_dim

        # 处理输入维度（类似原始 TemporalUnet）
        input_dim = state_dim
        if conditioning_type == "concatenate":
            # 如果 context 维度明显比 state_dim 大，就先把 state 映射到更高维
            if state_dim < conditioning_embed_dim // 4:
                state_emb_dim = conditioning_embed_dim // 4
                self.state_encoder = MLP(
                    state_dim,
                    state_emb_dim,
                    hidden_dim=state_emb_dim,
                    n_layers=2,
                    act="mish",
                )
            else:
                state_emb_dim = state_dim
                self.state_encoder = nn.Identity()
            input_dim = state_emb_dim + conditioning_embed_dim
        else:
            # default / None: 不在 token 维度上拼 context
            self.state_encoder = nn.Identity()
            input_dim = state_dim

        # 模型维度：用 unet_input_dim * max(dim_mults) 当作 base dim
        model_dim = unet_input_dim * max(dim_mults)
        self.model_dim = model_dim

        print(f"[ models/temporal_transformer ] input_dim={input_dim}, model_dim={model_dim}")

        # 把输入 token [state_dim or state+context] 映射到 model_dim
        self.input_proj = nn.Linear(input_dim, model_dim)

        # 时间编码
        self.time_mlp = TimeEncoder(32, time_emb_dim)

        # 条件编码（time + context）
        if self.conditioning_type == "default":
            cond_in_dim = time_emb_dim + conditioning_embed_dim
        else:
            cond_in_dim = time_emb_dim  # 只用时间
        self.cond_mlp = nn.Sequential(
            nn.SiLU(),
            nn.Linear(cond_in_dim, model_dim),
        )

        # Transformer 堆叠层数：如果没指定，就按 dim_mults 推一个合理的深度
        if num_layers is None:
            num_layers = len(dim_mults) * 2  # 比如 dim_mults=(1,2,4,8) -> 8 层
        self.num_layers = num_layers

        self.blocks = nn.ModuleList(
            [
                TransformerBlock(
                    dim=model_dim,
                    num_heads=attention_num_heads,
                    dim_ff=model_dim * 4,
                    dropout=dropout,
                )
                for _ in range(num_layers)
            ]
        )

        self.out_norm = nn.LayerNorm(model_dim)
        self.out_proj = nn.Linear(model_dim, state_dim)

    def forward(self, x, time, context):
        """
        x : [batch, horizon, state_dim]
        time : [batch]
        context : [batch, context_dim]  (取决于 conditioning_type)
        """
        # x 编码
        # 注意: 如果是 "concatenate"，要把 context 扩展到每个时间步
        if self.conditioning_type == "concatenate":
            # 先编码 state
            x_emb = self.state_encoder(x)  # [B, H, state_emb_dim]
            # 把 context 拓展到时间维度
            context_rep = einops.repeat(context, "b c -> b h c", h=x.shape[1])
            x_cat = torch.cat([x_emb, context_rep], dim=-1)  # [B, H, state_emb_dim + cond_dim]
            x_tok = self.input_proj(x_cat)  # [B, H, model_dim]
        else:
            # default / None
            x_emb = self.state_encoder(x)       # 通常是 Identity
            x_tok = self.input_proj(x_emb)      # [B, H, model_dim]

        # 时间 / 条件编码
        t_emb = self.time_mlp(time)  # [B, time_emb_dim]

        if self.conditioning_type == "default":
            # 用 time + context 作为全局条件
            cond_in = torch.cat([t_emb, context], dim=-1)  # [B, time_emb_dim + cond_dim]
        else:
            # 只用 time
            cond_in = t_emb  # [B, time_emb_dim]

        cond = self.cond_mlp(cond_in)  # [B, model_dim]

        # 把条件加到 token 上，相当于 FiLM-like 的简单加法
        x_tok = x_tok + cond.unsqueeze(1)  # [B, H, model_dim]

        # 经过多层 Transformer self-attention
        for blk in self.blocks:
            x_tok = blk(x_tok)  # [B, H, model_dim]

        # 输出映射回 state_dim
        x_tok = self.out_norm(x_tok)
        out = self.out_proj(x_tok)  # [B, H, state_dim]

        return out


if __name__ == "__main__":
    import torch

    device = "cuda:0"

    batch_size = 1000
    n_support_points = 16  # horizon

    model = TemporalTransformer(
        n_support_points=n_support_points,
        state_dim=7,
        unet_input_dim=32,
        dim_mults=UNET_DIM_MULTS[1],
        time_emb_dim=32,
        self_attention=True,
        conditioning_embed_dim=128,
        conditioning_type="default",
        attention_num_heads=4,
        dropout=0.0,
    ).to(device)

    x = torch.randn(batch_size, n_support_points, 7, device=device)      # [B, H, state_dim]
    t = torch.randn(batch_size, device=device)                           # [B]
    context = torch.randn(batch_size, 128, device=device)                # [B, cond_dim]

    t_elapsed_l = []
    for i in range(20):
        with TimerCUDA() as t_forward:
            output = model(x, t, context)
        t_elapsed_l.append(t_forward.elapsed)

    print("output shape:", output.shape)
    print("Time taken average (CUDA):", np.mean(t_elapsed_l[10:]))
