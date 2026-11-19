"""
sim_source.py — 生成测试用的上升沿序列（模拟 GPIO16 上的流量计脉冲）

真实流量曲线（mL/min）：
  0–3 s    300
  3–6 s    150（阶跃）
  6–9 s     60（低流量）
  9–11 s     0（停泵，检验超时判零）
  11–14 s  0 -> 300 斜坡

叠加的干扰：
  - 蠕动泵脉动：±PULSE_AMP 的正弦，频率与流量成正比
    （滚轮每次压管排出 ML_PER_ROLLER mL，这是假设值）
  - 间隔抖动：每个上升沿时间加高斯噪声（局部周期的 JITTER_FRAC）
  - 毛刺：随机插入 N_GLITCH 个假上升沿，紧跟在真实上升沿后 0.2–1 ms
"""

import numpy as np

K_HZ_PER_LPM = 98.0
DURATION_S = 14.0
PULSE_AMP = 0.15          # 泵脉动幅度 ±15%
ML_PER_ROLLER = 0.4       # 每次滚轮压管的排量（假设）：300 mL/min 时脉动约 12.5 Hz
JITTER_FRAC = 0.02        # 间隔抖动 2%
N_GLITCH = 10
SEED = 5725


def q_true(t):
    """真实（平均）流量，mL/min，t 可以是数组"""
    t = np.asarray(t, dtype=float)
    q = np.zeros_like(t)
    q[(t >= 0) & (t < 3)] = 300
    q[(t >= 3) & (t < 6)] = 150
    q[(t >= 6) & (t < 9)] = 60
    m = (t >= 11) & (t < 14)
    q[m] = 300 * (t[m] - 11) / 3
    return q


def generate_edges(duration_s=DURATION_S, dt=10e-6, seed=SEED):
    """
    返回 (edges_ns, glitch_ns)：
      edges_ns  — 全部上升沿（含毛刺），已排序，int64 纳秒
      glitch_ns — 其中的毛刺，便于检验
    """
    rng = np.random.default_rng(seed)
    t = np.arange(0, duration_s, dt)
    qm = q_true(t)

    # 泵脉动：相位随排出体积推进
    vol_ml = np.cumsum(qm / 60.0 * dt)
    pulse_phase = 2 * np.pi * vol_ml / ML_PER_ROLLER
    q_inst = qm * (1 + PULSE_AMP * np.sin(pulse_phase))

    # 流量计脉冲：f = K * Q[L/min]，相位每过一个整数产生一个上升沿
    f = K_HZ_PER_LPM * q_inst / 1000.0
    phase = np.cumsum(f * dt)
    idx = np.nonzero(np.diff(np.floor(phase)) > 0)[0] + 1
    t_edges = t[idx]

    # 抖动
    local_period = 1.0 / np.maximum(f[idx], 1e-6)
    t_edges = t_edges + rng.normal(0, JITTER_FRAC, size=t_edges.size) * local_period
    t_edges = np.sort(t_edges)

    # 毛刺
    if t_edges.size > 0:
        pick = rng.choice(t_edges.size, size=min(N_GLITCH, t_edges.size), replace=False)
        glitches = t_edges[pick] + rng.uniform(0.2e-3, 1.0e-3, size=pick.size)
    else:
        glitches = np.array([])

    all_edges = np.sort(np.concatenate([t_edges, glitches]))
    to_ns = lambda a: np.round(a * 1e9).astype(np.int64)
    return to_ns(all_edges), to_ns(np.sort(glitches))


if __name__ == "__main__":
    e, g = generate_edges()
    print(f"{e.size} edges ({g.size} glitches) over {DURATION_S} s")
