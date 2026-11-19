"""
flow_core.py — 流量计算核心（不依赖 gpiod，仿真和实机共用）

输入：上升沿的内核时间戳（ns，CLOCK_MONOTONIC）
输出：滤波前流量 q_raw、FIR 滤波后流量 q_fir（mL/min）

流量换算（厂家参数）：F[Hz] = K * Q[L/min]，K = 98
  => Q[mL/min] = 1000 * F / K = 1e12 / (K * period_ns)
  10.2*K ml/s
  量程 0.3-1 L
"""

from collections import deque

NS_PER_S = 1_000_000_000


class FlowEstimator:
    def __init__(self,
                 k_hz_per_lpm: float = 98.0,     # 厂家系数，标定后替换
                 fir_window_s: float = 0.200,    # FIR 时间窗口
                 min_interval_s: float = 0.010,  # 小于此间隔视为毛刺
                 timeout_s: float = 0.500,       # 超过此时间无上升沿 => 流量 0
                 max_edges: int = 256):
        self.k = k_hz_per_lpm
        self.win_ns = int(fir_window_s * NS_PER_S)
        self.min_ns = int(min_interval_s * NS_PER_S)
        self.timeout_ns = int(timeout_s * NS_PER_S)

        self.edges = deque(maxlen=max_edges)   # 已接受的上升沿时间戳
        self.last_interval_ns = None           # 最近一个有效间隔
        self.accepted = 0
        self.rejected = 0

    # ---------- 换算 ----------
    def q_from_period(self, period_ns: float) -> float:
        """周期(ns) -> 流量(mL/min)"""
        if period_ns <= 0:
            return 0.0
        return 1e12 / (self.k * period_ns)

    # ---------- 输入 ----------
    def add_edge(self, ts_ns: int) -> bool:
        """加入一个上升沿；返回 True 表示接受，False 表示当作毛刺丢弃"""
        if self.edges:
            dt = ts_ns - self.edges[-1]
            if dt < self.min_ns:
                self.rejected += 1
                return False
            self.last_interval_ns = dt
        self.edges.append(ts_ns)
        self.accepted += 1
        return True

    # ---------- 输出 ----------
    def _timed_out(self, now_ns: int) -> bool:
        return (not self.edges) or (now_ns - self.edges[-1] > self.timeout_ns)

    def raw(self, now_ns: int) -> float:
        """滤波前：最近一个间隔直接换算，保持到下一个上升沿"""
        if self.last_interval_ns is None or self._timed_out(now_ns):
            return 0.0
        return self.q_from_period(self.last_interval_ns)

    def fir(self, now_ns: int):
        """
        FIR（时间窗口滑动平均）：
          窗口 [now - win, now] 内有 n>=2 个上升沿时，
          平均周期 = (t_最新 - t_最早) / (n - 1)
          不足 2 个时，退回用最近两个上升沿。
        另外：距上一个上升沿的时间若已超过平均周期，
          把“此刻”当作一个虚拟上升沿重新求平均（流量上限），
          让流量下降时读数能及时往下走，又不会被单个偏长的间隔拉低。
        返回 (q_fir, n_in_window)
        """
        if len(self.edges) < 2 or self._timed_out(now_ns):
            return 0.0, 0

        t0 = now_ns - self.win_ns
        n = 0
        first = None
        for ts in reversed(self.edges):     # 从新到旧
            if ts < t0:
                break
            first = ts
            n += 1

        last = self.edges[-1]
        if n < 2:
            first, n_int = self.edges[-2], 1
        else:
            n_int = n - 1
        period = (last - first) / n_int

        # 等待下一个上升沿的时间已超过平均周期：
        # 假设“此刻”刚好来一个上升沿，重新求平均周期（流量上限），读数平滑下降
        elapsed = now_ns - last
        if elapsed > period:
            period = (now_ns - first) / (n_int + 1)

        return self.q_from_period(period), n
