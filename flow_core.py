"""
flow_core.py - flow calculation core (no gpiod dependency, shared by sim and hardware).

Input:  kernel timestamps of rising edges (ns, CLOCK_MONOTONIC)
Output: unfiltered flow q_raw and FIR-filtered flow q_fir (mL/min)

Conversion: F[Hz] = K * Q[L/min]
  => Q[mL/min] = 1000 * F / K = 1e12 / (K * period_ns)

Sensor (replaced 2026-11): low-flow Hall water flow sensor, 1/4" quick connect
  Range 0.05-0.8 L/min, DC 5 V / 10 mA max, <= 0.35 MPa, liquid <= 60 C, IP65
  Connectors XH-3P / XH-2P
  The datasheet gives no K (pulses/L). K_HZ_PER_LPM below is a placeholder carried
  over from the old sensor - replace it after weigh calibration with count_pulse.py.
"""

from collections import deque

NS_PER_S = 1_000_000_000

# ---------- sensor parameters (all scripts take their defaults from here) ----------
K_HZ_PER_LPM = 98.0       # PLACEHOLDER - calibrate by weighing
Q_MIN_MLMIN = 50.0        # rated min 0.05 L/min
Q_MAX_MLMIN = 800.0       # rated max 0.8 L/min
# Glitch threshold: at K=98 the full-scale period is ~12.8 ms and pump ripple makes it
# shorter, so the old 10 ms was too close -> use 5 ms
MIN_INTERVAL_S = 0.005
# Zero timeout: at K=98 the period at min flow is ~204 ms; 0.5 s is ~2.5 periods
TIMEOUT_S = 0.500


class FlowEstimator:
    def __init__(self,
                 k_hz_per_lpm: float = K_HZ_PER_LPM,
                 fir_window_s: float = 0.200,             # FIR time window
                 min_interval_s: float = MIN_INTERVAL_S,  # shorter interval = glitch
                 timeout_s: float = TIMEOUT_S,            # no edge for this long => flow 0
                 max_edges: int = 256):
        self.k = k_hz_per_lpm
        self.win_ns = int(fir_window_s * NS_PER_S)
        self.min_ns = int(min_interval_s * NS_PER_S)
        self.timeout_ns = int(timeout_s * NS_PER_S)

        self.edges = deque(maxlen=max_edges)   # accepted edge timestamps
        self.last_interval_ns = None           # latest valid interval
        self.accepted = 0
        self.rejected = 0

    # ---------- conversion ----------
    def q_from_period(self, period_ns: float) -> float:
        """period (ns) -> flow (mL/min)"""
        if period_ns <= 0:
            return 0.0
        return 1e12 / (self.k * period_ns)

    # ---------- input ----------
    def add_edge(self, ts_ns: int) -> bool:
        """Add one rising edge. Returns True if accepted, False if dropped as a glitch."""
        if self.edges:
            dt = ts_ns - self.edges[-1]
            if dt < self.min_ns:
                self.rejected += 1
                return False
            self.last_interval_ns = dt
        self.edges.append(ts_ns)
        self.accepted += 1
        return True

    # ---------- output ----------
    def _timed_out(self, now_ns: int) -> bool:
        return (not self.edges) or (now_ns - self.edges[-1] > self.timeout_ns)

    def raw(self, now_ns: int) -> float:
        """Unfiltered: convert the latest interval directly, hold until the next edge."""
        if self.last_interval_ns is None or self._timed_out(now_ns):
            return 0.0
        return self.q_from_period(self.last_interval_ns)

    def fir(self, now_ns: int):
        """
        FIR (moving average over a time window):
          With n >= 2 edges in [now - win, now],
          mean period = (t_newest - t_oldest) / (n - 1).
          With fewer than 2, fall back to the last two edges.
        Also: if the time since the last edge already exceeds the mean period,
          treat "now" as a virtual edge and re-average (an upper bound on flow).
          This lets the reading fall promptly when flow drops, without being
          pulled down by a single long interval.
        Returns (q_fir, n_in_window)
        """
        if len(self.edges) < 2 or self._timed_out(now_ns):
            return 0.0, 0

        t0 = now_ns - self.win_ns
        n = 0
        first = None
        for ts in reversed(self.edges):     # newest to oldest
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

        # Waiting longer than the mean period: assume an edge arrives right now and
        # re-average (upper bound on flow), so the reading decays smoothly
        elapsed = now_ns - last
        if elapsed > period:
            period = (now_ns - first) / (n_int + 1)

        return self.q_from_period(period), n
