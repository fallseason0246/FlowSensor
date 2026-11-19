"""
sim_source.py - generate a test rising-edge sequence (simulated flow sensor on GPIO16).

True flow profile (mL/min):
  0-3 s    300
  3-6 s    150 (step)
  6-9 s     60 (low flow)
  9-11 s     0 (pump off, tests zero timeout)
  11-14 s  0 -> 300 ramp

Disturbances added:
  - Peristaltic pump ripple: sine of +/-PULSE_AMP, frequency proportional to flow
    (each roller stroke moves ML_PER_ROLLER mL - assumed value)
  - Interval jitter: Gaussian noise on each edge time (JITTER_FRAC of local period)
  - Glitches: N_GLITCH fake edges, 0.2-1 ms after a real edge
"""

import numpy as np

from flow_core import K_HZ_PER_LPM
DURATION_S = 14.0
PULSE_AMP = 0.15          # pump ripple amplitude +/-15%
ML_PER_ROLLER = 0.4       # mL per roller stroke (assumed): ~12.5 Hz ripple at 300 mL/min
JITTER_FRAC = 0.02        # interval jitter 2%
N_GLITCH = 10
SEED = 5725


def q_true(t):
    """True (mean) flow in mL/min. t can be an array."""
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
    Returns (edges_ns, glitch_ns):
      edges_ns  - all rising edges incl. glitches, sorted, int64 ns
      glitch_ns - the glitches only, for checking
    """
    rng = np.random.default_rng(seed)
    t = np.arange(0, duration_s, dt)
    qm = q_true(t)

    # pump ripple: phase advances with pumped volume
    vol_ml = np.cumsum(qm / 60.0 * dt)
    pulse_phase = 2 * np.pi * vol_ml / ML_PER_ROLLER
    q_inst = qm * (1 + PULSE_AMP * np.sin(pulse_phase))

    # sensor pulses: f = K * Q[L/min]; one edge each time phase crosses an integer
    f = K_HZ_PER_LPM * q_inst / 1000.0
    phase = np.cumsum(f * dt)
    idx = np.nonzero(np.diff(np.floor(phase)) > 0)[0] + 1
    t_edges = t[idx]

    # jitter
    local_period = 1.0 / np.maximum(f[idx], 1e-6)
    t_edges = t_edges + rng.normal(0, JITTER_FRAC, size=t_edges.size) * local_period
    t_edges = np.sort(t_edges)

    # glitches
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
