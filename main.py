"""
main.py - 1 ms main loop: read edges -> compute flow -> log to CSV.

Usage:
  Simulation (any PC, virtual clock, finishes in seconds):
    python3 main.py --sim
  Hardware (Raspberry Pi 4, GPIO16):
    python3 main.py --gpio --duration 20
    python3 main.py --gpio --bias pull-up      # direct loopback, no BSS138

Output (in data/, created if missing):
  data/flow_<mode>_<time>.csv   one row per 1 ms tick
  data/edges_<mode>_<time>.csv  one row per rising edge (accepted or rejected as glitch)
Then run python3 plot_flow.py to plot the latest run.
"""

import argparse
import csv
import time
from datetime import datetime
from pathlib import Path

from flow_core import FlowEstimator, NS_PER_S, K_HZ_PER_LPM, MIN_INTERVAL_S, TIMEOUT_S

LOOP_NS = 1_000_000   # 1 ms
DATA_DIR = Path(__file__).resolve().parent / "data"   # independent of cwd


def process_tick(est, now_ns, new_edges, t0_ns, edge_log):
    """Per-tick processing, shared by sim and hardware."""
    for ts in new_edges:
        ok = est.add_edge(ts)
        edge_log.append(((ts - t0_ns) / NS_PER_S, int(ok)))
    q_raw = est.raw(now_ns)
    q_fir, n_win = est.fir(now_ns)
    return q_raw, q_fir, n_win


def run_sim(est, args):
    import numpy as np
    from sim_source import generate_edges, q_true, DURATION_S

    edges_ns, glitch_ns = generate_edges()
    duration_ns = int((args.duration or DURATION_S) * NS_PER_S)

    rows, edge_log = [], []
    t0 = 0
    i = 0
    for now in range(0, duration_ns, LOOP_NS):
        # same as GpioEdgeSource.drain(): all edges that arrived up to now
        new = []
        while i < edges_ns.size and edges_ns[i] <= now:
            new.append(int(edges_ns[i]))
            i += 1
        q_raw, q_fir, n_win = process_tick(est, now, new, t0, edge_log)
        t_s = now / NS_PER_S
        rows.append([f"{t_s:.3f}", f"{q_raw:.2f}", f"{q_fir:.2f}", n_win,
                     len(new), est.rejected, f"{float(q_true(t_s)):.2f}"])

    header = ["t_s", "q_raw_mlmin", "q_fir_mlmin", "n_win", "new_edges",
              "rejected_total", "q_true_mlmin"]
    print(f"sim: {edges_ns.size} edges, {glitch_ns.size} glitches injected, "
          f"{est.rejected} rejected")
    return header, rows, edge_log


def run_gpio(est, args):
    from gpio_source import GpioEdgeSource

    src = GpioEdgeSource(chip=args.chip, line=args.line, bias=args.bias)
    src.start()
    rows, edge_log = [], []
    t0 = time.monotonic_ns()
    next_t = t0
    end_t = t0 + int(args.duration * NS_PER_S)
    print(f"gpio: listening on {args.chip} line {args.line} for {args.duration} s ...")
    try:
        while next_t < end_t:
            # absolute-time scheduling: target += 1 ms each tick, no drift
            next_t += LOOP_NS
            delay = next_t - time.monotonic_ns()
            if delay > 0:
                time.sleep(delay / NS_PER_S)
            now = time.monotonic_ns()
            lag_us = (now - next_t) / 1000

            new = src.drain()
            q_raw, q_fir, n_win = process_tick(est, now, new, t0, edge_log)
            rows.append([f"{(now - t0) / NS_PER_S:.4f}", f"{q_raw:.2f}",
                         f"{q_fir:.2f}", n_win, len(new), est.rejected,
                         f"{lag_us:.0f}"])
    except KeyboardInterrupt:
        print("stopped by user")
    finally:
        src.stop()

    header = ["t_s", "q_raw_mlmin", "q_fir_mlmin", "n_win", "new_edges",
              "rejected_total", "loop_lag_us"]
    print(f"gpio: {est.accepted} edges accepted, {est.rejected} rejected")
    return header, rows, edge_log


def main():
    ap = argparse.ArgumentParser()
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--sim", action="store_true")
    mode.add_argument("--gpio", action="store_true")
    ap.add_argument("--duration", type=float, default=None, help="seconds; gpio default 20")
    ap.add_argument("--chip", default="/dev/gpiochip0") 
    ap.add_argument("--line", type=int, default=16)
    ap.add_argument("--bias", default="disabled",
                    choices=["disabled", "pull-up", "pull-down"])
    ap.add_argument("--k", type=float, default=K_HZ_PER_LPM, help="Hz per L/min")
    ap.add_argument("--window", type=float, default=0.200, help="FIR window, s")
    ap.add_argument("--min-interval", type=float, default=MIN_INTERVAL_S, help="glitch threshold, s")
    ap.add_argument("--timeout", type=float, default=TIMEOUT_S, help="zero-flow timeout, s")
    ap.add_argument("--out", default=None, help="CSV file name prefix (still saved in data/)")
    args = ap.parse_args()

    est = FlowEstimator(k_hz_per_lpm=args.k, fir_window_s=args.window,
                        min_interval_s=args.min_interval, timeout_s=args.timeout)

    if args.sim:
        header, rows, edge_log = run_sim(est, args)
        tag = "sim"
    else:
        if args.duration is None:
            args.duration = 20.0
        header, rows, edge_log = run_gpio(est, args)
        tag = "gpio"

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    prefix = args.out or f"{tag}_{stamp}"
    DATA_DIR.mkdir(exist_ok=True)
    flow_csv = DATA_DIR / f"flow_{prefix}.csv"
    edge_csv = DATA_DIR / f"edges_{prefix}.csv"
    with open(flow_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)
    with open(edge_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "accepted"])
        w.writerows([[f"{t:.6f}", a] for t, a in edge_log])
    print(f"wrote data/{flow_csv.name} ({len(rows)} rows), "
          f"data/{edge_csv.name} ({len(edge_log)} edges)")
    print("plot: python3 plot_flow.py")


if __name__ == "__main__":
    main()
