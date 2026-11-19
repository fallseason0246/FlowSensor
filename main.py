"""
main.py — 1 ms 主循环：取上升沿 -> 计算流量 -> 记录 CSV

用法：
  仿真（任何电脑都能跑，虚拟时钟，几秒跑完）：
    python3 main.py --sim
  实机（Raspberry Pi 4，GPIO16）：
    python3 main.py --gpio --duration 20
    python3 main.py --gpio --bias pull-up      # 不经过 BSS138、直接回环测试时

输出（都在 data/ 子文件夹，不存在时自动创建）：
  data/flow_<mode>_<时间>.csv   每 1 ms 一行
  data/edges_<mode>_<时间>.csv  每个上升沿一行（是否被当作毛刺丢弃）
之后直接 python3 plot_flow.py 就会画最新这次的结果
"""

import argparse
import csv
import time
from datetime import datetime
from pathlib import Path

from flow_core import FlowEstimator, NS_PER_S

LOOP_NS = 1_000_000   # 1 ms
DATA_DIR = Path(__file__).resolve().parent / "data"   # 与运行目录无关


def process_tick(est, now_ns, new_edges, t0_ns, edge_log):
    """主循环每一轮的处理（仿真和实机共用）"""
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
    irqsign = False
    pending = []
    for now in range(0, duration_ns, LOOP_NS):
        # —— 模拟“中断回调”：这 1 ms 内到达的上升沿进入队列并置位 irqsign ——
        while i < edges_ns.size and edges_ns[i] <= now:
            pending.append(int(edges_ns[i]))
            i += 1
            irqsign = True

        # —— 主循环本体 ——
        new = []
        if irqsign:
            irqsign = False
            new, pending = pending, []
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
            # 绝对时间排程：每轮目标时间 next_t += 1 ms，不累积误差
            next_t += LOOP_NS
            delay = next_t - time.monotonic_ns()
            if delay > 0:
                time.sleep(delay / NS_PER_S)
            now = time.monotonic_ns()
            lag_us = (now - next_t) / 1000

            new = []
            if src.irqsign.is_set():
                src.irqsign.clear()
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
    ap.add_argument("--duration", type=float, default=None, help="秒；gpio 默认 20")
    ap.add_argument("--chip", default="/dev/gpiochip0") 
    ap.add_argument("--line", type=int, default=16)
    ap.add_argument("--bias", default="disabled",
                    choices=["disabled", "pull-up", "pull-down"])
    ap.add_argument("--k", type=float, default=98.0, help="Hz per L/min")
    ap.add_argument("--window", type=float, default=0.200, help="FIR 窗口 s")
    ap.add_argument("--min-interval", type=float, default=0.010, help="毛刺门限 s")
    ap.add_argument("--timeout", type=float, default=0.500, help="判零超时 s")
    ap.add_argument("--out", default=None, help="CSV 文件名前缀（仍存到 data/）")
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
