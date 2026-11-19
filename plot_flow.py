"""
plot_flow.py — 画滤波前 / 滤波后（仿真时加真值）流量随时间变化

用法：
  python3 plot_flow.py                       画 data/ 里最新的一次
  python3 plot_flow.py sim_20261115_143707   指定某次（前缀、文件名或路径都行）
同名的 edges_*.csv 自动配对；输出：data/ 里同名 .png
"""

import csv
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

DATA_DIR = Path(__file__).resolve().parent / "data"


def find_flow_csv(arg):
    """无参数 -> data/ 里最新的 flow_*.csv；有参数 -> 路径 / 文件名 / 前缀"""
    if arg is None:
        files = sorted(DATA_DIR.glob("flow_*.csv"), key=lambda p: p.stat().st_mtime)
        if not files:
            sys.exit(f"{DATA_DIR} 里没有 flow_*.csv，先运行 main.py")
        return files[-1]
    name = arg if arg.endswith(".csv") else f"{arg}.csv"
    for cand in (Path(arg), DATA_DIR / name, DATA_DIR / f"flow_{name}"):
        if cand.is_file():
            return cand
    sys.exit(f"找不到 {arg}（查过当前目录和 {DATA_DIR}）")


def load(path):
    with open(path) as f:
        r = csv.DictReader(f)
        cols = {k: [] for k in r.fieldnames}
        for row in r:
            for k in cols:
                cols[k].append(float(row[k]))
    return cols


def main():
    flow_csv = find_flow_csv(sys.argv[1] if len(sys.argv) > 1 else None)
    edge_csv = flow_csv.with_name("edges_" + flow_csv.name.removeprefix("flow_"))
    if not edge_csv.is_file():
        edge_csv = None
    print(f"plotting {flow_csv.name}" + (f" + {edge_csv.name}" if edge_csv else ""))
    d = load(flow_csv)
    t = d["t_s"]
    has_true = "q_true_mlmin" in d

    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True,
                             gridspec_kw={"height_ratios": [3, 1]})
    ax = axes[0]
    ax.plot(t, d["q_raw_mlmin"], color="#9aa5b1", lw=0.8, label="raw (single interval)")
    ax.plot(t, d["q_fir_mlmin"], color="#1f6feb", lw=1.8, label="FIR (200 ms window)")
    if has_true:
        ax.plot(t, d["q_true_mlmin"], color="#d1242f", lw=1.2, ls="--", label="true mean flow")
    ax.set_ylabel("Flow (mL/min)")
    ax.set_title(f"Flow sensor GPIO16 — {flow_csv.name}")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper right")

    ax2 = axes[1]
    if has_true:
        err = [a - b for a, b in zip(d["q_fir_mlmin"], d["q_true_mlmin"])]
        ax2.plot(t, err, color="#1f6feb", lw=1)
        ax2.axhline(0, color="k", lw=0.6)
        ax2.set_ylabel("FIR − true\n(mL/min)")
    else:
        ax2.plot(t, d["loop_lag_us"], color="#8250df", lw=0.6)
        ax2.set_ylabel("loop lag (µs)")
    if edge_csv:
        e = load(edge_csv)
        rej = [ts for ts, a in zip(e["t_s"], e["accepted"]) if a == 0]
        for ts in rej:
            ax.axvline(ts, color="orange", lw=0.6, alpha=0.7)
        if rej:
            ax.plot([], [], color="orange", lw=0.6, label=f"rejected glitch ({len(rej)})")
            ax.legend(loc="upper right")
    ax2.set_xlabel("Time (s)")
    ax2.grid(alpha=0.3)

    fig.tight_layout()
    png = flow_csv.with_suffix(".png")
    fig.savefig(png, dpi=130)
    print(f"wrote {png.parent.name}/{png.name}")


if __name__ == "__main__":
    main()
