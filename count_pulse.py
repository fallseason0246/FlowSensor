"""
count_pulse.py — 称重标定用的脉冲计数器

在设定的计数时间内统计流量计脉冲：原始计数 + 去毛刺计数，可以暂停。
结束后（可选）输入称重质量，直接算出 mL/脉冲 和 K。

用法：
  python3 count_pulse.py              # 一直计数，按 q 结束
  python3 count_pulse.py 120          # 计数 120 s（只算计数状态的时间，暂停不计）
  python3 count_pulse.py 60 --sim     # 不接硬件，用仿真脉冲测试（任何电脑）
  python3 count_pulse.py --mass 152.3 --temp 31   # 质量和水温也可以在命令行直接给

运行时按键：
  p 或 空格   暂停 / 继续（暂停期间的脉冲写进 CSV，但不计数）
  q           提前结束（Ctrl-C 也会正常保存）

输出（data/ 子文件夹）：
  count_<mode>_<时间>.csv   每个上升沿、每次开始/暂停/继续/结束各一行
  count_log.csv             每次运行追加一行汇总，重复标定时直接看这张表
"""

import argparse
import csv
import os
import sys
import time
from datetime import datetime
from pathlib import Path

from flow_core import FlowEstimator, NS_PER_S

DATA_DIR = Path(__file__).resolve().parent / "data"
LOG_CSV = DATA_DIR / "count_log.csv"
TICK_S = 0.02        # 主循环周期：读按键、取边沿
STATUS_S = 0.25      # 状态行刷新周期
FACTORY_K = 98.0     # 厂家系数 Hz per L/min，只用于显示估算流量


def water_density(t_c):
    """纯水密度 g/mL（Thiesen 公式，0–40°C 足够准）"""
    return 1.0 - (t_c + 288.9414) / (508929.2 * (t_c + 68.12963)) * (t_c - 3.9863) ** 2


# ---------------------------------------------------------------- 输入源
class SimEdgeSource:
    """仿真输入：把 sim_source 的脉冲序列按真实时间循环播放，接口同 GpioEdgeSource"""

    def __init__(self):
        from sim_source import generate_edges, DURATION_S
        edges, _ = generate_edges()
        self.pattern = [int(x) for x in edges]
        self.period = int(DURATION_S * NS_PER_S)
        self.i = 0
        self.t0 = None

    def start(self):
        self.t0 = time.monotonic_ns()

    def drain(self):
        now = time.monotonic_ns()
        out = []
        n = len(self.pattern)
        while True:
            cyc, k = divmod(self.i, n)
            ts = self.t0 + cyc * self.period + self.pattern[k]
            if ts > now:
                return out
            out.append(ts)
            self.i += 1

    def stop(self):
        pass


# ---------------------------------------------------------------- 按键
class Keys:
    """非阻塞读单个按键；stdin 不是终端（如重定向）时不读按键"""

    def __init__(self):
        self.enabled = sys.stdin.isatty()
        self.win = os.name == "nt"
        self.old = None

    def __enter__(self):
        if self.enabled and not self.win:
            import termios
            import tty
            self.old = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        return self

    def __exit__(self, *exc):
        if self.old is not None:
            import termios
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old)
            self.old = None

    def read(self):
        if not self.enabled:
            return []
        out = []
        if self.win:
            import msvcrt
            while msvcrt.kbhit():
                out.append(msvcrt.getwch())
        else:
            import select
            fd = sys.stdin.fileno()
            while select.select([fd], [], [], 0)[0]:
                b = os.read(fd, 1)
                if not b:
                    break
                out.append(b.decode(errors="ignore"))
        return out


# ---------------------------------------------------------------- 计数
class PulseCounter:
    HEADER = ["t_s", "active_s", "event", "dt_ms", "accepted", "raw_total", "clean_total"]

    def __init__(self, min_interval_s, t0):
        # 去毛刺沿用 flow_core 的规则：距上一个有效边沿 < min_interval 视为毛刺
        self.est = FlowEstimator(min_interval_s=min_interval_s)
        self.t0 = t0
        self.raw = 0
        self.paused_edges = 0
        self.pauses = 0
        self.prev_ts = None
        self.rows = []

    @property
    def clean(self):
        return self.est.accepted

    @property
    def rejected(self):
        return self.est.rejected

    def _t(self, ts):
        return f"{(ts - self.t0) / NS_PER_S:.6f}"

    def edge(self, ts, counting, active_s):
        dt_ms = "" if self.prev_ts is None else f"{(ts - self.prev_ts) / 1e6:.3f}"
        self.prev_ts = ts
        if counting:
            self.raw += 1
            ok = int(self.est.add_edge(ts))
            self.rows.append([self._t(ts), f"{active_s:.3f}", "edge", dt_ms, ok,
                              self.raw, self.clean])
        else:
            self.paused_edges += 1
            self.rows.append([self._t(ts), f"{active_s:.3f}", "edge_paused", dt_ms, "",
                              self.raw, self.clean])

    def event(self, name, ts, active_s):
        if name == "pause":
            self.pauses += 1
        self.rows.append([self._t(ts), f"{active_s:.3f}", name, "", "",
                          self.raw, self.clean])


def run(src, duration_s, counter, keys):
    """主循环；返回 (计数时间 s, 结束原因)"""
    dur_ns = None if duration_s is None else int(duration_s * NS_PER_S)
    counting = True
    active_ns = 0
    last = counter.t0                 # 上次结算到的时刻
    next_status = 0.0
    reason = "duration"
    show_status = sys.stdout.isatty()   # 输出被重定向时不刷状态行
    counter.event("start", last, 0.0)

    def settle(end):
        """把 (last, end] 内的边沿按当前状态记账，并推进计时"""
        nonlocal active_ns, last
        for ts in edges:
            if ts < counter.t0 or ts > end:      # 开始前 / 计时到点后的边沿不算
                continue
            a = active_ns + (ts - last) if counting else active_ns
            counter.edge(ts, counting, a / NS_PER_S)
        if counting:
            active_ns += end - last
        last = end

    try:
        while True:
            time.sleep(TICK_S)
            edges = src.drain()
            now = time.monotonic_ns()     # 在取边沿之后读时间：已取到的边沿都 <= now

            if counting and dur_ns is not None and active_ns + (now - last) >= dur_ns:
                settle(last + (dur_ns - active_ns))   # 截在到点的那一刻
                break
            settle(now)

            quit_ = False
            for ch in keys.read():
                if ch in ("p", "P", " "):
                    counting = not counting
                    counter.event("resume" if counting else "pause", now, active_ns / NS_PER_S)
                elif ch in ("q", "Q"):
                    quit_ = True
            if quit_:
                reason = "key q"
                break

            if show_status and time.monotonic() >= next_status:
                next_status = time.monotonic() + STATUS_S
                state = "计数" if counting else "暂停"
                total = f"/{duration_s:g}" if duration_s is not None else ""
                print(f"\r[{state}] {active_ns / NS_PER_S:7.1f}{total} s   "
                      f"原始 {counter.raw:6d}   去毛刺 {counter.clean:6d}   ",
                      end="", flush=True)
    except KeyboardInterrupt:
        reason = "Ctrl-C"
        edges = src.drain()
        settle(time.monotonic_ns())

    counter.event("end", last, active_ns / NS_PER_S)
    print()
    return active_ns / NS_PER_S, reason


# ---------------------------------------------------------------- 结果
def ask_float(prompt):
    if not sys.stdin.isatty():
        return None
    while True:
        s = input(prompt).strip()
        if not s:
            return None
        try:
            return float(s)
        except ValueError:
            print("  请输入数字，或直接回车跳过")


def append_log(row):
    header = ["time", "csv", "mode", "active_s", "raw", "clean", "rejected",
              "paused_edges", "pauses", "min_interval_ms", "mass_g", "temp_c",
              "volume_ml", "ml_per_pulse", "k_hz_per_lpm", "q_mean_mlmin", "stop"]
    new = not LOG_CSV.exists()
    with open(LOG_CSV, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(header)
        w.writerow([row.get(h, "") for h in header])


def main():
    ap = argparse.ArgumentParser(description="称重标定用的脉冲计数器")
    ap.add_argument("duration", nargs="?", type=float, default=None,
                    help="计数时间 s（暂停不计）；不给则一直计到按 q")
    ap.add_argument("--sim", action="store_true", help="用仿真脉冲，不需要硬件")
    ap.add_argument("--chip", default="/dev/gpiochip0")
    ap.add_argument("--line", type=int, default=16)
    ap.add_argument("--bias", default="disabled",
                    choices=["disabled", "pull-up", "pull-down"])
    ap.add_argument("--min-interval", type=float, default=0.010, help="毛刺门限 s")
    ap.add_argument("--mass", type=float, default=None, help="称重质量 g（不给则结束时询问）")
    ap.add_argument("--temp", type=float, default=None, help="水温 °C（不给则结束时询问）")
    ap.add_argument("--out", default=None, help="CSV 文件名前缀（仍存到 data/）")
    args = ap.parse_args()

    if args.sim:
        src, tag = SimEdgeSource(), "sim"
    else:
        from gpio_source import GpioEdgeSource
        src, tag = GpioEdgeSource(chip=args.chip, line=args.line, bias=args.bias), "gpio"

    keys = Keys()
    src.start()
    counter = PulseCounter(args.min_interval, time.monotonic_ns())
    how = "p/空格 暂停，q 结束" if keys.enabled else "stdin 不是终端，按键不可用，Ctrl-C 结束"
    print(f"开始计数（{tag}，{how}）")
    try:
        with keys:
            active_s, reason = run(src, args.duration, counter, keys)
    finally:
        src.stop()

    # —— CSV ——
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    DATA_DIR.mkdir(exist_ok=True)
    out_csv = DATA_DIR / f"count_{args.out or f'{tag}_{stamp}'}.csv"
    with open(out_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(PulseCounter.HEADER)
        w.writerows(counter.rows)

    # —— 结果 ——
    clean, raw = counter.clean, counter.raw
    freq = clean / active_s if active_s > 0 else 0.0
    print("=" * 44)
    print(f"计数时间      {active_s:.2f} s（暂停 {counter.pauses} 次，结束：{reason}）")
    print(f"原始脉冲      {raw}")
    print(f"去毛刺脉冲    {clean}（丢弃 {counter.rejected} 个，门限 {args.min_interval * 1000:g} ms）")
    if counter.paused_edges:
        print(f"暂停期间脉冲  {counter.paused_edges}（不计入）")
    print(f"平均频率      {freq:.3f} Hz ≈ {freq / FACTORY_K * 1000:.1f} mL/min（按厂家 K={FACTORY_K:g}）")
    print(f"CSV           data/{out_csv.name}")

    mass = args.mass if args.mass is not None else ask_float("称重质量 g（回车跳过）：")
    log = {"time": stamp, "csv": out_csv.name, "mode": tag, "active_s": f"{active_s:.3f}",
           "raw": raw, "clean": clean, "rejected": counter.rejected,
           "paused_edges": counter.paused_edges, "pauses": counter.pauses,
           "min_interval_ms": f"{args.min_interval * 1000:g}", "stop": reason}
    if mass is not None and clean > 0:
        temp = args.temp if args.temp is not None else ask_float("水温 °C（回车按 25）：")
        temp = 25.0 if temp is None else temp
        vol = mass / water_density(temp)
        ml_per_pulse = vol / clean
        k = clean / vol * 1000 / 60            # 脉冲/L ÷ 60 = Hz per (L/min)
        q = vol / active_s * 60 if active_s > 0 else 0.0
        print(f"体积          {vol:.2f} mL（{mass:g} g，{temp:g}°C，ρ={water_density(temp):.5f}）")
        print(f"每脉冲        {ml_per_pulse:.5f} mL/脉冲")
        print(f"K             {k:.2f} Hz/(L/min)")
        print(f"平均流量      {q:.1f} mL/min（体积 / 计数时间，含启停过渡）")
        log.update(mass_g=f"{mass:g}", temp_c=f"{temp:g}", volume_ml=f"{vol:.3f}",
                   ml_per_pulse=f"{ml_per_pulse:.6f}", k_hz_per_lpm=f"{k:.3f}",
                   q_mean_mlmin=f"{q:.2f}")
    append_log(log)
    print("=" * 44)
    print(f"最终脉冲数（去毛刺）：{clean}    原始：{raw}")


if __name__ == "__main__":
    main()
