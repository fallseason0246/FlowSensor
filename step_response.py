"""
step_response.py - pump PWM steps + synchronized flow logging, to measure dead time and time constant.

Drives one pump (TB6612FNG + hardware PWM) through a given PWM sequence while reading
the flow sensor every 1 ms. Afterwards each step is fitted with the two-point method
(28.3% / 63.2%) to a first-order-plus-dead-time model:
  theta dead time, tau time constant, steady flow before/after, gain
The fit uses the FIR-filtered flow (what the controller actually sees),
so theta includes both the pump lag and the FIR window delay.

Usage:
  python3 step_response.py --pwm1 30 2 --pwm2 60 2 --pwm3 30 2          # hardware (Pi)
  python  step_response.py --pwm1 30 2 --pwm2 60 2 --pwm3 30 2 --sim    # simulation on a PC
  Each segment is --pwmN <duty%> <seconds>, run in increasing N. The pump is off
  before the first segment and after the last.
  Use the first segment as warm-up (e.g. --pwm1 30 3); the fit for a step from standstill is unreliable.
  Ctrl-C stops the pump at any time; data collected so far is still saved and fitted.

Wiring (defaults; override with arguments if yours differs):
  PWMA -> GPIO12 (hardware PWM channel 0, --pwm-channel 0); second pump on GPIO13 (--pwm-channel 1)
  AIN1 -> GPIO5   AIN2 -> GPIO6   STBY -> GPIO26   (-1 = pin is hard-wired, not driven)
  Flow sensor signal -> GPIO16 (through BSS138)
  /boot/firmware/config.txt needs: dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4

Output (in data/):
  step_<mode>_<time>.csv        one row per 1 ms: duty, q_raw, q_fir ...
  step_edges_<mode>_<time>.csv  one row per rising edge (accepted or rejected as glitch)
  step_<mode>_<time>.png        flow + duty plot, with the 28.3% / 63.2% points of each step
  step_log.csv                  one fit result per step, appended across runs
"""

import argparse
import csv
import math
import re
import sys
import time
from datetime import datetime
from pathlib import Path

from flow_core import FlowEstimator, NS_PER_S, K_HZ_PER_LPM, MIN_INTERVAL_S, TIMEOUT_S

DATA_DIR = Path(__file__).resolve().parent / "data"
STEP_LOG = DATA_DIR / "step_log.csv"
LOOP_NS = 1_000_000      # 1 ms sample period
STEADY_FRAC = 0.3        # mean of the last 30% of a segment = steady state
MIN_DELTA = 10.0         # skip fit if steady-state change is below this (mL/min)


# ======================================================================
# Hardware: hardware PWM (sysfs) + TB6612 direction / STBY pins (libgpiod)
# ======================================================================
class SysfsPwm:
    def __init__(self, chip, channel, freq_hz):
        self.base = Path(f"/sys/class/pwm/pwmchip{chip}")
        self.ch = self.base / f"pwm{channel}"
        if not self.base.exists():
            sys.exit(f"{self.base} not found. Add this line to /boot/firmware/config.txt\n"
                     f"  dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4\nthen reboot")
        if not self.ch.exists():
            (self.base / "export").write_text(str(channel))
        # after export, udev needs a moment to make the files writable by the gpio group
        deadline = time.monotonic() + 2.0
        while not all(os_access(self.ch / f) for f in ("period", "duty_cycle", "enable")):
            if time.monotonic() > deadline:
                sys.exit(f"No write access to {self.ch}: add the user to the gpio group, or run with sudo")
            time.sleep(0.02)
        self.period = int(round(1e9 / freq_hz))
        if int((self.ch / "period").read_text()) > 0:
            self._w("duty_cycle", 0)          # zero duty first so the new period is never below the old duty
        self._w("period", self.period)
        self._w("duty_cycle", 0)
        self._w("enable", 1)

    def _w(self, name, val):
        (self.ch / name).write_text(str(val))

    def set_duty(self, pct):
        self._w("duty_cycle", int(self.period * pct / 100))

    def close(self):
        try:
            self._w("duty_cycle", 0)
            self._w("enable", 0)
        except OSError:
            pass


def os_access(path):
    import os
    return os.access(path, os.W_OK)


class TB6612:
    """Direction pins + STBY. IN1/IN2 = H/L is forward; PWM=0 is short brake; on close STBY goes low (outputs hi-Z)."""

    def __init__(self, chip, in1, in2, stby, reverse):
        import gpiod
        from gpiod.line import Direction, Value
        self.Value = Value
        self.levels = {}
        if in1 >= 0:
            self.levels[in1] = not reverse
        if in2 >= 0:
            self.levels[in2] = reverse
        if stby >= 0:
            self.levels[stby] = True
        self.req = None
        if self.levels:
            self.req = gpiod.request_lines(
                chip, consumer="pump",
                config={tuple(self.levels): gpiod.LineSettings(
                    direction=Direction.OUTPUT, output_value=Value.INACTIVE)})

    def enable(self):
        if self.req:
            self.req.set_values({ln: self.Value.ACTIVE if v else self.Value.INACTIVE
                                 for ln, v in self.levels.items()})

    def close(self):
        if self.req:
            try:
                self.req.set_values({ln: self.Value.INACTIVE for ln in self.levels})
            finally:
                self.req.release()
                self.req = None


class HardwareIO:
    extra_name = "loop_lag_us"

    def __init__(self, args):
        from gpio_source import GpioEdgeSource   # exits with a message if gpiod is missing
        self.src = self.drv = self.pwm = None
        self.src = GpioEdgeSource(chip=args.chip, line=args.line, bias=args.bias)
        self.drv = TB6612(args.chip, args.in1, args.in2, args.stby, args.reverse)
        self.pwm = SysfsPwm(args.pwm_chip, args.pwm_channel, args.hz)
        self.drv.enable()          # duty is still 0, so the pump does not run yet
        self.src.start()

    def now(self):
        return time.monotonic_ns()

    def wait_until(self, t_ns):
        d = t_ns - time.monotonic_ns()
        if d > 0:
            time.sleep(d / NS_PER_S)

    def set_duty(self, pct):
        self.pwm.set_duty(pct)

    def edges(self):
        return self.src.drain()

    def extra(self, now, target):
        return f"{(now - target) / 1000:.0f}"

    def close(self):
        # order: stop PWM, then pull STBY low, then release the edge input
        for part in (self.pwm, self.drv, self.src):
            if part is None:
                continue
            try:
                part.close() if hasattr(part, "close") else part.stop()
            except Exception as e:
                print(f"Error closing {type(part).__name__}: {e}")


# ======================================================================
# Simulation: duty -> steady flow -> dead time -> first-order lag -> pump ripple -> sensor pulses
# ======================================================================
class SimIO:
    extra_name = "q_model_mlmin"
    SUB_NS = 100_000           # 0.1 ms integration step

    def __init__(self, k_hz_per_lpm, gain=4.0, d0=8.0, dead_s=0.05, tau_s=0.25):
        from sim_source import PULSE_AMP, ML_PER_ROLLER
        self.k = k_hz_per_lpm
        self.gain, self.d0 = gain, d0          # steady Q = gain * (duty - d0); d0 = start-up deadband
        self.dead_ns = int(dead_s * NS_PER_S)
        self.tau_ns = tau_s * NS_PER_S
        self.amp, self.ml_roller = PULSE_AMP, ML_PER_ROLLER
        self.t = 0
        self.duty_hist = [(0, 0.0)]
        self.q = self.vol = self.phase = 0.0
        self.pending = []

    def now(self):
        return self.t

    def wait_until(self, t_ns):
        while self.t < t_ns:
            self.t += self.SUB_NS
            d = self._duty_at(self.t - self.dead_ns)
            q_ss = max(0.0, self.gain * (d - self.d0))
            self.q += (q_ss - self.q) * self.SUB_NS / self.tau_ns
            q_inst = self.q * (1 + self.amp * math.sin(2 * math.pi * self.vol / self.ml_roller))
            dt_s = self.SUB_NS / NS_PER_S
            self.vol += q_inst / 60 * dt_s
            self.phase += self.k * q_inst / 1000 * dt_s
            if self.phase >= 1.0:
                self.phase -= 1.0
                self.pending.append(self.t)

    def _duty_at(self, t):
        for ts, d in reversed(self.duty_hist):
            if ts <= t:
                return d
        return 0.0

    def set_duty(self, pct):
        self.duty_hist.append((self.t, pct))

    def edges(self):
        out, self.pending = self.pending, []
        return out

    def extra(self, now, target):
        return f"{self.q:.2f}"

    def close(self):
        pass


# ======================================================================
# Acquisition main loop
# ======================================================================
def run(io, sched, est):
    bounds, acc = [], 0.0
    for _, sec in sched:
        bounds.append(int(acc * NS_PER_S))
        acc += sec
    total_ns = int(acc * NS_PER_S)

    rows, edge_log, steps = [], [], []
    seg, duty = -1, 0.0
    t0 = io.now()
    next_t = t0
    try:
        while True:
            io.wait_until(next_t)          # absolute-time scheduling, no drift
            new = io.edges()
            now = io.now()
            el = now - t0
            if el >= total_ns:
                break
            while seg + 1 < len(sched) and el >= bounds[seg + 1]:
                seg += 1
                prev, duty = duty, sched[seg][0]
                io.set_duty(duty)
                t_cmd = (io.now() - t0) / NS_PER_S
                steps.append((t_cmd, prev, duty))
                print(f"  t={t_cmd:6.2f} s  PWM {prev:g}% -> {duty:g}%")
            for ts in new:
                if ts < t0:
                    continue
                ok = est.add_edge(ts)
                edge_log.append(((ts - t0) / NS_PER_S, int(ok)))
            q_raw = est.raw(now)
            q_fir, n_win = est.fir(now)
            rows.append([f"{el / NS_PER_S:.4f}", f"{duty:g}", seg + 1, f"{q_raw:.2f}",
                         f"{q_fir:.2f}", n_win, est.rejected, io.extra(now, next_t)])
            next_t += LOOP_NS
    except KeyboardInterrupt:
        print("\nInterrupted: pump stopped, collected data will still be saved")
    finally:
        try:
            io.set_duty(0)
        except Exception:
            pass
    return rows, edge_log, steps


# ======================================================================
# Two-point fit: first-order lag + dead time
# ======================================================================
def fit_steps(rows, steps):
    t = [float(r[0]) for r in rows]
    q = [float(r[4]) for r in rows]          # q_fir
    t_last = t[-1] if t else 0.0

    def steady(a, b):
        lo = b - STEADY_FRAC * (b - a)
        vals = [qq for tt, qq in zip(t, q) if lo <= tt <= b]
        return sum(vals) / len(vals) if vals else float("nan")

    results = []
    for i, (tc, d0, d1) in enumerate(steps):
        end = steps[i + 1][0] if i + 1 < len(steps) else t_last
        y0 = 0.0 if i == 0 else results[-1]["q_to"]    # pump is off before the first step
        y1 = steady(tc, end)
        dq = y1 - y0
        r = {"step": i + 1, "duty_from": d0, "duty_to": d1, "t_cmd": tc,
             "seg_s": end - tc, "q_from": y0, "q_to": y1,
             "gain": dq / (d1 - d0) if d1 != d0 else float("nan"),
             "theta": None, "tau": None, "t28": None, "t63": None, "note": ""}
        if math.isnan(dq) or abs(dq) < MIN_DELTA:
            r["note"] = "change too small, not fitted"
            results.append(r)
            continue
        t28 = t63 = None
        for tt, qq in zip(t, q):
            if tt < tc:
                continue
            if tt > end:
                break
            frac = (qq - y0) / dq
            if t28 is None and frac >= 0.283:
                t28 = tt
            if frac >= 0.632:
                t63 = tt
                break
        if t28 is None or t63 is None:
            r["note"] = "never reached 63.2%: segment too short or bad signal"
        else:
            tau = 1.5 * (t63 - t28)
            theta = (t63 - tc) - tau
            notes = []
            if d0 == 0:
                notes.append("from standstill: reading waits for the first two pulses, theta/tau not pump dynamics")
            if d1 == 0:
                notes.append("pump stop: drop is set by the zero-flow timeout")
            if theta < 0:
                notes.append("theta<0: not first-order, check the plot")
            if r["seg_s"] < max(theta, 0) + 4 * tau:
                notes.append("segment too short, not fully settled")
            r.update(theta=theta, tau=tau, t28=t28, t63=t63, note="; ".join(notes))
        results.append(r)
    return results


def fmt(x, spec):
    return "—" if x is None or (isinstance(x, float) and math.isnan(x)) else format(x, spec)


def print_results(results):
    print("=" * 78)
    print(f"{'#':>2}  {'PWM':>11}  {'steady mL/min':>18}  {'theta s':>9}  {'tau s':>11}  "
          f"{'gain mL/min/%':>12}")
    for r in results:
        pwm = f"{r['duty_from']:g}→{r['duty_to']:g}%"
        flows = f"{fmt(r['q_from'], '.1f')} → {fmt(r['q_to'], '.1f')}"
        print(f"{r['step']:>2}  {pwm:>11}  {flows:>18}  {fmt(r['theta'], '.3f'):>9}  "
              f"{fmt(r['tau'], '.3f'):>11}  {fmt(r['gain'], '.2f'):>12}  {r['note']}")
    print("theta = pump lag + FIR window delay (fit uses q_fir). Step 1 starts from standstill; its gain includes the start-up deadband")
    print("=" * 78)


def plot(rows, steps, results, png, title, extra_name):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed, skipping plot")
        return
    t = [float(r[0]) for r in rows]
    fig, ax = plt.subplots(figsize=(12, 6))
    ax.plot(t, [float(r[3]) for r in rows], color="#9aa5b1", lw=0.7, label="raw (single interval)")
    ax.plot(t, [float(r[4]) for r in rows], color="#1f6feb", lw=1.6, label="FIR")
    if extra_name == "q_model_mlmin":
        ax.plot(t, [float(r[7]) for r in rows], color="#d1242f", lw=1, ls="--",
                label="model mean flow (sim)")
    for r in results:
        ax.axvline(r["t_cmd"], color="#f59e0b", lw=0.6, ls=":")
        if r["t63"] is not None:
            dq = r["q_to"] - r["q_from"]
            ax.plot([r["t28"], r["t63"]],
                    [r["q_from"] + 0.283 * dq, r["q_from"] + 0.632 * dq], "o", color="k", ms=4)
            ax.annotate(f"θ={r['theta']:.2f}s\nτ={r['tau']:.2f}s", (r["t63"], r["q_from"] + 0.632 * dq),
                        xytext=(8, -4), textcoords="offset points", fontsize=8)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Flow (mL/min)")
    ax.grid(alpha=0.3)
    ax2 = ax.twinx()
    ax2.step(t, [float(r[1]) for r in rows], where="post", color="#f59e0b", lw=1.2, label="PWM duty")
    ax2.set_ylabel("Duty (%)")
    ax2.set_ylim(0, 105)
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, loc="upper left", fontsize=8)
    ax.set_title(title)
    fig.tight_layout()
    fig.savefig(png, dpi=130)
    plt.close(fig)


def append_log(stamp, csv_name, mode, results):
    header = ["time", "csv", "mode", "step", "duty_from", "duty_to", "t_cmd_s", "seg_s",
              "q_from_mlmin", "q_to_mlmin", "theta_s", "tau_s", "gain_mlmin_per_pct", "note"]
    new = not STEP_LOG.exists()
    with open(STEP_LOG, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(header)
        for r in results:
            w.writerow([stamp, csv_name, mode, r["step"], f"{r['duty_from']:g}", f"{r['duty_to']:g}",
                        f"{r['t_cmd']:.4f}", f"{r['seg_s']:.3f}", fmt(r["q_from"], ".2f"),
                        fmt(r["q_to"], ".2f"), fmt(r["theta"], ".4f"), fmt(r["tau"], ".4f"),
                        fmt(r["gain"], ".3f"), r["note"]])


# ======================================================================
def parse_args():
    ap = argparse.ArgumentParser(
        description="Pump PWM steps + synchronized flow logging", allow_abbrev=False,
        epilog="Example: python3 step_response.py --pwm1 30 2 --pwm2 60 2 --pwm3 30 2")
    nums = sorted({int(m.group(1)) for a in sys.argv[1:]
                   if (m := re.fullmatch(r"--pwm(\d+)", a))})
    for n in nums:
        ap.add_argument(f"--pwm{n}", nargs=2, type=float, metavar=("DUTY%", "SECONDS"),
                        required=True, help=f"segment {n}: duty %% and duration in s")
    ap.add_argument("--sim", action="store_true", help="simulation, no hardware")
    ap.add_argument("--pwm-chip", type=int, default=0, help="/sys/class/pwm/pwmchipN")
    ap.add_argument("--pwm-channel", type=int, default=0, help="0=GPIO12, 1=GPIO13")
    ap.add_argument("--hz", type=float, default=20000, help="PWM frequency, Hz")
    ap.add_argument("--in1", type=int, default=5, help="AIN1 GPIO, -1 = hard-wired")
    ap.add_argument("--in2", type=int, default=6, help="AIN2 GPIO, -1 = hard-wired")
    ap.add_argument("--stby", type=int, default=26, help="STBY GPIO, -1 = hard-wired")
    ap.add_argument("--reverse", action="store_true", help="reverse (swap IN1/IN2)")
    ap.add_argument("--chip", default="/dev/gpiochip0")
    ap.add_argument("--line", type=int, default=16, help="flow sensor GPIO")
    ap.add_argument("--bias", default="disabled", choices=["disabled", "pull-up", "pull-down"])
    ap.add_argument("--k", type=float, default=K_HZ_PER_LPM, help="Hz per L/min (replace after calibration)")
    ap.add_argument("--window", type=float, default=0.200, help="FIR window, s")
    ap.add_argument("--min-interval", type=float, default=MIN_INTERVAL_S, help="glitch threshold, s")
    ap.add_argument("--timeout", type=float, default=TIMEOUT_S, help="zero-flow timeout, s")
    ap.add_argument("--out", default=None, help="file name prefix (still saved in data/)")
    args = ap.parse_args()
    if not nums:
        ap.error("give at least one segment, e.g. --pwm1 30 2 --pwm2 60 2")
    sched = [tuple(getattr(args, f"pwm{n}")) for n in nums]
    for n, (d, s) in zip(nums, sched):
        if not 0 <= d <= 100:
            ap.error(f"--pwm{n}: duty {d:g} is not in 0-100")
        if s <= 0:
            ap.error(f"--pwm{n}: time {s:g} must be > 0")
    return args, sched


def main():
    args, sched = parse_args()
    est = FlowEstimator(k_hz_per_lpm=args.k, fir_window_s=args.window,
                        min_interval_s=args.min_interval, timeout_s=args.timeout)
    tag = "sim" if args.sim else "gpio"
    plan = " → ".join(f"{d:g}%×{s:g}s" for d, s in sched)
    print(f"Sequence ({tag}): {plan}, total {sum(s for _, s in sched):g} s. Ctrl-C stops the pump")

    io = SimIO(args.k) if args.sim else HardwareIO(args)
    try:
        rows, edge_log, steps = run(io, sched, est)
    finally:
        io.close()
    if not rows:
        sys.exit("No data collected")

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    prefix = args.out or f"{tag}_{stamp}"
    DATA_DIR.mkdir(exist_ok=True)
    data_csv = DATA_DIR / f"step_{prefix}.csv"
    edge_csv = DATA_DIR / f"step_edges_{prefix}.csv"
    png = DATA_DIR / f"step_{prefix}.png"
    with open(data_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "duty_pct", "segment", "q_raw_mlmin", "q_fir_mlmin", "n_win",
                    "rejected_total", io.extra_name])
        w.writerows(rows)
    with open(edge_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "accepted"])
        w.writerows([[f"{t:.6f}", a] for t, a in edge_log])

    results = fit_steps(rows, steps)
    print_results(results)
    plot(rows, steps, results, png, f"Step response — {data_csv.name}", io.extra_name)
    append_log(stamp, data_csv.name, tag, results)
    print(f"Wrote data/{data_csv.name} ({len(rows)} rows), data/{edge_csv.name} ({len(edge_log)} edges), "
          f"data/{png.name}. Fit results appended to data/{STEP_LOG.name}")
    print(f"Pulses: {est.accepted} accepted, {est.rejected} dropped as glitches")


if __name__ == "__main__":
    main()
