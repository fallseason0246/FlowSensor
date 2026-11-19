"""
count_pulse.py - pulse counter for weigh-based calibration.

Counts sensor pulses over a set time: raw count + deglitched count, with pause.
At the end, optionally enter the weighed mass to get mL/pulse and K.

Usage:
  python3 count_pulse.py              # count until q is pressed
  python3 count_pulse.py 120          # count for 120 s of active time (pauses excluded)
  python3 count_pulse.py 60 --sim     # simulated pulses, no hardware (any PC)
  python3 count_pulse.py --mass 152.3 --temp 31   # give mass and water temp on the command line

Keys while running:
  p or space  pause / resume (edges during pause are logged but not counted)
  q           stop early (Ctrl-C also saves normally)

Output (in data/):
  count_<mode>_<time>.csv   one row per edge and per start/pause/resume/end event
  count_log.csv             one summary row appended per run, for comparing repeat runs
"""

import argparse
import csv
import os
import sys
import time
from datetime import datetime
from pathlib import Path

from flow_core import FlowEstimator, NS_PER_S, K_HZ_PER_LPM, MIN_INTERVAL_S

DATA_DIR = Path(__file__).resolve().parent / "data"
LOG_CSV = DATA_DIR / "count_log.csv"
TICK_S = 0.02        # main loop period: read keys, drain edges
STATUS_S = 0.25      # status line refresh period
FACTORY_K = K_HZ_PER_LPM   # nominal K from flow_core, only used to display estimated flow


def water_density(t_c):
    """Pure water density in g/mL (Thiesen formula, good for 0-40 C)."""
    return 1.0 - (t_c + 288.9414) / (508929.2 * (t_c + 68.12963)) * (t_c - 3.9863) ** 2


# ---------------------------------------------------------------- input source
class SimEdgeSource:
    """Sim input: replays the sim_source edges in real time, looped. Same interface as GpioEdgeSource."""

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


# ---------------------------------------------------------------- keys
class Keys:
    """Non-blocking single-key read. Disabled when stdin is not a terminal (e.g. redirected)."""

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


# ---------------------------------------------------------------- counting
class PulseCounter:
    HEADER = ["t_s", "active_s", "event", "dt_ms", "accepted", "raw_total", "clean_total"]

    def __init__(self, min_interval_s, t0):
        # same glitch rule as flow_core: < min_interval after the last accepted edge = glitch
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
    """Main loop. Returns (active time in s, stop reason)."""
    dur_ns = None if duration_s is None else int(duration_s * NS_PER_S)
    counting = True
    active_ns = 0
    last = counter.t0                 # time up to which edges are settled
    next_status = 0.0
    reason = "duration"
    show_status = sys.stdout.isatty()   # no status line when stdout is redirected
    counter.event("start", last, 0.0)

    def settle(end):
        """Book edges in (last, end] under the current state and advance the timer."""
        nonlocal active_ns, last
        for ts in edges:
            if ts < counter.t0 or ts > end:      # skip edges before start / after time is up
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
            now = time.monotonic_ns()     # read after drain, so every drained edge is <= now

            if counting and dur_ns is not None and active_ns + (now - last) >= dur_ns:
                settle(last + (dur_ns - active_ns))   # cut off exactly at the time limit
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
                state = "COUNT" if counting else "PAUSE"
                total = f"/{duration_s:g}" if duration_s is not None else ""
                print(f"\r[{state}] {active_ns / NS_PER_S:7.1f}{total} s   "
                      f"raw {counter.raw:6d}   clean {counter.clean:6d}   ",
                      end="", flush=True)
    except KeyboardInterrupt:
        reason = "Ctrl-C"
        edges = src.drain()
        settle(time.monotonic_ns())

    counter.event("end", last, active_ns / NS_PER_S)
    print()
    return active_ns / NS_PER_S, reason


# ---------------------------------------------------------------- results
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
            print("  Enter a number, or press Enter to skip")


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
    ap = argparse.ArgumentParser(description="Pulse counter for weigh calibration")
    ap.add_argument("duration", nargs="?", type=float, default=None,
                    help="count time in s (pauses excluded); omit to count until q")
    ap.add_argument("--sim", action="store_true", help="use simulated pulses, no hardware")
    ap.add_argument("--chip", default="/dev/gpiochip0")
    ap.add_argument("--line", type=int, default=16)
    ap.add_argument("--bias", default="disabled",
                    choices=["disabled", "pull-up", "pull-down"])
    ap.add_argument("--min-interval", type=float, default=MIN_INTERVAL_S, help="glitch threshold, s")
    ap.add_argument("--mass", type=float, default=None, help="weighed mass, g (asked at the end if omitted)")
    ap.add_argument("--temp", type=float, default=None, help="water temp, C (asked at the end if omitted)")
    ap.add_argument("--out", default=None, help="CSV file name prefix (still saved in data/)")
    args = ap.parse_args()

    if args.sim:
        src, tag = SimEdgeSource(), "sim"
    else:
        from gpio_source import GpioEdgeSource
        src, tag = GpioEdgeSource(chip=args.chip, line=args.line, bias=args.bias), "gpio"

    keys = Keys()
    src.start()
    counter = PulseCounter(args.min_interval, time.monotonic_ns())
    how = "p/space = pause, q = stop" if keys.enabled else "stdin is not a terminal, keys disabled, Ctrl-C to stop"
    print(f"Counting ({tag}, {how})")
    try:
        with keys:
            active_s, reason = run(src, args.duration, counter, keys)
    finally:
        src.stop()

    # --- CSV ---
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    DATA_DIR.mkdir(exist_ok=True)
    out_csv = DATA_DIR / f"count_{args.out or f'{tag}_{stamp}'}.csv"
    with open(out_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(PulseCounter.HEADER)
        w.writerows(counter.rows)

    # --- results ---
    clean, raw = counter.clean, counter.raw
    freq = clean / active_s if active_s > 0 else 0.0
    print("=" * 44)
    print(f"Active time   {active_s:.2f} s ({counter.pauses} pauses, stop: {reason})")
    print(f"Raw pulses    {raw}")
    print(f"Clean pulses  {clean} ({counter.rejected} dropped, threshold {args.min_interval * 1000:g} ms)")
    if counter.paused_edges:
        print(f"Paused pulses {counter.paused_edges} (not counted)")
    print(f"Mean freq     {freq:.3f} Hz ~ {freq / FACTORY_K * 1000:.1f} mL/min (nominal K={FACTORY_K:g})")
    print(f"CSV           data/{out_csv.name}")

    mass = args.mass if args.mass is not None else ask_float("Weighed mass, g (Enter to skip): ")
    log = {"time": stamp, "csv": out_csv.name, "mode": tag, "active_s": f"{active_s:.3f}",
           "raw": raw, "clean": clean, "rejected": counter.rejected,
           "paused_edges": counter.paused_edges, "pauses": counter.pauses,
           "min_interval_ms": f"{args.min_interval * 1000:g}", "stop": reason}
    if mass is not None and clean > 0:
        temp = args.temp if args.temp is not None else ask_float("Water temp, C (Enter = 25): ")
        temp = 25.0 if temp is None else temp
        vol = mass / water_density(temp)
        ml_per_pulse = vol / clean
        k = clean / vol * 1000 / 60            # pulses/L / 60 = Hz per (L/min)
        q = vol / active_s * 60 if active_s > 0 else 0.0
        print(f"Volume        {vol:.2f} mL ({mass:g} g, {temp:g} C, rho={water_density(temp):.5f})")
        print(f"Per pulse     {ml_per_pulse:.5f} mL/pulse")
        print(f"K             {k:.2f} Hz/(L/min)")
        print(f"Mean flow     {q:.1f} mL/min (volume / active time, incl. start/stop transients)")
        log.update(mass_g=f"{mass:g}", temp_c=f"{temp:g}", volume_ml=f"{vol:.3f}",
                   ml_per_pulse=f"{ml_per_pulse:.6f}", k_hz_per_lpm=f"{k:.3f}",
                   q_mean_mlmin=f"{q:.2f}")
    append_log(log)
    print("=" * 44)
    print(f"Final count (clean): {clean}    raw: {raw}")


if __name__ == "__main__":
    main()
