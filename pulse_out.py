"""
pulse_out.py - hardware loopback test: output the sim test pulse train on GPIO7.

Wiring: jumper GPIO7 (pin 26, PiTFT side header) directly to GPIO16 (pin 36).
        No BSS138 in this test.

Usage (two terminals):
  Terminal 1: python3 main.py --gpio --bias pull-down --duration 16
  Terminal 2: python3 pulse_out.py
Then: python3 plot_flow.py

Notes:
  - Timing uses busy-wait. Python jitter is tens of us, negligible vs ~30 ms periods.
  - Normal pulse high time 1 ms, glitch pulse high time 100 us.
  - If GPIO7 is held by spidev0.1 you get "Device or resource busy".
    Add dtoverlay=spi0-1cs to config.txt or pick a free pin (--line).
"""

import argparse
import time

import gpiod
from gpiod.line import Direction, Value

from sim_source import generate_edges


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--chip", default="/dev/gpiochip0")
    ap.add_argument("--line", type=int, default=7)
    ap.add_argument("--high-us", type=int, default=1000)
    args = ap.parse_args()

    edges_ns, glitch_ns = generate_edges()
    glitch_set = set(glitch_ns.tolist())

    req = gpiod.request_lines(
        args.chip, consumer="flow-pulse-out",
        config={args.line: gpiod.LineSettings(direction=Direction.OUTPUT,
                                               output_value=Value.INACTIVE)})
    print(f"output {edges_ns.size} edges on line {args.line} ...")
    t0 = time.monotonic_ns() + 500_000_000   # start in 0.5 s
    try:
        for e in edges_ns.tolist():
            target = t0 + e
            while time.monotonic_ns() < target:
                pass
            req.set_value(args.line, Value.ACTIVE)
            high = 100_000 if e in glitch_set else args.high_us * 1000
            while time.monotonic_ns() < target + high:
                pass
            req.set_value(args.line, Value.INACTIVE)
    finally:
        req.set_value(args.line, Value.INACTIVE)
        req.release()
    print("done")


if __name__ == "__main__":
    main()
