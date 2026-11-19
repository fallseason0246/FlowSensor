#!/usr/bin/env python3
"""
pwm_control.py - PWM output with pigpio (Raspberry Pi 4).

Pins use BCM numbering. Default BCM13 = physical pin 33 (hardware PWM1).
Pins 12/13/18/19 use hardware PWM; any other pin falls back to pigpio software PWM.

Setup:
    sudo apt install pigpio python3-pigpio
    sudo systemctl enable --now pigpiod

Usage:
    python3 pwm_control.py                  # interactive: enter duty, q to quit (output set to 0 on exit)
    python3 pwm_control.py 50               # set 50 % and exit (pigpiod keeps the output running)
    python3 pwm_control.py 0                # turn off
    python3 pwm_control.py 30 --pin 12 --freq 20000
"""

import argparse
import sys

import pigpio

HW_PWM_PINS = (12, 13, 18, 19)


class Pwm:
    def __init__(self, pin, freq):
        self.pi = pigpio.pi()
        if not self.pi.connected:
            sys.exit("Cannot connect to pigpiod. Run: sudo systemctl start pigpiod")
        self.pin, self.freq = pin, freq
        self.hw = pin in HW_PWM_PINS
        if not self.hw:
            self.pi.set_PWM_frequency(pin, freq)
            self.pi.set_PWM_range(pin, 1000)

    def set(self, pct):
        """Set duty in percent, clamped to 0-100."""
        pct = max(0.0, min(100.0, pct))
        if self.hw:
            self.pi.hardware_PWM(self.pin, self.freq, round(pct * 10_000))  # 0-1e6
        else:
            self.pi.set_PWM_dutycycle(self.pin, round(pct * 10))           # 0-1000
        return pct

    def close(self, stop=True):
        if stop:
            self.set(0)
        self.pi.stop()


def main():
    ap = argparse.ArgumentParser(description="PWM output with pigpio")
    ap.add_argument("duty", nargs="?", type=float, help="duty 0-100; omit for interactive mode")
    ap.add_argument("--pin", type=int, default=13, help="BCM pin, default 13 (physical pin 33)")
    ap.add_argument("--freq", type=int, default=1000, help="frequency Hz, default 1000")
    args = ap.parse_args()

    pwm = Pwm(args.pin, args.freq)
    mode = "hardware" if pwm.hw else "software"

    if args.duty is not None:
        pct = pwm.set(args.duty)
        print(f"GPIO{args.pin} {mode} PWM: {args.freq} Hz, {pct:.2f} %")
        pwm.close(stop=False)
        return

    print(f"GPIO{args.pin} {mode} PWM, {args.freq} Hz. Enter duty 0-100, q to quit")
    try:
        while True:
            s = input("duty % > ").strip().rstrip("%")
            if s.lower() in ("q", "quit", "exit"):
                break
            try:
                print(f"  -> {pwm.set(float(s)):.2f} %")
            except ValueError:
                print("  Enter a number 0-100")
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        pwm.close()
        print("PWM off")


if __name__ == "__main__":
    main()
