#!/usr/bin/env python3
"""
pwm_control.py — 用 pigpio 输出 PWM（Raspberry Pi 4）

引脚用 BCM 编号。默认 BCM13 = 物理 33 脚（硬件 PWM1）。
12/13/18/19 走硬件 PWM，其他引脚自动改用 pigpio 的软件 PWM。

准备：
    sudo apt install pigpio python3-pigpio
    sudo systemctl enable --now pigpiod

用法：
    python3 pwm_control.py                  # 交互模式，输入占空比，q 退出（退出时输出归零）
    python3 pwm_control.py 50               # 直接设成 50 %，然后退出（pigpiod 会保持输出）
    python3 pwm_control.py 0                # 关掉
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
            sys.exit("连不上 pigpiod，请先运行: sudo systemctl start pigpiod")
        self.pin, self.freq = pin, freq
        self.hw = pin in HW_PWM_PINS
        if not self.hw:
            self.pi.set_PWM_frequency(pin, freq)
            self.pi.set_PWM_range(pin, 1000)

    def set(self, pct):
        """设占空比（百分比，自动限幅到 0~100）"""
        pct = max(0.0, min(100.0, pct))
        if self.hw:
            self.pi.hardware_PWM(self.pin, self.freq, round(pct * 10_000))  # 0~1e6
        else:
            self.pi.set_PWM_dutycycle(self.pin, round(pct * 10))           # 0~1000
        return pct

    def close(self, stop=True):
        if stop:
            self.set(0)
        self.pi.stop()


def main():
    ap = argparse.ArgumentParser(description="pigpio PWM 输出")
    ap.add_argument("duty", nargs="?", type=float, help="占空比 0~100；不给就进入交互模式")
    ap.add_argument("--pin", type=int, default=13, help="BCM 编号，默认 13（物理 33 脚）")
    ap.add_argument("--freq", type=int, default=1000, help="频率 Hz，默认 1000")
    args = ap.parse_args()

    pwm = Pwm(args.pin, args.freq)
    mode = "硬件" if pwm.hw else "软件"

    if args.duty is not None:
        pct = pwm.set(args.duty)
        print(f"GPIO{args.pin} {mode} PWM: {args.freq} Hz, {pct:.2f} %")
        pwm.close(stop=False)
        return

    print(f"GPIO{args.pin} {mode} PWM, {args.freq} Hz。输入占空比 0~100，q 退出")
    try:
        while True:
            s = input("duty % > ").strip().rstrip("%")
            if s.lower() in ("q", "quit", "exit"):
                break
            try:
                print(f"  -> {pwm.set(float(s)):.2f} %")
            except ValueError:
                print("  请输入 0~100 的数字")
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        pwm.close()
        print("PWM 已关闭")


if __name__ == "__main__":
    main()
