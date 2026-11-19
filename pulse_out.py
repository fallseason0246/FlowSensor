"""
pulse_out.py — 实机回环测试：在 GPIO7 上输出和仿真相同的测试脉冲序列

接线：GPIO7（物理 26 脚，PiTFT 侧边排针）用杜邦线直接接 GPIO16（物理 36 脚），
     测试时不经过 BSS138。

用法（开两个终端）：
  终端 1：python3 main.py --gpio --bias pull-down --duration 16
  终端 2：python3 pulse_out.py
然后：python3 plot_flow.py flow_gpio_xxx.csv edges_gpio_xxx.csv

说明：
  - 用 busy-wait 控制时间，Python 抖动约几十 µs，相对 30 ms 级的周期可以忽略。
  - 每个脉冲高电平 1 ms，毛刺脉冲高电平 100 µs。
  - GPIO7 若被 spidev0.1 占用会报 "Device or resource busy"，
    在 config.txt 加 dtoverlay=spi0-1cs 或换一个空闲脚（--line）。
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
    t0 = time.monotonic_ns() + 500_000_000   # 0.5 s 后开始
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
