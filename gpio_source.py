"""
gpio_source.py — 实机输入：用 libgpiod v2 监听 GPIO16 上升沿

Python 版 libgpiod 没有真正的回调函数，这里用后台线程阻塞等待边沿事件，
效果等同于中断回调：
  - 收到事件 -> 内核时间戳（ns，CLOCK_MONOTONIC）放进队列
  - 置位 irqsign，通知主循环

需要 libgpiod v2 的 Python 绑定（import gpiod; gpiod.__version__ >= 2）。
"""

import queue
import threading
from datetime import timedelta

import sys

try:
    import gpiod
    from gpiod.line import Bias, Edge
except ImportError:
    sys.exit("实机模式需要 Linux 的 libgpiod（只能在 Raspberry Pi 上运行）。\n"
             "在 Windows / Mac 上测试请加 --sim，例如：python count_pulse.py 10 --sim\n"
             "已经在 Pi 上的话：先 source ~/flowenv/bin/activate，再 pip install -r requirements.txt")


class GpioEdgeSource:
    def __init__(self, chip="/dev/gpiochip0", line=16, bias="disabled"):
        bias_map = {
            "disabled": Bias.DISABLED,   # 经过 BSS138 时用这个（板上已有上拉）
            "pull-up": Bias.PULL_UP,
            "pull-down": Bias.PULL_DOWN,
        }
        self.line = line
        self.edge_q = queue.SimpleQueue()   # 时间戳队列（线程安全）
        self.irqsign = threading.Event()    # “中断标志”
        self._stop = threading.Event()

        self.req = gpiod.request_lines(
            chip,
            consumer="flow-sensor",
            config={
                line: gpiod.LineSettings(
                    edge_detection=Edge.RISING,
                    bias=bias_map[bias],
                    # event_clock 默认就是 MONOTONIC，与 time.monotonic_ns() 同一时钟
                )
            },
        )
        self._th = threading.Thread(target=self._isr_thread, daemon=True)

    def start(self):
        self._th.start()

    def _isr_thread(self):
        while not self._stop.is_set():
            # 最多阻塞 100 ms，以便能响应 stop
            if self.req.wait_edge_events(timedelta(milliseconds=100)):
                for ev in self.req.read_edge_events():
                    self.edge_q.put(ev.timestamp_ns)
                self.irqsign.set()

    def drain(self):
        """取出队列里所有时间戳"""
        out = []
        while True:
            try:
                out.append(self.edge_q.get_nowait())
            except queue.Empty:
                return out

    def stop(self):
        self._stop.set()
        self._th.join(timeout=1)
        self.req.release()
