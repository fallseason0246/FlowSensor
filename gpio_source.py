"""
gpio_source.py - flow sensor input on a real Pi, using libgpiod v2.

The kernel timestamps each rising edge in its IRQ handler (ns, CLOCK_MONOTONIC)
and buffers the events. drain() does a non-blocking read of that buffer once
per main-loop tick. No thread, no queue, no locks.

Buffer check: max flow 0.8 L/min * K=98 -> ~78 Hz, so even a 20 ms tick
only collects 1-2 events. The kernel buffer holds far more than that.

Requires the libgpiod v2 Python binding (gpiod.__version__ >= 2).
"""

import sys
from datetime import timedelta

try:
    import gpiod
    from gpiod.line import Bias, Edge
except ImportError:
    sys.exit("Hardware mode needs Linux libgpiod (Raspberry Pi only).\n"
             "To test on Windows / Mac add --sim, e.g. python count_pulse.py 10 --sim\n"
             "On the Pi: source ~/flowenv/bin/activate, then pip install -r requirements.txt")

BIAS = {
    "disabled": Bias.DISABLED,   # use with BSS138 board (it has its own pull-up)
    "pull-up": Bias.PULL_UP,
    "pull-down": Bias.PULL_DOWN,
}


class GpioEdgeSource:
    def __init__(self, chip="/dev/gpiochip0", line=16, bias="disabled"):
        self.line = line
        self.req = gpiod.request_lines(
            chip,
            consumer="flow-sensor",
            # event clock defaults to MONOTONIC, same clock as time.monotonic_ns()
            config={line: gpiod.LineSettings(edge_detection=Edge.RISING,
                                             bias=BIAS[bias])},
        )

    def start(self):
        """Kept for interface compatibility; edges are captured from request_lines on."""

    def drain(self):
        """Return timestamps (ns) of all edges since the last call. Never blocks."""
        out = []
        while self.req.wait_edge_events(timedelta(0)):
            out.extend(ev.timestamp_ns for ev in self.req.read_edge_events())
        return out

    def stop(self):
        self.req.release()
