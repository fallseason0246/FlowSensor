# FlowSensor — flow sensor readout test (GPIO16)

Sensor: low-flow Hall water flow sensor, 1/4" quick connect, range 0.05–0.8 L/min (50–800 mL/min), DC 5 V / ≤10 mA, ≤0.35 MPa, liquid ≤60 °C, XH-3P / XH-2P connectors.
Conversion: F = K × Q (Hz, Q in L/min) => Q [mL/min] = 1000 / (K × t), where t is the interval between two rising edges (s).

> ⚠ The datasheet gives no K. `K = 98` in the code is a placeholder carried over from the old sensor (G1/4 turbine).
> Calibrate by weighing with `count_pulse.py` first, then update `K_HZ_PER_LPM` in `flow_core.py` (all scripts take their defaults from there).

## Files

| File | Purpose |
|---|---|
| `flow_core.py` | Algorithm core: glitch threshold (drop < 5 ms), FIR time-window average (200 ms), zero-flow timeout (500 ms). No gpiod dependency |
| `sim_source.py` | Test pulse generator: 300 → 150 → 60 → 0 → ramp 0→300 mL/min, with ±15% pump ripple, 2% interval jitter, 10 glitches |
| `gpio_source.py` | Hardware input: libgpiod v2. The kernel timestamps edges in its IRQ handler and buffers them; the main loop reads them non-blocking every tick (no thread) |
| `main.py` | 1 ms main loop (absolute-time scheduling). Writes `data/flow_*.csv` (one row per ms) and `data/edges_*.csv` (one row per rising edge) |
| `plot_flow.py` | Plots raw / FIR flow (plus true flow in sim); lower subplot shows error or main-loop lag. Plots the latest run in `data/` by default |
| `data/` | All CSV and PNG output, created automatically |
| `count_pulse.py` | Pulse counter for weigh calibration: set count time, pause, raw / deglitched counts. Enter the weighed mass to get mL/pulse and K; each result is appended to `data/count_log.csv` |
| `step_response.py` | Pump PWM steps + synchronized flow logging (`--pwm1 30 2 --pwm2 60 2 ...`). Two-point fit of dead time θ and time constant τ for each step; writes CSV and PNG, appends results to `data/step_log.csv` |
| `pulse_out.py` | Hardware loopback: outputs the same test sequence on GPIO7, jumpered to GPIO16 |
| `pwm_control.py` | Pump PWM via pigpio, default BCM13 (pin 33). `python3 pwm_control.py 50` sets 50 %; no argument = interactive |

## Running

```bash
# Simulation (any PC)
python3 main.py --sim
python3 plot_flow.py                        # plot the latest run in data/, edges paired automatically
python3 plot_flow.py sim_20261115_143707    # plot a given run (prefix, file name or path)

# Hardware: flow sensor connected (through BSS138)
python3 main.py --gpio --duration 30

# Hardware loopback test: GPIO7 (pin 26) -> GPIO16 (pin 36)
python3 main.py --gpio --bias pull-down --duration 16   # terminal 1
python3 pulse_out.py                                    # terminal 2
```

Tunable parameters: `--k 98 --window 0.2 --min-interval 0.005 --timeout 0.5` (defaults at the top of `flow_core.py`)

## Weigh calibration (count_pulse.py)

```bash
python3 count_pulse.py          # count until q; press q after the pump stops and the outlet stops dripping
python3 count_pulse.py 120      # count for 120 s (pauses not timed)
python3 count_pulse.py 10 --sim # test without hardware
```

While running: `p` / space to pause or resume, `q` to stop; Ctrl-C also saves. At the end, enter the weighed mass (g)
and water temperature (°C) to get mL/pulse, K and mean flow. You can also pass `--mass 152.3 --temp 31` on the command line.

- `data/count_<mode>_<time>.csv`: one row per rising edge (interval, glitch or not), plus start / pause / resume / end events
- `data/count_log.csv`: one summary row per run; repeat each flow level 3 times and compare in this table

## Step response (step_response.py)

```bash
python3 step_response.py --pwm1 30 3 --pwm2 60 2 --pwm3 30 2          # hardware
python  step_response.py --pwm1 30 3 --pwm2 60 2 --pwm3 30 2 --sim    # simulation on a PC
```

Each segment is `--pwmN <duty%> <seconds>`, run in order of N. The pump stops at the end or on Ctrl-C. Use the first segment as warm-up.
Default wiring: PWMA→GPIO12 (`--pwm-channel 0`), AIN1→GPIO5, AIN2→GPIO6, STBY→GPIO26, flow sensor→GPIO16.
Change with `--in1/--in2/--stby` if yours differs (-1 = hard-wired). Requires `dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4`.

## Dependencies

Python dependencies are listed in `requirements.txt`; install them in a virtual environment (Debian / Raspberry Pi OS):

```bash
# System packages: venv, build tools for the gpiod binding, gpioinfo and other CLI tools
sudo apt update
sudo apt install -y python3-venv python3-dev build-essential gpiod

# Virtual environment
python3 -m venv ~/flowenv
source ~/flowenv/bin/activate        # run this in every new terminal
pip install --upgrade pip
pip install -r requirements.txt

# Check that gpiod is v2
python3 -c "import gpiod; print(gpiod.__version__)"
```

- The user must be in the `gpio` group (check with `groups`; if not, `sudo usermod -aG gpio $USER` and log in again).
- The Pi 4 40-pin header GPIO is `/dev/gpiochip0`; confirm with `gpioinfo`.
- Simulation mode only needs numpy and matplotlib and also runs on Windows / Mac.

## Simulation results (data/flow_test.csv / data/flow_test.png, old sensor parameters, 10 ms glitch threshold)

| Condition | Raw mean / std | FIR mean / std |
|---|---|---|
| 300 mL/min | 303.7 / 24.7 | 300.4 / 2.9 |
| 150 mL/min | 151.7 / 12.4 | 150.5 / 3.7 |
| 60 mL/min | 60.8 / 4.6 | 60.6 / 4.5 |

- All 10 injected glitches were dropped
- 300→150 step: within ±5% after 0.19 s; 150→60: within ±5% after 0.62 s
- Reads zero 0.33 s after the pump stops

Pump ripple frequency and amplitude (`ML_PER_ROLLER`, `PULSE_AMP`) are assumed values; update them in `sim_source.py` once a measured waveform is available.
