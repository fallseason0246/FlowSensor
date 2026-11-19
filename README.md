# FlowSensor — 流量计读取测试（GPIO16）

流量计：G1/4 霍尔涡轮，厂家参数 F = 98 × Q（Hz，Q 单位 L/min）
=> Q [mL/min] = 1000 / (98 × t)，t 为两个上升沿的间隔（s）

## 文件

| 文件 | 作用 |
|---|---|
| `flow_core.py` | 算法核心：毛刺门限（<10 ms 丢弃）、FIR 时间窗口平均（200 ms）、超时判零（500 ms）。不依赖 gpiod |
| `sim_source.py` | 生成测试脉冲：300 → 150 → 60 → 0 → 斜坡 0→300 mL/min，叠加 ±15% 泵脉动、2% 间隔抖动、10 个毛刺 |
| `gpio_source.py` | 实机输入：libgpiod v2 后台线程等边沿事件，内核时间戳入队并置位 `irqsign` |
| `main.py` | 1 ms 主循环（绝对时间排程），写 `data/flow_*.csv`（每 ms 一行）和 `data/edges_*.csv`（每个上升沿一行） |
| `plot_flow.py` | 画滤波前 / 滤波后（仿真时加真值）流量，下方子图为误差或主循环延迟；默认画 `data/` 里最新的一次 |
| `data/` | 所有 CSV 和 PNG，自动创建 |
| `count_pulse.py` | 称重标定用的脉冲计数：可设计数时间、可暂停，输出原始 / 去毛刺计数；输入称重质量后直接算 mL/脉冲 和 K，每次结果追加到 `data/count_log.csv` |
| `step_response.py` | 泵 PWM 阶跃 + 流量计同步采集（`--pwm1 30 2 --pwm2 60 2 ...`），两点法拟合每个阶跃的死区时间 θ 和时间常数 τ，输出 CSV、PNG，结果追加到 `data/step_log.csv` |
| `pulse_out.py` | 实机回环：GPIO7 输出同一测试序列，杜邦线接 GPIO16 |

## 运行

```bash
# 仿真（任何电脑）
python3 main.py --sim
python3 plot_flow.py                        # 画 data/ 里最新的一次，edges 自动配对
python3 plot_flow.py sim_20261115_143707    # 画指定某次（前缀、文件名或路径都行）

# 实机：接流量计（经 BSS138）
python3 main.py --gpio --duration 30

# 实机回环测试：GPIO7(26脚) -> GPIO16(36脚)
python3 main.py --gpio --bias pull-down --duration 16   # 终端 1
python3 pulse_out.py                                    # 终端 2
```

可调参数：`--k 98 --window 0.2 --min-interval 0.01 --timeout 0.5`

## 称重标定（count_pulse.py）

```bash
python3 count_pulse.py          # 一直计数，泵停、出水口不滴水后按 q 结束
python3 count_pulse.py 120      # 计数 120 s（暂停不计时）
python3 count_pulse.py 10 --sim # 不接硬件测试
```

运行时：`p` / 空格 暂停或继续，`q` 结束，Ctrl-C 也会保存。结束后输入称重质量（g）和水温（°C），
直接给出 mL/脉冲、K 和平均流量；也可以用 `--mass 152.3 --temp 31` 在命令行给。

- `data/count_<mode>_<时间>.csv`：每个上升沿一行（间隔、是否被当作毛刺），加上开始 / 暂停 / 继续 / 结束
- `data/count_log.csv`：每次一行汇总，同一流量档重复 3 次后直接在这张表里比较

## 阶跃响应（step_response.py）

```bash
python3 step_response.py --pwm1 30 3 --pwm2 60 2 --pwm3 30 2          # 实机
python  step_response.py --pwm1 30 3 --pwm2 60 2 --pwm3 30 2 --sim    # 电脑仿真
```

每段 `--pwmN <占空比%> <秒>`，按 N 顺序执行，结束或 Ctrl-C 后泵停。第一段当预热段。
默认接线：PWMA→GPIO12（`--pwm-channel 0`），AIN1→GPIO5，AIN2→GPIO6，STBY→GPIO26，流量计→GPIO16；
不符就用 `--in1/--in2/--stby`（-1 表示硬接线）改。需要 `dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4`。

## 依赖

Python 依赖统一写在 `requirements.txt`，在虚拟环境里安装（Debian / Raspberry Pi OS）：

```bash
# 系统包：venv、编译 gpiod 绑定用的工具、gpioinfo 等命令行工具
sudo apt update
sudo apt install -y python3-venv python3-dev build-essential gpiod

# 虚拟环境
python3 -m venv ~/flowenv
source ~/flowenv/bin/activate        # 每次开新终端都要先执行
pip install --upgrade pip
pip install -r requirements.txt

# 确认 gpiod 是 v2
python3 -c "import gpiod; print(gpiod.__version__)"
```

- 用户需要在 `gpio` 组里（`groups` 查看；没有就 `sudo usermod -aG gpio $USER` 后重新登录）。
- Pi 4 的 40 针 GPIO 在 `/dev/gpiochip0`，可用 `gpioinfo` 确认。
- 仿真模式只需要 numpy 和 matplotlib，在 Windows / Mac 上也能跑。

## 仿真结果（data/flow_test.csv / data/flow_test.png）

| 工况 | 滤波前 均值 / 标准差 | FIR 均值 / 标准差 |
|---|---|---|
| 300 mL/min | 303.7 / 24.7 | 300.4 / 2.9 |
| 150 mL/min | 151.7 / 12.4 | 150.5 / 3.7 |
| 60 mL/min | 60.8 / 4.6 | 60.6 / 4.5 |

- 10 个注入毛刺全部被丢弃
- 300→150 阶跃：0.19 s 进入 ±5%；150→60：0.62 s 进入 ±5%
- 停泵后 0.33 s 判零

泵脉动的频率和幅度（`ML_PER_ROLLER`、`PULSE_AMP`）是假设值，拿到实测波形后在 `sim_source.py` 里改。
