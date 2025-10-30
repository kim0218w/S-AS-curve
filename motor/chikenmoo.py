import time
import math
import sys
import os
import re
import csv
import numpy as np

try:
    import lgpio
except Exception:
    lgpio = None

try:
    import smbus
except Exception:
    smbus = None

try:
    from graph import plot_results as plot_csv_results
except Exception:
    plot_csv_results = None
# -------------------- Pins --------------------
DIR_PIN_NAMA_17  = 24
STEP_PIN_NAMA_17 = 23
ENA_PIN_NAMA_17  = 25

DIR_PIN_NAMA_23  = 20
STEP_PIN_NAMA_23 = 21
ENA_PIN_NAMA_23  = 16

IN1_PIN = 5
IN2_PIN = 6
PWM_PIN = 13

# -------------------- Motion params --------------------
MIN_DELAY_17 = 0.0005
MAX_DELAY_17 = 0.002
STEPS_17     = 1320

MIN_DELAY_23 = 0.0003   # fastest (shortest delay)
MAX_DELAY_23 = 0.0015   # slowest (longest delay)
STEPS_23     = 10000

# Safety
MIN_SAFE_DELAY = 0.00025  # 250 us
TX_BACKLOG = 8

# Actuator / encoder
ACT_TIME       = 7
ENC_SAMPLE_DT  = 0.015
MUX_ADDR       = 0x70
ENCODER_ADDR   = 0x36
ENC_REG_ANGLE  = 0x0E

DEG_PER_STEP = 0.018  # 10000 steps = 180 deg
PULSES_PER_REV = 360.0 / DEG_PER_STEP if DEG_PER_STEP else float("nan")
PULSE_SPEED_FILTER_TAU = 0.05  # seconds, low-pass filter time constant

# -------------------- Output folders --------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(SCRIPT_DIR, "logs")
os.makedirs(LOG_DIR, exist_ok=True)

def _safe_name(s: str) -> str:
    return re.sub(r'[^A-Za-z0-9._-]+', '_', s)

def plot_time_series(*_args, **_kwargs):
    """Graph plotting disabled."""
    return None

# -------------------- Encoder via PCA9548A --------------------
class EncoderMux:
    def __init__(self, bus_num=1, settle=0.002, retries=3, retry_wait=0.001):
        if smbus is None:
            raise RuntimeError("smbus not available. Run on Raspberry Pi with I2C enabled.")
        self.bus = smbus.SMBus(bus_num)
        self.settle = settle
        self.retries = retries
        self.retry_wait = retry_wait

    def select_channel(self, ch: int):
        self.bus.write_byte(MUX_ADDR, 1 << ch)
        time.sleep(self.settle)

    def read_angle_deg_once(self) -> float:
        data = self.bus.read_i2c_block_data(ENCODER_ADDR, ENC_REG_ANGLE, 2)
        raw  = ((data[0] << 8) | data[1]) & 0x0FFF
        return raw * 360.0 / 4096.0

    def read_angle_deg(self) -> float:
        for _ in range(self.retries):
            try:
                return self.read_angle_deg_once()
            except Exception:
                time.sleep(self.retry_wait)
        raise

    def try_read_channel(self, ch: int):
        try:
            self.select_channel(ch)
            return self.read_angle_deg()
        except Exception:
            return float("nan")

    def detect_working_channels(self, candidates=None):
        if candidates is None:
            candidates = list(range(8))
        ok = []
        for ch in candidates:
            v = self.try_read_channel(ch)
            if not (np.isnan(v) or np.isinf(v)):
                ok.append(ch)
        try:
            self.bus.write_byte(MUX_ADDR, 0x00)
        except Exception:
            pass
        return ok

# -------------------- Stepper helpers --------------------
def enable_motor(h, ena_pin, enable=True):
    state = 0 if enable else 1
    lgpio.gpio_write(h, ena_pin, state)

def stop_actuator(h):
    lgpio.gpio_write(h, IN1_PIN, 0)
    lgpio.gpio_write(h, IN2_PIN, 0)
    lgpio.tx_pwm(h, PWM_PIN, 1000, 0)

def extend(h, speed=1.0):
    duty = int(max(0, min(100, speed * 100)))
    lgpio.gpio_write(h, IN1_PIN, 1)
    lgpio.gpio_write(h, IN2_PIN, 0)
    lgpio.tx_pwm(h, PWM_PIN, 1000, duty)

def retract(h, speed=1.0):
    duty = int(max(0, min(100, speed * 100)))
    lgpio.gpio_write(h, IN1_PIN, 0)
    lgpio.gpio_write(h, IN2_PIN, 1)
    lgpio.tx_pwm(h, PWM_PIN, 1000, duty)

def smooth_cos_delay(i, n, min_delay, max_delay, gamma=1.0):
    if min_delay >= max_delay:
        min_delay, max_delay = sorted([min_delay, max_delay])
    min_delay = max(min_delay, MIN_SAFE_DELAY)
    if n <= 1:
        return min_delay
    t_norm = i / (n - 1)
    if gamma not in (None, 1.0):
        if gamma <= 0:
            gamma = 1.0
        t_norm = min(max(t_norm, 0.0), 1.0) ** gamma
    k = 0.5 * (1 - math.cos(math.pi * t_norm))
    return max_delay - (max_delay - min_delay) * k

# -------------------- Math helpers --------------------
def unwrap_deg(deg_series: np.ndarray) -> np.ndarray:
    return np.degrees(np.unwrap(np.radians(deg_series)))

def finite_diff(y: np.ndarray, t: np.ndarray) -> np.ndarray:
    if len(y) < 2 or len(y) != len(t):
        return np.zeros_like(y)
    dy = np.empty_like(y)
    dy[0]  = (y[1] - y[0]) / (t[1] - t[0] + 1e-12)
    dy[-1] = (y[-1] - y[-2]) / (t[-1] - t[-2] + 1e-12)
    for i in range(1, len(y)-1):
        dt = (t[i+1] - t[i-1])
        dy[i] = (y[i+1] - y[i-1]) / (dt + 1e-12)
    return dy

def cumtrapz(y: np.ndarray, x: np.ndarray):
    if y is None or x is None or len(y) < 2 or len(y) != len(x):
        return np.zeros_like(x)
    out = np.zeros_like(y)
    for i in range(1, len(y)):
        dx = x[i] - x[i-1]
        out[i] = out[i-1] + 0.5 * (y[i] + y[i-1]) * dx
    return out

# -------------------- TX helpers (no tx_pwm for steps) --------------------
def queue_pulse(h, gpio, delay_s):
    high_us = int(delay_s * 1_000_000)
    while lgpio.tx_room(h, gpio, lgpio.TX_PWM) <= 0:
        time.sleep(delay_s * 0.25)
    lgpio.tx_pulse(h, gpio, high_us, high_us, 0, 1)

# -------------------- Rate/Delay utils --------------------
def rate_to_delay(rate_steps_per_s, min_delay, max_delay):
    if rate_steps_per_s <= 0:
        return max_delay
    d = 1.0 / (2.0 * rate_steps_per_s)
    d = max(min(d, max_delay), max(min_delay, MIN_SAFE_DELAY))
    return d

def delay_to_rate(delay_s):
    return 1.0 / (2.0 * max(delay_s, 1e-6))

def pulses_to_speed(pulse_count: int, dt: float, pulses_per_rev: float):
    if dt <= 0.0 or pulses_per_rev <= 0.0:
        return float("nan"), float("nan")
    revs = pulse_count / pulses_per_rev
    revs_per_s = revs / dt
    rpm = revs_per_s * 60.0
    rad_s = revs_per_s * math.tau
    return rpm, rad_s

def lowpass_first_order(prev_value: float, new_value: float, dt: float, tau: float) -> float:
    if not math.isfinite(new_value):
        return prev_value
    if tau <= 0.0:
        return new_value
    if not math.isfinite(prev_value):
        return new_value
    dt = max(dt, 1e-9)
    alpha = dt / (tau + dt)
    alpha = min(max(alpha, 0.0), 1.0)
    return prev_value + alpha * (new_value - prev_value)

def apply_speed_target_to_delays(
    min_delay: float,
    max_delay: float,
    target_deg_per_s: float | None
) -> tuple[float, float, float | None]:
    if target_deg_per_s is None or target_deg_per_s <= 0:
        return min_delay, max_delay, None

    fast_limit = max(min_delay, MIN_SAFE_DELAY)
    slow_limit = max(max_delay, fast_limit)

    rate_steps_per_s = target_deg_per_s / max(DEG_PER_STEP, 1e-9)
    if rate_steps_per_s <= 0:
        return min_delay, max_delay, None

    target_delay = 1.0 / (2.0 * rate_steps_per_s)
    target_delay = max(target_delay, fast_limit)

    if target_delay > slow_limit:
        slow_limit = target_delay

    min_delay = target_delay
    max_delay = slow_limit

    achieved_deg_per_s = (1.0 / (2.0 * min_delay)) * DEG_PER_STEP
    return min_delay, max_delay, achieved_deg_per_s

def export_motion_csv(
    t: np.ndarray,
    cmd_angle: np.ndarray,
    pulse_deg: np.ndarray,
    cmd_ang_vel: np.ndarray,
    pulse_vel: np.ndarray,
    filename_base: str = "motion_run"
) -> str | None:
    if t is None or len(t) < 2:
        return None
    t = np.asarray(t, dtype=float)
    if not np.all(np.isfinite(t)):
        return None
    sample_dt = 0.005  # 5 ms
    t_start = float(t[0])
    t_end = float(t[-1])
    if t_end <= t_start:
        return None
    samples = np.arange(t_start, t_end + sample_dt * 0.5, sample_dt, dtype=float)

    def _prepare(series):
        if series is None:
            return None
        arr = np.asarray(series, dtype=float)
        if len(arr) != len(t):
            return None
        return arr

    cmd_angle_arr = _prepare(cmd_angle)
    pulse_deg_arr = _prepare(pulse_deg)
    cmd_vel_arr = _prepare(cmd_ang_vel)
    pulse_vel_arr = _prepare(pulse_vel)

    if cmd_angle_arr is None and cmd_vel_arr is None:
        return None

    def _grad(values, spacing):
        if values is None:
            return np.zeros_like(samples)
        if len(values) < 2:
            return np.zeros_like(values)
        edge_order = 2 if len(values) > 2 else 1
        return np.gradient(values, spacing, edge_order=edge_order)

    if cmd_angle_arr is None and cmd_vel_arr is not None:
        cmd_angle_arr = np.zeros_like(t, dtype=float)
        for i in range(1, len(t)):
            dt = max(t[i] - t[i - 1], 1e-6)
            cmd_angle_arr[i] = cmd_angle_arr[i - 1] + 0.5 * (cmd_vel_arr[i] + cmd_vel_arr[i - 1]) * dt

    if cmd_vel_arr is None and cmd_angle_arr is not None:
        cmd_vel_arr = _grad(cmd_angle_arr, t)

    com_pos = np.interp(samples, t, cmd_angle_arr) if cmd_angle_arr is not None else np.zeros_like(samples)
    com_vel = np.interp(samples, t, cmd_vel_arr) if cmd_vel_arr is not None else _grad(com_pos, sample_dt)

    if pulse_deg_arr is None:
        pul_pos = np.zeros_like(samples)
    else:
        pul_pos = np.interp(samples, t, pulse_deg_arr - pulse_deg_arr[0])

    if pulse_vel_arr is not None:
        pul_vel = np.interp(samples, t, pulse_vel_arr)
    elif len(samples) > 1:
        pul_vel = _grad(pul_pos, sample_dt)
    else:
        pul_vel = np.zeros_like(samples)

    time_ms = (samples - t_start) * 1000.0

    data = zip(time_ms, com_pos, pul_pos, com_vel, pul_vel)
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    fname = _safe_name(f"{filename_base}_{timestamp}.csv")
    path = os.path.join(LOG_DIR, fname)
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([
            "Time_ms", "com_Pos_deg", "pul_Pos_deg",
            "com_Vel_deg_per_s", "pul_Vel_deg_per_s"
        ])
        writer.writerows(data)
    return path

# -------------------- PID --------------------
class PID:
    def __init__(self, kp=2.0, ki=0.0, kd=0.0, out_min=-300.0, out_max=300.0, tau=0.02):
        # output unit: deg/s
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self.tau = tau
        self._i = 0.0
        self._prev_e = 0.0
        self._prev_d = 0.0

    def reset(self):
        self._i = 0.0
        self._prev_e = 0.0
        self._prev_d = 0.0

    def update(self, e, dt, anti_windup_ref=None):
        if dt <= 0.0:
            dt = 1e-3
        p = self.kp * e
        self._i += self.ki * e * dt
        d_raw = (e - self._prev_e) / dt
        d = self._prev_d + (dt / (self.tau + dt)) * (d_raw - self._prev_d)
        out = p + self._i + self.kd * d
        if out > self.out_max:
            out = self.out_max
            if anti_windup_ref is not None and self.ki > 0:
                self._i += 0.1 * (anti_windup_ref - out)
        elif out < self.out_min:
            out = self.out_min
            if anti_windup_ref is not None and self.ki > 0:
                self._i += 0.1 * (anti_windup_ref - out)
        self._prev_e = e
        self._prev_d = d
        return out

# -------------------- Open-loop S-curve with logging --------------------
def move_stepper_scurve_with_logging(
    h, dir_pin, step_pin, total_steps, direction, min_delay, max_delay,
    accel_ratio=0.2, s_curve_gamma: float = 1.0,
    log_enc: bool = True, enc: EncoderMux = None, enc_channels=None,
    enc_sample_dt: float = ENC_SAMPLE_DT
):
    if enc_channels is None:
        enc_channels = []
    if min_delay >= max_delay:
        min_delay, max_delay = sorted([min_delay, max_delay])
    min_delay = max(min_delay, MIN_SAFE_DELAY)

    lgpio.gpio_write(h, dir_pin, direction)

    accel_steps = int(total_steps * accel_ratio)
    decel_steps = accel_steps
    const_steps = total_steps - accel_steps - decel_steps
    if const_steps < 0:
        accel_steps = total_steps // 2
        decel_steps = total_steps - accel_steps
        const_steps = 0

    pulses_per_rev = PULSES_PER_REV if math.isfinite(PULSES_PER_REV) and PULSES_PER_REV > 0 else float("nan")

    t0 = time.monotonic()
    last_sample_t = t0
    last_sample_pulse_count = 0
    total_pulses = 0
    time_log = []
    enc_logs = {ch: [] for ch in enc_channels}
    cmd_rate_log = []
    pulse_rpm_log = []
    pulse_deg_log = []
    pulse_deg_per_s_log = []
    pulse_deg_per_s_filtered_log = []
    pulse_rpm_filtered_log = []
    pulse_deg_per_s_lpf = float("nan")
    current_cmd_rate = 0.0

    def sample_all(ts):
        nonlocal last_sample_t, last_sample_pulse_count, pulse_deg_per_s_lpf
        if ts - last_sample_t >= enc_sample_dt:
            if log_enc and enc is not None:
                for ch in enc_channels:
                    ang = enc.try_read_channel(ch)
                    enc_logs[ch].append(ang)
            dt = ts - last_sample_t
            if dt <= 0.0:
                dt = 1e-6
            pulse_delta = total_pulses - last_sample_pulse_count
            rpm, _ = pulses_to_speed(pulse_delta, dt, pulses_per_rev)
            deg_per_s = (pulse_delta * DEG_PER_STEP) / dt
            if (not math.isfinite(rpm)) and math.isfinite(deg_per_s):
                rpm = deg_per_s / 6.0
            pulse_rpm_log.append(rpm)
            pulse_deg_per_s_log.append(deg_per_s)
            pulse_deg_per_s_lpf = lowpass_first_order(pulse_deg_per_s_lpf, deg_per_s, dt, PULSE_SPEED_FILTER_TAU)
            if math.isfinite(pulse_deg_per_s_lpf):
                pulse_deg_per_s_filtered_log.append(pulse_deg_per_s_lpf)
                pulse_rpm_filtered_log.append(pulse_deg_per_s_lpf / 6.0)
            else:
                pulse_deg_per_s_filtered_log.append(float("nan"))
                pulse_rpm_filtered_log.append(float("nan"))
            pulse_deg_log.append(total_pulses * DEG_PER_STEP)
            cmd_rate_log.append(current_cmd_rate)
            time_log.append(ts - t0)
            last_sample_pulse_count = total_pulses
            last_sample_t = ts

    for i in range(accel_steps):
        d = smooth_cos_delay(i, max(accel_steps, 1), min_delay, max_delay, gamma=s_curve_gamma)
        current_cmd_rate = 1.0 / (2.0 * d)
        queue_pulse(h, step_pin, d)
        total_pulses += 1
        sample_all(time.monotonic())

    for _ in range(const_steps):
        d = min_delay
        current_cmd_rate = 1.0 / (2.0 * d)
        queue_pulse(h, step_pin, d)
        total_pulses += 1
        sample_all(time.monotonic())

    for i in range(decel_steps):
        d = smooth_cos_delay(decel_steps - 1 - i, max(decel_steps, 1), min_delay, max_delay, gamma=s_curve_gamma)
        current_cmd_rate = 1.0 / (2.0 * d)
        queue_pulse(h, step_pin, d)
        total_pulses += 1
        sample_all(time.monotonic())

    time.sleep(min_delay * TX_BACKLOG)
    current_cmd_rate = 0.0
    sample_all(time.monotonic())

    out = {"t": np.array(time_log, dtype=float),
           "cmd_rate": np.array(cmd_rate_log, dtype=float),
           "pulse_rpm": np.array(pulse_rpm_log, dtype=float),
           "pulse_deg": np.array(pulse_deg_log, dtype=float),
           "pulse_deg_per_s": np.array(pulse_deg_per_s_log, dtype=float),
           "pulse_deg_per_s_filtered": np.array(pulse_deg_per_s_filtered_log, dtype=float),
           "pulse_rpm_filtered": np.array(pulse_rpm_filtered_log, dtype=float)}
    for ch, vals in enc_logs.items():
        out[f"enc_{ch}"] = np.array(vals, dtype=float)
    return out

# -------------------- S-curve + PID (closed-loop) --------------------
def move_stepper_scurve_with_pid(
    h, dir_pin, step_pin, total_steps, direction,
    min_delay, max_delay, accel_ratio=0.2,
    s_curve_gamma: float = 1.0,
    pid_gains=(2.0, 0.2, 0.0),  # (Kp, Ki, Kd) in (deg/s) per deg
    log_enc: bool = True, enc: EncoderMux = None, enc_channels=None,
    enc_sample_dt: float = ENC_SAMPLE_DT
):
    if enc is None or not enc_channels:
        return move_stepper_scurve_with_logging(
            h, dir_pin, step_pin, total_steps, direction, min_delay, max_delay,
            accel_ratio, s_curve_gamma, log_enc=False, enc=None, enc_channels=[], enc_sample_dt=enc_sample_dt
        )

    if min_delay >= max_delay:
        min_delay, max_delay = sorted([min_delay, max_delay])
    min_delay = max(min_delay, MIN_SAFE_DELAY)

    lgpio.gpio_write(h, dir_pin, direction)

    accel_steps = int(total_steps * accel_ratio)
    decel_steps = accel_steps
    const_steps = total_steps - accel_steps - decel_steps
    if const_steps < 0:
        accel_steps = total_steps // 2
        decel_steps = total_steps - accel_steps
        const_steps = 0

    ff_delays = []
    for i in range(accel_steps):
        ff_delays.append(smooth_cos_delay(i, max(accel_steps, 1), min_delay, max_delay, gamma=s_curve_gamma))
    for _ in range(const_steps):
        ff_delays.append(min_delay)
    for i in range(decel_steps):
        ff_delays.append(smooth_cos_delay(decel_steps - 1 - i, max(decel_steps, 1), min_delay, max_delay, gamma=s_curve_gamma))

    pulses_per_rev = PULSES_PER_REV if math.isfinite(PULSES_PER_REV) and PULSES_PER_REV > 0 else float("nan")

    t0 = time.monotonic()
    last_sample_t = t0
    last_sample_pulse_count = 0
    total_pulses = 0
    time_log, cmd_rate_log = [], []
    pulse_rpm_log = []
    pulse_deg_log = []
    pulse_deg_per_s_log = []
    pulse_deg_per_s_filtered_log = []
    pulse_rpm_filtered_log = []
    pulse_deg_per_s_lpf = float("nan")
    enc_logs = {ch: [] for ch in enc_channels}

    cmd_rate_ff = np.array([delay_to_rate(d) for d in ff_delays], dtype=float)
    cmd_ang_vel_ff = cmd_rate_ff * DEG_PER_STEP
    t_grid = np.cumsum(np.array(ff_delays, dtype=float))
    if len(t_grid) > 0:
        t_grid -= t_grid[0]
    cmd_angle_ff = np.zeros_like(t_grid)
    for i in range(1, len(t_grid)):
        dt = t_grid[i] - t_grid[i-1]
        cmd_angle_ff[i] = cmd_angle_ff[i-1] + 0.5 * (cmd_ang_vel_ff[i] + cmd_ang_vel_ff[i-1]) * dt

    Kp, Ki, Kd = pid_gains
    pid = PID(kp=Kp, ki=Ki, kd=Kd, out_min=-600.0, out_max=600.0, tau=0.01)
    last_t = t0

    def sample_all(ts):
        nonlocal last_sample_t, last_sample_pulse_count, pulse_deg_per_s_lpf
        if ts - last_sample_t >= enc_sample_dt:
            for ch in enc_channels:
                val = enc.try_read_channel(ch)
                enc_logs[ch].append(val)
            dt = ts - last_sample_t
            if dt <= 0.0:
                dt = 1e-6
            pulse_delta = total_pulses - last_sample_pulse_count
            rpm, _ = pulses_to_speed(pulse_delta, dt, pulses_per_rev)
            deg_per_s = (pulse_delta * DEG_PER_STEP) / dt
            if (not math.isfinite(rpm)) and math.isfinite(deg_per_s):
                rpm = deg_per_s / 6.0
            pulse_rpm_log.append(rpm)
            pulse_deg_per_s_log.append(deg_per_s)
            pulse_deg_per_s_lpf = lowpass_first_order(pulse_deg_per_s_lpf, deg_per_s, dt, PULSE_SPEED_FILTER_TAU)
            if math.isfinite(pulse_deg_per_s_lpf):
                pulse_deg_per_s_filtered_log.append(pulse_deg_per_s_lpf)
                pulse_rpm_filtered_log.append(pulse_deg_per_s_lpf / 6.0)
            else:
                pulse_deg_per_s_filtered_log.append(float("nan"))
                pulse_rpm_filtered_log.append(float("nan"))
            pulse_deg_log.append(total_pulses * DEG_PER_STEP)
            time_log.append(ts - t0)
            cmd_rate_log.append(current_rate_steps)
            last_sample_pulse_count = total_pulses
            last_sample_t = ts

    fb_ch = enc_channels[0]
    enc.select_channel(fb_ch)
    ang0 = enc.read_angle_deg()
    target_offset = ang0
    current_rate_steps = delay_to_rate(ff_delays[0]) if ff_delays else 0.0

    for i, ff_d in enumerate(ff_delays):
        now = time.monotonic()
        dt = max(now - last_t, 1e-4)
        last_t = now

        target_ang = cmd_angle_ff[i] + target_offset

        enc.select_channel(fb_ch)
        try:
            ang_meas = enc.read_angle_deg()
        except Exception:
            ang_meas = float('nan')

        if not np.isnan(ang_meas):
            e = target_ang - ang_meas
            trim_deg_per_s = pid.update(e, dt, anti_windup_ref=0.0)
        else:
            trim_deg_per_s = 0.0

        ff_rate_steps = delay_to_rate(ff_d)
        trim_steps = trim_deg_per_s / max(DEG_PER_STEP, 1e-9)
        desired_rate_steps = max(ff_rate_steps + trim_steps, 1.0)

        d = rate_to_delay(desired_rate_steps, min_delay, max_delay)
        current_rate_steps = delay_to_rate(d)

        queue_pulse(h, step_pin, d)
        total_pulses += 1
        sample_all(time.monotonic())

    time.sleep(min_delay * TX_BACKLOG)
    current_rate_steps = 0.0
    sample_all(time.monotonic())

    out = {"t": np.array(time_log, dtype=float),
           "cmd_rate": np.array(cmd_rate_log, dtype=float),
           "pulse_rpm": np.array(pulse_rpm_log, dtype=float),
           "pulse_deg": np.array(pulse_deg_log, dtype=float),
           "pulse_deg_per_s": np.array(pulse_deg_per_s_log, dtype=float),
           "pulse_deg_per_s_filtered": np.array(pulse_deg_per_s_filtered_log, dtype=float),
           "pulse_rpm_filtered": np.array(pulse_rpm_filtered_log, dtype=float)}
    for ch, vals in enc_logs.items():
        out[f"enc_{ch}"] = np.array(vals, dtype=float)
    return out

# -------------------- Main --------------------
def main():
    if lgpio is None:
        print("[ERROR] lgpio is not available in this environment.")
        return
    try:
        enc = EncoderMux(bus_num=1, settle=0.002, retries=3, retry_wait=0.001) if smbus is not None else None
        candidates = [0, 1]
        working = enc.detect_working_channels(candidates) if enc is not None else []
        if enc is None or not working:
            print("[WARN] No encoder channels responded - proceeding without encoder logging.")
            enc = None
            working = []
        else:
            print("[INFO] Working encoder channels:", working)
    except Exception as e:
        print(f"[WARN] Encoder init failed: {e}")
        enc = None
        working = []

    h = lgpio.gpiochip_open(0)
    for pin in (
        DIR_PIN_NAMA_17, STEP_PIN_NAMA_17, ENA_PIN_NAMA_17,
        DIR_PIN_NAMA_23, STEP_PIN_NAMA_23, ENA_PIN_NAMA_23,
        IN1_PIN, IN2_PIN, PWM_PIN
    ):
        lgpio.gpio_claim_output(h, pin)

    try:
        print("Commands:")
        print("  ACT e/r                              -> actuator extend/retract")
        print("  M1 f/b [steps] [accel] [gamma <shape>] [speed <deg/s>|rpm <value>] [pid? Kp Ki Kd]  -> motor1")
        print("  M2 f/b [steps] [accel] [gamma <shape>] [speed <deg/s>|rpm <value>] [pid? Kp Ki Kd]  -> motor2")
        print("  q                                    -> quit")

        while True:
            cmd = input(">> ").strip().lower().split()
            if not cmd:
                continue
            if cmd[0] == 'q':
                break

            if cmd[0] == 'act' and len(cmd) >= 2:
                if cmd[1] == 'e':
                    extend(h, 1.0)
                elif cmd[1] == 'r':
                    retract(h, 1.0)
                else:
                    print("Specify e (extend) or r (retract)")
                    continue
                time.sleep(ACT_TIME)
                stop_actuator(h)
                continue

            target_speed_deg = None
            s_curve_gamma = 1.0
            use_pid = False
            pid_gains = (2.0, 0.2, 0.0)

            def parse_motion_args(motor_name):
                nonlocal use_pid, pid_gains, target_speed_deg, s_curve_gamma
                if len(cmd) < 2:
                    print("Invalid command.")
                    return None
                direction = 0 if cmd[1] == 'f' else 1
                steps = int(cmd[2]) if len(cmd) >= 3 else (STEPS_17 if motor_name == 'M1' else STEPS_23)
                accel_ratio = float(cmd[3]) if len(cmd) >= 4 else 0.2
                idx = 4
                while idx < len(cmd):
                    token = cmd[idx]
                    if token == 'pid':
                        use_pid = True
                        if idx + 3 < len(cmd):
                            try:
                                Kp = float(cmd[idx + 1]); Ki = float(cmd[idx + 2]); Kd = float(cmd[idx + 3])
                                pid_gains = (Kp, Ki, Kd)
                            except Exception:
                                pass
                            idx += 4
                        else:
                            idx = len(cmd)
                    elif token == 'gamma' and idx + 1 < len(cmd):
                        try:
                            s_curve_gamma = max(float(cmd[idx + 1]), 0.1)
                        except Exception:
                            print("[WARN] Invalid gamma value; ignoring.")
                        idx += 2
                    elif token == 'speed' and idx + 1 < len(cmd):
                        try:
                            target_speed_deg = float(cmd[idx + 1])
                        except Exception:
                            print("[WARN] Invalid speed value; ignoring.")
                        idx += 2
                    elif token == 'rpm' and idx + 1 < len(cmd):
                        try:
                            target_speed_deg = float(cmd[idx + 1]) * 6.0
                        except Exception:
                            print("[WARN] Invalid RPM value; ignoring.")
                        idx += 2
                    else:
                        idx += 1
                return direction, steps, accel_ratio, s_curve_gamma

            achieved_speed_deg = None

            if cmd[0] == 'm1':
                parsed = parse_motion_args('M1')
                if not parsed:
                    continue
                direction, steps, accel_ratio, s_curve_gamma = parsed
                name = 'M1'
                dir_pin, step_pin, ena_pin = DIR_PIN_NAMA_17, STEP_PIN_NAMA_17, ENA_PIN_NAMA_17
                min_delay, max_delay = MIN_DELAY_17, MAX_DELAY_17
            elif cmd[0] == 'm2':
                parsed = parse_motion_args('M2')
                if not parsed:
                    continue
                direction, steps, accel_ratio, s_curve_gamma = parsed
                name = 'M2'
                dir_pin, step_pin, ena_pin = DIR_PIN_NAMA_23, STEP_PIN_NAMA_23, ENA_PIN_NAMA_23
                min_delay, max_delay = MIN_DELAY_23, MAX_DELAY_23
            else:
                print("Unknown command.")
                continue

            min_delay, max_delay, achieved_speed_deg = apply_speed_target_to_delays(
                min_delay, max_delay, target_speed_deg
            )
            speed_info = ""
            if target_speed_deg is not None:
                if achieved_speed_deg is not None:
                    speed_info = f", target_speed~={achieved_speed_deg:.1f} deg/s ({achieved_speed_deg/6.0:.1f} RPM)"
                else:
                    speed_info = ", target_speed ignored"

            print(f"[{name}] steps={steps}, accel_ratio={accel_ratio}, gamma={s_curve_gamma}, direction={direction}, PID={use_pid} gains={pid_gains if use_pid else '-'}{speed_info}")

            enable_motor(h, ena_pin, True)
            if use_pid and enc is not None and working:
                logs = move_stepper_scurve_with_pid(
                    h, dir_pin, step_pin, steps, direction,
                    min_delay, max_delay, accel_ratio=accel_ratio,
                    s_curve_gamma=s_curve_gamma,
                    pid_gains=pid_gains,
                    log_enc=True, enc=enc, enc_channels=working, enc_sample_dt=ENC_SAMPLE_DT
                )
            else:
                logs = move_stepper_scurve_with_logging(
                    h, dir_pin, step_pin, steps, direction,
                    min_delay, max_delay, accel_ratio=accel_ratio,
                    s_curve_gamma=s_curve_gamma,
                    log_enc=(enc is not None), enc=enc, enc_channels=(working if enc else []), enc_sample_dt=ENC_SAMPLE_DT
                )
            enable_motor(h, ena_pin, False)

            if logs is None or 't' not in logs or len(logs['t']) < 3:
                print("[INFO] No data to plot.")
                continue

            t = logs['t']
            cmd_rate = logs.get('cmd_rate', None)
            cmd_ang_vel = cmd_rate * DEG_PER_STEP if cmd_rate is not None else None
            cmd_angle = cumtrapz(cmd_ang_vel, t) if cmd_ang_vel is not None else None
            pulse_deg = logs.get('pulse_deg', None)
            pulse_deg_per_s = logs.get('pulse_deg_per_s', None)
            pulse_deg_per_s_filtered = logs.get('pulse_deg_per_s_filtered', None)
            pulse_rpm = logs.get('pulse_rpm', None)
            pulse_rpm_filtered = logs.get('pulse_rpm_filtered', None)

            csv_pulse_vel = None
            if pulse_deg_per_s_filtered is not None and len(pulse_deg_per_s_filtered) == len(t):
                csv_pulse_vel = pulse_deg_per_s_filtered
            elif pulse_deg_per_s is not None and len(pulse_deg_per_s) == len(t):
                csv_pulse_vel = pulse_deg_per_s

            csv_path = export_motion_csv(
                t=t,
                cmd_angle=cmd_angle,
                pulse_deg=pulse_deg,
                cmd_ang_vel=cmd_ang_vel,
                pulse_vel=csv_pulse_vel,
                filename_base=f"{name.lower()}_{'pid' if use_pid else 'open'}"
            )
            if csv_path:
                print(f"[INFO] Motion CSV saved: {csv_path}")
                if plot_csv_results is not None:
                    try:
                        plot_csv_results(csv_path)
                    except Exception as exc:
                        print(f"[WARN] Plotting CSV failed: {exc}")

            cmd_rpm = None
            if cmd_rate is not None and math.isfinite(PULSES_PER_REV) and PULSES_PER_REV > 0:
                cmd_rpm = cmd_rate * (60.0 / PULSES_PER_REV)

            rpm_for_stats = None
            rpm_stats_label = ""
            if pulse_rpm_filtered is not None and len(pulse_rpm_filtered):
                rpm_for_stats = pulse_rpm_filtered
                rpm_stats_label = "filtered"
            elif pulse_rpm is not None and len(pulse_rpm):
                rpm_for_stats = pulse_rpm
                rpm_stats_label = "raw"

            if rpm_for_stats is not None and len(rpm_for_stats):
                finite_mask = np.isfinite(rpm_for_stats)
                if finite_mask.any():
                    rpm_values = rpm_for_stats[finite_mask]
                    avg_rpm = float(np.mean(rpm_values))
                    max_rpm = float(np.max(rpm_values))
                    print(f"[INFO] Pulse-based RPM ({rpm_stats_label}) avg={avg_rpm:.2f}, max={max_rpm:.2f}")
                else:
                    print(f"[WARN] Pulse-based RPM ({rpm_stats_label}) contains only NaNs.")

            # Summaries of what has already been plotted; prevents duplicate command-only charts later.
            pulse_vel_plot = False
            pulse_pos_plot = False

            # Compare commanded angular velocity vs. pulse-derived velocities (raw and filtered).
            has_pulse_vel = pulse_deg_per_s is not None and len(pulse_deg_per_s) == len(t)
            has_pulse_vel_f = pulse_deg_per_s_filtered is not None and len(pulse_deg_per_s_filtered) == len(t)
            if has_pulse_vel or has_pulse_vel_f:
                vel_series = []
                vel_labels = []
                if cmd_ang_vel is not None:
                    vel_series.append(cmd_ang_vel)
                    vel_labels.append("commanded ω [deg/s]")
                if has_pulse_vel:
                    vel_series.append(pulse_deg_per_s)
                    vel_labels.append("pulse ω raw [deg/s]")
                if has_pulse_vel_f:
                    vel_series.append(pulse_deg_per_s_filtered)
                    vel_labels.append("pulse ω LPF [deg/s]")
                plot_time_series(t, vel_series, vel_labels, "Pulse-derived Angular Velocity", "Angular velocity (deg/s)", "pulse_speed_deg")
                pulse_vel_plot = True

            # Overlay commanded vs. pulse-derived RPM traces (raw / filtered).
            has_pulse_rpm = pulse_rpm is not None and len(pulse_rpm) == len(t)
            has_pulse_rpm_f = pulse_rpm_filtered is not None and len(pulse_rpm_filtered) == len(t)
            if has_pulse_rpm or has_pulse_rpm_f:
                rpm_series = []
                rpm_labels = []
                if cmd_rpm is not None:
                    rpm_series.append(cmd_rpm)
                    rpm_labels.append("commanded RPM")
                if has_pulse_rpm:
                    rpm_series.append(pulse_rpm)
                    rpm_labels.append("pulse RPM raw")
                if has_pulse_rpm_f:
                    rpm_series.append(pulse_rpm_filtered)
                    rpm_labels.append("pulse RPM LPF")
                plot_time_series(t, rpm_series, rpm_labels, "Pulse-derived Speed (RPM)", "Speed (RPM)", "pulse_speed_rpm")

            # Compare commanded angle to pulse-integrated angle for displacement accuracy.
            if pulse_deg is not None and len(pulse_deg) == len(t):
                pos_series = []
                pos_labels = []
                if cmd_angle is not None:
                    pos_series.append(cmd_angle)
                    pos_labels.append("commanded θ [deg]")
                    pulse_deg_aligned = pulse_deg - pulse_deg[0] + cmd_angle[0]
                else:
                    pulse_deg_aligned = pulse_deg
                pos_series.append(pulse_deg_aligned)
                pos_labels.append("pulse θ [deg]")
                plot_time_series(t, pos_series, pos_labels, "Pulse-derived Angle", "Angle (deg)", "pulse_angle_deg")
                pulse_pos_plot = True

            usable = []
            if enc is not None:
                for ch in (working if working else []):
                    key = f"enc_{ch}"
                    y = logs.get(key, None)
                    if y is None or len(y) != len(t):
                        continue
                    nan_ratio = np.isnan(y).mean()
                    if nan_ratio < 0.3:
                        usable.append(ch)
                    else:
                        print(f"[WARN] ch{ch} has too many NaNs ({nan_ratio:.0%}); skipping plot.")

            # Minimal RPM plot (raw) even if filtered version missing.
            if pulse_rpm is not None and len(pulse_rpm) == len(t):
                rpm_series = [pulse_rpm]
                rpm_labels = ["pulse RPM"]
                if cmd_rpm is not None:
                    rpm_series.append(cmd_rpm)
                    rpm_labels.append("commanded RPM")
                plot_time_series(t, rpm_series, rpm_labels, "Pulse-derived Speed", "Speed (RPM)", "pulse_speed_rpm")

            # Plots
            if not usable:
                # No encoder data available: fall back to command-only plots (unless already shown).
                if cmd_rate is not None:
                    if cmd_ang_vel is not None and not pulse_vel_plot:
                        plot_time_series(t, [cmd_ang_vel], ["commanded ω"], "Commanded Angular Velocity", "Angular velocity (deg/s)", "cmd_angular_velocity")
                        if cmd_angle is not None and not pulse_pos_plot:
                            plot_time_series(t, [cmd_angle], ["commanded θ"], "Commanded Angle (integrated)", "Angle (deg)", "cmd_angle")
                    plot_time_series(t, [cmd_rate], ["cmd step rate"], "Commanded Step Rate", "Steps per second", "cmd_step_rate")
                continue

            for ch in usable:
                key = f"enc_{ch}"
                ang_raw = logs[key]
                mask = ~np.isnan(ang_raw)
                ang_filled = ang_raw.copy()
                if not mask.all():
                    valid_t = t[mask]
                    valid_y = ang_raw[mask]
                    ang_filled = np.interp(t, valid_t, valid_y)
                ang_unwrapped = unwrap_deg(ang_filled)
                vel = finite_diff(ang_unwrapped, t)
                acc = finite_diff(vel, t)

                # Plot encoder angles against commands when available.
                if cmd_angle is not None:
                    cmd_angle_aligned = cmd_angle - cmd_angle[0] + ang_unwrapped[0]
                    plot_time_series(t, [ang_unwrapped, cmd_angle_aligned], [f"ch{ch} measured θ", "commanded θ"], f"Encoder ch{ch} Angle", "Angle (deg)", f"ch{ch}_angle_overlay")
                else:
                    plot_time_series(t, [ang_unwrapped], [f"ch{ch} measured θ"], f"Encoder ch{ch} Angle", "Angle (deg)", f"ch{ch}_angle")

                # Likewise compare encoder velocity and acceleration.
                if cmd_ang_vel is not None:
                    plot_time_series(t, [vel, cmd_ang_vel], [f"ch{ch} measured ω", "commanded ω"], f"Encoder ch{ch} Angular Velocity", "Angular velocity (deg/s)", f"ch{ch}_angular_velocity_overlay")
                else:
                    plot_time_series(t, [vel], [f"ch{ch} measured ω"], f"Encoder ch{ch} Angular Velocity", "Angular velocity (deg/s)", f"ch{ch}_angular_velocity")

                plot_time_series(t, [acc], [f"ch{ch} measured α"], f"Encoder ch{ch} Angular Acceleration", "Angular acceleration (deg/s^2)", f"ch{ch}_angular_acceleration")

    except KeyboardInterrupt:
        print("\n[Interrupted]")
    finally:
        try:
            stop_actuator(h)
        except Exception:
            pass
        try:
            if lgpio is not None:
                enable_motor(h, ENA_PIN_NAMA_17, False)
                enable_motor(h, ENA_PIN_NAMA_23, False)
                lgpio.gpiochip_close(h)
        except Exception:
            pass
        print("GPIO released.")

if __name__ == "__main__":
    main()
