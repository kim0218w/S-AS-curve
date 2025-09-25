#!/usr/bin/env python3
import time
import math
import sys
import os
import re
import numpy as np
import matplotlib.pyplot as plt

try:
    import lgpio
except Exception:
    lgpio = None

try:
    import smbus
except Exception:
    smbus = None

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

# -------------------- Output: graph folder --------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
GRAPH_DIR = os.path.join(SCRIPT_DIR, "graph")
os.makedirs(GRAPH_DIR, exist_ok=True)

def _safe_name(s: str) -> str:
    return re.sub(r'[^A-Za-z0-9._-]+', '_', s)

def _save_or_show(fig, filename_base: str):
    ts = time.strftime("%Y%mcd%d_%H%M%S")
    fname = f"{filename_base}_{ts}.png"
    path = os.path.join(GRAPH_DIR, _safe_name(fname))
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.show(block=False)
    plt.pause(0.001)

def plot_time_series(t, ys, labels, title, ylabel, filename_base):
    fig = plt.figure()
    for y in ys:
        plt.plot(t, y)
    if title:
        plt.title(title)
    plt.xlabel("Time (s)")
    plt.ylabel(ylabel)
    plt.grid(True)
    if labels and len(labels) == len(ys):
        plt.legend(labels)
    plt.tight_layout()
    _save_or_show(fig, filename_base)
    plt.close(fig)

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

def smooth_cos_delay(i, n, min_delay, max_delay):
    if min_delay >= max_delay:
        min_delay, max_delay = sorted([min_delay, max_delay])
    min_delay = max(min_delay, MIN_SAFE_DELAY)
    if n <= 1:
        return min_delay
    t_norm = i / (n - 1)
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
    accel_ratio=0.2, log_enc: bool = True, enc: EncoderMux = None, enc_channels=None,
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

    t0 = time.monotonic()
    last_sample_t = t0
    time_log = []
    enc_logs = {ch: [] for ch in enc_channels}
    cmd_rate_log = []
    current_cmd_rate = 0.0

    def sample_all(ts):
        nonlocal last_sample_t
        if ts - last_sample_t >= enc_sample_dt:
            if log_enc and enc is not None:
                for ch in enc_channels:
                    ang = enc.try_read_channel(ch)
                    enc_logs[ch].append(ang)
            cmd_rate_log.append(current_cmd_rate)
            time_log.append(ts - t0)
            last_sample_t = ts

    for i in range(accel_steps):
        d = smooth_cos_delay(i, max(accel_steps, 1), min_delay, max_delay)
        current_cmd_rate = 1.0 / (2.0 * d)
        sample_all(time.monotonic())
        queue_pulse(h, step_pin, d)

    for _ in range(const_steps):
        d = min_delay
        current_cmd_rate = 1.0 / (2.0 * d)
        sample_all(time.monotonic())
        queue_pulse(h, step_pin, d)

    for i in range(decel_steps):
        d = smooth_cos_delay(decel_steps - 1 - i, max(decel_steps, 1), min_delay, max_delay)
        current_cmd_rate = 1.0 / (2.0 * d)
        sample_all(time.monotonic())
        queue_pulse(h, step_pin, d)

    time.sleep(min_delay * TX_BACKLOG)
    sample_all(time.monotonic())

    out = {"t": np.array(time_log, dtype=float),
           "cmd_rate": np.array(cmd_rate_log, dtype=float)}
    for ch, vals in enc_logs.items():
        out[f"enc_{ch}"] = np.array(vals, dtype=float)
    return out

# -------------------- S-curve + PID (closed-loop) --------------------
def move_stepper_scurve_with_pid(
    h, dir_pin, step_pin, total_steps, direction,
    min_delay, max_delay, accel_ratio=0.2,
    pid_gains=(2.0, 0.2, 0.0),  # (Kp, Ki, Kd) in (deg/s) per deg
    log_enc: bool = True, enc: EncoderMux = None, enc_channels=None,
    enc_sample_dt: float = ENC_SAMPLE_DT
):
    if enc is None or not enc_channels:
        return move_stepper_scurve_with_logging(
            h, dir_pin, step_pin, total_steps, direction, min_delay, max_delay,
            accel_ratio, log_enc=False, enc=None, enc_channels=[], enc_sample_dt=enc_sample_dt
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
        ff_delays.append(smooth_cos_delay(i, max(accel_steps, 1), min_delay, max_delay))
    for _ in range(const_steps):
        ff_delays.append(min_delay)
    for i in range(decel_steps):
        ff_delays.append(smooth_cos_delay(decel_steps - 1 - i, max(decel_steps, 1), min_delay, max_delay))

    t0 = time.monotonic()
    last_sample_t = t0
    time_log, cmd_rate_log = [], []
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
        nonlocal last_sample_t
        if ts - last_sample_t >= enc_sample_dt:
            for ch in enc_channels:
                val = enc.try_read_channel(ch)
                enc_logs[ch].append(val)
            time_log.append(ts - t0)
            cmd_rate_log.append(current_rate_steps)
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

        ts = time.monotonic()
        sample_all(ts)

        queue_pulse(h, step_pin, d)

    time.sleep(min_delay * TX_BACKLOG)
    sample_all(time.monotonic())

    out = {"t": np.array(time_log, dtype=float),
           "cmd_rate": np.array(cmd_rate_log, dtype=float)}
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
        print("  M1 f/b [steps] [accel] [pid? Kp Ki Kd]  -> motor1")
        print("  M2 f/b [steps] [accel] [pid? Kp Ki Kd]  -> motor2")
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

            target = None
            use_pid = False
            pid_gains = (2.0, 0.2, 0.0)

            def parse_motion_args(motor_name):
                nonlocal use_pid, pid_gains
                if len(cmd) < 2:
                    print("Invalid command.")
                    return None
                direction = 0 if cmd[1] == 'f' else 1
                steps = int(cmd[2]) if len(cmd) >= 3 else (STEPS_17 if motor_name == 'M1' else STEPS_23)
                accel_ratio = float(cmd[3]) if len(cmd) >= 4 else 0.2
                # optional: 'pid' [Kp Ki Kd]
                if len(cmd) >= 5 and cmd[4] == 'pid':
                    use_pid = True
                    if len(cmd) >= 8:
                        try:
                            Kp = float(cmd[5]); Ki = float(cmd[6]); Kd = float(cmd[7])
                            pid_gains = (Kp, Ki, Kd)
                        except Exception:
                            pass
                return direction, steps, accel_ratio

            if cmd[0] == 'm1':
                parsed = parse_motion_args('M1')
                if not parsed:
                    continue
                direction, steps, accel_ratio = parsed
                name = 'M1'
                dir_pin, step_pin, ena_pin = DIR_PIN_NAMA_17, STEP_PIN_NAMA_17, ENA_PIN_NAMA_17
                min_delay, max_delay = MIN_DELAY_17, MAX_DELAY_17
            elif cmd[0] == 'm2':
                parsed = parse_motion_args('M2')
                if not parsed:
                    continue
                direction, steps, accel_ratio = parsed
                name = 'M2'
                dir_pin, step_pin, ena_pin = DIR_PIN_NAMA_23, STEP_PIN_NAMA_23, ENA_PIN_NAMA_23
                min_delay, max_delay = MIN_DELAY_23, MAX_DELAY_23
            else:
                print("Unknown command.")
                continue

            print(f"[{name}] steps={steps}, accel_ratio={accel_ratio}, direction={direction}, PID={use_pid} gains={pid_gains if use_pid else '-'}")

            enable_motor(h, ena_pin, True)
            if use_pid and enc is not None and working:
                logs = move_stepper_scurve_with_pid(
                    h, dir_pin, step_pin, steps, direction,
                    min_delay, max_delay, accel_ratio=accel_ratio,
                    pid_gains=pid_gains,
                    log_enc=True, enc=enc, enc_channels=working, enc_sample_dt=ENC_SAMPLE_DT
                )
            else:
                logs = move_stepper_scurve_with_logging(
                    h, dir_pin, step_pin, steps, direction,
                    min_delay, max_delay, accel_ratio=accel_ratio,
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

            # Plots
            if not usable:
                if cmd_rate is not None:
                    if cmd_ang_vel is not None:
                        plot_time_series(t, [cmd_ang_vel], ["commanded ω"], "Commanded Angular Velocity", "Angular velocity (deg/s)", "cmd_angular_velocity")
                        if cmd_angle is not None:
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

                if cmd_angle is not None:
                    cmd_angle_aligned = cmd_angle - cmd_angle[0] + ang_unwrapped[0]
                    plot_time_series(t, [ang_unwrapped, cmd_angle_aligned], [f"ch{ch} measured θ", "commanded θ"], f"Encoder ch{ch} Angle", "Angle (deg)", f"ch{ch}_angle_overlay")
                else:
                    plot_time_series(t, [ang_unwrapped], [f"ch{ch} measured θ"], f"Encoder ch{ch} Angle", "Angle (deg)", f"ch{ch}_angle")

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
