import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from encoder import DEG_PER_STEP, STEPS_PER_REV

# -------------------- Pulse Limits --------------------
MIN_PULSE_INTERVAL = 0.001   # 1 kHz
MAX_PULSE_INTERVAL = 0.05    # 20 Hz

def vmax_effective(v_max: float, min_pulse_interval: float = MIN_PULSE_INTERVAL) -> float:
    """하드웨어 펄스 한계 고려한 최대 속도 (steps/s)"""
    hw_limit = 1.0 / min_pulse_interval
    return min(v_max, hw_limit)

# -------------------- Parameter Calculator --------------------
def calc_scurve_params(total_steps=None, v_max=None, total_time=None, show=True):
    if [total_steps, v_max, total_time].count(None) != 1:
        raise ValueError("세 변수 중 정확히 2개만 지정해야 합니다.")

    if total_steps is None:
        total_steps = int((v_max * total_time) / 2)
    elif v_max is None:
        v_max = (2 * total_steps) / total_time
    elif total_time is None:
        total_time = (2 * total_steps) / v_max

    result = {"total_steps": total_steps, "v_max": v_max, "total_time": total_time}

    if show:
        print("[S-curve Parameters]")
        print(result, "\n")
        df = pd.DataFrame([result])
        print(df)

        t = np.linspace(0, total_time, 500)
        v = v_max * (np.sin(np.pi * t / total_time))**2
        plt.figure()
        plt.plot(t, v, label="S-curve velocity (symmetric)")
        plt.xlabel("Time [s]")
        plt.ylabel("Velocity [steps/s]")
        plt.grid()
        plt.legend()
        plt.show()

    return result

# -------------------- Velocity Profiles --------------------
def shaped_s_curve_velocity(t: float, v_max: float, t_acc: float, t_const: float, t_dec: float) -> float:
    total_time = t_acc + t_const + t_dec
    if t < 0 or t > total_time:
        return 0.0
    if t < t_acc:  # 가속
        return v_max * (np.sin(np.pi * t / (2 * t_acc)))**2
    elif t < t_acc + t_const:  # 정속
        return v_max
    else:  # 감속
        tau = t - (t_acc + t_const)
        return v_max * (np.cos(np.pi * tau / (2 * t_dec)))**2

def as_curve_velocity(t: float, v_max: float, t_acc: float, t_dec: float, total_time: float) -> float:
    t_const = total_time - t_acc - t_dec
    if t_const < 0:
        raise ValueError("가속+감속 시간이 전체 시간보다 짧아야 합니다.")
    if t < t_acc:
        return v_max * (np.sin(np.pi * t / (2 * t_acc)))**2
    elif t < t_acc + t_const:
        return v_max
    elif t < total_time:
        tau = t - (t_acc + t_const)
        return v_max * (np.cos(np.pi * tau / (2 * t_dec)))**2
    return 0.0

# -------------------- Total Time Calculator --------------------
def compute_total_time_scurve(total_steps: int, v_max: float, shape="mid") -> float:
    v_eff = vmax_effective(v_max)
    if shape == "short": avg = 0.5
    elif shape == "long": avg = 0.9
    else: avg = 0.75
    return total_steps / (avg * v_eff)

def compute_total_time_ascurve(total_steps: int, v_max: float, shape="mid"):
    if shape == "short":  r_acc, r_dec, r_const = 0.4, 0.6, 0.0
    elif shape == "long": r_acc, r_dec, r_const = 0.15, 0.25, 0.6
    else:                 r_acc, r_dec, r_const = 0.2, 0.4, 0.4
    v_eff = vmax_effective(v_max)
    coeff = 0.5*(r_acc+r_dec) + r_const
    T = total_steps / (v_eff * coeff)
    return T, r_acc*T, r_dec*T, r_const*T

# -------------------- Run Motor with S-Curve --------------------
def run_motor_scurve(gpio, encoder, motor_id, direction, total_steps, v_max, shape="mid"):
    total_time = compute_total_time_scurve(total_steps, v_max, shape)
    if shape == "short":
        t_acc, t_dec, t_const = total_time*0.5, total_time*0.5, 0
    elif shape == "long":
        t_acc, t_dec, t_const = total_time*0.1, total_time*0.1, total_time*0.8
    else:
        t_acc, t_dec, t_const = total_time*0.25, total_time*0.25, total_time*0.5
    v_eff = vmax_effective(v_max)

    gpio.set_dir(motor_id, direction == 'f')
    gpio.set_enable(motor_id, True)

    moved_steps = 0
    prev_enc = encoder.read()
    enc_init = (prev_enc / 1000) * 360
    start = time.time()
    data_log = []
    enc_positions = []
    enc_times = []

    while moved_steps < total_steps:
        t = time.time() - start
        if t > total_time: break

        com_vel_steps = shaped_s_curve_velocity(t, v_eff, t_acc, t_const, t_dec)
        if com_vel_steps < 1e-6:
            time.sleep(0.001)
            continue

        pulse_interval = max(MIN_PULSE_INTERVAL, min(1.0/com_vel_steps, MAX_PULSE_INTERVAL))
        gpio.pulse_step(motor_id, high_time=0.00002, low_time=pulse_interval-0.00002)
        moved_steps += 1

        com_pos = moved_steps * DEG_PER_STEP
        com_vel_deg = com_vel_steps * DEG_PER_STEP

        enc_now = encoder.read()
        enc_positions.append((enc_now/1000)*360 - enc_init)
        enc_times.append(t)

        # 속도는 위치 신호 smoothing 후 graph.py에서 추정
        data_log.append([int(t*1000), com_pos, enc_positions[-1], com_vel_deg, 0.0])

    return data_log

# -------------------- Run Motor with AS-Curve --------------------
def run_motor_ascurve(gpio, encoder, motor_id, direction, total_steps, v_max, shape="mid"):
    total_time, t_acc, t_dec, t_const = compute_total_time_ascurve(total_steps, v_max, shape)
    v_eff = vmax_effective(v_max)

    gpio.set_dir(motor_id, direction == 'f')
    gpio.set_enable(motor_id, True)

    moved_steps = 0
    prev_enc = encoder.read()
    enc_init = (prev_enc / 1000) * 360
    start = time.time()
    data_log = []
    enc_positions = []
    enc_times = []

    while moved_steps < total_steps:
        t = time.time() - start
        if t > total_time: break

        com_vel_steps = as_curve_velocity(t, v_eff, t_acc, t_dec, total_time)
        if com_vel_steps < 1e-6:
            time.sleep(0.001)
            continue

        pulse_interval = max(MIN_PULSE_INTERVAL, min(1.0/com_vel_steps, MAX_PULSE_INTERVAL))
        gpio.pulse_step(motor_id, high_time=0.00002, low_time=pulse_interval-0.00002)
        moved_steps += 1

        com_pos = moved_steps * DEG_PER_STEP
        com_vel_deg = com_vel_steps * DEG_PER_STEP

        enc_now = encoder.read()
        enc_positions.append((enc_now/1000)*360 - enc_init)
        enc_times.append(t)

        data_log.append([int(t*1000), com_pos, enc_positions[-1], com_vel_deg, 0.0])

    return data_log
