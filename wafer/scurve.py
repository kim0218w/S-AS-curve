import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from encoder import DEG_PER_STEP, STEPS_PER_REV
MIN_PULSE_INTERVAL = 0.001
MAX_PULSE_INTERVAL = 0.05

def vmax_effective(v_max: float, pulse_interval: float) -> float:
    return min(v_max, 1.0 / MIN_PULSE_INTERVAL)

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
    """short/mid/long 형태의 S-curve 속도"""
    total_time = t_acc + t_const + t_dec
    if t < 0 or t > total_time:
        return 0.0

    # 가속 구간
    if t < t_acc:
        return v_max * (np.sin(np.pi * t / (2 * t_acc)))**2

    # 정속 구간
    elif t < t_acc + t_const:
        return v_max

    # 감속 구간
    else:
        tau = t - (t_acc + t_const)
        return v_max * (np.cos(np.pi * tau / (2 * t_dec)))**2


def as_curve_velocity(t: float, v_max: float, t_acc: float, t_dec: float, total_time: float) -> float:
    """비대칭 AS-curve 속도"""
    t_const = total_time - t_acc - t_dec
    if t_const < 0:
        raise ValueError("가속+감속 시간이 전체 시간보다 짧아야 합니다.")

    if 0 <= t < t_acc:  # 가속
        return v_max * (np.sin(np.pi * t / (2 * t_acc)))**2
    elif t_acc <= t < t_acc + t_const:  # 정속
        return v_max
    elif t_acc + t_const <= t <= total_time:  # 감속
        tau = t - (t_acc + t_const)
        return v_max * (np.cos(np.pi * tau / (2 * t_dec)))**2
    return 0.0


# -------------------- Constants --------------------
ENC_CPR = 1000
DEG_PER_STEP = 180.0 / 10000.0   # 0.018°/step


# -------------------- Run Motor with S-Curve (short/mid/long) --------------------
def compute_total_time_scurve(total_steps: int, v_max: float, shape="mid") -> float:
    v_eff = vmax_effective(v_max)
    if shape == "short":   avg = 0.5    # t_acc=0.5T, t_const=0, t_dec=0.5T
    elif shape == "long":  avg = 0.9    # 0.1/0.8/0.1
    else:                  avg = 0.75   # 0.25/0.5/0.25
    return total_steps / (avg * v_eff)


def run_motor_scurve(gpio, encoder, motor_id: int, direction: str,
                     total_steps: int, v_max: float,
                     shape: str = "mid"):
    total_time = compute_total_time_scurve(total_steps, v_max, shape)
    # shape 비율
    if shape == "short": t_acc, t_dec, t_const = total_time*0.5, total_time*0.5, 0.0
    elif shape == "long": t_acc, t_dec, t_const = total_time*0.1, total_time*0.1, total_time*0.8
    else: t_acc, t_dec, t_const = total_time*0.25, total_time*0.25, total_time*0.5
    v_eff = vmax_effective(v_max)

    gpio.set_dir(motor_id, direction == 'f'); gpio.set_enable(motor_id, True)
    moved_steps, prev_enc = 0, encoder.read()
    enc_init = (prev_enc / ENC_CPR) * 360.0
    start_time = time.time(); data_log = []

    while moved_steps < total_steps:
        t = time.time() - start_time
        if t > (t_acc + t_const + t_dec):  # 이론상 충분히 맞춰서 거의 안 걸림
            break

        com_vel_steps = shaped_s_curve_velocity(t, v_eff, t_acc, t_const, t_dec)
        if com_vel_steps < 1e-6:
            time.sleep(0.001); continue

        pulse_interval = max(MIN_PULSE_INTERVAL, min(1.0 / com_vel_steps, MAX_PULSE_INTERVAL))
        gpio.pulse_step(motor_id, high_time=0.00002, low_time=pulse_interval - 0.00002)
        moved_steps += 1

        # (로그 기록은 기존 그대로)


        # 위치/속도 기록
        com_pos = moved_steps * DEG_PER_STEP
        com_vel_deg = com_vel_steps * DEG_PER_STEP
        enc_now = encoder.read()
        delta = enc_now - prev_enc
        prev_enc = enc_now
        enc_pos_deg = (enc_now / ENC_CPR) * 360.0 - enc_init
        enc_vel_deg = (delta / pulse_interval) * (360.0 / ENC_CPR) if delta != 0 else 0.0

        t_ms = int(round(t * 1000))
        data_log.append([t_ms, com_pos, enc_pos_deg, com_vel_deg, enc_vel_deg])

    return data_log

# -------------------- Run Motor with AS-Curve --------------------
def compute_total_time_ascurve(total_steps: int, v_max: float, shape="mid"):
    # shape → 비율
    if shape == "short":  r_acc, r_dec, r_const = 0.4, 0.6, 0.0
    elif shape == "long": r_acc, r_dec, r_const = 0.15, 0.25, 0.6
    else:                 r_acc, r_dec, r_const = 0.2, 0.4, 0.4
    v_eff = vmax_effective(v_max)
    coeff = 0.5*(r_acc + r_dec) + r_const   # 면적 계수
    T = total_steps / (v_eff * coeff)
    return T, r_acc*T, r_dec*T, r_const*T


def run_motor_ascurve(gpio, encoder, motor_id: int, direction: str,
                      total_steps: int, v_max: float,
                      shape: str = "mid"):

    # 이동시간 자동계산
    total_time, t_acc, t_dec, t_const = compute_total_time_ascurve(total_steps, v_max, shape)
    v_eff = vmax_effective(v_max)

    gpio.set_dir(motor_id, direction == 'f')
    gpio.set_enable(motor_id, True)

    moved_steps = 0
    prev_enc = encoder.read()
    enc_init = (prev_enc / ENC_CPR) * 360.0
    data_log = []
    start_time = time.time()

    while moved_steps < total_steps:
        t = time.time() - start_time
        if t > total_time:
            break

        com_vel_steps = as_curve_velocity(t, v_eff, t_acc, t_dec, total_time)
        pulse_interval = max(MIN_PULSE_INTERVAL, min(1.0 / com_vel_steps, MAX_PULSE_INTERVAL))
    
        if com_vel_steps < 1e-6:
            time.sleep(0.001)
            continue

        pulse_interval = 1.0 / com_vel_steps
        pulse_interval = max(0.001, min(pulse_interval, 0.05))

        gpio.pulse_step(motor_id, high_time=0.00002, low_time=pulse_interval - 0.00002)
        moved_steps += 1

        # 위치/속도 기록
        com_pos = moved_steps * DEG_PER_STEP
        com_vel_deg = com_vel_steps * DEG_PER_STEP
        enc_now = encoder.read()
        delta = enc_now - prev_enc
        prev_enc = enc_now
        enc_pos_deg = (enc_now / ENC_CPR) * 360.0 - enc_init
        enc_vel_deg = (delta / pulse_interval) * (360.0 / ENC_CPR) if delta != 0 else 0.0

        t_ms = int(round(t * 1000))
        data_log.append([t_ms, com_pos, enc_pos_deg, com_vel_deg, enc_vel_deg])

    return data_log
