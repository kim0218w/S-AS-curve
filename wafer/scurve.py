import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

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
def run_motor_scurve(gpio, encoder, motor_id: int, direction: str,
                     total_steps: int, v_max: float,
                     shape: str = "mid"):
    """
    shape: "short" (정속=0), "mid" (정속=50%), "long" (정속=80%)
    """

    # 기본 total_time 계산 (평균 속도 = v_max/2 가정)
    total_time = (2 * total_steps) / v_max

    # shape에 따른 비율
    if shape == "short":
        t_acc = total_time * 0.5
        t_dec = total_time * 0.5
        t_const = 0.0
    elif shape == "long":
        t_acc = total_time * 0.1
        t_dec = total_time * 0.1
        t_const = total_time * 0.8
    else:  # mid
        t_acc = total_time * 0.25
        t_dec = total_time * 0.25
        t_const = total_time * 0.5

    gpio.set_dir(motor_id, direction == 'f')
    gpio.set_enable(motor_id, True)

    moved_steps = 0
    prev_enc = encoder.read()
    enc_init = (prev_enc / ENC_CPR) * 360.0
    data_log = []
    start_time = time.time()
    total_time = t_acc + t_const + t_dec

    while moved_steps < total_steps:
        t = time.time() - start_time
        if t > total_time:
            break

        com_vel_steps = shaped_s_curve_velocity(t, v_max, t_acc, t_const, t_dec)
        if com_vel_steps < 1e-6:
            time.sleep(0.001)
            continue

        pulse_interval = 1.0 / com_vel_steps
        pulse_interval = max(0.001, min(pulse_interval, 0.05))

        gpio.pulse_step(motor_id, high_time=0.00002, low_time=pulse_interval - 0.00002)
        moved_steps += 1

        # 명령 위치/속도 [deg]
        com_pos = moved_steps * DEG_PER_STEP
        com_vel_deg = com_vel_steps * DEG_PER_STEP

        # 엔코더 위치/속도 [deg], [deg/s]
        enc_now = encoder.read()
        delta = enc_now - prev_enc
        prev_enc = enc_now
        enc_pos_deg = (enc_now / ENC_CPR) * 360.0 - enc_init
        enc_vel_deg = (delta / pulse_interval) * (360.0 / ENC_CPR) if delta != 0 else 0.0

        t_ms = int(round(t * 1000))
        data_log.append([t_ms, com_pos, enc_pos_deg, com_vel_deg, enc_vel_deg])

    return data_log


# -------------------- Run Motor with AS-Curve --------------------
def run_motor_ascurve(gpio, encoder, motor_id: int, direction: str,
                      total_steps: int, v_max: float,
                      shape: str = "mid", t_acc: float = None, t_dec: float = None, total_time: float = None):

    if total_time is None:
        total_time = (2 * total_steps) / v_max

    # shape에 따라 자동 분배
    if t_acc is None or t_dec is None:
        if shape == "short":
            t_acc = total_time * 0.4
            t_dec = total_time * 0.6
            t_const = 0
        elif shape == "long":
            t_acc = total_time * 0.15
            t_dec = total_time * 0.25
            t_const = total_time - t_acc - t_dec
        else:  # mid
            t_acc = total_time * 0.2
            t_dec = total_time * 0.4
            t_const = total_time - t_acc - t_dec
    else:
        t_const = total_time - t_acc - t_dec

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

        com_vel_steps = as_curve_velocity(t, v_max, t_acc, t_dec, total_time)
        if com_vel_steps < 1e-6:
            time.sleep(0.001)
            continue

        pulse_interval = 1.0 / com_vel_steps
        pulse_interval = max(0.001, min(pulse_interval, 0.05))

        gpio.pulse_step(motor_id, high_time=0.00002, low_time=pulse_interval - 0.00002)
        moved_steps += 1

        # 명령 위치/속도 [deg]
        com_pos = moved_steps * DEG_PER_STEP
        com_vel_deg = com_vel_steps * DEG_PER_STEP

        # 엔코더 위치/속도 [deg], [deg/s]
        enc_now = encoder.read()
        delta = enc_now - prev_enc
        prev_enc = enc_now
        enc_pos_deg = (enc_now / ENC_CPR) * 360.0 - enc_init
        enc_vel_deg = (delta / pulse_interval) * (360.0 / ENC_CPR) if delta != 0 else 0.0

        t_ms = int(round(t * 1000))
        data_log.append([t_ms, com_pos, enc_pos_deg, com_vel_deg, enc_vel_deg])

    return data_log
