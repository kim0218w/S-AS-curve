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
        plt.plot(t, v, label="S-curve velocity")
        plt.xlabel("Time [s]")
        plt.ylabel("Velocity [steps/s]")
        plt.grid()
        plt.legend()
        plt.show()

    return result


# -------------------- Velocity Profiles --------------------
def s_curve_velocity(t: float, v_max: float, total_time: float) -> float:
    """대칭 S-curve 속도 [steps/s]"""
    if total_time <= 0:
        return 0.0
    return v_max * (np.sin(np.pi * t / total_time))**2


def as_curve_velocity(t: float, v_max: float, t_acc: float, t_dec: float, total_time: float) -> float:
    """비대칭 AS-curve 속도 [steps/s]"""
    t_const = total_time - t_acc - t_dec
    if t_const < 0:
        raise ValueError("가속+감속 시간이 전체 시간보다 짧아야 합니다.")

    # 가속 구간
    if 0 <= t < t_acc:
        return v_max * (np.sin(np.pi * t / (2 * t_acc)))**2

    # 정속 구간
    elif t_acc <= t < t_acc + t_const:
        return v_max

    # 감속 구간
    elif t_acc + t_const <= t <= total_time:
        tau = t - (t_acc + t_const)
        return v_max * (np.cos(np.pi * tau / (2 * t_dec)))**2

    return 0.0


# -------------------- Constants --------------------
ENC_CPR = 1000   # encoder counts per revolution

# 10000 steps = 180 deg 기준
DEG_PER_STEP = 180.0 / 10000.0   # 0.018°/step


# -------------------- Run Motor with S-Curve --------------------
def run_motor_scurve(gpio, encoder, motor_id: int, direction: str,
                     total_steps: int, v_max: float, total_time: float):
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

        com_vel_steps = s_curve_velocity(t, v_max, total_time)
        if com_vel_steps < 1e-6:
            time.sleep(0.001)
            continue

        pulse_interval = 1.0 / com_vel_steps
        pulse_interval = max(0.001, min(pulse_interval, 0.05))  # 안전 범위 제한

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
                      total_steps: int, v_max: float, total_time: float,
                      t_acc: float, t_dec: float):
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
