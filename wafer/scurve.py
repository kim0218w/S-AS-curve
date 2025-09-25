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
def calc_scurve_params(total_steps=None, v_max=None, total_time=None,
                       pitch_mm=5.0, steps_per_rev=200, microstep=16,
                       show=True):
    """
    S-curve 프로파일 파라미터 계산 및 표시
    - v(t) = v_max * sin^2(pi * t / T)
    - total_steps, v_max, total_time 중 2개를 지정하면 나머지 1개를 계산
    - 출력은 mm/s 단위로 환산된 속도 프로파일
    """
    if [total_steps, v_max, total_time].count(None) != 1:
        raise ValueError("세 변수 중 정확히 2개만 지정해야 합니다.")

    # --- 계산 ---
    if total_steps is None:
        total_steps = int((v_max * total_time) / 2)
    elif v_max is None:
        v_max = (2 * total_steps) / total_time
    elif total_time is None:
        total_time = (2 * total_steps) / v_max

    result = {"total_steps": total_steps, "v_max_steps": v_max, "total_time": total_time}

    if show:
        # --- 콘솔 출력 ---
        print("[S-curve Parameters]")
        print(f"총 스텝 수 : {total_steps} steps")
        print(f"최대 속도  : {v_max:.2f} steps/s")
        print(f"총 이동 시간: {total_time:.3f} s\n")

        # --- DataFrame 출력 ---
        df = pd.DataFrame([result])
        print(df, "\n")

        # --- 그래프 표시 ---
        t = np.linspace(0, total_time, 500)
        v_steps = v_max * (np.sin(np.pi * t / total_time))**2  # step/s
        v_mm = (v_steps / (steps_per_rev * microstep)) * pitch_mm  # mm/s 변환

        plt.figure(figsize=(6, 4))
        plt.plot(t, v_mm, label="S-curve velocity [mm/s]")
        plt.xlabel("Time [s]")
        plt.ylabel("Velocity [mm/s]")
        plt.title("S-curve Motion Profile (Commanded)")
        plt.grid()
        plt.legend()
        plt.tight_layout()
        plt.show()

    return result

# -------------------- Velocity Profiles --------------------
def s_curve_velocity(t: float, v_max: float, total_time: float,
                     pitch_mm=5.0, steps_per_rev=200, microstep=16) -> float:
    """
    주어진 시간 t에서 S-curve 속도를 mm/s 단위로 반환
    """
    if total_time <= 0:
        return 0.0
    v_steps = v_max * (np.sin(np.pi * t / total_time))**2  # step/s
    return (v_steps / (steps_per_rev * microstep)) * pitch_mm  # mm/s

def as_curve_velocity(t: float, v_max: float, t_acc: float, t_dec: float, total_time: float,
                      pitch_mm=5.0, steps_per_rev=200, microstep=16) -> float:
    """
    AS-curve 속도를 mm/s 단위로 반환
    """
    t_const = total_time - t_acc - t_dec
    if t_const < 0:
        raise ValueError("가속+감속 시간이 전체 시간보다 짧아야 합니다.")
    if t < t_acc:
        v_steps = v_max * (np.sin(np.pi * t / (2 * t_acc)))**2
    elif t < t_acc + t_const:
        v_steps = v_max
    elif t < total_time:
        tau = t - (t_acc + t_const)
        v_steps = v_max * (np.cos(np.pi * tau / (2 * t_dec)))**2
    else:
        v_steps = 0.0
    return (v_steps / (steps_per_rev * microstep)) * pitch_mm  # mm/s

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
def run_motor_scurve(gpio, encoder, motor_id, direction, total_steps, v_max, shape="mid",
                     pitch_mm=5.0, steps_per_rev=200, microstep=16):
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
    enc_init = (prev_enc / 1000) * pitch_mm
    start = time.time()
    data_log = []

    while moved_steps < total_steps:
        t = time.time() - start
        if t > total_time: break

        com_vel_mm = s_curve_velocity(t, v_eff, total_time,
                                      pitch_mm=pitch_mm,
                                      steps_per_rev=steps_per_rev,
                                      microstep=microstep)
        if com_vel_mm < 1e-6:
            time.sleep(0.001)
            continue

        pulse_interval = max(MIN_PULSE_INTERVAL,
                             min(1.0/(com_vel_mm/(pitch_mm/(steps_per_rev*microstep))),
                                 MAX_PULSE_INTERVAL))
        gpio.pulse_step(motor_id, high_time=0.00002, low_time=pulse_interval-0.00002)
        moved_steps += 1

        com_pos = (moved_steps / (steps_per_rev * microstep)) * pitch_mm

        enc_now = encoder.read()
        enc_pos_mm = (enc_now/1000)*pitch_mm - enc_init

        # 속도는 EncoderVelEstimator를 run loop에서 계산
        data_log.append([int(t*1000), com_pos, enc_pos_mm, com_vel_mm, 0.0])

    return data_log

# -------------------- Run Motor with AS-Curve --------------------
def run_motor_ascurve(gpio, encoder, motor_id, direction, total_steps, v_max, shape="mid",
                      pitch_mm=5.0, steps_per_rev=200, microstep=16):
    total_time, t_acc, t_dec, t_const = compute_total_time_ascurve(total_steps, v_max, shape)
    v_eff = vmax_effective(v_max)

    gpio.set_dir(motor_id, direction == 'f')
    gpio.set_enable(motor_id, True)

    moved_steps = 0
    prev_enc = encoder.read()
    enc_init = (prev_enc / 1000) * pitch_mm
    start = time.time()
    data_log = []

    while moved_steps < total_steps:
        t = time.time() - start
        if t > total_time: break

        com_vel_mm = as_curve_velocity(t, v_eff, t_acc, t_dec, total_time,
                                       pitch_mm=pitch_mm,
                                       steps_per_rev=steps_per_rev,
                                       microstep=microstep)
        if com_vel_mm < 1e-6:
            time.sleep(0.001)
            continue

        pulse_interval = max(MIN_PULSE_INTERVAL,
                             min(1.0/(com_vel_mm/(pitch_mm/(steps_per_rev*microstep))),
                                 MAX_PULSE_INTERVAL))
        gpio.pulse_step(motor_id, high_time=0.00002, low_time=pulse_interval-0.00002)
        moved_steps += 1

        com_pos = (moved_steps / (steps_per_rev * microstep)) * pitch_mm

        enc_now = encoder.read()
        enc_pos_mm = (enc_now/1000)*pitch_mm - enc_init

        data_log.append([int(t*1000), com_pos, enc_pos_mm, com_vel_mm, 0.0])

    return data_log
