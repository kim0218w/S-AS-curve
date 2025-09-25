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

# -------------------- Profile --------------------
def s_curve_velocity(t: float, v_max: float, total_time: float) -> float:
    if total_time <= 0:
        return 0.0
    return v_max * (np.sin(np.pi * t / total_time))**2


# -------------------- Run Motor (Pulse Interval Based) --------------------
def run_motor_scurve(gpio, encoder, motor_id: int, direction: str,
                     total_steps: int, v_max: float, total_time: float):
    """
    motor_id: 17 or 23 (라즈베리 파이 핀 그룹 기준)
    direction: 'f' (forward), 'b' (backward)
    """
    STEPS_PER_REV = 200
    MICROSTEP = 16
    PITCH_MM = 5.0
    ENC_CPR = 1000

    # 방향 및 Enable 설정
    gpio.set_dir(motor_id, direction == 'f')
    gpio.set_enable(motor_id, True)

    moved_steps = 0
    com_pos = 0.0

    prev_enc = encoder.read()
    enc_init = (prev_enc / ENC_CPR) * PITCH_MM
    data_log = []

    start_time = time.time()

    while moved_steps < total_steps:
        t = time.time() - start_time
        if t > total_time:
            break

        # 현재 목표 속도 [steps/s]
        com_vel_steps = s_curve_velocity(t, v_max, total_time)
        if com_vel_steps < 1e-6:
            time.sleep(0.001)
            continue

        # 속도 → 펄스 간격
        pulse_interval = 1.0 / com_vel_steps

        # 모터에 펄스 출력
        gpio.pulse_step(motor_id, high_time=0.00002, low_time=pulse_interval - 0.00002)
        moved_steps += 1

        # 위치 계산 (mm)
        com_pos = (moved_steps / (STEPS_PER_REV * MICROSTEP)) * PITCH_MM

        # 엔코더 읽기
        enc_now = encoder.read()
        delta = enc_now - prev_enc
        prev_enc = enc_now
        enc_pos_mm = (enc_now / ENC_CPR) * PITCH_MM - enc_init
        enc_vel_mm = ((delta / pulse_interval) / ENC_CPR) * PITCH_MM if delta != 0 else 0.0

        # 로그 저장
        t_ms = int(round(t * 1000))
        com_vel_mm = (com_vel_steps / (STEPS_PER_REV * MICROSTEP)) * PITCH_MM
        data_log.append([t_ms, com_pos, enc_pos_mm, com_vel_mm, enc_vel_mm])

    return data_log

