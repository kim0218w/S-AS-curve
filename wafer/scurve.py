import os, time, csv
import numpy as np
import pandas as pd
from graph import plot_scurve_params, plot_scurve_logs

# -------------------- S-curve --------------------
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
        plot_scurve_params(result)
    return result


def s_curve_velocity(t, v_max, total_time):
    if total_time <= 0:
        return 0.0
    return v_max * (np.sin(np.pi * t / total_time))**2


def run_motor_scurve(gpio, encoder, direction, total_steps, v_max, total_time,
                     csv_filename="scurve_run.csv"):
    """
    gpio     : GPIOHelper 객체
    encoder  : Encoder(AS5600 I2C) 객체
    direction: 'f' 또는 'b'
    total_steps: 총 스텝 수
    v_max    : 최대 속도 (step/s)
    total_time: 총 이동 시간 (s)
    """
    STEPS_PER_REV = 200
    MICROSTEP = 16
    PITCH_MM = 5.0
    ENC_CPR = 4096   # AS5600 = 12bit = 4096 counts/rev

    # 방향 설정
    gpio.write(20, 1 if direction == 'f' else 0)  # DIR_PIN
    gpio.write(16, 0)  # ENA_PIN (Enable)

    moved_steps = 0
    data_log = []

    start_time = time.time()

    prev_enc = encoder.read()
    enc_init = (prev_enc / ENC_CPR) * PITCH_MM

    while moved_steps < total_steps:
        # 현재 경과 시간
        t = time.time() - start_time
        if t > total_time:
            break

        # 현재 목표 속도 (steps/s)
        com_vel_steps = s_curve_velocity(t, v_max, total_time)
        if com_vel_steps < 1:   # 속도가 1 step/s 이하이면 skip
            continue

        # 현재 스텝 주기 (s)
        step_delay = 1.0 / com_vel_steps

        # --- 펄스 발생 ---
        gpio.pulse(21, high_time_s=0.0003, low_time_s=0.0003)  # STEP_PIN
        moved_steps += 1

        # --- 엔코더 ---
        enc_now = encoder.read()
        delta = enc_now - prev_enc
        prev_enc = enc_now

        enc_pos_mm = (enc_now / ENC_CPR) * PITCH_MM - enc_init
        enc_vel_mm = ((delta / step_delay) / ENC_CPR) * PITCH_MM if step_delay > 0 else 0.0

        com_pos_mm = (moved_steps / (STEPS_PER_REV * MICROSTEP)) * PITCH_MM
        com_vel_mm = (com_vel_steps / (STEPS_PER_REV * MICROSTEP)) * PITCH_MM

        t_ms = int(round(t * 1000))
        data_log.append([t_ms, com_pos_mm, enc_pos_mm, com_vel_mm, enc_vel_mm])

        # --- 스텝 주기만큼 sleep ---
        time.sleep(step_delay)

    # CSV 저장 & 그래프 출력
    os.makedirs("logs", exist_ok=True)
    filepath = os.path.join("logs", csv_filename)
    with open(filepath, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "Time_ms", "com_Pos_mm", "enc_Pos_mm",
            "com_Vel_mm_per_s", "enc_Vel_mm_per_s"
        ])
        writer.writerows(data_log)

    print(f"-- 실행 완료! CSV 저장: {filepath} --")

    plot_scurve_logs(data_log)
