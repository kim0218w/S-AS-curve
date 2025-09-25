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


def run_motor_scurve(gpio, encoder, direction, total_steps, v_max, total_time, csv_filename="scurve_run.csv"):
    """
    gpio: GPIOHelper 인스턴스
    encoder: Encoder 인스턴스
    """
    STEPS_PER_REV = 200
    MICROSTEP = 16
    PITCH_MM = 5.0
    ENC_CPR = 1000

    # 방향 설정
    gpio.write(20, 1 if direction == 'f' else 0)  # DIR_PIN = 20
    gpio.write(16, 0)  # ENA_PIN = 16 (Enable)

    dt = 0.01
    t = 0.0
    moved_steps = 0
    step_accumulator = 0.0
    com_pos = 0.0

    prev_enc = encoder.read()
    enc_init = (prev_enc / ENC_CPR) * PITCH_MM
    data_log = []
    sample_count = 0

    while t <= total_time and moved_steps < total_steps:
        # --- 명령 속도 계산 ---
        com_vel_steps = s_curve_velocity(t, v_max, total_time)
        com_vel_mm = (com_vel_steps / (STEPS_PER_REV * MICROSTEP)) * PITCH_MM
        com_pos += com_vel_mm * dt

        # --- 스텝 펄스 발생 ---
        step_accumulator += com_vel_steps * dt
        while step_accumulator >= 1.0 and moved_steps < total_steps:
            gpio.pulse(21, high_time_s=0.0005, low_time_s=0.0005)  # STEP_PIN = 21
            moved_steps += 1
            step_accumulator -= 1.0

        # --- 엔코더 읽기 ---
        enc_now = encoder.read()
        delta = enc_now - prev_enc
        prev_enc = enc_now
        enc_pos_mm = (enc_now / ENC_CPR) * PITCH_MM - enc_init
        enc_vel_mm = ((delta / dt) / ENC_CPR) * PITCH_MM if sample_count > 2 else 0.0

        # --- 로그 저장 ---
        t_ms = int(round(t * 1000))
        data_log.append([t_ms, com_pos, enc_pos_mm, com_vel_mm, enc_vel_mm])

        sample_count += 1
        t += dt
        time.sleep(dt)

    # === CSV 저장 ===
    os.makedirs("logs", exist_ok=True)
    filepath = os.path.join("logs", csv_filename)
    with open(filepath, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Time_ms","com_Pos_mm","enc_Pos_mm","com_Vel_mm_per_s","enc_Vel_mm_per_s"])
        writer.writerows(data_log)

    print(f"-- 실행 완료! CSV 저장: {filepath} --")

    # === 그래프 출력 ===
    plot_scurve_logs(data_log)
