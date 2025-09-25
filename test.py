import sys
import os
import time
import csv
import numpy as np
import pandas as pd
from graph import plot_run_results
from scurve import calc_scurve_params, s_curve_velocity, split_motion_time
from encoder import Encoder, EncoderVelEstimator

try:
    import lgpio  # Raspberry Pi 5 GPIO 라이브러리
except ImportError:
    lgpio = None

# -------------------- Pin Assignments --------------------
DIR_PIN = 20
STEP_PIN = 21
ENA_PIN = 16
ENCODER_A_PIN = 3
ENCODER_B_PIN = 2
ENCODER_INVERT = True   # 기본값, 실제로는 ENC_SIGN으로 처리
DEG_PER_STEP = 0.018  # 1 step = 0.018°, 10000 steps = 180°

# -------------------- GPIO Helper --------------------
class GPIOHelper:
    def __init__(self):
        self.sim = (lgpio is None)
        self.h = None
        if not self.sim:
            try:
                self.h = lgpio.gpiochip_open(0)
                for pin in [DIR_PIN, STEP_PIN, ENA_PIN]:
                    lgpio.gpio_claim_output(self.h, pin)
                lgpio.gpio_write(self.h, ENA_PIN, 0)  # Enable = LOW (A4988)
            except Exception:
                self.sim = True
                self.h = None
                print("[GPIO] GPIO 초기화 실패, 시뮬레이션 모드로 실행됩니다.")

    def write(self, pin, val):
        if not self.sim:
            lgpio.gpio_write(self.h, pin, 1 if val else 0)

    def pulse(self, pin, high_time_s, low_time_s):
        if self.sim:
            time.sleep(high_time_s + low_time_s)
            return
        lgpio.gpio_write(self.h, pin, 1)
        time.sleep(high_time_s)
        lgpio.gpio_write(self.h, pin, 0)
        time.sleep(low_time_s)

    def cleanup(self):
        if not self.sim and self.h:
            try:
                lgpio.gpiochip_close(self.h)
            except Exception:
                pass

gpio = GPIOHelper()

# -------------------- Encoder Instance --------------------
encoder = Encoder(gpio=gpio, a_pin=ENCODER_A_PIN, b_pin=ENCODER_B_PIN, invert=ENCODER_INVERT)

# -------------------- Run Motor with S-curve --------------------
def run_motor_scurve(direction: str, total_steps: int, v_max: float, total_time: float,
                     accel_ratio: float = 0.2, csv_filename="S_curve_run.csv"):
    """
    accel_ratio를 적용해 accel/const/decel 구간을 sin² 프로파일로 나눈 각도 기반 스텝퍼 제어
    """
    ENC_CPR = 1000   # 엔코더 카운트 per rev (360도)

    # 방향 설정
    gpio.write(DIR_PIN, 1 if direction == 'f' else 0)
    gpio.write(ENA_PIN, 0)

    # accel_ratio 기반 시간 분할 (모듈화 완료)
    accel_time, const_time, decel_time = split_motion_time(total_time, accel_ratio)

    # 루프 주기 (15 ms)
    dt = 0.015
    t = 0.0
    moved_steps = 0
    step_accumulator = 0.0
    com_pos = 0.0   # 명령 위치 (deg)

    prev_enc = encoder.read()
    # 엔코더 초기 오프셋 (deg 단위)
    enc_init = (prev_enc / ENC_CPR) * 360.0
    data_log = []

    warmup_ms = 20
    warmup_samples = int(warmup_ms / (dt * 1000))
    sample_count = 0

    while t <= total_time and moved_steps < total_steps:
        # --- 명령 속도 (step/s → deg/s) ---
        com_vel_steps = s_curve_velocity(t, v_max, accel_time, const_time, decel_time)
        com_vel_deg = com_vel_steps * DEG_PER_STEP

        # --- 명령 위치 (적분 누적) ---
        com_pos += com_vel_deg * dt

        # --- 스텝 펄스 발생 ---
        step_accumulator += com_vel_steps * dt
        while step_accumulator >= 1.0 and moved_steps < total_steps:
            gpio.pulse(STEP_PIN, high_time_s=0.0005, low_time_s=0.0005)
            moved_steps += 1
            step_accumulator -= 1.0

        # --- 엔코더 읽기 ---
        enc_now = encoder.read()
        delta = enc_now - prev_enc
        prev_enc = enc_now

        # 엔코더 위치 (deg)
        enc_pos_deg = (enc_now / ENC_CPR) * 360.0 - enc_init

        # 엔코더 속도 (deg/s)
        if sample_count < warmup_samples:
            enc_vel_deg = 0.0
        else:
            enc_vel_deg = ((delta / dt) / ENC_CPR) * 360.0

        # --- 로그 기록 ---
        t_ms = int(round(t * 1000))
        data_log.append([t_ms, com_pos, enc_pos_deg, com_vel_deg, enc_vel_deg])

        # --- 시간 갱신 ---
        sample_count += 1
        t += dt
        time.sleep(dt)

    # === CSV 저장 ===
    os.makedirs("logs", exist_ok=True)
    filepath = os.path.join("logs", csv_filename)
    with open(filepath, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "Time_ms",
            "com_Pos_deg",
            "enc_Pos_deg",
            "com_Vel_deg_per_s",
            "enc_Vel_deg_per_s"
        ])
        writer.writerows(data_log)

    print(f"-- 실행 완료! CSV 저장: {filepath} --")

    # === 그래프 ===
    plot_run_results(data_log)


# -------------------- Main --------------------
if __name__ == "__main__":
    try:
        mode = input("실행 모드 선택 (1: 모터 실행, 2: 파라미터 계산): ").strip()

        if mode == "1":
            v_max = float(input("Vmax 입력 [100~5000]: ").strip())
            move_steps = int(input("이동할 스텝 수 입력 [100~10000]: ").strip())
            direction = input("모터 방향 입력 (f/b): ").strip().lower()
            total_time = float(input("총 이동 시간 입력 [초, 0.5~30]: ").strip())

            run_motor_scurve(direction, move_steps, v_max, total_time)

        elif mode == "2":
            # 파라미터 계산 모드
            steps_in = input("총 스텝 수 입력 (또는 Enter): ").strip()
            vmax_in = input("최대 속도 입력 (steps/s, 또는 Enter): ").strip()
            t_in = input("총 이동 시간 입력 (초, 또는 Enter): ").strip()

            total_steps = int(steps_in) if steps_in else None
            v_max = float(vmax_in) if vmax_in else None
            total_time = float(t_in) if t_in else None

            calc_scurve_params(total_steps=total_steps, v_max=v_max, total_time=total_time)

        else:
            print("잘못된 모드 선택")

    except Exception as e:
        print(f"[ERROR] {e}")
    finally:
        try:
            encoder.stop()
        except:
            pass
        try:
            gpio.cleanup()
        except:
            pass
