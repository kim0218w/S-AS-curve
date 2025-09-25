import sys
import os
import time
import csv
import threading
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from collections import deque

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

# -------------------- Encoder Helper --------------------
class Encoder:
    def __init__(self):
        self.position = 0
        self.sim = True
        self._stop = False
        self._thread = None
        try:
            if lgpio is not None and gpio.h is not None:
                lgpio.gpio_claim_input(gpio.h, ENCODER_A_PIN)
                lgpio.gpio_claim_input(gpio.h, ENCODER_B_PIN)
                self.sim = False
                self._thread = threading.Thread(target=self._poll_loop, daemon=True)
                self._thread.start()
        except Exception:
            self.sim = True

    def _read_ab(self):
        if self.sim:
            return 0, 0
        a = lgpio.gpio_read(gpio.h, ENCODER_A_PIN)
        b = lgpio.gpio_read(gpio.h, ENCODER_B_PIN)
        return a, b

    def _poll_loop(self):
        prev_a, prev_b = self._read_ab()
        prev = (prev_a << 1) | prev_b
        transition_to_delta = {
            0b0001: +1, 0b0011: +1, 0b0110: +1, 0b0100: +1,
            0b0010: -1, 0b0111: -1, 0b1111: -1, 0b1100: -1,
        }
        while not self._stop:
            a, b = self._read_ab()
            curr = (a << 1) | b
            key = ((prev << 2) | curr) & 0b1111
            delta = transition_to_delta.get(key, 0)
            if ENCODER_INVERT:
                delta = -delta
            if delta != 0:
                self.position += delta
                prev = curr
            time.sleep(0.001)  # 1 kHz polling

    def reset(self):
        self.position = 0

    def read(self) -> int:
        return int(self.position)

    def stop(self):
        self._stop = True
        if self._thread:
            try:
                self._thread.join(timeout=0.5)
            except Exception:
                pass

encoder = Encoder()

# -------------------- Velocity Estimator --------------------
class EncoderVelEstimator:
    def __init__(self, cpr, pitch_mm, win_size=10, lpf_alpha=0.2):
        self.cpr = cpr
        self.pitch_mm = pitch_mm
        self.win_size = win_size
        self.lpf_alpha = lpf_alpha

        self.buffer = deque(maxlen=win_size)
        self.lpf_val = 0.0
        self.initialized = False

    def update(self, delta_count, dt):
        # 순간 속도 (mm/s)
        vel_raw = ((delta_count / self.cpr) * self.pitch_mm) / dt

        # Moving Average
        self.buffer.append(vel_raw)
        vel_ma = sum(self.buffer) / len(self.buffer)

        # Low-pass Filter
        if not self.initialized:
            self.lpf_val = vel_ma
            self.initialized = True
        else:
            self.lpf_val = self.lpf_alpha * vel_ma + (1 - self.lpf_alpha) * self.lpf_val

        return self.lpf_val
# --------------------calc_S-curve_params------------------
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def calc_scurve_params(total_steps=None, v_max=None, total_time=None, show=True):
    """
    S-curve 프로파일 파라미터 자동 계산 + 표시 + 그래프 출력
    v(t) = v_max * sin^2(pi * t / T)
    ∫ v(t) dt = (v_max * total_time) / 2 ≈ total_steps
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

    result = {"total_steps": total_steps, "v_max": v_max, "total_time": total_time}

    if show:
        # --- 콘솔 출력 ---
        print("[S-curve Parameters]")
        print(f"총 스텝 수 : {total_steps} steps")
        print(f"최대 속도  : {v_max:.2f} steps/s")
        print(f"총 이동 시간: {total_time:.3f} s\n")

        # --- 표 출력 ---
        df = pd.DataFrame([result])
        print(df, "\n")

        # --- 그래프 표시 ---
        t = np.linspace(0, total_time, 500)
        v = v_max * (np.sin(np.pi * t / total_time))**2

        plt.figure(figsize=(6, 4))
        plt.plot(t, v, label="S-curve velocity")
        plt.xlabel("Time [s]")
        plt.ylabel("Velocity [steps/s]")
        plt.title("S-curve Motion Profile")
        plt.grid()
        plt.legend()
        plt.tight_layout()
        plt.show()

    return result

# -------------------- S-curve Profile --------------------
def s_curve_velocity(t: float, v_max: float,
                     accel_time: float, const_time: float, decel_time: float) -> float:
    """
    sin² 기반의 accel/const/decel 구간 속도 프로파일 (단위: step/s)
    """
    if t < 0:
        return 0.0
    elif t < accel_time:  # 가속 구간
        return v_max * (np.sin(np.pi * t / (2 * accel_time)))**2
    elif t < accel_time + const_time:  # 정속 구간
        return v_max
    elif t < accel_time + const_time + decel_time:  # 감속 구간
        td = t - (accel_time + const_time)
        return v_max * (np.sin(np.pi * td / (2 * decel_time)))**2
    else:
        return 0.0

# -------------------- S-curve Profile --------------------
def run_motor_scurve(direction: str, total_steps: int, v_max: float, total_time: float,
                     accel_ratio: float = 0.2, csv_filename="S_curve_run.csv"):
    """
    accel_ratio를 적용해 accel/const/decel 구간을 sin² 프로파일로 나눈 각도 기반 스텝퍼 제어
    """
    ENC_CPR = 1000   # 엔코더 카운트 per rev (360도)

    # 방향 설정
    sign = 1 if direction == 'f' else -1
    gpio.write(DIR_PIN, 1 if direction == 'f' else 0)
    gpio.write(ENA_PIN, 0)

    # accel_ratio 기반 시간 분할
    accel_time = total_time * accel_ratio
    decel_time = total_time * accel_ratio
    const_time = total_time - accel_time - decel_time
    if const_time < 0:
        accel_time = total_time / 2
        decel_time = total_time / 2
        const_time = 0

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
            "com_Pos_mm",
            "enc_Pos_mm",
            "com_Vel_mm_per_s",
            "enc_Vel_mm_per_s"
        ])
        writer.writerows(data_log)

    print(f"-- 실행 완료! CSV 저장: {filepath} --")



    # === 그래프 ===
    df = pd.DataFrame(data_log, columns=[
        "Time_ms", "com_Pos_mm", "enc_Pos_mm",
        "com_Vel_mm_per_s", "enc_Vel_mm_per_s"
    ])

    plt.figure(figsize=(8, 4))  # 세로 크기도 줄임

    # 속도 그래프
    plt.subplot(2, 1, 1)
    plt.plot(df["Time_ms"], df["com_Vel_mm_per_s"], label="com_Vel_mm_per_s", linestyle="--")
    plt.plot(df["Time_ms"], df["enc_Vel_mm_per_s"], label="enc_Vel_mm_per_s", alpha=0.8)
    plt.ylabel("Velocity [mm/s]")
    plt.legend()
    plt.grid()

    # 위치 그래프
    plt.subplot(2, 1, 2)
    plt.plot(df["Time_ms"], df["com_Pos_mm"], label="com_Pos_mm")
    plt.plot(df["Time_ms"], df["enc_Pos_mm"], label="enc_Pos_mm", alpha=0.8)
    plt.xlabel("Time [ms]")
    plt.ylabel("Position [mm]")
    plt.legend()
    plt.grid()

    plt.tight_layout()
    plt.show()


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
        try: encoder.stop()
        except: pass
        try: gpio.cleanup()
        except: pass

