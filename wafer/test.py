# test.py
import sys
import time
import numpy as np
import pandas as pd

from graph import plot_run_results
from scurve import (
    calc_scurve_params,
    s_curve_velocity,
    split_motion_time,
    smooth_cos_delay,
    rate_to_delay,
    delay_to_rate,
    move_stepper_scurve_with_logging,
    move_stepper_scurve_with_pid
)
from encoder import Encoder, PID

try:
    import lgpio
except ImportError:
    lgpio = None


# -------------------- Pin Assignments (BCM) --------------------
DIR_PIN_NAMA_17 = 24
STEP_PIN_NAMA_17 = 23
ENA_PIN_NAMA_17 = 25

DIR_PIN_NAMA_23 = 20
STEP_PIN_NAMA_23 = 21
ENA_PIN_NAMA_23 = 16

ENCODER_A_PIN = 3
ENCODER_B_PIN = 2
ENCODER_INVERT = True

# 모터 스펙: 스텝당 각도(모터축 기준) 예: 0.018°/step
DEG_PER_STEP_MOTOR = 0.018
GEAR_RATIO = 58.0
# 출력축 기준 스텝각: 모터축 스텝각 / 기어비
DEG_PER_STEP_OUT = DEG_PER_STEP_MOTOR / GEAR_RATIO


MOTORS = {
    "M1": {"dir": DIR_PIN_NAMA_17, "step": STEP_PIN_NAMA_17, "ena": ENA_PIN_NAMA_17},
    "M2": {"dir": DIR_PIN_NAMA_23, "step": STEP_PIN_NAMA_23, "ena": ENA_PIN_NAMA_23},
}


# -------------------- GPIO Helper --------------------
class GPIOHelper:
    def __init__(self, extra_output_pins=None):
        self.sim = (lgpio is None)
        self.h = None
        self._claimed = set()
        if not self.sim:
            try:
                self.h = lgpio.gpiochip_open(0)
                if extra_output_pins:
                    for pin in extra_output_pins:
                        self._claim_output(pin)
            except Exception as e:
                self.sim = True
                self.h = None
                print("[GPIO] 초기화 실패 → 시뮬레이션 모드:", e)

    def _claim_output(self, pin, init_val=0):
        if self.sim or pin in self._claimed:
            return
        try:
            # 정식 시그니처: (handle, flags, gpio, value)
            lgpio.gpio_claim_output(self.h, 0, pin, init_val)
            self._claimed.add(pin)
        except Exception as e:
            self.sim = True
            print(f"[GPIO] 출력 claim 실패(pin={pin}): {e} → 시뮬레이션 모드 전환")

    def _claim_input(self, pin):
        if self.sim or pin in self._claimed:
            return
        try:
            # 정식 시그니처: (handle, flags, gpio)
            lgpio.gpio_claim_input(self.h, 0, pin)
            self._claimed.add(pin)
        except Exception as e:
            self.sim = True
            print(f"[GPIO] 입력 claim 실패(pin={pin}): {e} → 시뮬레이션 모드 전환")

    def write(self, pin, val):
        if self.sim: return
        self._claim_output(pin)
        lgpio.gpio_write(self.h, pin, 1 if val else 0)

    def pulse(self, pin, high_time_s, low_time_s):
        if self.sim:
            time.sleep(high_time_s + low_time_s)
            return
        self._claim_output(pin)
        lgpio.gpio_write(self.h, pin, 1)
        time.sleep(high_time_s)
        lgpio.gpio_write(self.h, pin, 0)
        time.sleep(low_time_s)

    def enable_motor(self, ena_pin, enable=True):
        """
        A4988: Enable LOW 활성
        """
        if self.sim:
            return
        self._claim_output(ena_pin, init_val=1)  # 초기 HIGH(Disable)
        lgpio.gpio_write(self.h, ena_pin, 0 if enable else 1)

    def set_dir(self, dir_pin, forward=True):
        if self.sim:
            return
        self._claim_output(dir_pin)
        lgpio.gpio_write(self.h, dir_pin, 1 if forward else 0)

    def queue_pulse(self, step_pin, half_period_s):
        """
        Sleep 기반 한 스텝 (토글)
        """
        if self.sim:
            time.sleep(2 * half_period_s)
            return
        self._claim_output(step_pin)
        lgpio.gpio_write(self.h, step_pin, 1)
        time.sleep(half_period_s)
        lgpio.gpio_write(self.h, step_pin, 0)
        time.sleep(half_period_s)

    def cleanup(self):
        if not self.sim and self.h:
            try:
                lgpio.gpiochip_close(self.h)
            except Exception:
                pass


# -------------------- Initialize --------------------
gpio = GPIOHelper()
encoder = Encoder(gpio=gpio, a_pin=ENCODER_A_PIN, b_pin=ENCODER_B_PIN, invert=ENCODER_INVERT)

# -------------------- Main --------------------
if __name__ == "__main__":
    try:
        print("Commands:")
        print("  M1|M2 f|b [steps] [accel_ratio] ol/cl [pid Kp Ki Kd]")
        print("  q -> quit")

        while True:
            cmd = input(">> ").strip().split()
            if not cmd:
                continue
            if cmd[0].lower() == "q":
                break

            target = cmd[0].upper()
            if target not in MOTORS:
                print("Unknown motor. Use M1 or M2.")
                continue

            forward = (cmd[1].lower() == "f")
            steps = int(cmd[2]) if len(cmd) >= 3 else 2000
            accel_ratio = float(cmd[3]) if len(cmd) >= 4 else 0.2
            mode = cmd[4].lower() if len(cmd) >= 5 else "ol"

            kp, ki, kd = 2.0, 0.2, 0.0
            if len(cmd) >= 6 and cmd[5].lower() == "pid":
                kp, ki, kd = map(float, cmd[6:9])

            pins = MOTORS[target]
            dir_pin, step_pin, ena_pin = pins["dir"], pins["step"], pins["ena"]

            MIN_DELAY = 0.0008   # 너무 빠르게 시작하면 발진/미구동 가능 → 조금 여유
            MAX_DELAY = 0.0030

            if mode == "ol":
                logs = move_stepper_scurve_with_logging(
                    gpio, dir_pin, step_pin, ena_pin,
                    total_steps=steps, forward=forward,
                    min_delay=MIN_DELAY, max_delay=MAX_DELAY,
                    accel_ratio=accel_ratio, sample_dt=0.015,
                    deg_per_step=DEG_PER_STEP_OUT   # 출력축 기준으로 로그 변환
                )
            elif mode == "cl":
                logs = move_stepper_scurve_with_pid(
                    gpio, dir_pin, step_pin, ena_pin,
                    total_steps=steps, forward=forward,
                    min_delay=MIN_DELAY, max_delay=MAX_DELAY,
                    accel_ratio=accel_ratio,
                    deg_per_step=DEG_PER_STEP_OUT,  # ★ PID도 출력축 기준으로
                    encoder=encoder,
                    sample_dt=0.015,
                    pid_gains=(kp, ki, kd)
                )
            else:
                print("Mode must be 'ol' or 'cl'")
                continue

            print(f"[{target}] Done. Logged {len(logs.get('t', []))} samples.")

            # --- 그래프 ---
            try:
                plot_run_results(logs)
            except Exception as e:
                print("[WARN] 그래프 그리기 실패:", e)

            # --- CSV 저장 ---
            try:
                df = pd.DataFrame(logs)
                cols_order = [c for c in [
                    "t", "com_pos_deg", "enc_pos_deg", "com_vel_dps", "enc_vel_dps", "cmd_rate"
                ] if c in df.columns]
                if cols_order:
                    df = df[cols_order]
                log_filename = f"{target}_log.csv"
                df.to_csv(log_filename, index=False)
                print(f"[{target}] 로그 저장 완료 → {log_filename}")
            except Exception as e:
                print("[WARN] 로그 저장 실패:", e)

    except KeyboardInterrupt:
        print("\n[Interrupted]")
    finally:
        try:
            encoder.stop()
        except:
            pass
        try:
            gpio.cleanup()
        except:
            pass
