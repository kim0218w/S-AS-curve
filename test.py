import sys
import os
import time
import csv
import numpy as np
import pandas as pd
from graph import plot_run_results,unwrap_deg, finite_diff, cumtrapz, plot_overlay
from scurve import calc_scurve_params, s_curve_velocity, split_motion_time, smooth_cos_delay, rate_to_delay, delay_to_rate,move_stepper_scurve_with_logging,move_stepper_scurve_with_pid
from encoder import Encoder, EncoderVelEstimator, PID, EncoderMux

try:
    import lgpio  # Raspberry Pi 5 GPIO 라이브러리
except ImportError:
    lgpio = None

# -------------------- Pin Assignments --------------------
DIR_PIN_NAMA_17 = 24
STEP_PIN_NAMA_17 = 23
ENA_PIN_NAMA_17 = 25
ENCODER_A_PIN = 3
ENCODER_B_PIN = 2
ENCODER_INVERT = True   # 기본값, 실제로는 ENC_SIGN으로 처리
DEG_PER_STEP = 0.018  # 1 step = 0.018°, 10000 steps = 180°

DIR_PIN_NAMA_23 = 20
STEP_PIN_NAMA_23 = 21
ENA_PIN_NAMA_23 = 16

IN1_PIN = 5
IN2_PIN = 6
PWM_PIN = 13

MOTORS = {
    "M1": {
        "dir": DIR_PIN_NAMA_17, "step": STEP_PIN_NAMA_17, "ena": ENA_PIN_NAMA_17},
    "M2": {
        "dir": DIR_PIN_NAMA_23, "step": STEP_PIN_NAMA_23, "ena": ENA_PIN_NAMA_23},
}

# -------------------- GPIO Helper --------------------
c# -------------------- GPIO Helper (업그레이드) --------------------
class GPIOHelper:
    def __init__(self, extra_output_pins=None):
        """
        extra_output_pins: 추가로 출력으로 잡아둘 핀 리스트(선택).
                           예: [DIR_PIN_NEMA17, STEP_PIN_NEMA17, ENA_PIN_NEMA17]
        """
        self.sim = (lgpio is None)
        self.h = None
        self._claimed = set()

        if not self.sim:
            try:
                self.h = lgpio.gpiochip_open(0)

                # 기본 핀(현재 프로젝트의 기본 축)
                for pin in [DIR_PIN, STEP_PIN, ENA_PIN]:
                    self._claim_output(pin)

                # 옵션: 추가 핀들까지 미리 출력으로 claim
                if extra_output_pins:
                    for pin in extra_output_pins:
                        self._claim_output(pin)

                # A4988: Enable LOW
                lgpio.gpio_write(self.h, ENA_PIN, 0)
            except Exception:
                self.sim = True
                self.h = None
                print("[GPIO] GPIO 초기화 실패, 시뮬레이션 모드로 실행됩니다.")

    # --- 내부 유틸 ---
    def _claim_output(self, pin):
        if self.sim or pin in self._claimed:
            return
        try:
            lgpio.gpio_claim_output(self.h, pin)
            self._claimed.add(pin)
        except Exception:
            # Claim 실패 시 시뮬로 폴백
            self.sim = True
            print(f"[GPIO] 핀 {pin} claim 실패, 시뮬레이션 모드로 전환됩니다.")

    @property
    def tx_supported(self) -> bool:
        """
        하드웨어 TX 큐 기능 사용 가능 여부.
        - 실제 보드(lgpio 사용 가능)
        - lgpio에 tx_pulse/tx_room 심볼 존재
        """
        return (not self.sim) and hasattr(lgpio, "tx_pulse") and hasattr(lgpio, "tx_room")

    # --- 기본 API ---
    def write(self, pin, val):
        if self.sim:
            return
        self._claim_output(pin)
        lgpio.gpio_write(self.h, pin, 1 if val else 0)

    def pulse(self, pin, high_time_s, low_time_s):
        """
        Sleep 기반 단일 펄스 (폴백용)
        """
        if self.sim:
            time.sleep(high_time_s + low_time_s)
            return
        self._claim_output(pin)
        lgpio.gpio_write(self.h, pin, 1)
        time.sleep(high_time_s)
        lgpio.gpio_write(self.h, pin, 0)
        time.sleep(low_time_s)

    # --- 업그레이드: 모터 제어 유틸 ---
    def enable_motor(self, ena_pin, enable=True):
        """
        A4988 기준: Enable LOW가 모터 활성화
        """
        if self.sim:
            return
        self._claim_output(ena_pin)
        lgpio.gpio_write(self.h, ena_pin, 0 if enable else 1)

    def set_dir(self, dir_pin, forward=True):
        """
        DIR 핀 방향 설정
        """
        if self.sim:
            return
        self._claim_output(dir_pin        )
        lgpio.gpio_write(self.h, dir_pin, 1 if forward else 0)

    # --- 업그레이드: 큐 기반 스텝 펄스 ---
    def queue_pulse(self, step_pin, half_period_s):
        """
        하드웨어 큐 기반 '한 스텝' 펄스.
        - 하프주기(half-period): high == low == half_period_s
        - tx_pulse 미지원/시뮬이면 자동으로 pulse(sleep)로 폴백
        """
        if not self.tx_supported:
            # 폴백: sleep 기반
            self.pulse(step_pin, half_period_s, half_period_s)
            return

        self._claim_output(step_pin)

        # us 단위 변환(최소 1us 보장)
        on_us = int(max(half_period_s, 1e-6) * 1_000_000)

        # 큐 여유 없으면 조금 대기 (너무 길게 대기하지 않도록 비율)
        try:
            while lgpio.tx_room(self.h, step_pin, lgpio.TX_PWM) <= 0:
                time.sleep(half_period_s * 0.25)
            # (on, off, offset, repeat=1)
            lgpio.tx_pulse(self.h, step_pin, on_us, on_us, 0, 1)
        except Exception:
            # 예외 시 안전 폴백
            self.pulse(step_pin, half_period_s, half_period_s)

    def cleanup(self):
        if not self.sim and self.h:
            try:
                lgpio.gpiochip_close(self.h)
            except Exception:
                pass


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
        print("Commands:")
        print("  M1|M2 f|b [steps] [accel_ratio] ol/cl [pid Kp Ki Kd]")
        print("  q -> quit")

        # I2C 엔코더 준비 (있으면 closed-loop 가능, 없으면 open-loop만 가능)
        try:
            encmux = EncoderMux(bus_num=1)
            working = encmux.detect_working_channels([0,1])
            fb_ch = working[0] if working else None
            def enc_read_deg():
                encmux.select_channel(fb_ch)
                return encmux.read_angle_deg()
        except Exception:
            encmux = None
            enc_read_deg = None
            print("[WARN] I2C encoder not available")

        while True:
            cmd = input(">> ").strip().split()
            if not cmd:
                continue
            if cmd[0].lower() == "q":
                break

            # --- 명령어 파싱 ---
            target = cmd[0].upper()   # M1 / M2
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

            MIN_DELAY = 0.0005
            MAX_DELAY = 0.002

            # --- 실행 ---
            if mode == "ol":
                logs = move_stepper_scurve_with_logging(
                    gpio, dir_pin, step_pin, ena_pin,
                    total_steps=steps, forward=forward,
                    min_delay=MIN_DELAY, max_delay=MAX_DELAY,
                    accel_ratio=accel_ratio, sample_dt=0.015
                )
            elif mode == "cl":
                if enc_read_deg is None:
                    print("[ERR] Closed-loop requires I2C encoder.")
                    continue
                logs = move_stepper_scurve_with_pid(
                    gpio, dir_pin, step_pin, ena_pin,
                    total_steps=steps, forward=forward,
                    min_delay=MIN_DELAY, max_delay=MAX_DELAY,
                    accel_ratio=accel_ratio, deg_per_step=DEG_PER_STEP,
                    enc_read_deg=enc_read_deg, sample_dt=0.015,
                    pid_gains=(kp, ki, kd)
                )
            else:
                print("Mode must be 'ol' or 'cl'")
                continue

            print(f"[{target}] Done. Logged {len(logs['t'])} samples.")

            # --- 그래프 자동 출력 ---
            try:
                t = logs["t"]
                cmd_rate = logs["cmd_rate"]
                cmd_vel = cmd_rate * DEG_PER_STEP
                cmd_angle = cumtrapz(cmd_vel, t)

                enc_angle_raw = logs.get("enc_deg", None)  # 만약 엔코더 각도를 로그에 추가했다면
                if enc_angle_raw is not None:
                    enc_angle = unwrap_deg(enc_angle_raw)
                    enc_vel = finite_diff(enc_angle, t)
                    enc_acc = finite_diff(enc_vel, t)
                else:
                    enc_angle = enc_vel = enc_acc = None

                plot_overlay(t,
                            cmd_angle=cmd_angle, enc_angle=enc_angle,
                            cmd_vel=cmd_vel, enc_vel=enc_vel,
                            enc_acc=enc_acc,
                            title_prefix=f"{target} {mode.upper()}")
            except Exception as e:
                print("[WARN] Could not plot overlay:", e)

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
