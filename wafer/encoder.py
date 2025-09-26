import time
import threading

try:
    import lgpio
except ImportError:
    lgpio = None

# -------------------- Pin Assignments --------------------
STEP_PIN_17 = 23
ENA_PIN_17 = 25
DIR_PIN_17 = 24

STEP_PIN_23 = 21
ENA_PIN_23 = 16
DIR_PIN_23 = 20

ENCODER_A_PIN = 3
ENCODER_B_PIN = 2
ENCODER_INVERT = True

IN1_PIN = 5
IN2_PIN = 6
PWM_PIN = 13
# -------------------- Motor Parameters --------------------
MOTOR_STEP_PER_REV = 200    # 보통 200 (1.8°/step 모터)
MICROSTEP_SETTING  = 16     # A4988, TMC 등 드라이버 DIP 스위치 값
GEAR_RATIO         = 1.0    # 감속기 기어비 (없으면 1.0)

STEPS_PER_REV = MOTOR_STEP_PER_REV * MICROSTEP_SETTING * GEAR_RATIO
DEG_PER_STEP  = 360.0 / STEPS_PER_REV

def set_motion_params(*, motor_step_per_rev=None, microstep=None, gear_ratio=None):
    global MOTOR_STEP_PER_REV, MICROSTEP_SETTING, GEAR_RATIO, STEPS_PER_REV, DEG_PER_STEP
    if motor_step_per_rev is not None:
        MOTOR_STEP_PER_REV = int(motor_step_per_rev)
    if microstep is not None:
        MICROSTEP_SETTING = int(microstep)
    if gear_ratio is not None:
        GEAR_RATIO = float(gear_ratio)
    STEPS_PER_REV = MOTOR_STEP_PER_REV * MICROSTEP_SETTING * GEAR_RATIO
    DEG_PER_STEP  = 360.0 / STEPS_PER_REV

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
                print("[GPIO]  GPIO 초기화 실패, 시뮬레이션 모드 실행")

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

# -------------------- Encoder --------------------
class Encoder:
    def __init__(self, gpio: GPIOHelper):
        self.gpio = gpio
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
        a = lgpio.gpio_read(self.gpio.h, ENCODER_A_PIN)
        b = lgpio.gpio_read(self.gpio.h, ENCODER_B_PIN)
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

from collections import deque


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
        vel_raw = ((delta_count / self.cpr) * self.pitch_mm) / dt
        self.buffer.append(vel_raw)
        vel_ma = sum(self.buffer) / len(self.buffer)
        if not self.initialized:
            self.lpf_val = vel_ma
            self.initialized = True
        else:
            self.lpf_val = self.lpf_alpha * vel_ma + (1 - self.lpf_alpha) * self.lpf_val
        return self.lpf_val
# -------------------- Pin Aliases (for scurve.py) --------------------
USE_PIN_SET = 23   # <--- 여기서 17 또는 23으로 변경

if USE_PIN_SET == 17:
    DIR_PIN  = DIR_PIN_17
    STEP_PIN = STEP_PIN_17
    ENA_PIN  = ENA_PIN_17
elif USE_PIN_SET == 23:
    DIR_PIN  = DIR_PIN_23
    STEP_PIN = STEP_PIN_23
    ENA_PIN  = ENA_PIN_23
else:
    raise ValueError("USE_PIN_SET은 17 또는 23만 가능합니다.")
