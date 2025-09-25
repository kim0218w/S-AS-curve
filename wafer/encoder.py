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

DIR_PIN_23 = 20
STEP_PIN_23 = 21
ENA_PIN_23 = 16

ENCODER_A_PIN = 3
ENCODER_B_PIN = 2
ENCODER_INVERT = True  # 엔코더 방향 설정 (True: 정방향, False: 역방향)

IN1_PIN = 5
IN2_PIN = 6
PWM_PIN = 13


# -------------------- GPIO Helper --------------------
class GPIOHelper:
    def __init__(self):
        self.sim = (lgpio is None)
        self.h = None
        if not self.sim:
            try:
                self.h = lgpio.gpiochip_open(0)
                # 두 모터 핀 전부 출력으로 등록
                for pin in [DIR_PIN_17, STEP_PIN_17, ENA_PIN_17,
                            DIR_PIN_23, STEP_PIN_23, ENA_PIN_23]:
                    lgpio.gpio_claim_output(self.h, pin)
                # 초기값 (모터 Disable 상태)
                for pin in [ENA_PIN_17, ENA_PIN_23]:
                    lgpio.gpio_write(self.h, pin, 1)
                for pin in [DIR_PIN_17, DIR_PIN_23, STEP_PIN_17, STEP_PIN_23]:
                    lgpio.gpio_write(self.h, pin, 0)
            except Exception:
                self.sim = True
                self.h = None
                print("[GPIO] 초기화 실패 → 시뮬레이션 모드")

    def set_enable(self, motor: int, enable: bool):
        """모터 선택 (17/23), enable True=활성"""
        pin = ENA_PIN_17 if motor == 17 else ENA_PIN_23
        val = 0 if enable else 1
        if not self.sim:
            lgpio.gpio_write(self.h, pin, val)

    def set_dir(self, motor: int, forward: bool):
        """모터 방향"""
        pin = DIR_PIN_17 if motor == 17 else DIR_PIN_23
        if not self.sim:
            lgpio.gpio_write(self.h, pin, 1 if forward else 0)

    def write(self, pin, val):
        if not self.sim:
            lgpio.gpio_write(self.h, pin, 1 if val else 0)

    def pulse(self, pin, high_time_s, low_time_s):
        if self.sim or self.h is None:
            time.sleep(high_time_s + low_time_s)
            return
        lgpio.gpio_write(self.h, pin, 1)
        time.sleep(high_time_s)

    def pulse(self, pin, high_time_s, low_time_s):
        if self.sim:
            time.sleep(high_time_s + low_time_s)
            return
        lgpio.gpio_write(self.h, pin, 1)
        time.sleep(high_time_s)
        lgpio.gpio_write(self.h, pin, 0)
        time.sleep(low_time_s)   
    
    def pulse_step(self, motor: int, high_time=0.00002, low_time=0.00002):
        """STEP 펄스 발생"""
        pin = STEP_PIN_17 if motor == 17 else STEP_PIN_23
        if self.sim:
            time.sleep(high_time + low_time)
            return
        lgpio.gpio_write(self.h, pin, 1)
        time.sleep(high_time)
        lgpio.gpio_write(self.h, pin, 0)
        time.sleep(low_time)

    def cleanup(self):
        if not self.sim and self.h:
            try:
                for motor in [17, 23]:
                    self.set_enable(motor, False)  # 종료 시 disable
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


# -------------------- Velocity Estimator --------------------
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
