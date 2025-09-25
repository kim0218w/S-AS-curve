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
            self.h = lgpio.gpiochip_open(0)
            for pin in [DIR_PIN_17, STEP_PIN_17, ENA_PIN_17,
                        DIR_PIN_23, STEP_PIN_23, ENA_PIN_23]:
                lgpio.gpio_claim_output(self.h, pin)

    def set_enable(self, motor: int, enable: bool):
        pin = ENA_PIN_17 if motor == 17 else ENA_PIN_23
        val = 0 if enable else 1
        lgpio.gpio_write(self.h, pin, val)

    def set_dir(self, motor: int, forward: bool):
        pin = DIR_PIN_17 if motor == 17 else DIR_PIN_23
        lgpio.gpio_write(self.h, pin, 1 if forward else 0)

    def pulse_step(self, motor: int, high_time=0.00002, low_time=0.00002):
        pin = STEP_PIN_17 if motor == 17 else STEP_PIN_23
        lgpio.gpio_write(self.h, pin, 1)
        time.sleep(high_time)
        lgpio.gpio_write(self.h, pin, 0)
        time.sleep(low_time)

    def cleanup(self):
        if self.h:
            for motor in [17, 23]:
                self.set_enable(motor, False)
            lgpio.gpiochip_close(self.h)

# -------------------- Encoder --------------------
class Encoder:
    def __init__(self, gpio: GPIOHelper):
        self.gpio = gpio
        self.position = 0
        self._stop = False
        self._thread = None
        if lgpio and gpio.h:
            lgpio.gpio_claim_input(gpio.h, ENCODER_A_PIN)
            lgpio.gpio_claim_input(gpio.h, ENCODER_B_PIN)
            self._thread = threading.Thread(target=self._poll_loop, daemon=True)
            self._thread.start()

    def _read_ab(self):
        a = lgpio.gpio_read(self.gpio.h, ENCODER_A_PIN)
        b = lgpio.gpio_read(self.gpio.h, ENCODER_B_PIN)
        return a, b

    def _poll_loop(self):
        prev_a, prev_b = self._read_ab()
        prev = (prev_a << 1) | prev_b
        trans = {0b0001:+1, 0b0011:+1, 0b0110:+1, 0b0100:+1,
                 0b0010:-1, 0b0111:-1, 0b1111:-1, 0b1100:-1}
        while not self._stop:
            a, b = self._read_ab()
            curr = (a << 1) | b
            key = ((prev << 2) | curr) & 0b1111
            delta = trans.get(key, 0)
            if ENCODER_INVERT:
                delta = -delta
            self.position += delta
            prev = curr
            time.sleep(0.001)

    def reset(self): self.position = 0
    def read(self) -> int: return int(self.position)
    def stop(self):
        self._stop = True
        if self._thread: self._thread.join(timeout=0.5)


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
