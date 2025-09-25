import time
import threading
from collections import deque

try:
    import lgpio  # Raspberry Pi 5 GPIO 라이브러리
except ImportError:
    lgpio = None

# -------------------- Encoder --------------------
class Encoder:
    def __init__(self, gpio, a_pin, b_pin, invert=True):
        self.position = 0
        self.sim = True
        self._stop = False
        self._thread = None
        self.invert = invert
        self.gpio = gpio
        self.a_pin = a_pin
        self.b_pin = b_pin

        try:
            if lgpio is not None and gpio.h is not None:
                lgpio.gpio_claim_input(gpio.h, self.a_pin)
                lgpio.gpio_claim_input(gpio.h, self.b_pin)
                self.sim = False
                self._thread = threading.Thread(target=self._poll_loop, daemon=True)
                self._thread.start()
        except Exception:
            self.sim = True

    def _read_ab(self):
        if self.sim:
            return 0, 0
        a = lgpio.gpio_read(self.gpio.h, self.a_pin)
        b = lgpio.gpio_read(self.gpio.h, self.b_pin)
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
            if self.invert:
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
