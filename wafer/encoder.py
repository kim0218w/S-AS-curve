import time
import threading
from collections import deque

try:
    import lgpio  # Raspberry Pi 5 GPIO 라이브러리
except ImportError:
    lgpio = None
#-------------------EncoderMUX--------------------

try:
    import smbus
except Exception:
    smbus = None

MUX_ADDR = 0x70          # PCA9548A 기본 주소
ENCODER_ADDR = 0x36      # AS5600 기본 주소
ENC_REG_ANGLE = 0x0E     # AS5600 각도 레지스터

class EncoderMux:
    def __init__(self, bus_num=1, settle=0.002, retries=3, retry_wait=0.001):
        if smbus is None:
            raise RuntimeError("smbus not available. Enable I2C and install smbus.")
        self.bus = smbus.SMBus(bus_num)
        self.settle = settle
        self.retries = retries
        self.retry_wait = retry_wait

    def select_channel(self, ch: int):
        """
        MUX 채널 선택 (0~7)
        """
        self.bus.write_byte(MUX_ADDR, 1 << ch)
        time.sleep(self.settle)

    def read_angle_deg_once(self) -> float:
        """
        AS5600 각도 레지스터에서 각도 읽기
        """
        data = self.bus.read_i2c_block_data(ENCODER_ADDR, ENC_REG_ANGLE, 2)
        raw  = ((data[0] << 8) | data[1]) & 0x0FFF
        return raw * 360.0 / 4096.0

    def read_angle_deg(self) -> float:
        """
        안정적으로 각도 읽기 (재시도 포함)
        """
        for _ in range(self.retries):
            try:
                return self.read_angle_deg_once()
            except Exception:
                time.sleep(self.retry_wait)
        raise RuntimeError("Encoder read failed after retries")

    def try_read_channel(self, ch: int):
        """
        특정 채널에서 읽기 (에러 발생 시 NaN 반환)
        """
        try:
            self.select_channel(ch)
            return self.read_angle_deg()
        except Exception:
            return float("nan")

    def detect_working_channels(self, candidates=None):
        """
        후보 채널 중 정상 동작하는 채널 자동 탐지
        """
        if candidates is None:
            candidates = list(range(8))
        ok = []
        for ch in candidates:
            v = self.try_read_channel(ch)
            if not (v != v or v == float("inf")):  # NaN, Inf 체크
                ok.append(ch)
        try:
            self.bus.write_byte(MUX_ADDR, 0x00)  # 모든 채널 해제
        except Exception:
            pass
        return ok


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


# pid.py
class PID:
    def __init__(self, kp=2.0, ki=0.2, kd=0.0,
                 out_min=-600.0, out_max=600.0, tau=0.01):
        """
        출력 단위: deg/s
        kp, ki, kd: 게인
        out_min, out_max: 출력 제한
        tau: derivative filter 시간상수
        """
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.tau = tau
        self._i = 0.0
        self._prev_e = 0.0
        self._prev_d = 0.0

    def reset(self):
        self._i = 0.0
        self._prev_e = 0.0
        self._prev_d = 0.0

    def update(self, e, dt, anti_windup_ref=0.0):
        if dt <= 0: dt = 1e-3

        # P
        p = self.kp * e
        # I
        self._i += self.ki * e * dt
        # D (1차 저역통과 필터 포함)
        d_raw = (e - self._prev_e) / dt
        d = self._prev_d + (dt / (self.tau + dt)) * (d_raw - self._prev_d)

        out = p + self._i + self.kd * d

        # 출력 제한 및 anti-windup
        if out > self.out_max:
            out = self.out_max
            if self.ki > 0:
                self._i += 0.1 * (anti_windup_ref - out)
        elif out < self.out_min:
            out = self.out_min
            if self.ki > 0:
                self._i += 0.1 * (anti_windup_ref - out)

        self._prev_e = e
        self._prev_d = d
        return out
