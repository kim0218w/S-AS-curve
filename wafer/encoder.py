# encoder.py
import time
import threading
from collections import deque

try:
    import lgpio
except ImportError:
    lgpio = None

#--------GPIO Helper--------------------

class GPIOHelper:
    def __init__(self, extra_output_pins=None, max_chips=8):
        self.sim = (lgpio is None)
        self._handles = {}          # chip_idx -> handle
        self._pin2handle = {}       # pin -> handle
        self._claimed = set()
        self.max_chips = max_chips

        if not self.sim:
            # 칩을 미리 하나만 여는 대신, 필요할 때 열도록 둡니다.
            try:
                pass
            except Exception as e:
                self.sim = True
                print("[GPIO] 초기화 실패 → 시뮬레이션 모드:", e)

    def _open_chip(self, chip):
        if chip in self._handles: 
            return self._handles[chip]
        h = lgpio.gpiochip_open(chip)
        self._handles[chip] = h
        return h

    def _claim_output(self, pin, init_val=0):
        if self.sim or pin in self._claimed:
            return
        last_err = None
        for chip in range(self.max_chips):
            try:
                h = self._open_chip(chip)
                # (handle, flags, gpio, value)
                lgpio.gpio_claim_output(h, 0, pin, init_val)
                self._pin2handle[pin] = h
                self._claimed.add(pin)
                return
            except Exception as e:
                last_err = e
                # 다음 칩으로 계속 시도
        # 여기까지 왔다면 어떤 칩에도 해당 pin이 없음
        self.sim = True
        print(f"[GPIO] 출력 claim 실패(pin={pin}): {last_err} → 시뮬레이션 모드 전환")

    def _claim_input(self, pin):
        if self.sim or pin in self._claimed:
            return
        last_err = None
        for chip in range(self.max_chips):
            try:
                h = self._open_chip(chip)
                # (handle, flags, gpio)
                lgpio.gpio_claim_input(h, 0, pin)
                self._pin2handle[pin] = h
                self._claimed.add(pin)
                return
            except Exception as e:
                last_err = e
        self.sim = True
        print(f"[GPIO] 입력 claim 실패(pin={pin}): {last_err} → 시뮬레이션 모드 전환")

    def write(self, pin, val):
        if self.sim: return
        if pin not in self._pin2handle:
            self._claim_output(pin)
        h = self._pin2handle.get(pin, None)
        if h is None: 
            return
        lgpio.gpio_write(h, pin, 1 if val else 0)

    def pulse(self, pin, high_time_s, low_time_s):
        if self.sim:
            time.sleep(high_time_s + low_time_s)
            return
        if pin not in self._pin2handle:
            self._claim_output(pin)
        h = self._pin2handle.get(pin, None)
        if h is None: 
            return
        lgpio.gpio_write(h, pin, 1); time.sleep(high_time_s)
        lgpio.gpio_write(h, pin, 0); time.sleep(low_time_s)

    def enable_motor(self, ena_pin, enable=True):
        if self.sim: return
        self._claim_output(ena_pin, init_val=1)  # 기본 HIGH(Disable)
        h = self._pin2handle.get(ena_pin, None)
        if h is None: 
            return
        lgpio.gpio_write(h, ena_pin, 0 if enable else 1)

    def set_dir(self, dir_pin, forward=True):
        if self.sim: return
        self._claim_output(dir_pin, init_val=0)
        h = self._pin2handle.get(dir_pin, None)
        if h is None: 
            return
        lgpio.gpio_write(h, dir_pin, 1 if forward else 0)

    def queue_pulse(self, step_pin, half_period_s):
        if self.sim:
            time.sleep(2*half_period_s); return
        if step_pin not in self._pin2handle:
            self._claim_output(step_pin, init_val=0)
        h = self._pin2handle.get(step_pin, None)
        if h is None: 
            return
        lgpio.gpio_write(h, step_pin, 1); time.sleep(half_period_s)
        lgpio.gpio_write(h, step_pin, 0); time.sleep(half_period_s)

    def cleanup(self):
        for h in set(self._handles.values()):
            try:
                lgpio.gpiochip_close(h)
            except Exception:
                pass


# -------------------- Quadrature Encoder (A/B) --------------------
class Encoder:
    """
    A/B 쿼드러처 입력을 polling 스레드로 읽어 position 카운트를 누적합니다.
    encoder.read() -> 현재 카운트(int)
    """
    def __init__(self, gpio, a_pin, b_pin, invert=True, poll_dt=0.001):
        self.position = 0
        self.sim = True
        self._stop = False
        self._thread = None
        self.invert = invert
        self.gpio = gpio
        self.a_pin = a_pin
        self.b_pin = b_pin
        self.poll_dt = poll_dt

        try:
            if lgpio is not None and gpio.h is not None:
                # 정식 시그니처: (handle, lFlags, gpio)
                lgpio.gpio_claim_input(gpio.h, 0, self.a_pin)
                lgpio.gpio_claim_input(gpio.h, 0, self.b_pin)
               
                self.sim = False
                self._thread = threading.Thread(target=self._poll_loop, daemon=True)
                self._thread.start()
            else:
                self.sim = True
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
        # (prev<<2 | curr) → delta
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
            if delta:
                self.position += delta
                prev = curr
            time.sleep(self.poll_dt)

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


# -------------------- Optional: Velocity Estimator (count→mm/s) --------------------
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
        vel_raw = ((delta_count / self.cpr) * self.pitch_mm) / max(dt, 1e-6)
        self.buffer.append(vel_raw)
        vel_ma = sum(self.buffer) / len(self.buffer)
        if not self.initialized:
            self.lpf_val = vel_ma
            self.initialized = True
        else:
            self.lpf_val = self.lpf_alpha * vel_ma + (1 - self.lpf_alpha) * self.lpf_val
        return self.lpf_val


# -------------------- PID --------------------
class PID:
    def __init__(self, kp=2.0, ki=0.2, kd=0.0,
                 out_min=-600.0, out_max=600.0, tau=0.01):
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
        dt = max(dt, 1e-6)
        p = self.kp * e
        self._i += self.ki * e * dt
        d_raw = (e - self._prev_e) / dt
        # 1st-order filter on derivative
        d = self._prev_d + (dt / (self.tau + dt)) * (d_raw - self._prev_d)
        out = p + self._i + self.kd * d
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
