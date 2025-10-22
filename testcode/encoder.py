import time
import threading
from collections import deque

try:
    import lgpio
except ImportError:
    lgpio = None


# ----- IRQ 타임소스 (ns) -----
LAST_IRQ_TIME_NS = None

def _set_irq_time(ts_ns: int):
    """엔코더 엣지 콜백에서 호출해 마지막 IRQ 시각(ns)을 갱신."""
    global LAST_IRQ_TIME_NS
    LAST_IRQ_TIME_NS = int(ts_ns)

def get_irq_time_s_or_none():
    """마지막 IRQ 시각(초)을 반환. 없으면 None."""
    if LAST_IRQ_TIME_NS is None:
        return None
    return LAST_IRQ_TIME_NS * 1e-9

# -------------------- Pin Assignments --------------------
STEP_PIN_17 = 23
ENA_PIN_17  = 25
DIR_PIN_17  = 24

STEP_PIN_23 = 21
ENA_PIN_23  = 16
DIR_PIN_23  = 20

ENCODER_A_PIN = 3
ENCODER_B_PIN = 2
ENCODER_INVERT = True

IN1_PIN = 5
IN2_PIN = 6
PWM_PIN = 13

# -------------------- Motor Parameters --------------------
MOTOR_STEP_PER_REV = 200    # 보통 200 (1.8°/step 모터)
MICROSTEP_SETTING  = 8     # A4988, TMC 등 드라이버 DIP 스위치 값
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


# -------------------- Pin Aliases (for scurve.py) --------------------
USE_PIN_SET = 23   

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


# -------------------- GPIO Helper --------------------
class GPIOHelper:
    def __init__(self):
        self.h = None
        try:
            self.h = lgpio.gpiochip_open(0)
            for pin in [DIR_PIN, STEP_PIN, ENA_PIN]:
                lgpio.gpio_claim_output(self.h, pin)
            lgpio.gpio_write(self.h, ENA_PIN, 1)  # 모터 비활성(Active-LOW 가정)
            print(f"[GPIO] 초기화 성공. STEP_PIN:{STEP_PIN}, DIR_PIN:{DIR_PIN}, ENA_PIN:{ENA_PIN}")
            # ---- 타임소스: IRQ가 있으면 그 시각, 아니면 perf_counter ----
            def now(self) -> float:
                ts = get_irq_time_s_or_none()
                return ts if ts is not None else time.perf_counter()
        except Exception as ex:
            print(f"[GPIO] 초기화 실패, 프로그램이 종료될 수 있습니다. ({ex})")
            raise

    # ---------- scurve.py용 ----------
    def set_dir(self, motor_id: int, forward: bool):
        
        lgpio.gpio_write(self.h, DIR_PIN, 1 if forward else 0)

    def set_enable(self, motor_id: int, enable: bool):
        """
        A4988 기준: Enable Active LOW
        - enable=True  -> ENA 핀 LOW (0)
        - enable=False -> ENA 핀 HIGH (1)
        """
        lgpio.gpio_write(self.h, ENA_PIN, 0 if enable else 1)

    def pulse_step(self, motor_id: int, high_time: float, low_time: float):
        lgpio.gpio_write(self.h, STEP_PIN, 1)
        time.sleep(high_time)
        lgpio.gpio_write(self.h, STEP_PIN, 0)
        time.sleep(low_time)
    def pulse_burst(self, motor_id: int, *, high_time: float, low_time: float, count: int, base_period: float):
        """
        scurve.py 호출 시그니처에 맞춘 버전.
        - base_period는 안전 주기(>= high_time+low_time). 여기선 high/low에서 직접 계산.
        - 하드웨어 버스트 API 없는 경우, 안전 루프 폴백.
        """
        if count <= 0:
            return

        period = max(base_period, high_time + low_time)
        duty = max(0.1, min(0.9, high_time / period))
        try:
            # lgpio에 tx_pwm(버스트)가 있다면 사용
            period_us = int(period * 1_000_000)
            duty_lgpio = int(duty * 1_000_000)
            lgpio.tx_pwm(self.h, STEP_PIN, period_us, duty_lgpio, count, 0, 0, 0)
        except Exception:
            # 폴백: 소프트웨어로 count회 펄스
            for _ in range(count):
                lgpio.gpio_write(self.h, STEP_PIN, 1)
                time.sleep(high_time)
                lgpio.gpio_write(self.h, STEP_PIN, 0)
                time.sleep(low_time)

    # ---------- 하위 호환 ----------
    def write(self, pin, val):
        
        lgpio.gpio_write(self.h, pin, 1 if val else 0)

    def pulse(self, pin, high_time_s, low_time_s):
        
        lgpio.gpio_write(self.h, pin, 1)
        time.sleep(high_time_s)
        lgpio.gpio_write(self.h, pin, 0)
        time.sleep(low_time_s)

    def cleanup(self):
        if self.h: # self.sim 검사 로직 삭제, self.h가 유효한지 여부만 검사
            try:
                lgpio.gpio_write(self.h, ENA_PIN, 1) # 모터 비활성화
                lgpio.tx_pwm(self.h, STEP_PIN, 0, 0, 0, 0, 0, 0) # STEP 핀 PWM 중지
                lgpio.gpiochip_close(self.h)
                print("[GPIO] GPIO 정리 완료.")
            except Exception as e:
                print(f"[GPIO] GPIO 정리 중 오류 발생: {e}")


# -------------------- Encoder --------------------
class Encoder:
    def __init__(self, gpio: GPIOHelper):
        self.gpio = gpio
        self.position = 0
        # self.sim = True 로직 삭제
        self._stop = False
        self._thread = None
        try:
            # if lgpio is not None and gpio.h is not None: 로직 삭제
            lgpio.gpio_claim_input(gpio.h, ENCODER_A_PIN) # lgpio가 없거나 gpio.h가 None이면 AttributeError 발생
            lgpio.gpio_claim_input(gpio.h, ENCODER_B_PIN)
            # self.sim = False 로직 삭제
            self._thread = threading.Thread(target=self._poll_loop, daemon=True)
            self._thread.start()
        except Exception:
            # 초기화 실패 시 더 이상 self.sim = True; 로 전환되지 않고,
            # 예외가 main.py로 전달될 수 있습니다.
            print(f"[Encoder] Encoder 초기화 실패. 프로그램이 종료될 수 있습니다.")
            raise # 예외를 다시 발생시켜 프로그램 종료

    def _read_ab(self):
        # if self.sim: return 0, 0 로직 삭제
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
