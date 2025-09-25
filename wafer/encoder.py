import time
import threading
from collections import deque

try:
    import smbus   # I2C 통신 라이브러리
except ImportError:
    smbus = None

# -------------------- AS5600 I2C 설정 --------------------
ENCODER_ADDR   = 0x36   # AS5600 기본 I2C 주소
ENC_REG_ANGLE  = 0x0E   # 각도 레지스터 시작 주소
ENCODER_INVERT = True   # True → 회전 방향 반전

# -------------------- Encoder (AS5600 I2C 전용) --------------------
class Encoder:
    def __init__(self, bus_num=1):
        if smbus is None:
            raise RuntimeError("smbus 모듈이 필요합니다. (sudo apt install python3-smbus)")

        self.bus = smbus.SMBus(bus_num)
        self.addr = ENCODER_ADDR
        self.reg_angle = ENC_REG_ANGLE

        self.position = 0   # 누적 카운트 (스텝 기반)
        self._stop = False
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    def _read_angle_raw(self):
        """ I2C에서 12비트 원시 각도값 읽기 (0~4095) """
        try:
            data = self.bus.read_i2c_block_data(self.addr, self.reg_angle, 2)
            raw = (data[0] << 8) | data[1]
            raw &= 0x0FFF  # 12bit 마스크
            return raw
        except Exception as e:
            print(f"[Encoder] I2C read error: {e}")
            return 0

    def _poll_loop(self):
        prev_raw = self._read_angle_raw()
        while not self._stop:
            raw = self._read_angle_raw()
            delta = raw - prev_raw

            # wrap-around 보정 (4096 카운트 = 360도)
            if delta > 2048:
                delta -= 4096
            elif delta < -2048:
                delta += 4096

            if ENCODER_INVERT:
                delta = -delta

            self.position += delta
            prev_raw = raw
            time.sleep(0.001)  # 1 kHz polling

    def reset(self):
        """엔코더 누적값 초기화"""
        self.position = 0

    def read(self):
        """누적된 엔코더 카운트 반환"""
        return int(self.position)

    def stop(self):
        """스레드 안전 종료"""
        self._stop = True
        if self._thread:
            try:
                self._thread.join(timeout=0.5)
            except Exception:
                pass


# -------------------- 속도 추정기 --------------------
class EncoderVelEstimator:
    def __init__(self, cpr=4096, pitch_mm=1.0, win_size=10, lpf_alpha=0.2):
        """
        cpr: 엔코더 카운트/회전 (AS5600은 4096)
        pitch_mm: 리드(mm) (볼스크류 사용 시)
        win_size: 이동 평균 윈도우
        lpf_alpha: 저역통과 필터 계수
        """
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
