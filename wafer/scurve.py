import time
import math
from typing import Tuple, List, Callable, Optional

# -------------------- (참고) Pulse Limits --------------------
# 원본과 호환을 위해 상수만 남겨두되, 실행 로직에서는 사용하지 않습니다.
MIN_PULSE_INTERVAL = 0.001   # 1 kHz (미사용)
MAX_PULSE_INTERVAL = 0.05    # 20 Hz (미사용)

# -------------------- 기본 모터 파라미터 --------------------
STEPS_PER_REV = 200
MICROSTEP = 1
DEG_PER_STEP = 360.0 / (STEPS_PER_REV * MICROSTEP)

# LPF 계수 (펄스 기반 속도 평활)
LPF_ALPHA = 0.1

# 최소 안전 하이 펄스폭(드라이버 데이터시트에 맞게 조정 권장)
DEFAULT_HIGH_TIME_MIN = 2e-6  # 2 µs

# -------------------- 최대 속도 제한 (제거 버전) --------------------
def vmax_effective(v_max_steps: float,
                   min_pulse_interval: float = MIN_PULSE_INTERVAL) -> float:
    """이제는 하드웨어 펄스 상한을 적용하지 않고 그대로 반환합니다."""
    return float(v_max_steps)

# -------------------- Shape 분할 --------------------
def _shape_fractions(shape: str) -> Tuple[float, float, float]:
    """(r_acc, r_const, r_dec) 합 = 1"""
    s = (shape or "mid").lower()
    if s == "short":
        return 0.5, 0.0, 0.5
    if s == "long":
        return 0.1, 0.8, 0.1
    return 0.25, 0.5, 0.25  # mid

# -------------------- 총 시간 계산 (S-curve) --------------------
def compute_total_time_scurve(total_steps: int, v_max_steps: float,
                              shape: str = "mid") -> float:
    v_eff = vmax_effective(v_max_steps)
    r_acc, r_const, r_dec = _shape_fractions(shape)
    coeff = 0.5 * (r_acc + r_dec) + r_const
    if coeff <= 0 or v_eff <= 0:
        raise ValueError("v_max나 shape 파라미터가 유효하지 않습니다.")
    return float(total_steps) / (v_eff * coeff)

def compute_segments(total_steps: int, v_max_steps: float,
                     shape: str = "mid") -> Tuple[float, float, float, float]:
    T = compute_total_time_scurve(total_steps, v_max_steps, shape)
    r_acc, r_const, r_dec = _shape_fractions(shape)
    return T, r_acc * T, r_const * T, r_dec * T

# -------------------- 총 시간 계산 (AS-curve, 직접 비율) --------------------
def compute_total_time_ascurve(
    total_steps: int,
    v_max_steps: float,
    shape: str = "mid",
    r_acc: float = None,
    r_const: float = None,
    r_dec: float = None,
) -> Tuple[float, float, float, float]:
    """AS-curve 총 시간 및 구간 계산"""
    if r_acc is None or r_const is None or r_dec is None:
        if shape == "short":
            r_acc, r_dec, r_const = 0.4, 0.6, 0.0
        elif shape == "long":
            r_acc, r_dec, r_const = 0.15, 0.25, 0.6
        else:  # mid
            r_acc, r_dec, r_const = 0.2, 0.4, 0.4

    total = r_acc + r_const + r_dec
    if total <= 0:
        raise ValueError("r_acc + r_const + r_dec > 0 이어야 합니다.")
    r_acc /= total
    r_const /= total
    r_dec /= total

    v_eff = vmax_effective(v_max_steps)
    coeff = 0.5 * (r_acc + r_dec) + r_const
    if coeff <= 0 or v_eff <= 0:
        raise ValueError("비율/속도 설정이 잘못되었습니다.")
    T = total_steps / (v_eff * coeff)

    return T, r_acc * T, r_const * T, r_dec * T

# -------------------- 속도 프로파일 (math 사용, 저오버헤드) --------------------
def s_curve_velocity_steps(t: float, v_max_steps: float,
                           t_acc: float, t_const: float, t_dec: float,
                           T_total: float) -> float:
    """S-curve (sin^2 프로파일) — 단위: steps/s"""
    if T_total <= 0 or v_max_steps <= 0:
        return 0.0
    if t < t_acc:
        x = 0.5 * math.pi * (t / (t_acc if t_acc > 0 else 1e-12))
        s = math.sin(x)
        return v_max_steps * (s * s)
    t1 = t_acc + t_const
    if t < t1:
        return v_max_steps
    if t < T_total:
        tau = t - t1
        x = 0.5 * math.pi * (tau / (t_dec if t_dec > 0 else 1e-12))
        c = math.cos(x)
        return v_max_steps * (c * c)
    return 0.0

def as_curve_velocity(t: float, v_max: float,
                      t_acc: float, t_const: float, t_dec: float,
                      T_total: float) -> float:
    """AS-curve (sine easing accel/decel) — 단위: steps/s"""
    if t < t_acc:
        return v_max * math.sin((math.pi / 2) * (t / (t_acc if t_acc > 0 else 1e-12)))
    elif t < t_acc + t_const:
        return v_max
    elif t < T_total:
        tau = t - (t_acc + t_const)
        return v_max * math.sin((math.pi / 2) * (1 - tau / (t_dec if t_dec > 0 else 1e-12)))
    return 0.0

# -------------------- 실행 (고속·안전, 5ms 고정 로깅 포함) --------------------
def _run_motor_profile(
    gpio,
    motor_id: int, direction: str,
    total_steps: int, v_eff: float,
    T_total: float, t_acc: float, t_const: float, t_dec: float,
    vel_func: Callable[[float, float, float, float, float, float], float],
    *,
    high_time_min: float = DEFAULT_HIGH_TIME_MIN,   # 하이 펄스 최소 폭(안전)
    duty: float = 0.5,                              # 듀티비(0.05~0.9 권장)
    sample_period_s: float = 0.005                  # ★ CSV용 고정 샘플 주기(5ms)
) -> List[List[float]]:
    """
    - 속도 상한(clamp) 없음: v_eff 그대로 사용
    - 적응형 펄스 스케줄러: 다음 펄스 시각(next_due)에 맞춰 sleep/busy-wait
    - 고정 5ms 샘플러 내장: CSV가 항상 0.005s 그리드로 기록됨
    - lgpio 전제: pulse_burst 없이 pulse_step만 사용
    반환: [t_ms, com_pos_deg, pul_pos_deg, com_vel_deg, pul_vel_deg] 리스트
    """
    gpio.set_dir(motor_id, direction.lower() == 'f')
    gpio.set_enable(motor_id, True)

    start = time.perf_counter()
    now = start
    t = 0.0

    # 상태 변수
    moved_steps = 0
    last_pulse_t: Optional[float] = None
    pul_based_vel = 0.0
    com_pos_deg = 0.0
    last_loop_abs = start

    data_log: List[List[float]] = []

    # 듀티/하이폭 안전 보정
    duty = min(0.9, max(0.05, float(duty)))
    high_time_min = max(0.0, float(high_time_min))

    # 첫 속도/주기 및 첫 펄스 예정 시각
    v_steps = max(0.0, vel_func(0.0, v_eff, t_acc, t_const, t_dec, T_total))
    period = (1.0 / v_steps) if v_steps > 0.0 else 1e9
    next_due = start + period

    # ★ 고정 샘플러: CSV용 5ms 그리드
    next_sample_due = start + sample_period_s

    # 메인 루프
    while (t <= T_total) and (moved_steps < total_steps):
        now = time.perf_counter()
        t = now - start

        # 현재 목표 속도와 주기 업데이트
        v_steps = vel_func(t, v_eff, t_acc, t_const, t_dec, T_total)
        if v_steps > 0.0:
            period = 1.0 / v_steps
        else:
            period = 1e9  # 사실상 무한대(대기)

        # 다음 펄스 만기까지 적절히 휴면
        sleep_time = next_due - now
        if sleep_time > 0.0002:
            # 너무 깊이 재우지 않고 소폭 여유를 둠
            time.sleep(sleep_time - 0.0001)
            now = time.perf_counter()

        # 마감 직전 촘촘히 대기
        while now < next_due:
            now = time.perf_counter()
        t = now - start  # 펄스 직전의 실제 시간

        # ---- 펄스 1회 출력 (lgpio: pulse_burst 없음) ----
        high_time = max(high_time_min, period * duty)   # 듀티 반영
        low_time = max(0.0, period - high_time)
        gpio.pulse_step(motor_id, high_time=high_time, low_time=low_time)
        moved_steps += 1

        # 다음 펄스 예정 시각 갱신(현재 주기 기준)
        next_due = now + period

        # 연속 지령속도로 위치 적분(단순 직사각형 적분)
        loop_dt = now - last_loop_abs
        com_pos_deg += (v_steps * DEG_PER_STEP) * loop_dt
        last_loop_abs = now

        # 펄스 기반 속도 추정(저역 통과)
        if last_pulse_t is not None:
