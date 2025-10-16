import time
import math
from typing import Tuple, List, Callable, Optional

# ============================================================
# 변경 요약
# - [FIX] v_max 단위: 모든 v_max는 **steps/s**로 가정. deg/s → steps/s 변환 헬퍼 추가
# - [FIX] Enable 극성/세틀: set_enable(True) 직후 소량 대기
# - [FIX] 최소 펄스폭: DEFAULT_HIGH_TIME_MIN 및 MIN_LOW_TIME 기본 상향(드라이버 권장치 반영)
# - [FIX] 듀티 적용: high_time = max(Hmin, period*duty), low_time = max(Lmin, period-high_time)
# - [FIX] 초기 next_due 계산 및 v=0 구간 처리 강화
# - [FIX] burst 경로: horizon 내 예상 count 계산 시 현재 v_steps 사용, base_period 안전치
# - [FIX] 종료 시 모터 Disable
# ============================================================

# -------------------- 기본 모터 파라미터 --------------------
STEPS_PER_REV = 200
MICROSTEP = 1
DEG_PER_STEP = 360.0 / (STEPS_PER_REV * MICROSTEP)

# LPF 계수 (펄스 기반 속도 평활)
LPF_ALPHA = 0.1

# 최소 안전 펄스폭(하드웨어 데이터시트에 맞춰 조정 권장)
# 많은 드라이버가 3.5~10us의 STEP high 최소 폭을 요구합니다.
DEFAULT_HIGH_TIME_MIN = 8e-6  # [FIX] 기본 8 µs로 상향
MIN_LOW_TIME = 8e-6           # [FIX] 최소 low 시간도 보장

# Enable 세틀 시간(드라이버 깨어날 시간 확보)
ENABLE_SETTLE_S = 0.001       # [FIX] 1ms 대기

# -------------------- 유틸: 단위 변환 --------------------
def deg_per_s_to_steps_per_s(vel_deg_s: float) -> float:
    """deg/s → steps/s 변환"""
    return float(vel_deg_s) / max(DEG_PER_STEP, 1e-12)

# -------------------- Shape 분할 --------------------
def _shape_fractions(shape: str) -> Tuple[float, float, float]:
    s = (shape or "mid").lower()
    if s == "short":
        return 0.5, 0.0, 0.5
    if s == "long":
        return 0.1, 0.8, 0.1
    return 0.25, 0.5, 0.25  # mid

# -------------------- 총 시간 계산 (S-curve) --------------------
def compute_total_time_scurve(total_steps: int, v_max_steps: float, shape: str = "mid") -> float:
    r_acc, r_const, r_dec = _shape_fractions(shape)
    coeff = 0.5 * (r_acc + r_dec) + r_const
    if coeff <= 0 or v_max_steps <= 0:
        raise ValueError("v_max나 shape 파라미터가 유효하지 않습니다.")
    return float(total_steps) / (v_max_steps * coeff)

def compute_segments(total_steps: int, v_max_steps: float, shape: str = "mid") -> Tuple[float, float, float, float]:
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

    coeff = 0.5 * (r_acc + r_dec) + r_const
    if coeff <= 0 or v_max_steps <= 0:
        raise ValueError("비율/속도 설정이 잘못되었습니다.")
    T = total_steps / (v_max_steps * coeff)

    return T, r_acc * T, r_const * T, r_dec * T

# -------------------- 속도 프로파일 (math 사용, 저오버헤드) --------------------
def s_curve_velocity_steps(t: float, v_max_steps: float,
                           t_acc: float, t_const: float, t_dec: float,
                           T_total: float) -> float:
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
    if t < t_acc:
        return v_max * math.sin((math.pi / 2) * (t / (t_acc if t_acc > 0 else 1e-12)))
    elif t < t_acc + t_const:
        return v_max
    elif t < T_total:
        tau = t - (t_acc + t_const)
        return v_max * math.sin((math.pi / 2) * (1 - tau / (t_dec if t_dec > 0 else 1e-12)))
    return 0.0

# -------------------- 고속 실행기 --------------------
def _run_motor_profile_fast(
    gpio,
    motor_id: int,
    direction: str,
    total_steps: int,
    v_max_steps: float,
    T_total: float, t_acc: float, t_const: float, t_dec: float,
    vel_func: Callable[[float, float, float, float, float, float], float],
    *,
    high_time_min: float = DEFAULT_HIGH_TIME_MIN,
    min_low_time: float = MIN_LOW_TIME,     # [FIX] 로우 최소 보장
    duty: float = 0.5,
    log_stride: int = 10,
    burst_horizon_s: float = 0.004,
) -> List[List[float]]:
    """
    - 다음 펄스 만기시각(next_due)을 계산해 그때만 깨운다 → 소프트웨어 지터 최소화
    - gpio.pulse_burst(...)가 있으면 horizon 시간창의 펄스를 일괄 전송해 하드웨어/드라이버에 오프로딩
    - 모든 속도 단위는 steps/s
    """

    # 방향/Enable
    gpio.set_dir(motor_id, direction.lower() == 'f')
    gpio.set_enable(motor_id, True)
    time.sleep(ENABLE_SETTLE_S)  # [FIX] Enable 세틀 타임

    start = time.perf_counter()
    now = start
    t = 0.0

    moved_steps = 0
    last_pulse_t: Optional[float] = None
    pul_based_vel = 0.0

    data_log: List[List[float]] = []
    com_pos_deg = 0.0
    last_loop_abs = start

    # 펄스 타이밍 파라미터
    high_time_min = max(0.0, float(high_time_min))
    min_low_time  = max(0.0, float(min_low_time))
    duty = float(min(0.9, max(0.1, duty)))  # [FIX] 10%~90%

    # burst 기능 감지
    has_burst = hasattr(gpio, "pulse_burst")

    # 초기 주기/만기 시각 설정
    v0 = max(0.0, vel_func(0.0, v_max_steps, t_acc, t_const, t_dec, T_total))
    period0 = (1.0 / v0) if v0 > 0 else 1e9
    next_due = start + period0  # [FIX] v0 기준으로 초기화

    log_counter = 0

    try:
        while t <= T_total and moved_steps < total_steps:
            now = time.perf_counter()
            t = now - start

            # 현재 목표 속도(steps/s)와 주기
            v_steps = vel_func(t, v_max_steps, t_acc, t_const, t_dec, T_total)
            if v_steps <= 0.0:
                # 유의미한 속도까지 짧게 쉼
                time.sleep(0.0005)
                last_loop_dt = now - last_loop_abs
                # v_steps가 0이라도 좌표 적분 식 자체는 유지
                com_pos_deg += (v_steps * DEG_PER_STEP) * last_loop_dt
                last_loop_abs = now

                # [FIX] v=0 구간에서는 next_due를 약간 미래로 밀어 놓음
                next_due = now + 0.001
                continue

            period = 1.0 / v_steps

            # [FIX] 듀티 및 최소 펄스 폭
            high_time = max(high_time_min, period * duty)
            low_time  = max(min_low_time, period - high_time)

            # (보정) high + low가 period보다 커졌다면, high를 다시 줄여 정합
            if high_time + low_time > period:
                # 남는 공간이 없으면 low 우선 보장, high를 깎음
                high_time = max(high_time_min, period - min_low_time)
                low_time  = max(min_low_time, period - high_time)

            if has_burst:
                # [FIX] 현재 v_steps로 horizon 내 펄스 수 추정
                count = int(max(1, min(total_steps - moved_steps, v_steps * burst_horizon_s)))
                # [FIX] base_period는 안전하게 high+low 이상
                base_period = max(period, high_time + low_time)

                gpio.pulse_burst(motor_id,
                                 high_time=high_time,
                                 low_time=low_time,
                                 count=count,
                                 base_period=base_period)
                moved_steps += count

                # 로깅/좌표 적분(버스트 평균속도 기준)
                dt_burst = count * period
                com_pos_deg += (v_steps * DEG_PER_STEP) * dt_burst
                now = time.perf_counter()
                t = now - start

                log_counter += count
                if log_counter >= log_stride:
                    # 펄스 기반 속도 추정(버스트 끝 기준)
                    if last_pulse_t is not None:
                        dt_pul = max(now - last_pulse_t, 1e-12)
                        inst_vel = (count / dt_pul) * DEG_PER_STEP
                        pul_based_vel = LPF_ALPHA * inst_vel + (1 - LPF_ALPHA) * pul_based_vel
                    last_pulse_t = now

                    data_log.append([
                        int(round(t * 1000)),
                        com_pos_deg,
                        moved_steps * DEG_PER_STEP,
                        v_steps * DEG_PER_STEP,
                        pul_based_vel
                    ])
                    log_counter = 0

                # 다음 만기 시각 갱신
                next_due = now + period
                continue

            # --- 단일 펄스 스케줄러 ---
            sleep_time = next_due - now
            if sleep_time > 0.0002:
                time.sleep(sleep_time - 0.0001)  # 마감 직전까지 조금만 재움
                now = time.perf_counter()

            # 바짝 대기
            while now < next_due:
                now = time.perf_counter()

            # 펄스 1회
            gpio.pulse_step(motor_id, high_time=high_time, low_time=low_time)
            moved_steps += 1

            # 다음 만기시각 갱신
            next_due = now + period

            # 연속 좌표 적분
            loop_dt = now - last_loop_abs
            com_pos_deg += (v_steps * DEG_PER_STEP) * loop_dt
            last_loop_abs = now

            # 펄스 기반 속도 추정(저역통과)
            if last_pulse_t is not None:
                dt_pul = max(now - last_pulse_t, 1e-12)
                inst_vel = (1.0 / dt_pul) * DEG_PER_STEP
                pul_based_vel = LPF_ALPHA * inst_vel + (1 - LPF_ALPHA) * pul_based_vel
            last_pulse_t = now

            # 로그 (간격 기반)
            log_counter += 1
            if log_counter >= log_stride:
                data_log.append([
                    int(round(t * 1000)),
                    com_pos_deg,
                    moved_steps * DEG_PER_STEP,
                    v_steps * DEG_PER_STEP,
                    pul_based_vel
                ])
                log_counter = 0

        # 종료 보정
        com_pos_deg = total_steps * DEG_PER_STEP
        t_end = min(T_total, time.perf_counter() - start)
        data_log.append([int(round(t_end * 1000)), com_pos_deg, com_pos_deg, 0.0, 0.0])

        return data_log

    finally:
        # [FIX] 모터 Disable 보장
        try:
            gpio.set_enable(motor_id, False)
        except Exception:
            pass

# -------------------- (S-curve) --------------------
def run_motor_scurve(
    gpio, motor_id, direction, total_steps, v_max_steps, shape="mid",
    *, log_stride: int = 10, high_time_min: float = DEFAULT_HIGH_TIME_MIN
) -> List[List[float]]:
    """
    v_max_steps는 반드시 steps/s 단위로 넣으세요.
    deg/s로 가지고 있다면: v_max_steps = deg_per_s_to_steps_per_s(v_deg_s)
    """
    if v_max_steps <= 0:
        raise ValueError("v_max_steps(steps/s)는 양수여야 합니다.")
    T_total, t_acc, t_const, t_dec = compute_segments(total_steps, v_max_steps, shape)
    return _run_motor_profile_fast(
        gpio, motor_id, direction, total_steps, v_max_steps,
        T_total, t_acc, t_const, t_dec, s_curve_velocity_steps,
        high_time_min=high_time_min, log_stride=log_stride
    )

# -------------------- (AS-curve) --------------------
def run_motor_ascurve(
    gpio, motor_id: int, direction: str,
    total_steps: int, v_max_steps: float,
    shape: str = "mid",
    r_acc: float = None, r_const: float = None, r_dec: float = None,
    *, log_stride: int = 10, high_time_min: float = DEFAULT_HIGH_TIME_MIN
) -> List[List[float]]:
    """
    v_max_steps는 반드시 steps/s 단위.
    """
    if v_max_steps <= 0:
        raise ValueError("v_max_steps(steps/s)는 양수여야 합니다.")
    T_total, t_acc, t_const, t_dec = compute_total_time_ascurve(
        total_steps, v_max_steps, shape, r_acc, r_const, r_dec
    )
    return _run_motor_profile_fast(
        gpio, motor_id, direction, total_steps, v_max_steps,
        T_total, t_acc, t_const, t_dec, as_curve_velocity,
        high_time_min=high_time_min, log_stride=log_stride
    )
