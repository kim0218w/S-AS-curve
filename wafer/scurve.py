import time
import math
from typing import Tuple, List, Callable, Optional

# -------------------- (참고) Pulse Limits --------------------
MIN_PULSE_INTERVAL = 0.001   # (미사용)
MAX_PULSE_INTERVAL = 0.05    # (미사용)

# -------------------- 기본 모터 파라미터 --------------------
STEPS_PER_REV = 200
MICROSTEP = 1
DEG_PER_STEP = 360.0 / (STEPS_PER_REV * MICROSTEP)

# LPF 계수 (펄스 기반 속도 평활)
LPF_ALPHA = 0.1

# 최소 안전 하이 펄스폭(드라이버 데이터시트 기준으로 조정 권장)
DEFAULT_HIGH_TIME_MIN = 2e-6  # 2 µs

# -------------------- 최대 속도 제한 (제거 버전) --------------------
def vmax_effective(v_max_steps: float,
                   min_pulse_interval: float = MIN_PULSE_INTERVAL) -> float:
    """하드웨어 상한 적용 안 함: 그대로 반환"""
    return float(v_max_steps)

# -------------------- Shape 분할 --------------------
def _shape_fractions(shape: str) -> Tuple[float, float, float]:
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

# -------------------- 총 시간 계산 (AS-curve) --------------------
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

    v_eff = vmax_effective(v_max_steps)
    coeff = 0.5 * (r_acc + r_dec) + r_const
    if coeff <= 0 or v_eff <= 0:
        raise ValueError("비율/속도 설정이 잘못되었습니다.")
    T = total_steps / (v_eff * coeff)

    return T, r_acc * T, r_const * T, r_dec * T

# -------------------- 속도 프로파일 --------------------
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

# -------------------- 실행 (고속·안전, 5ms 고정 로깅) --------------------
def _run_motor_profile(
    gpio,
    motor_id: int, direction: str,
    total_steps: int, v_eff: float,
    T_total: float, t_acc: float, t_const: float, t_dec: float,
    vel_func: Callable[[float, float, float, float, float, float], float],
    *,
    high_time_min: float = DEFAULT_HIGH_TIME_MIN,
    duty: float = 0.5,
    sample_period_s: float = 0.005
) -> List[List[float]]:
    """
    - 사전 대기(next_due) 제거: 주기 제어는 pulse_step(high, low)에만 맡김(이중 대기 방지)
    - 루프 종료 조건: 스텝 완료까지(시간 먼저 끝나도 계속)
    - 5ms 고정 샘플러: CSV 시간열 고정 보장
    """
    gpio.set_dir(motor_id, direction.lower() == 'f')
    gpio.set_enable(motor_id, True)

    start = time.perf_counter()
    last_abs = start

    moved_steps = 0
    last_pulse_t: Optional[float] = None
    pul_based_vel = 0.0
    com_pos_deg = 0.0

    data_log: List[List[float]] = []

    duty = min(0.9, max(0.05, float(duty)))
    high_time_min = max(0.0, float(high_time_min))

    # 샘플러(5ms 그리드)
    next_sample_due = start + sample_period_s

    # 메인 루프: 스텝이 끝날 때까지
    while moved_steps < total_steps:
        now = time.perf_counter()
        t = now - start

        # 현재 목표 속도와 주기
        v_steps = vel_func(min(t, T_total), v_eff, t_acc, t_const, t_dec, T_total)
        if v_steps <= 0.0:
            # 속도가 0인 구간이면 잠깐 쉼 (바운드리에서 바운스 방지)
            time.sleep(0.0005)
            # 연속 좌표 적분(0이므로 변화 없음), 샘플러만 처리
            now = time.perf_counter()
            # 샘플러 처리
            while now >= next_sample_due:
                t_samp = next_sample_due - start
                v_steps_samp = vel_func(min(t_samp, T_total), v_eff, t_acc, t_const, t_dec, T_total)
                data_log.append([
                    int(round(t_samp * 1000)),
                    com_pos_deg,
                    moved_steps * DEG_PER_STEP,
                    v_steps_samp * DEG_PER_STEP,
                    pul_based_vel
                ])
                next_sample_due += sample_period_s
            continue

        period = 1.0 / v_steps
        # 듀티 적용(하이 펄스폭은 최소 요구치 이상)
        high_time = max(high_time_min, period * duty)
        low_time = max(0.0, period - high_time)

        # ---- 펄스 1회 ----
        gpio.pulse_step(motor_id, high_time=high_time, low_time=low_time)
        moved_steps += 1

        # 펄스 동안 흐른 실제 시간(= high_time + low_time + 함수 내부 오버헤드)
        now2 = time.perf_counter()
        dt = now2 - last_abs
        last_abs = now2

        # 연속 지령속도로 위치 적분
        com_pos_deg += (v_steps * DEG_PER_STEP) * dt

        # 펄스 기반 속도 추정
        if last_pulse_t is not None:
            dt_p = max(now2 - last_pulse_t, 1e-12)
            inst_vel = (1.0 / dt_p) * DEG_PER_STEP
            pul_based_vel = LPF_ALPHA * inst_vel + (1 - LPF_ALPHA) * pul_based_vel
        last_pulse_t = now2

        # 5ms 고정 샘플러
        while now2 >= next_sample_due:
            t_samp = next_sample_due - start
            v_steps_samp = vel_func(min(t_samp, T_total), v_eff, t_acc, t_const, t_dec, T_total)
            data_log.append([
                int(round(t_samp * 1000)),
                com_pos_deg,
                moved_steps * DEG_PER_STEP,
                v_steps_samp * DEG_PER_STEP,
                pul_based_vel
            ])
            next_sample_due += sample_period_s

    # 종료 보정
    final_t = time.perf_counter() - start
    com_pos_deg = total_steps * DEG_PER_STEP
    data_log.append([int(round(final_t * 1000)), com_pos_deg, com_pos_deg, 0.0, 0.0])

    return data_log

# -------------------- 실행 (S-curve) --------------------
def run_motor_scurve(gpio, motor_id, direction,
                     total_steps, v_max, shape="mid") -> List[List[float]]:
    T_total, t_acc, t_const, t_dec = compute_segments(total_steps, v_max, shape)
    v_eff = vmax_effective(v_max)
    return _run_motor_profile(
        gpio, motor_id, direction,
        total_steps, v_eff,
        T_total, t_acc, t_const, t_dec,
        s_curve_velocity_steps
    )

# -------------------- 실행 (AS-curve) --------------------
def run_motor_ascurve(gpio, motor_id: int, direction: str,
                      total_steps: int, v_max_steps: float,
                      shape: str = "mid",
                      r_acc: float = None, r_const: float = None, r_dec: float = None
                      ) -> List[List[float]]:
    T_total, t_acc, t_const, t_dec = compute_total_time_ascurve(
        total_steps, v_max_steps, shape, r_acc, r_const, r_dec
    )
    v_eff = vmax_effective(v_max_steps)
    return _run_motor_profile(
        gpio, motor_id, direction,
        total_steps, v_eff,
        T_total, t_acc, t_const, t_dec,
        as_curve_velocity
    )
