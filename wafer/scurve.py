import time
import numpy as np
from typing import Tuple, List

# -------------------- Pulse Limits --------------------
MIN_PULSE_INTERVAL = 0.001   # 1 kHz
MAX_PULSE_INTERVAL = 0.05    # 20 Hz

# -------------------- 기본 모터 파라미터 --------------------
STEPS_PER_REV = 200
MICROSTEP = 1
DEG_PER_STEP = 360.0 / (STEPS_PER_REV * MICROSTEP)

# LPF 계수
LPF_ALPHA = 0.1


# -------------------- 최대 속도 제한 --------------------
def vmax_effective(v_max_steps: float,
                   min_pulse_interval: float = MIN_PULSE_INTERVAL) -> float:
    """하드웨어 펄스 한계 고려한 최대 속도 (steps/s)"""
    hw_limit = 1.0 / min_pulse_interval
    return min(float(v_max_steps), hw_limit)


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


# -------------------- 속도 프로파일 --------------------
def s_curve_velocity_steps(t: float, v_max_steps: float,
                           t_acc: float, t_const: float, t_dec: float,
                           T_total: float) -> float:
    """S-curve (sin^2 프로파일)"""
    if T_total <= 0 or v_max_steps <= 0:
        return 0.0
    if t < t_acc:
        return v_max_steps * (np.sin(0.5 * np.pi * (t / max(t_acc, 1e-9)))) ** 2
    t1 = t_acc + t_const
    if t < t1:
        return v_max_steps
    if t < T_total:
        tau = t - t1
        return v_max_steps * (np.cos(0.5 * np.pi * (tau / max(t_dec, 1e-9)))) ** 2
    return 0.0


def as_curve_velocity(t: float, v_max: float,
                      t_acc: float, t_const: float, t_dec: float,
                      T_total: float) -> float:
    """AS-curve (sine easing accel/decel)"""
    if t < t_acc:
        return v_max * np.sin((np.pi / 2) * (t / max(t_acc, 1e-9)))
    elif t < t_acc + t_const:
        return v_max
    elif t < T_total:
        tau = t - (t_acc + t_const)
        return v_max * np.sin((np.pi / 2) * (1 - tau / max(t_dec, 1e-9)))
    return 0.0


# -------------------- 실행 (공통 루틴) --------------------

# ...existing code...
def _run_motor_profile(gpio, motor_id: int, direction: str,
                       total_steps: int, v_eff: float,
                       T_total: float, t_acc: float, t_const: float, t_dec: float,
                       vel_func) -> List[List[float]]:
    """S/AS-curve 실행 (시간 기반, 5ms 단위 샘플링, 일정 간격)"""
    gpio.set_dir(motor_id, direction.lower() == 'f')
    gpio.set_enable(motor_id, True)

    moved_steps = 0
    data_log: List[List[float]] = []
    last_pulse_t = None
    pul_based_vel = 0.0
    start_t = time.perf_counter()

    com_pos_deg = 0.0
    last_t = start_t

    dt_sample = 0.005
    n = 0
    while True:
        t = n * dt_sample
        if t > T_total:
            break

        v_steps = vel_func(t, v_eff, t_acc, t_const, t_dec, T_total)
        com_vel_deg = v_steps * DEG_PER_STEP

        now_abs = time.perf_counter()
        dt = now_abs - last_t
        com_pos_deg += com_vel_deg * dt
        last_t = now_abs

        # 펄스 발생 (동일)
        if v_steps > 1e-6:
            pulse_interval = 1.0 / v_steps
            pulse_interval = max(MIN_PULSE_INTERVAL, min(pulse_interval, MAX_PULSE_INTERVAL))
            high_time = min(2e-5, 0.5 * pulse_interval)
            low_time = max(0.0, pulse_interval - high_time)

            gpio.pulse_step(motor_id, high_time=high_time, low_time=low_time)
            moved_steps += 1

            if last_pulse_t is not None:
                dt_pul = max(now_abs - last_pulse_t, 1e-9)
                inst_vel = (1.0 / dt_pul) * DEG_PER_STEP
                pul_based_vel = LPF_ALPHA * inst_vel + (1 - LPF_ALPHA) * pul_based_vel
            last_pulse_t = now_abs

        pul_pos_deg = moved_steps * DEG_PER_STEP
        pul_vel_deg = pul_based_vel
        t_ms = int(round(t * 1000))
        data_log.append([t_ms, com_pos_deg, pul_pos_deg, com_vel_deg, pul_vel_deg])

        n += 1
        time.sleep(dt_sample)

    # 마지막 보정 (동일)
    com_pos_deg = total_steps * DEG_PER_STEP
    pul_pos_deg = com_pos_deg
    t_ms = int(round(T_total * 1000))
    data_log.append([t_ms, com_pos_deg, pul_pos_deg, 0.0, 0.0])

    return data_log

# -------------------- 실행 (S-curve) --------------------
def run_motor_scurve(gpio, motor_id, direction,
                     total_steps, v_max, shape="mid") -> List[List[float]]:
    T_total, t_acc, t_const, t_dec = compute_segments(total_steps, v_max, shape)
    v_eff = vmax_effective(v_max)
    return _run_motor_profile(gpio, motor_id, direction,
                              total_steps, v_eff,
                              T_total, t_acc, t_const, t_dec,
                              s_curve_velocity_steps)


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
    return _run_motor_profile(gpio, motor_id, direction,
                              total_steps, v_eff,
                              T_total, t_acc, t_const, t_dec,
                              as_curve_velocity)




