# scurve.py
import time
import numpy as np
from typing import Tuple, List
from encoder import EncoderVelEstimator  # Δcount → (MA+LPF) → mm/s

# -------------------- Pulse Limits --------------------
MIN_PULSE_INTERVAL = 0.001   # 1 kHz (최대 펄스 속도 한계)
MAX_PULSE_INTERVAL = 0.05    # 20 Hz  (최저 펄스 속도 한계)

def _mm_per_step(pitch_mm: float, steps_per_rev: int, microstep: int) -> float:
    """한 스텝당 이동거리 [mm/step]"""
    return pitch_mm / (steps_per_rev * microstep)

def vmax_effective(v_max_steps: float,
                   min_pulse_interval: float = MIN_PULSE_INTERVAL) -> float:
    """하드웨어 펄스 한계 고려한 최대 속도 (steps/s)"""
    hw_limit = 1.0 / min_pulse_interval
    return min(float(v_max_steps), hw_limit)

# -------------------- Shape 분할 --------------------
def _shape_fractions(shape: str) -> Tuple[float, float, float]:
    """
    (r_acc, r_const, r_dec) 합 = 1
    - short: 정속 0% (가속 50% / 감속 50%)
    - mid  : 정속 50% (가속 25% / 감속 25%)
    - long : 정속 80% (가속 10% / 감속 10%)
    """
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
def compute_total_time_ascurve(total_steps: int, v_max_steps: float, shape="mid"):
    """
    AS-curve: 삼각형/사다리꼴 가속도 곡선 (acc-const-dec)
    """
    if shape == "short":  r_acc, r_dec, r_const = 0.4, 0.6, 0.0
    elif shape == "long": r_acc, r_dec, r_const = 0.15, 0.25, 0.6
    else:                 r_acc, r_dec, r_const = 0.2, 0.4, 0.4
    v_eff = vmax_effective(v_max_steps)
    coeff = 0.5*(r_acc+r_dec) + r_const
    T = total_steps / (v_eff * coeff)
    return T, r_acc*T, r_const*T, r_dec*T

# -------------------- 속도 프로파일 --------------------
def s_curve_velocity_steps(t: float, v_max_steps: float,
                           t_acc: float, t_const: float, t_dec: float,
                           T_total: float) -> float:
    if T_total <= 0 or v_max_steps <= 0:
        return 0.0
    if t < t_acc:
        return v_max_steps * (np.sin(0.5 * np.pi * (t / max(t_acc, 1e-9))))**2
    t1 = t_acc + t_const
    if t < t1:
        return v_max_steps
    if t < T_total:
        tau = t - t1
        return v_max_steps * (np.cos(0.5 * np.pi * (tau / max(t_dec, 1e-9))))**2
    return 0.0

def as_curve_velocity(t: float, v_max: float, t_acc: float, t_const: float, t_dec: float, T_total: float) -> float:
    if t < t_acc:
        return v_max * (t / t_acc)  # 선형 가속
    elif t < t_acc + t_const:
        return v_max               # 정속
    elif t < T_total:
        tau = t - (t_acc + t_const)
        return v_max * (1 - tau / t_dec)  # 선형 감속
    return 0.0

# -------------------- 실행 (S-curve) --------------------
def run_motor_scurve(
    gpio, encoder, motor_id: int, direction: str,
    total_steps: int, v_max_steps: float, shape: str = "mid",
    *, pitch_mm: float = 5.0, steps_per_rev: int = 200, microstep: int = 16,
    enc_cpr: int = 1000, dt: float = 0.01, estimator_win: int = 10, estimator_alpha: float = 0.2,
) -> List[List[float]]:
    T_total, t_acc, t_const, t_dec = compute_segments(total_steps, v_max_steps, shape)
    v_eff = vmax_effective(v_max_steps)
    mm_per_step = _mm_per_step(pitch_mm, steps_per_rev, microstep)
    est = EncoderVelEstimator(cpr=enc_cpr, pitch_mm=pitch_mm,
                              win_size=estimator_win, lpf_alpha=estimator_alpha)

    gpio.set_dir(motor_id, direction.lower() == 'f')
    gpio.set_enable(motor_id, True)

    moved_steps, step_acc, com_pos_mm = 0, 0.0, 0.0
    prev_count = encoder.read()
    enc_init_mm = (prev_count / enc_cpr) * pitch_mm
    data_log = []

    t0 = time.time()
    while moved_steps < total_steps:
        t = time.time() - t0
        if t > T_total: break

        v_steps = s_curve_velocity_steps(t, v_eff, t_acc, t_const, t_dec, T_total)
        com_vel_mm_s = v_steps * mm_per_step
        dir_sign = 1.0 if direction.lower() == 'f' else -1.0

        step_acc += v_steps * dt
        while step_acc >= 1.0 and moved_steps < total_steps:
            gpio.pulse_step(motor_id, high_time=0.00002,
                            low_time=max(MIN_PULSE_INTERVAL-0.00002,0.0))
            moved_steps += 1
            step_acc -= 1.0

        com_pos_mm += dir_sign * (com_vel_mm_s * dt)
        now = encoder.read()
        delta = now - prev_count
        prev_count = now

        enc_pos_mm = (now / enc_cpr) * pitch_mm - enc_init_mm
        enc_vel_mm_s = est.update(delta*dir_sign, dt)

        data_log.append([int(round(t*1000)), com_pos_mm, enc_pos_mm,
                         com_vel_mm_s*dir_sign, enc_vel_mm_s])

        sleep_left = dt - (time.time() - (t0 + t))
        if sleep_left > 0: time.sleep(sleep_left)

    return data_log

# -------------------- 실행 (AS-curve) --------------------
def run_motor_ascurve(
    gpio, encoder, motor_id: int, direction: str,
    total_steps: int, v_max_steps: float, shape: str = "mid",
    *, pitch_mm: float = 5.0, steps_per_rev: int = 200, microstep: int = 16,
    enc_cpr: int = 1000, dt: float = 0.01, estimator_win: int = 10, estimator_alpha: float = 0.2,
) -> List[List[float]]:
    T_total, t_acc, t_const, t_dec = compute_total_time_ascurve(total_steps, v_max_steps, shape)
    v_eff = vmax_effective(v_max_steps)
    mm_per_step = _mm_per_step(pitch_mm, steps_per_rev, microstep)
    est = EncoderVelEstimator(cpr=enc_cpr, pitch_mm=pitch_mm,
                              win_size=estimator_win, lpf_alpha=estimator_alpha)

    gpio.set_dir(motor_id, direction.lower() == 'f')
    gpio.set_enable(motor_id, True)

    moved_steps, step_acc, com_pos_mm = 0, 0.0, 0.0
    prev_count = encoder.read()
    enc_init_mm = (prev_count / enc_cpr) * pitch_mm
    data_log = []

    t0 = time.time()
    while moved_steps < total_steps:
        t = time.time() - t0
        if t > T_total: break

        v_steps = as_curve_velocity(t, v_eff, t_acc, t_const, t_dec, T_total)
        com_vel_mm_s = v_steps * mm_per_step
        dir_sign = 1.0 if direction.lower() == 'f' else -1.0

        step_acc += v_steps * dt
        while step_acc >= 1.0 and moved_steps < total_steps:
            gpio.pulse_step(motor_id, high_time=0.00002,
                            low_time=max(MIN_PULSE_INTERVAL-0.00002,0.0))
            moved_steps += 1
            step_acc -= 1.0

        com_pos_mm += dir_sign * (com_vel_mm_s * dt)
        now = encoder.read()
        delta = now - prev_count
        prev_count = now

        enc_pos_mm = (now / enc_cpr) * pitch_mm - enc_init_mm
        enc_vel_mm_s = est.update(delta*dir_sign, dt)

        data_log.append([int(round(t*1000)), com_pos_mm, enc_pos_mm,
                         com_vel_mm_s*dir_sign, enc_vel_mm_s])

        sleep_left = dt - (time.time() - (t0 + t))
        if sleep_left > 0: time.sleep(sleep_left)

    return data_log
