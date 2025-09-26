# scurve.py
import time
import numpy as np
from typing import Tuple, List

# -------------------- Pulse Limits --------------------
MIN_PULSE_INTERVAL = 0.001   # 1 kHz (최대 펄스 속도 한계)
MAX_PULSE_INTERVAL = 0.05    # 20 Hz  (최저 펄스 속도 한계)

# -------------------- 기본 모터 파라미터 --------------------
STEPS_PER_REV = 200       # 1.8° 모터
MICROSTEP = 16            # 드라이버 마이크로스텝 설정
DEG_PER_STEP = 360.0 / (STEPS_PER_REV * MICROSTEP)

LPF_ALPHA = 0.1 # 속도 필터링 강도(0~1, 작을수록 더 매끄러워짐)

def vmax_effective(v_max_steps: float,
                   min_pulse_interval: float = MIN_PULSE_INTERVAL) -> float:
    hw_limit = 1.0 / min_pulse_interval
    return min(float(v_max_steps), hw_limit)

# -------------------- Shape 분할 --------------------
def _shape_fractions(shape: str) -> Tuple[float, float, float]:
    s = (shape or "mid").lower()
    if s == "short":
        return 0.5, 0.0, 0.5
    if s == "long":
        return 0.1, 0.8, 0.1
    return 0.25, 0.5, 0.25  # mid

# -------------------- 총 시간 계산 (S-curve) --------------------
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

def compute_segments(total_steps: int, v_max_steps: float,
                     shape: str = "mid") -> Tuple[float, float, float, float]:
    T = s_curve_velocity_steps(total_steps, v_max_steps, shape)
    r_acc, r_const, r_dec = _shape_fractions(shape)
    return T, r_acc * T, r_const * T, r_dec * T

# -------------------- 총 시간 계산 (AS-curve) --------------------
def as_curve_velocity(t: float, v_max: float,
                      t_acc: float, t_const: float, t_dec: float,
                      T_total: float) -> float:
    if t < t_acc:
        return v_max * np.sin((np.pi/2) * (t / t_acc))
    elif t < t_acc + t_const:
        return v_max
    elif t < T_total:
        tau = t - (t_acc + t_const)
        return v_max * np.sin((np.pi/2) * (1 - tau / t_dec))
    return 0.0

# -------------------- 속도 프로파일 --------------------
def s_curve_velocity_steps(t: float, v_max_steps: float,
                           t_acc: float, t_const: float, t_dec: float,
                           T_total: float) -> float:
    """S-curve (sin^2 프로파일)"""
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

def as_curve_velocity(t: float, v_max: float,
                      t_acc: float, t_const: float, t_dec: float,
                      T_total: float) -> float:
    """AS-curve (cosine easing으로 매끄럽게)"""
    if t < t_acc:
        # 부드러운 가속 (0 → v_max)
        return  v_max * np.sin((np.pi/2) * (t / t_acc))
    elif t < t_acc + t_const:
        # 정속
        return v_max
    elif t < T_total:
        tau = t - (t_acc + t_const)
        # 부드러운 감속 (v_max → 0)
        return v_max * np.sin((np.pi/2) * (1 - tau / t_dec))
    return 0.0


# -------------------- 실행 (S-curve) --------------------
def run_motor_scurve(gpio, motor_id, direction, total_steps, v_max, shape="mid"):
    T_total, t_acc, t_const, t_dec = compute_segments(total_steps, v_max, shape)
    v_eff = vmax_effective(v_max)

    gpio.set_dir(motor_id, direction == 'f')
    gpio.set_enable(motor_id, True)

    moved_steps = 0
    data_log = []
    last_pulse_t = None
    pul_based_vel = 0.0  # LPF 초기값
    start_t = time.time()

    while moved_steps < total_steps:
        t = time.time() - start_t
        if t > T_total:
            break

        com_vel_steps = s_curve_velocity_steps(t, v_eff, t_acc, t_const, t_dec, T_total)
        if com_vel_steps < 1e-6:
            time.sleep(0.001)
            continue

        pulse_interval = 1.0 / com_vel_steps
        pulse_interval = max(MIN_PULSE_INTERVAL, min(pulse_interval, MAX_PULSE_INTERVAL))

        gpio.pulse_step(motor_id, high_time=0.00002, low_time=pulse_interval-0.00002)
        moved_steps += 1

        now = time.time()
        if last_pulse_t is not None:
            dt = now - last_pulse_t
            inst_vel = (1.0 / dt) * DEG_PER_STEP
            pul_based_vel = LPF_ALPHA*inst_vel + (1-LPF_ALPHA)*pul_based_vel
        last_pulse_t = now

        com_vel_deg = com_vel_steps * DEG_PER_STEP
        com_pos = moved_steps * DEG_PER_STEP

        t_ms = int(round(t * 1000))
        data_log.append([t_ms, com_pos, com_pos, com_vel_deg, pul_based_vel])

    # 마지막 샘플 보정
    com_pos = total_steps * DEG_PER_STEP
    t_ms = int(round(T_total * 1000))
    data_log.append([t_ms, com_pos, com_pos, 0.0, 0.0])

    return data_log

# -------------------- 실행 (AS-curve) --------------------
def run_motor_ascurve(
    gpio, motor_id: int, direction: str,
    total_steps: int, v_max_steps: float, shape: str = "mid",
) -> List[List[float]]:
    T_total, t_acc, t_const, t_dec = compute_total_time_ascurve(total_steps, v_max_steps, shape)
    v_eff = vmax_effective(v_max_steps)

    gpio.set_dir(motor_id, direction.lower() == 'f')
    gpio.set_enable(motor_id, True)

    moved_steps = 0
    data_log = []
    last_pulse_t = None
    pul_based_vel = 0.0
    start_t = time.time()

    while moved_steps < total_steps:
        t = time.time() - start_t
        if t > T_total:
            break

        v_steps = as_curve_velocity(t, v_eff, t_acc, t_const, t_dec, T_total)
        if v_steps < 1e-6:
            time.sleep(0.001)
            continue

        pulse_interval = 1.0 / v_steps
        pulse_interval = max(MIN_PULSE_INTERVAL, min(pulse_interval, MAX_PULSE_INTERVAL))

        gpio.pulse_step(motor_id, high_time=0.00002, low_time=pulse_interval-0.00002)
        moved_steps += 1

        now = time.time()
        if last_pulse_t is not None:
            dt = now - last_pulse_t
            inst_vel = (1.0 / dt) * DEG_PER_STEP
            pul_based_vel = LPF_ALPHA*inst_vel + (1-LPF_ALPHA)*pul_based_vel
        last_pulse_t = now

        com_vel_deg = v_steps * DEG_PER_STEP
        com_pos = moved_steps * DEG_PER_STEP

        t_ms = int(round(t * 1000))
        data_log.append([t_ms, com_pos, com_pos, com_vel_deg, pul_based_vel])

    # 마지막 샘플 보정
    com_pos = total_steps * DEG_PER_STEP
    t_ms = int(round(T_total * 1000))
    data_log.append([t_ms, com_pos, com_pos, 0.0, 0.0])

    return data_log
