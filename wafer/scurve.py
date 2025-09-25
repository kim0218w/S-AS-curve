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

def _steps_per_sec_from_mmps(mm_per_s: float, pitch_mm: float,
                             steps_per_rev: int, microstep: int) -> float:
    """mm/s → steps/s 변환"""
    mps = _mm_per_step(pitch_mm, steps_per_rev, microstep)
    return 0.0 if mps <= 0 else (mm_per_s / mps)

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

# -------------------- 총 시간 계산 --------------------
def compute_total_time_scurve(total_steps: int, v_max_steps: float,
                              shape: str = "mid") -> float:
    """
    steps 적분 = v_max * [0.5*(t_acc + t_dec) + t_const]
    r_* 비율로 쪼개면:
        total_steps = v_max * (0.5*(r_acc+r_dec) + r_const) * T
    → T 계산
    """
    v_eff = vmax_effective(v_max_steps)
    r_acc, r_const, r_dec = _shape_fractions(shape)
    coeff = 0.5 * (r_acc + r_dec) + r_const
    if coeff <= 0 or v_eff <= 0:
        raise ValueError("v_max나 shape 파라미터가 유효하지 않습니다.")
    return float(total_steps) / (v_eff * coeff)

def compute_segments(total_steps: int, v_max_steps: float,
                     shape: str = "mid") -> Tuple[float, float, float, float]:
    """
    세그먼트 시간 반환: (T_total, t_acc, t_const, t_dec)
    """
    T = compute_total_time_scurve(total_steps, v_max_steps, shape)
    r_acc, r_const, r_dec = _shape_fractions(shape)
    return T, r_acc * T, r_const * T, r_dec * T

# -------------------- S-curve 속도(steps/s) --------------------
def s_curve_velocity_steps(t: float, v_max_steps: float,
                           t_acc: float, t_const: float, t_dec: float,
                           T_total: float) -> float:
    """
    sin^2 엣지를 가진 S-curve (가속/감속), 중간 정속 구간 가능.
    반환 단위: steps/s (펄스 발생용)
    """
    if T_total <= 0 or v_max_steps <= 0:
        return 0.0
    if t < 0:
        return 0.0
    if t < t_acc:
        # 가속: 0 → v_max (sin^2)
        return v_max_steps * (np.sin(0.5 * np.pi * (t / max(t_acc, 1e-9))))**2
    t1 = t_acc + t_const
    if t < t1:
        # 정속
        return v_max_steps
    if t < T_total:
        # 감속: v_max → 0 (cos^2)
        tau = t - t1
        return v_max_steps * (np.cos(0.5 * np.pi * (tau / max(t_dec, 1e-9))))**2
    return 0.0

# -------------------- 모터 실행 (데이터 로깅만) --------------------
def run_motor_scurve(
    gpio,
    encoder,              # encoder.Encoder 인스턴스(카운트만 제공)
    motor_id: int,
    direction: str,       # 'f' or 'b'
    total_steps: int,     # 목표 스텝 수
    v_max_steps: float,   # 최대 속도(steps/s) - 드라이버 기준
    shape: str = "mid",   # 'short' | 'mid' | 'long'
    *,
    pitch_mm: float = 5.0,
    steps_per_rev: int = 200,
    microstep: int = 16,
    enc_cpr: int = 1000,  # 엔코더 카운트/회전
    dt: float = 0.01,     # 제어 루프 주기(초)
    estimator_win: int = 10,
    estimator_alpha: float = 0.2,
) -> List[List[float]]:
    """
    반환: data_log = [
        [Time_ms, com_Pos_mm, enc_Pos_mm, com_Vel_mm_per_s, enc_Vel_mm_per_s], ...
    ]
    - 펄스는 steps/s 기반으로 누적 방식으로 발생
    - com_*는 mm 단위로 기록 (그래프/CSV 일관성 유지)
    - enc_*는 A/B→Δcount를 EncoderVelEstimator로 mm/s 추정
    """
    # 방향/Enable
    gpio.set_dir(motor_id, direction.lower() == 'f')
    gpio.set_enable(motor_id, True)

    # 세그먼트 시간 계산
    T_total, t_acc, t_const, t_dec = compute_segments(total_steps, v_max_steps, shape)

    # 보정된 최대 속도
    v_eff = vmax_effective(v_max_steps)

    # 변환 상수
    mm_per_step = _mm_per_step(pitch_mm, steps_per_rev, microstep)

    # 속도 추정기 (Δcount, dt → mm/s)
    est = EncoderVelEstimator(cpr=enc_cpr, pitch_mm=pitch_mm,
                              win_size=estimator_win, lpf_alpha=estimator_alpha)

    # 상태 변수
    moved_steps = 0
    step_acc = 0.0
    com_pos_mm = 0.0

    prev_count = encoder.read()
    enc_init_mm = (prev_count / enc_cpr) * pitch_mm  # 초기 오프셋 mm

    data_log: List[List[float]] = []

    t0 = time.time()
    while moved_steps < total_steps:
        t = time.time() - t0
        if t > T_total:
            break

        # --- Commanded velocity (steps/s → mm/s) ---
        v_steps = s_curve_velocity_steps(t, v_eff, t_acc, t_const, t_dec, T_total)
        com_vel_mm_s = v_steps * mm_per_step
        # 방향 반영(필요 시): com_pos는 방향에 따라 누적
        dir_sign = 1.0 if direction.lower() == 'f' else -1.0

        # --- Step pulses (누적) ---
        step_acc += v_steps * dt
        while step_acc >= 1.0 and moved_steps < total_steps:
            gpio.pulse_step(motor_id, high_time=0.00002,
                            low_time=max(MIN_PULSE_INTERVAL - 0.00002, 0.0))
            moved_steps += 1
            step_acc -= 1.0

        # --- Commanded position (mm) ---
        com_pos_mm += dir_sign * (com_vel_mm_s * dt)

        # --- Encoder read → Δcount → 속도(mm/s) ---
        now = encoder.read()
        delta = now - prev_count
        prev_count = now

        enc_pos_mm = (now / enc_cpr) * pitch_mm - enc_init_mm
        enc_vel_mm_s = est.update(delta * dir_sign, dt)  # 방향 부호 일치

        # --- Log ---
        data_log.append([
            int(round(t * 1000.0)),  # Time_ms
            com_pos_mm,
            enc_pos_mm,
            com_vel_mm_s * dir_sign,   # (부호 맞춤)
            enc_vel_mm_s               # (Estimator 결과)
        ])

        # 루프 주기 맞추기
        sleep_left = dt - (time.time() - (t0 + t))
        if sleep_left > 0:
            time.sleep(sleep_left)

    return data_log
