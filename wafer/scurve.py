import numpy as np
import pandas as pd
from graph import plot_scurve_profile
import math
import time
from encoder import PID


#-----------------scurve--------------------
MIN_SAFE_DELAY = 0.00025  # 250us
TX_BACKLOG = 8  # (옵션) 큐가 모두 나갈 때까지 대기할 때 사용할 여유 팩터

def move_stepper_scurve_with_pid(
    gpio, dir_pin, step_pin, ena_pin,
    total_steps, forward: bool,
    min_delay, max_delay, accel_ratio=0.2,
    deg_per_step=0.018,
    enc_read_deg=None,     # 반드시 함수 형태 (현재 각도를 deg로 리턴)
    sample_dt=0.015,
    pid_gains=(2.0, 0.2, 0.0)
):
    if enc_read_deg is None:
        raise ValueError("enc_read_deg callable is required for closed-loop mode.")

    # ... (생략: 기존 안전가드/enable/방향/ff_delays 생성 동일)

    # --- 명령 궤적(각도/각속도) 계산 (기존) ---
    cmd_rate_ff = np.array([delay_to_rate(d) for d in ff_delays], dtype=float)   # step/s
    cmd_w_ff = cmd_rate_ff * deg_per_step                                       # deg/s
    t_grid = np.cumsum(np.array(ff_delays, dtype=float))
    t_grid -= (t_grid[0] if len(t_grid) else 0.0)
    cmd_theta_ff = np.zeros_like(t_grid)
    for i in range(1, len(t_grid)):
        dt_i = t_grid[i] - t_grid[i-1]
        cmd_theta_ff[i] = cmd_theta_ff[i-1] + 0.5 * (cmd_w_ff[i] + cmd_w_ff[i-1]) * dt_i

    # --- PID 초기화 (기존) ---
    Kp, Ki, Kd = pid_gains
    pid = PID(kp=Kp, ki=Ki, kd=Kd, out_min=-600.0, out_max=600.0, tau=0.01)

    t0 = time.monotonic()
    last_t = t0
    last_sample = t0

    # 🔧 추가: 확장 로그 버퍼
    t_log = []
    cmd_rate_log = []
    com_pos_log = []
    enc_pos_log = []
    com_vel_log = []
    enc_vel_log = []

    theta0 = enc_read_deg()     # 시작 오프셋
    enc_prev = theta0           # 🔧 추가: 엔코더 속도 계산용

    # --- 실행 루프 ---
    for i, ff_d in enumerate(ff_delays):
        now = time.monotonic()
        dt = max(now - last_t, 1e-4)
        last_t = now

        # 목표 각도 vs 측정
        target_deg = cmd_theta_ff[i] + theta0
        meas_deg = enc_read_deg()
        e = target_deg - meas_deg

        # 엔코더 속도(추정)
        enc_vel_dps = (meas_deg - enc_prev) / dt
        enc_prev = meas_deg

        # PID 보정 (deg/s → steps/s)
        trim_deg_per_s = pid.update(e, dt, anti_windup_ref=0.0)
        trim_steps = trim_deg_per_s / max(deg_per_step, 1e-9)

        # Feedforward + Feedback 합성
        ff_rate_steps = delay_to_rate(ff_d)
        desired_rate_steps = max(ff_rate_steps + trim_steps, 1.0)
        d = rate_to_delay(desired_rate_steps, min_delay, max_delay)

        # 🔧 로그(샘플링)
        if now - last_sample >= sample_dt:
            t_log.append(now - t0)
            cmd_rate_log.append(desired_rate_steps)  # step/s
            # 지령 각도/각속도
            com_pos_log.append(target_deg)
            com_vel_log.append(cmd_w_ff[i])
            # 실측 각도/각속도
            enc_pos_log.append(meas_deg)
            enc_vel_log.append(enc_vel_dps)
            last_sample = now

        gpio.queue_pulse(step_pin, d)

    # 큐 비우기 + 마지막 샘플
    time.sleep(min_delay * TX_BACKLOG)
    t_log.append(time.monotonic() - t0)
    cmd_rate_log.append(delay_to_rate(min_delay))
    # 마지막 값 재기록(보간 없이 직전값)
    if len(com_pos_log):
        com_pos_log.append(com_pos_log[-1])
        com_vel_log.append(0.0)
        enc_pos_log.append(enc_pos_log[-1] if len(enc_pos_log) else theta0)
        enc_vel_log.append(0.0)

    gpio.enable_motor(ena_pin, False)

    return {
        "t": np.array(t_log, dtype=float),
        "cmd_rate": np.array(cmd_rate_log, dtype=float),   # step/s (호환)
        "com_pos_deg": np.array(com_pos_log, dtype=float),
        "enc_pos_deg": np.array(enc_pos_log, dtype=float),
        "com_vel_dps": np.array(com_vel_log, dtype=float),
        "enc_vel_dps": np.array(enc_vel_log, dtype=float),
    }


# -------------------- S-curve 파라미터 자동 계산 --------------------
def calc_scurve_params(total_steps=None, v_max=None, total_time=None, show=True):
    """
    S-curve 프로파일 파라미터 자동 계산
    v(t) = v_max * sin^2(pi * t / T)
    ∫ v(t) dt = (v_max * total_time) / 2 ≈ total_steps
    """
    if [total_steps, v_max, total_time].count(None) != 1:
        raise ValueError("세 변수 중 정확히 2개만 지정해야 합니다.")

    if total_steps is None:
        total_steps = int((v_max * total_time) / 2)
    elif v_max is None:
        v_max = (2 * total_steps) / total_time
    elif total_time is None:
        total_time = (2 * total_steps) / v_max

    result = {"total_steps": total_steps, "v_max": v_max, "total_time": total_time}

    if show:
        print("[S-curve Parameters]")
        print(f"총 스텝 수 : {total_steps} steps")
        print(f"최대 속도  : {v_max:.2f} steps/s")
        print(f"총 이동 시간: {total_time:.3f} s\n")

        df = pd.DataFrame([result])
        print(df, "\n")

        plot_scurve_profile(total_time=total_time, v_max=v_max)

    return result


# -------------------- S-curve 속도 함수 --------------------
def s_curve_velocity(t: float, v_max: float,
                     accel_time: float, const_time: float, decel_time: float) -> float:
    if t < 0:
        return 0.0
    elif t < accel_time:  # 가속 구간
        return v_max * (np.sin(np.pi * t / (2 * accel_time)))**2
    elif t < accel_time + const_time:  # 정속 구간
        return v_max
    elif t < accel_time + const_time + decel_time:  # 감속 구간
        td = t - (accel_time + const_time)
        return v_max * (np.sin(np.pi * td / (2 * decel_time)))**2
    else:
        return 0.0


# -------------------- accel/const/decel 시간 계산 --------------------
def split_motion_time(total_time: float, accel_ratio: float = 0.2):
    accel_time = total_time * accel_ratio
    decel_time = total_time * accel_ratio
    const_time = total_time - accel_time - decel_time
    if const_time < 0:
        accel_time = total_time / 2
        decel_time = total_time / 2
        const_time = 0
    return accel_time, const_time, decel_time


#-----------------motion_utils--------------------
# 안전 최소 delay (하드웨어 스펙에 맞게 조정)
MIN_SAFE_DELAY = 0.00025  # 250us

def smooth_cos_delay(i, n, min_delay, max_delay):
    if min_delay >= max_delay:
        min_delay, max_delay = sorted([min_delay, max_delay])
    min_delay = max(min_delay, MIN_SAFE_DELAY)
    if n <= 1:
        return min_delay
    t_norm = i / (n - 1)                            # [0,1]
    k = 0.5 * (1 - math.cos(math.pi * t_norm))      # cosine easing
    return max_delay - (max_delay - min_delay) * k  # 큰→작게 보간

def rate_to_delay(rate_steps_per_s, min_delay, max_delay):
    if rate_steps_per_s <= 0:
        return max_delay
    d = 1.0 / (2.0 * rate_steps_per_s)
    d = max(min(d, max_delay), max(min_delay, MIN_SAFE_DELAY))
    return d

def delay_to_rate(delay_s):
    return 1.0 / (2.0 * max(delay_s, 1e-6))


# ==================== NEW: Open-loop 실행 함수 ====================
def move_stepper_scurve_with_logging(
    gpio, dir_pin, step_pin, ena_pin,
    total_steps, forward: bool,
    min_delay, max_delay, accel_ratio=0.2,
    sample_dt=0.015,
    deg_per_step=0.018,   # 🔧 추가(기본값): 기존 호출과 호환
):
    # ... (생략: 기존 안전가드/enable/방향/ff_delays 생성 동일)

    # --- 실행 + 로깅 ---
    t0 = time.monotonic()
    last_sample = t0

    # 🔧 추가: 확장 로그 버퍼
    t_log = []
    cmd_rate_log = []
    com_pos_log = []
    com_vel_log = []

    com_theta = 0.0  # deg 누적

    for d in ff_delays:
        now = time.monotonic()

        # 지령 각속도 (deg/s)
        com_vel_dps = delay_to_rate(d) * deg_per_step
        # dt를 샘플링 간격으로 사용해 적분(표본마다 누적)
        if now - last_sample >= sample_dt:
            dt_s = (now - last_sample)
            com_theta += com_vel_dps * dt_s

            t_log.append(now - t0)
            cmd_rate_log.append(delay_to_rate(d))
            com_pos_log.append(com_theta)
            com_vel_log.append(com_vel_dps)

            last_sample = now

        # 한 스텝 펄스
        gpio.queue_pulse(step_pin, d)

    # 큐 비우기 + 마지막 샘플
    time.sleep(min_delay * TX_BACKLOG)
    t_log.append(time.monotonic() - t0)
    cmd_rate_log.append(delay_to_rate(min_delay))
    com_pos_log.append(com_theta)
    com_vel_log.append(0.0)

    gpio.enable_motor(ena_pin, False)

    return {
        "t": np.array(t_log, dtype=float),
        "cmd_rate": np.array(cmd_rate_log, dtype=float),   # step/s (호환)
        "com_pos_deg": np.array(com_pos_log, dtype=float),
        "com_vel_dps": np.array(com_vel_log, dtype=float),
        # open-loop에는 enc_* 미포함 (None/결측으로 처리해도 됨)
    }
