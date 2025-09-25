# scurve.py
import math
import time
import numpy as np
from encoder import PID

# 안전 최소지연, TX 대기
MIN_SAFE_DELAY = 0.00025
TX_BACKLOG = 8


# --------- 보조 함수들 ---------
def smooth_cos_delay(i, n, min_delay, max_delay):
    if min_delay >= max_delay:
        min_delay, max_delay = sorted([min_delay, max_delay])
    min_delay = max(min_delay, MIN_SAFE_DELAY)
    if n <= 1:
        return min_delay
    t_norm = i / (n - 1)
    k = 0.5 * (1 - math.cos(math.pi * t_norm))
    return max_delay - (max_delay - min_delay) * k

def rate_to_delay(rate_steps_per_s, min_delay, max_delay):
    if rate_steps_per_s <= 0:
        return max_delay
    d = 1.0 / (2.0 * rate_steps_per_s)
    return max(min(d, max_delay), max(min_delay, MIN_SAFE_DELAY))

def delay_to_rate(delay_s):
    return 1.0 / (2.0 * max(delay_s, 1e-6))


# --------- Open-loop 실행 + 지령 궤적 로그 ---------
def move_stepper_scurve_with_logging(
    gpio, dir_pin, step_pin, ena_pin,
    total_steps, forward: bool,
    min_delay, max_delay, accel_ratio=0.2,
    sample_dt=0.015,
    deg_per_step=0.018,  # 출력축 기준 스텝당 각도 (그래프/CSV용 지령 변환)
):
    if total_steps <= 0:
        return {"t": np.zeros(0), "cmd_rate": np.zeros(0)}
    if min_delay >= max_delay:
        min_delay, max_delay = sorted([min_delay, max_delay])
    min_delay = max(min_delay, MIN_SAFE_DELAY)

    # Enable + DIR
    gpio.enable_motor(ena_pin, True)
    gpio.set_dir(dir_pin, forward)

    # FF delay 시퀀스
    accel_steps = int(total_steps * accel_ratio)
    decel_steps = accel_steps
    const_steps = total_steps - accel_steps - decel_steps
    if const_steps < 0:
        accel_steps = total_steps // 2
        decel_steps = total_steps - accel_steps
        const_steps = 0

    ff_delays = []
    for i in range(accel_steps):
        ff_delays.append(smooth_cos_delay(i, max(accel_steps, 1), min_delay, max_delay))
    for _ in range(const_steps):
        ff_delays.append(min_delay)
    for i in range(decel_steps):
        ff_delays.append(smooth_cos_delay(decel_steps - 1 - i, max(decel_steps, 1), min_delay, max_delay))

    # 지령 궤적(출력축 기준)
    cmd_rate_ff = np.array([delay_to_rate(d) for d in ff_delays], dtype=float)  # step/s
    cmd_w_ff = cmd_rate_ff * deg_per_step                                      # deg/s
    t_grid = np.cumsum(np.array(ff_delays, dtype=float))
    t_grid -= (t_grid[0] if len(t_grid) else 0.0)
    cmd_theta_ff = np.zeros_like(t_grid)
    for i in range(1, len(t_grid)):
        dt_i = t_grid[i] - t_grid[i-1]
        cmd_theta_ff[i] = cmd_theta_ff[i-1] + 0.5 * (cmd_w_ff[i] + cmd_w_ff[i-1]) * dt_i

    # 실행 + 로그
    t0 = time.monotonic()
    last_sample = t0
    t_log, cmd_rate_log, com_pos_log, com_vel_log = [], [], [], []

    # 펄스 실행
    for i, d in enumerate(ff_delays):
        now = time.monotonic()
        if now - last_sample >= sample_dt:
            t_log.append(now - t0)
            cmd_rate_log.append(delay_to_rate(d))
            # 샘플 시각에 맞춰 명령 궤적 보간 없이 가장 가까운 i 사용
            com_pos_log.append(cmd_theta_ff[i] if i < len(cmd_theta_ff) else cmd_theta_ff[-1])
            com_vel_log.append(cmd_w_ff[i] if i < len(cmd_w_ff) else cmd_w_ff[-1])
            last_sample = now

        gpio.queue_pulse(step_pin, d)

    time.sleep(min_delay * TX_BACKLOG)
    gpio.enable_motor(ena_pin, False)

    return {
        "t": np.array(t_log, dtype=float),
        "cmd_rate": np.array(cmd_rate_log, dtype=float),
        "com_pos_deg": np.array(com_pos_log, dtype=float),
        "com_vel_dps": np.array(com_vel_log, dtype=float),
        # open-loop: enc_* 없음
    }


# --------- Closed-loop (A4988 + Quadrature Encoder) ---------
def move_stepper_scurve_with_pid(
    gpio, dir_pin, step_pin, ena_pin,
    total_steps, forward: bool,
    min_delay, max_delay, accel_ratio=0.2,
    deg_per_step=0.018,        # ★ 출력축 기준 스텝당 각도 (모터축이면 /기어비 필요)
    encoder=None,              # encoder.Encoder 객체 (A/B)
    sample_dt=0.015,
    pid_gains=(2.0, 0.2, 0.0)
):
    if encoder is None:
        raise ValueError("encoder 객체가 필요합니다 (encoder.py의 Encoder).")

    # 고정 파라미터 (질문자 제공)
    CPR_MOTOR = 2000               # 0.18°/count → 2000 counts/rev
    GEAR_RATIO = 58                # 58:1 (모터:출력)
    ENCODER_ON_MOTOR_SHAFT = True  # 엔코더가 모터축에 있다면 True / 출력축이면 False

    CPR_EFFECTIVE = CPR_MOTOR * GEAR_RATIO if ENCODER_ON_MOTOR_SHAFT else CPR_MOTOR

    if total_steps <= 0:
        return {"t": np.zeros(0), "cmd_rate": np.zeros(0)}
    if min_delay >= max_delay:
        min_delay, max_delay = sorted([min_delay, max_delay])
    min_delay = max(min_delay, MIN_SAFE_DELAY)

    # Enable + DIR
    gpio.enable_motor(ena_pin, True)
    gpio.set_dir(dir_pin, forward)

    # FF delay 시퀀스
    accel_steps = int(total_steps * accel_ratio)
    decel_steps = accel_steps
    const_steps = total_steps - accel_steps - decel_steps
    if const_steps < 0:
        accel_steps = total_steps // 2
        decel_steps = total_steps - accel_steps
        const_steps = 0

    ff_delays = []
    for i in range(accel_steps):
        ff_delays.append(smooth_cos_delay(i, max(accel_steps, 1), min_delay, max_delay))
    for _ in range(const_steps):
        ff_delays.append(min_delay)
    for i in range(decel_steps):
        ff_delays.append(smooth_cos_delay(decel_steps - 1 - i, max(decel_steps, 1), min_delay, max_delay))

    # 명령 궤적(출력축)
    cmd_rate_ff = np.array([delay_to_rate(d) for d in ff_delays], dtype=float)  # step/s
    cmd_w_ff = cmd_rate_ff * deg_per_step                                      # deg/s
    t_grid = np.cumsum(np.array(ff_delays, dtype=float))
    t_grid -= (t_grid[0] if len(t_grid) else 0.0)
    cmd_theta_ff = np.zeros_like(t_grid)
    for i in range(1, len(t_grid)):
        dt_i = t_grid[i] - t_grid[i-1]
        cmd_theta_ff[i] = cmd_theta_ff[i-1] + 0.5 * (cmd_w_ff[i] + cmd_w_ff[i-1]) * dt_i

    # PID
    Kp, Ki, Kd = pid_gains
    pid = PID(kp=Kp, ki=Ki, kd=Kd, out_min=-600.0, out_max=600.0, tau=0.01)

    # 로그 버퍼
    t0 = time.monotonic()
    last_t = t0
    last_sample = t0
    t_log, cmd_rate_log = [], []
    com_pos_log, enc_pos_log = [], []
    com_vel_log, enc_vel_log = [], []

    # 초기 오프셋(출력축 deg)
    prev_count = encoder.read()
    pos_offset_deg = (prev_count / CPR_EFFECTIVE) * 360.0

    # 속도 EMA
    VEL_ALPHA = 0.2
    v_est = 0.0

    # 루프
    for i, ff_d in enumerate(ff_delays):
        now = time.monotonic()
        dt = max(now - last_t, 1e-4)
        last_t = now

        count = encoder.read()
        delta_count = count - prev_count
        prev_count = count

        meas_deg = (count / CPR_EFFECTIVE) * 360.0                  # 출력축 각도
        meas_vel_raw = (delta_count / CPR_EFFECTIVE) * 360.0 / dt   # 출력축 deg/s
        v_est = VEL_ALPHA * meas_vel_raw + (1 - VEL_ALPHA) * v_est
        enc_vel_dps = v_est

        target_deg = cmd_theta_ff[i] + pos_offset_deg
        e = target_deg - meas_deg

        trim_deg_per_s = pid.update(e, dt, anti_windup_ref=0.0)
        trim_steps = trim_deg_per_s / max(deg_per_step, 1e-9)

        ff_rate_steps = delay_to_rate(ff_d)
        desired_rate_steps = max(ff_rate_steps + trim_steps, 1.0)
        d = rate_to_delay(desired_rate_steps, min_delay, max_delay)

        if now - last_sample >= sample_dt:
            t_log.append(now - t0)
            cmd_rate_log.append(desired_rate_steps)
            com_pos_log.append(target_deg)
            com_vel_log.append(cmd_w_ff[i])
            enc_pos_log.append(meas_deg)
            enc_vel_log.append(enc_vel_dps)
            last_sample = now

        gpio.queue_pulse(step_pin, d)

    time.sleep(min_delay * TX_BACKLOG)
    gpio.enable_motor(ena_pin, False)

    return {
        "t": np.array(t_log, dtype=float),
        "cmd_rate": np.array(cmd_rate_log, dtype=float),
        "com_pos_deg": np.array(com_pos_log, dtype=float),
        "enc_pos_deg": np.array(enc_pos_log, dtype=float),
        "com_vel_dps": np.array(com_vel_log, dtype=float),
        "enc_vel_dps": np.array(enc_vel_log, dtype=float),
    }


# --------- (옵션) 파라미터 도우미들 ---------
def calc_scurve_params(total_steps=None, v_max=None, total_time=None, show=True):
    import pandas as pd
    if [total_steps, v_max, total_time].count(None) != 1:
        raise ValueError("세 변수 중 정확히 2개만 지정")
    if total_steps is None:
        total_steps = int((v_max * total_time) / 2)
    elif v_max is None:
        v_max = (2 * total_steps) / total_time
    elif total_time is None:
        total_time = (2 * total_steps) / v_max
    result = {"total_steps": total_steps, "v_max": v_max, "total_time": total_time}
    if show:
        print("[S-curve Parameters]")
        print(f"- 총 스텝수: {total_steps}")
        print(f"- 최대속도 : {v_max:.2f} step/s")
        print(f"- 총시간   : {total_time:.3f} s\n")
        print(pd.DataFrame([result]), "\n")
    return result

def s_curve_velocity(t, v_max, accel_time, const_time, decel_time):
    if t < 0: return 0.0
    if t < accel_time: return v_max * (math.sin(math.pi * t / (2*accel_time)))**2
    if t < accel_time + const_time: return v_max
    if t < accel_time + const_time + decel_time:
        td = t - (accel_time + const_time)
        return v_max * (math.sin(math.pi * td / (2*decel_time)))**2
    return 0.0

def split_motion_time(total_time: float, accel_ratio: float = 0.2):
    accel_time = total_time * accel_ratio
    decel_time = total_time * accel_ratio
    const_time = total_time - accel_time - decel_time
    if const_time < 0:
        accel_time = total_time / 2
        decel_time = total_time / 2
        const_time = 0
    return accel_time, const_time, decel_time
