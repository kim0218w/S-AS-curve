import numpy as np
import pandas as pd
from graph import plot_scurve_profile
import math
import time
from pid import PID
from scurve import smooth_cos_delay, delay_to_rate, rate_to_delay, MIN_SAFE_DELAY

#-----------------scurve--------------------
TX_BACKLOG = 8  # 큐가 모두 나갈 때까지 마무리 대기

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

    if min_delay >= max_delay:
        min_delay, max_delay = sorted([min_delay, max_delay])
    min_delay = max(min_delay, MIN_SAFE_DELAY)

    # 모터 Enable + 방향
    gpio.enable_motor(ena_pin, True)
    gpio.set_dir(dir_pin, forward)

    # --- Feedforward delay 시퀀스 생성 ---
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

    # --- 명령 궤적(각도) 계산 ---
    cmd_rate_ff = np.array([delay_to_rate(d) for d in ff_delays], dtype=float)
    cmd_w_ff = cmd_rate_ff * deg_per_step
    t_grid = np.cumsum(np.array(ff_delays, dtype=float))
    t_grid -= (t_grid[0] if len(t_grid) else 0.0)
    cmd_theta_ff = np.zeros_like(t_grid)
    for i in range(1, len(t_grid)):
        dt = t_grid[i] - t_grid[i-1]
        cmd_theta_ff[i] = cmd_theta_ff[i-1] + 0.5 * (cmd_w_ff[i] + cmd_w_ff[i-1]) * dt

    # --- PID 초기화 ---
    Kp, Ki, Kd = pid_gains
    pid = PID(kp=Kp, ki=Ki, kd=Kd, out_min=-600.0, out_max=600.0, tau=0.01)

    t0 = time.monotonic()
    last_t = t0
    last_sample = t0
    t_log, cmd_rate_log = [], []

    theta0 = enc_read_deg()  # 시작 오프셋

    # --- 실행 루프 ---
    for i, ff_d in enumerate(ff_delays):
        now = time.monotonic()
        dt = max(now - last_t, 1e-4)
        last_t = now

        # 목표 각도
        target_deg = cmd_theta_ff[i] + theta0
        meas_deg = enc_read_deg()
        e = target_deg - meas_deg

        # PID 보정 (deg/s → steps/s)
        trim_deg_per_s = pid.update(e, dt, anti_windup_ref=0.0)
        trim_steps = trim_deg_per_s / max(deg_per_step, 1e-9)

        # Feedforward + Feedback 합성
        ff_rate_steps = delay_to_rate(ff_d)
        desired_rate_steps = max(ff_rate_steps + trim_steps, 1.0)

        d = rate_to_delay(desired_rate_steps, min_delay, max_delay)

        # 샘플 로그
        if now - last_sample >= sample_dt:
            t_log.append(now - t0)
            cmd_rate_log.append(desired_rate_steps)
            last_sample = now

        gpio.queue_pulse(step_pin, d)

    # 큐 비우기 대기
    time.sleep(min_delay * 8)
    t_log.append(time.monotonic() - t0)
    cmd_rate_log.append(delay_to_rate(min_delay))
    gpio.enable_motor(ena_pin, False)

    return {
        "t": np.array(t_log, dtype=float),
        "cmd_rate": np.array(cmd_rate_log, dtype=float),
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

    # --- 계산 ---
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

        # 그래프 출력
        plot_scurve_profile(total_time=total_time, v_max=v_max)

    return result


# -------------------- S-curve 속도 함수 --------------------
def s_curve_velocity(t: float, v_max: float,
                     accel_time: float, const_time: float, decel_time: float) -> float:
    """
    sin² 기반의 accel/const/decel 구간 속도 프로파일 (단위: step/s)
    """
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
    """
    accel_ratio 기반으로 accel/const/decel 시간 분배
    """
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

    # [0,1] 구간으로 정규화
    t_norm = i / (n - 1)
    # 코사인 easing : 0 → 1
    k = 0.5 * (1 - math.cos(math.pi * t_norm))
    # delay 보간 (최대에서 최소로 줄어듦)
    return max_delay - (max_delay - min_delay) * k

def rate_to_delay(rate_steps_per_s, min_delay, max_delay):
    
    if rate_steps_per_s <= 0:
        return max_delay
    d = 1.0 / (2.0 * rate_steps_per_s)
    d = max(min(d, max_delay), max(min_delay, MIN_SAFE_DELAY))
    return d

def delay_to_rate(delay_s):
    
    return 1.0 / (2.0 * max(delay_s, 1e-6))
