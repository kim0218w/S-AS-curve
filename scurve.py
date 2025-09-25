import numpy as np
import pandas as pd
from graph import plot_scurve_profile

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
