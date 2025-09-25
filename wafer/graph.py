import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

# -------------------- Plot Functions --------------------
def plot_scurve_profile(total_time, v_max):
    """S-curve 속도 프로파일 그리기"""
    t = np.linspace(0, total_time, 500)
    v = v_max * (np.sin(np.pi * t / total_time))**2

    plt.figure(figsize=(6, 4))
    plt.plot(t, v, label="S-curve velocity")
    plt.xlabel("Time [s]")
    plt.ylabel("Velocity [steps/s]")
    plt.title("S-curve Motion Profile")
    plt.grid()
    plt.legend()
    plt.tight_layout()
    plt.show()


def plot_run_results(data_log):
    """실제 모터 실행 후 로그 데이터(DataFrame 또는 리스트)로 속도/위치 그래프 그리기"""
    if not isinstance(data_log, pd.DataFrame):
        df = pd.DataFrame(data_log, columns=[
            "Time_ms", "com_Pos_mm", "enc_Pos_mm",
            "com_Vel_mm_per_s", "enc_Vel_mm_per_s"
        ])
    else:
        df = data_log

    plt.figure(figsize=(8, 4))

    # 속도 그래프
    plt.subplot(2, 1, 1)
    plt.plot(df["Time_ms"], df["com_Vel_mm_per_s"], label="com_Vel_mm_per_s", linestyle="--")
    plt.plot(df["Time_ms"], df["enc_Vel_mm_per_s"], label="enc_Vel_mm_per_s", alpha=0.8)
    plt.ylabel("Velocity [mm/s]")
    plt.legend()
    plt.grid()

    # 위치 그래프
    plt.subplot(2, 1, 2)
    plt.plot(df["Time_ms"], df["com_Pos_mm"], label="com_Pos_mm")
    plt.plot(df["Time_ms"], df["enc_Pos_mm"], label="enc_Pos_mm", alpha=0.8)
    plt.xlabel("Time [ms]")
    plt.ylabel("Position [mm]")
    plt.legend()
    plt.grid()

    plt.tight_layout()
    plt.show()

# -------------------- Analysis Utilities --------------------
def unwrap_deg(deg_series: np.ndarray) -> np.ndarray:
    """0~360도 범위의 엔코더 데이터를 연속 각도로 변환"""
    return np.degrees(np.unwrap(np.radians(deg_series)))

def finite_diff(y: np.ndarray, t: np.ndarray) -> np.ndarray:
    """유한차분법으로 y(t)의 1차 미분 계산"""
    if len(y) < 2 or len(y) != len(t):
        return np.zeros_like(y)
    dy = np.empty_like(y)
    dy[0]  = (y[1] - y[0]) / (t[1] - t[0] + 1e-12)
    dy[-1] = (y[-1] - y[-2]) / (t[-1] - t[-2] + 1e-12)
    for i in range(1, len(y)-1):
        dt = (t[i+1] - t[i-1])
        dy[i] = (y[i+1] - y[i-1]) / (dt + 1e-12)
    return dy

def cumtrapz(y: np.ndarray, x: np.ndarray) -> np.ndarray:
    """트래피조이드 적분으로 ∫ y dx 계산"""
    if y is None or x is None or len(y) < 2 or len(y) != len(x):
        return np.zeros_like(x)
    out = np.zeros_like(y)
    for i in range(1, len(y)):
        dx = x[i] - x[i-1]
        out[i] = out[i-1] + 0.5 * (y[i] + y[i-1]) * dx
    return out

# -------------------- Overlay Plot --------------------
def plot_overlay(t, cmd_angle=None, enc_angle=None,
                 cmd_vel=None, enc_vel=None,
                 enc_acc=None, title_prefix=""):
    """
    명령/측정 궤적 비교 그래프
    t: 시간 벡터
    cmd_angle: 명령 각도 (deg)
    enc_angle: 측정 각도 (deg, unwrap된 값 권장)
    cmd_vel: 명령 속도 (deg/s)
    enc_vel: 측정 속도 (deg/s)
    enc_acc: 측정 가속도 (deg/s^2)
    """
    nrows = 0
    if (cmd_angle is not None or enc_angle is not None): nrows += 1
    if (cmd_vel is not None or enc_vel is not None): nrows += 1
    if enc_acc is not None: nrows += 1
    if nrows == 0: 
        print("[plot_overlay] Nothing to plot.")
        return

    plt.figure(figsize=(8, 3*nrows))

    row = 1
    if (cmd_angle is not None or enc_angle is not None):
        plt.subplot(nrows, 1, row); row += 1
        if cmd_angle is not None:
            plt.plot(t, cmd_angle, label="Commanded θ", linestyle="--")
        if enc_angle is not None:
            plt.plot(t, enc_angle, label="Measured θ")
        plt.ylabel("Angle [deg]")
        plt.legend(); plt.grid()
        if title_prefix: plt.title(f"{title_prefix} Angle")

    if (cmd_vel is not None or enc_vel is not None):
        plt.subplot(nrows, 1, row); row += 1
        if cmd_vel is not None:
            plt.plot(t, cmd_vel, label="Commanded ω", linestyle="--")
        if enc_vel is not None:
            plt.plot(t, enc_vel, label="Measured ω")
        plt.ylabel("Velocity [deg/s]")
        plt.legend(); plt.grid()
        if title_prefix: plt.title(f"{title_prefix} Velocity")

    if enc_acc is not None:
        plt.subplot(nrows, 1, row); row += 1
        plt.plot(t, enc_acc, label="Measured α")
        plt.ylabel("Acceleration [deg/s²]")
        plt.legend(); plt.grid()
        if title_prefix: plt.title(f"{title_prefix} Acceleration")

    plt.xlabel("Time [s]")
    plt.tight_layout()
    plt.show()
