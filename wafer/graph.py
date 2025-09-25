# graph.py
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# 간단한 프로파일 (디버그용)
def plot_scurve_profile(total_time, v_max):
    t = np.linspace(0, total_time, 500)
    v = v_max * (np.sin(np.pi * t / total_time))**2
    plt.figure(figsize=(6,4))
    plt.plot(t, v, label="S-curve velocity")
    plt.xlabel("Time [s]"); plt.ylabel("Velocity [steps/s]")
    plt.title("S-curve Motion Profile"); plt.grid(); plt.legend(); plt.tight_layout(); plt.show()


def plot_run_results(data):
    """
    data: dict 또는 DataFrame
      keys: t, com_pos_deg, enc_pos_deg, com_vel_dps, enc_vel_dps, cmd_rate
    """
    df = pd.DataFrame(data)

    have_enc = ("enc_pos_deg" in df.columns) or ("enc_vel_dps" in df.columns)

    rows = 2 if have_enc else 1
    plt.figure(figsize=(9, 3.5*rows))

    # 속도
    ax1 = plt.subplot(rows, 1, 1)
    if "com_vel_dps" in df.columns:
        ax1.plot(df["t"], df["com_vel_dps"], label="com_vel [deg/s]", linestyle="--")
    if "enc_vel_dps" in df.columns:
        ax1.plot(df["t"], df["enc_vel_dps"], label="enc_vel [deg/s]")
    elif "cmd_rate" in df.columns:
        ax1.plot(df["t"], df["cmd_rate"], label="cmd_rate [step/s]")
    ax1.set_ylabel("Velocity"); ax1.grid(); ax1.legend()

    # 위치
    if have_enc or ("com_pos_deg" in df.columns):
        ax2 = plt.subplot(rows, 1, rows)
        if "com_pos_deg" in df.columns:
            ax2.plot(df["t"], df["com_pos_deg"], label="com_pos [deg]")
        if "enc_pos_deg" in df.columns:
            ax2.plot(df["t"], df["enc_pos_deg"], label="enc_pos [deg]")
        ax2.set_xlabel("Time [s]"); ax2.set_ylabel("Angle [deg]"); ax2.grid(); ax2.legend()

    plt.tight_layout(); plt.show()


def unwrap_deg(deg_series: np.ndarray) -> np.ndarray:
    return np.degrees(np.unwrap(np.radians(deg_series)))

def finite_diff(y: np.ndarray, t: np.ndarray) -> np.ndarray:
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
    if y is None or x is None or len(y) < 2 or len(y) != len(x):
        return np.zeros_like(x)
    out = np.zeros_like(y)
    for i in range(1, len(y)):
        dx = x[i] - x[i-1]
        out[i] = out[i-1] + 0.5 * (y[i] + y[i-1]) * dx
    return out


def plot_overlay(t, cmd_angle=None, enc_angle=None, cmd_vel=None, enc_vel=None, enc_acc=None, title_prefix=""):
    nrows = 0
    if (cmd_angle is not None or enc_angle is not None): nrows += 1
    if (cmd_vel is not None or enc_vel is not None): nrows += 1
    if enc_acc is not None: nrows += 1
    if nrows == 0:
        print("[plot_overlay] Nothing to plot."); return
    plt.figure(figsize=(8, 3*nrows)); row = 1

    if (cmd_angle is not None or enc_angle is not None):
        plt.subplot(nrows, 1, row); row += 1
        if cmd_angle is not None: plt.plot(t, cmd_angle, label="com_pos [deg]", linestyle="--")
        if enc_angle is not None: plt.plot(t, enc_angle, label="enc_pos [deg]")
        plt.ylabel("Angle [deg]"); plt.legend(); plt.grid()
        if title_prefix: plt.title(f"{title_prefix} Angle")

    if (cmd_vel is not None or enc_vel is not None):
        plt.subplot(nrows, 1, row); row += 1
        if cmd_vel is not None: plt.plot(t, cmd_vel, label="com_vel [deg/s]", linestyle="--")
        if enc_vel is not None: plt.plot(t, enc_vel, label="enc_vel [deg/s]")
        plt.ylabel("Velocity [deg/s]"); plt.legend(); plt.grid()
        if title_prefix: plt.title(f"{title_prefix} Velocity")

    if enc_acc is not None:
        plt.subplot(nrows, 1, row); row += 1
        plt.plot(t, enc_acc, label="enc_acc [deg/s²]")
        plt.ylabel("Acceleration [deg/s²]"); plt.legend(); plt.grid()
        if title_prefix: plt.title(f"{title_prefix} Acceleration")

    plt.xlabel("Time [ms]"); plt.tight_layout(); plt.show()
