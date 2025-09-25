import os
import csv
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
try:
    from scipy.signal import savgol_filter, butter, filtfilt
    _HAS_SCIPY = True
except Exception:
    _HAS_SCIPY = False


# -------------------- CSV 저장 --------------------
def save_csv(data_log, filename="scurve_run.csv", pitch_mm=5.0, **meta):
    """
    enc_vel을 RPM 값으로 저장합니다.
    data_log: [Time_ms, com_Pos_mm, enc_Pos_mm, com_Vel_mm_per_s, enc_Vel_raw]
              enc_Vel_raw는 mm/s 라고 가정
    pitch_mm: 1회전 당 이송 거리(mm)
    """
    os.makedirs("logs", exist_ok=True)
    filepath = os.path.join("logs", filename)
    with open(filepath, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "Time_ms", "com_Pos_mm", "enc_Pos_mm",
            "com_Vel_mm_per_s", "enc_Vel_RPM"
        ])
        for row in data_log:
            Time_ms, com_Pos_mm, enc_Pos_mm, com_Vel_mm_per_s, enc_Vel_raw = row
            enc_Vel_RPM = ((enc_Vel_raw / pitch_mm) * 60.0) if (pitch_mm and pitch_mm != 0) else np.nan
            writer.writerow([Time_ms, com_Pos_mm, enc_Pos_mm, com_Vel_mm_per_s, enc_Vel_RPM])
    print(f"-- 실행 완료! CSV 저장: {filepath} --")
    return filepath


# -------------------- 실제 데이터 기반 플롯 --------------------
def plot_results(
    data_log,
    title="S-Curve Motion",
    motor_id=None, steps=None, shape=None,
    pitch_mm=5.0, step_angle_deg=1.8, microstep=16,
    smooth_method="savgol", smooth_ms=30, polyorder=3
):
    """
    실제 데이터 기반 S-curve 그래프 (속도[RPM] + 위치[mm])
    Short 모션의 경우 smooth_ms=20~40 권장
    """

    df = pd.DataFrame(data_log, columns=[
        "Time_ms", "com_Pos_mm", "enc_Pos_mm", "com_Vel_mm_per_s", "enc_Vel_raw"
    ])
    df["Time_s"] = df["Time_ms"] / 1000.0

    if len(df) < 3:
        raise ValueError("데이터가 너무 적습니다.")

    t = df["Time_s"].to_numpy()
    pos = df["enc_Pos_mm"].to_numpy()
    dt = np.median(np.diff(t))
    if not np.isfinite(dt) or dt <= 0:
        raise ValueError("시간 축이 올바르지 않습니다.")
    fs = 1.0 / dt

    # ---- 속도 계산 (mm/s) ----
    if smooth_method == "savgol":
        win_len = max(5, int(round((smooth_ms/1000.0) * fs)) | 1)
        win_len = min(win_len, len(pos) - (1 - len(pos) % 2))
        if win_len < 5: win_len = 5
        if win_len % 2 == 0: win_len += 1
        vel_mm_s = savgol_filter(pos, window_length=win_len,
                                 polyorder=polyorder, deriv=1, delta=dt)
    elif smooth_method == "butter":
        cutoff_hz = 10.0
        b, a = butter(N=3, Wn=cutoff_hz/(fs*0.5), btype='low')
        pos_f = filtfilt(b, a, pos, method="gust")
        vel_mm_s = np.gradient(pos_f, df["Time_s"])
    else:
        vel_mm_s = np.gradient(pos, df["Time_s"])

    # ---- 속도 부호 반전 (그래프 뒤집기) ----
    vel_mm_s = -vel_mm_s

    # ---- mm/s → RPM ----
    if pitch_mm and pitch_mm != 0:
        df["enc_RPM"] = (vel_mm_s / pitch_mm) * 60.0
        df["com_RPM"] = (df["com_Vel_mm_per_s"] / pitch_mm) * 60.0
    else:
        df["enc_RPM"] = np.nan
        df["com_RPM"] = np.nan

    # ---- 플롯 ----
    plt.figure(figsize=(8, 6))

    # 속도 그래프
    plt.subplot(2, 1, 1)
    plt.plot(df["Time_ms"], df["com_RPM"], label="Commanded (RPM)", linestyle="--")
    plt.plot(df["Time_ms"], df["enc_RPM"], label=f"Encoder (RPM) [{smooth_method}]")
    plt.ylabel("Speed [RPM]")
    plt.legend()
    plt.grid(True)

    rpm_min = 0
    rpm_max = max(df["enc_RPM"].max(), df["com_RPM"].max())
    plt.ylim(rpm_min, rpm_max * 1.1)

    # 위치 그래프
    plt.subplot(2, 1, 2)
    plt.plot(df["Time_ms"], df["com_Pos_mm"], label="Commanded Position")
    plt.plot(df["Time_ms"], df["enc_Pos_mm"], label="Encoder Position")
    plt.xlabel("Time [ms]")
    plt.ylabel("Position [mm]")
    plt.legend()
    plt.grid(True)

    plt.suptitle(title)
    plt.tight_layout()
    plt.show()

    return df
