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
              여기서 enc_Vel_raw는 mm/s 라고 가정
    pitch_mm: 1회전 당 이송 거리(mm)
    """
    os.makedirs("logs", exist_ok=True)
    filepath = os.path.join("logs", filename)
    with open(filepath, "w", newline="") as f:
        writer = csv.writer(f)
        # 헤더
        writer.writerow([
            "Time_ms", "com_Pos_mm", "enc_Pos_mm",
            "com_Vel_mm_per_s", "enc_Vel_RPM"  # enc_vel은 RPM으로 기록
        ])
        for row in data_log:
            Time_ms, com_Pos_mm, enc_Pos_mm, com_Vel_mm_per_s, enc_Vel_raw = row
            # enc_Vel_raw(mm/s) -> RPM
            enc_Vel_RPM = ((enc_Vel_raw / pitch_mm) * 60.0) if (pitch_mm and pitch_mm != 0) else np.nan
            writer.writerow([Time_ms, com_Pos_mm, enc_Pos_mm, com_Vel_mm_per_s, enc_Vel_RPM])
    print(f"-- 실행 완료! CSV 저장: {filepath} --")
    return filepath


# -------------------- 결과 플로팅 --------------------
def plot_results(
    data_log,
    title="S-Curve Motion",
    motor_id=None, steps=None, shape=None,
    pitch_mm=5.0,          # 1회전 당 이동(mm)
    step_angle_deg=1.8,    # 보통 1.8°
    microstep=16,          # 마이크로스텝
    smooth_method="savgol",# "savgol" | "butter" | "regress" | "none"
    smooth_ms=80,          # 평활 창 길이(ms)
    polyorder=3
):
    """
    그래프는 RPM 단위로 그립니다.
    - com_Vel_mm_per_s → com_RPM
    - enc_Pos_mm → 매끄럽게 미분해 enc_RPM
    """

    # ---------------- DataFrame ----------------
    df = pd.DataFrame(data_log, columns=[
        "Time_ms", "com_Pos_mm", "enc_Pos_mm", "com_Vel_mm_per_s", "enc_Vel_raw"
    ])
    df["Time_s"] = df["Time_ms"] / 1000.0

    if len(df) < 3:
        raise ValueError("데이터가 너무 적습니다.")

    # 표본 간격
    t = df["Time_s"].to_numpy()
    pos = df["enc_Pos_mm"].to_numpy()
    dt = np.median(np.diff(t))
    if not np.isfinite(dt) or dt <= 0:
        raise ValueError("시간 축이 올바르지 않습니다.")
    fs = 1.0 / dt

    # ----- 엔코더 위치 기반 속도(mm/s) 추정 -----
    if smooth_method == "savgol":
        win_len_samples = max(5, int(round((smooth_ms/1000.0) * fs)) | 1)  # 홀수
        win_len_samples = min(win_len_samples, len(pos) - (1 - len(pos) % 2))
        if win_len_samples < 5: win_len_samples = 5
        if win_len_samples % 2 == 0: win_len_samples += 1
        vel_mm_s = savgol_filter(
            pos, window_length=win_len_samples, polyorder=polyorder,
            deriv=1, delta=dt
        )
    elif smooth_method == "butter":
        cutoff_hz = 10.0  # 필요시 조정
        b, a = butter(N=3, Wn=cutoff_hz/(fs*0.5), btype='low')
        pos_f = filtfilt(b, a, pos, method="gust")
        vel_mm_s = np.gradient(pos_f, df["Time_s"])
    elif smooth_method == "regress":
        win_len_samples = max(5, int(round((smooth_ms/1000.0) * fs)))
        if win_len_samples % 2 == 0: win_len_samples += 1
        half = win_len_samples // 2
        vel_mm_s = np.full_like(pos, np.nan, dtype=float)
        for i in range(half, len(pos)-half):
            ti = t[i-half:i+half+1]
            yi = pos[i-half:i+half+1]
            A = np.vstack([ti, np.ones_like(ti)]).T
            a, b = np.linalg.lstsq(A, yi, rcond=None)[0]
            vel_mm_s[i] = a
        vel_mm_s = pd.Series(vel_mm_s).interpolate().bfill().ffill().to_numpy()
    else:
        vel_mm_s = np.gradient(pos, df["Time_s"])  # fallback

    # ----- mm/s -> RPM (부호 유지) -----
    if pitch_mm and pitch_mm != 0:
        df["enc_RPM"] = (vel_mm_s / pitch_mm) * 60.0
        df["com_RPM"] = (df["com_Vel_mm_per_s"] / pitch_mm) * 60.0
    else:
        df["enc_RPM"] = np.nan
        df["com_RPM"] = np.nan

    # ----- Plot -----
    plt.figure(figsize=(8, 6))

    # (1) 속도 그래프 (RPM)
    plt.subplot(2, 1, 1)
    plt.plot(df["Time_ms"], df["com_RPM"], label="Commanded (RPM)", linestyle="--")
    plt.plot(df["Time_ms"], df["enc_RPM"], label=f"Encoder (RPM) [{smooth_method}]")
    plt.ylabel("Speed [RPM]")
    plt.legend()
    plt.grid(True)

    # 여기서 y축 고정 (질문 주신 코드)
    rpm_min = 0
    rpm_max = max(df["enc_RPM"].max(), df["com_RPM"].max())
    plt.ylim(rpm_min, rpm_max * 1.1)

    # (2) 위치 그래프
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
