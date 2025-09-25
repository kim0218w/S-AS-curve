import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
try:
    from scipy.signal import savgol_filter, butter, filtfilt
    _HAS_SCIPY = True
except Exception:
    _HAS_SCIPY = False


# -------------------- CSV 저장 --------------------
def save_csv(df_or_list, filename="scurve_run.csv", **meta):
    """
    그래프에 사용된 DataFrame(df)을 그대로 CSV로 저장.
    - list가 들어오면 DataFrame으로 변환
    - 첫 줄에 메타데이터 주석
    - com_Vel_mm_per_s, enc_Vel_raw 제거
    """
    if isinstance(df_or_list, list):
        df = pd.DataFrame(df_or_list, columns=[
            "Time_ms", "com_Pos_mm", "enc_Pos_mm",
            "com_Vel_mm_per_s", "enc_Vel_raw"
        ])
    else:
        df = df_or_list.copy()

    # 불필요한 컬럼 제거
    for c in ["com_Vel_mm_per_s", "enc_Vel_raw"]:
        if c in df.columns:
            df.drop(columns=c, inplace=True)

    os.makedirs("logs", exist_ok=True)
    filepath = os.path.join("logs", filename)

    preferred_cols = [
        "Time_ms", "Time_s",
        "com_Pos_mm", "enc_Pos_mm",
        "com_RPM", "enc_RPM"
    ]
    cols = [c for c in preferred_cols if c in df.columns]
    other_cols = [c for c in df.columns if c not in cols]
    out_df = df[cols + other_cols]

    with open(filepath, "w", newline="") as f:
        if meta:
            meta_str = ", ".join(f"{k}={v}" for k, v in meta.items())
            f.write(f"# {meta_str}\n")
        out_df.to_csv(f, index=False)

    print(f"-- 실행 완료! CSV 저장: {filepath} --")
    return filepath


# -------------------- 데이터 플롯 --------------------
def plot_results(
    data_log,
    title="S-Curve Motion",
    filename="scurve_run.csv",
    pitch_mm=5.0,
    smooth_method="savgol",
    smooth_ms=30,
    polyorder=3,
    reverse_sign=False,
    align_to_command=True,
    max_lag_ms=200,
    fit_mode="scale",
    **meta
):
    """
    실제 데이터 기반 S-curve 그래프 (속도[RPM] + 위치[mm])
    - com_RPM: 명령 속도 (RPM)
    - enc_RPM: 보정된 엔코더 속도 (RPM)
    - com_Vel_mm_per_s, enc_Vel_raw은 저장에서 제외
    - CSV에는 그래프와 동일한 값 저장
    """
    # --- DataFrame 변환 ---
    df = pd.DataFrame(data_log, columns=[
        "Time_ms", "com_Pos_mm", "enc_Pos_mm", "com_Vel_mm_per_s", "enc_Vel_raw"
    ])
    df["Time_s"] = df["Time_ms"] / 1000.0

    # --- Commanded 속도 (mm/s → RPM) ---
    if pitch_mm:
        df["com_RPM"] = (df["com_Vel_mm_per_s"] / pitch_mm) * 60.0
    else:
        df["com_RPM"] = np.nan

    # --- Encoder 위치 → 속도(mm/s) ---
    t = df["Time_s"].to_numpy()
    pos = df["enc_Pos_mm"].to_numpy()
    dt = np.median(np.diff(t))
    fs = 1.0 / dt

    if smooth_method == "savgol":
        win = max(5, int(round((smooth_ms/1000.0)*fs)) | 1)
        win = min(win, len(pos)-(1-len(pos)%2))
        if win < 5: win = 5
        if win % 2 == 0: win += 1
        vel_mm_s = savgol_filter(pos, window_length=win,
                                 polyorder=polyorder, deriv=1, delta=dt)
    elif smooth_method == "butter":
        cutoff_hz = 10.0
        b,a = butter(N=3, Wn=cutoff_hz/(fs*0.5), btype="low")
        pos_f = filtfilt(b,a,pos,method="gust")
        vel_mm_s = np.gradient(pos_f, t)
    else:
        vel_mm_s = np.gradient(pos, t)

    if reverse_sign:
        vel_mm_s = -vel_mm_s

    # --- mm/s → RPM ---
    enc_rpm = (vel_mm_s / pitch_mm) * 60.0 if pitch_mm else np.nan
    com_rpm = df["com_RPM"].to_numpy()

    # --- 시간 지연 정렬 + 선형 보정 ---
    if align_to_command and np.isfinite(enc_rpm).all():
        max_lag = int(round((max_lag_ms/1000.0) * fs))
        x = com_rpm - np.mean(com_rpm)
        y = enc_rpm - np.mean(enc_rpm)
        corr = np.correlate(y, x, mode="full")
        lags = np.arange(-len(x)+1, len(x))
        mask = (lags >= -max_lag) & (lags <= max_lag)
        best_lag = lags[mask][np.argmax(corr[mask])]
        if best_lag != 0:
            enc_rpm = np.roll(enc_rpm, -best_lag)
            if best_lag > 0:
                enc_rpm[-best_lag:] = np.nan
            else:
                enc_rpm[:-best_lag] = np.nan

        # 스케일 보정
        valid = np.isfinite(enc_rpm) & np.isfinite(com_rpm)
        if valid.sum() > 5:
            X = enc_rpm[valid]
            Y = com_rpm[valid]
            if fit_mode == "scale+offset":
                A = np.vstack([X, np.ones_like(X)]).T
                a,b = np.linalg.lstsq(A, Y, rcond=None)[0]
            else:
                a = (X @ Y) / (X @ X) if np.any(X) else 1.0
                b = 0.0
            enc_rpm = a*enc_rpm + b

    df["enc_RPM"] = enc_rpm

    # --- Plot ---
    plt.figure(figsize=(8, 6))
    plt.subplot(2,1,1)
    plt.plot(df["Time_ms"], df["com_RPM"], label="Commanded (RPM)", linestyle="--")
    plt.plot(df["Time_ms"], df["enc_RPM"], label="Encoder (RPM, aligned)")
    plt.ylabel("Speed [RPM]")
    plt.legend(); plt.grid(True)
    y_max = np.nanmax([df["com_RPM"].max(), df["enc_RPM"].max()])
    plt.ylim(0, y_max*1.1 if np.isfinite(y_max) else None)

    plt.subplot(2,1,2)
    plt.plot(df["Time_ms"], df["com_Pos_mm"], label="Commanded Position")
    plt.plot(df["Time_ms"], df["enc_Pos_mm"], label="Encoder Position")
    plt.xlabel("Time [ms]"); plt.ylabel("Position [mm]")
    plt.legend(); plt.grid(True)

    plt.suptitle(title); plt.tight_layout(); plt.show()

    # --- CSV 자동 저장 ---
    save_csv(df, filename=filename, **meta)
    return df
