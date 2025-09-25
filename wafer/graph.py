import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
try:
    from scipy.signal import butter, filtfilt, savgol_filter  # savgol은 옵션
    _HAS_SCIPY = True
except Exception:
    _HAS_SCIPY = False


# -------------------- CSV 저장 --------------------
def save_csv(df_or_list, filename="scurve_run.csv", **meta):
    """
    그래프에 사용된 DataFrame을 그대로 CSV로 저장.
    - list면 표준 컬럼으로 DataFrame 변환
    - 첫 줄에 메타데이터(# k=v, ...)
    - com_Vel_mm_per_s, enc_Vel_raw은 CSV에서 제거
    """
    if isinstance(df_or_list, list):
        df = pd.DataFrame(df_or_list, columns=[
            "Time_ms", "com_Pos_mm", "enc_Pos_mm", "com_Vel_mm_per_s", "enc_Vel_raw"
        ])
    else:
        df = df_or_list.copy()

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
            f.write("# " + ", ".join(f"{k}={v}" for k,v in meta.items()) + "\n")
        out_df.to_csv(f, index=False)

    print(f"-- 실행 완료! CSV 저장: {filepath} --")
    return filepath


# -------------------- 유틸 --------------------
def _butter_lowpass(x, fs, cutoff_hz=8.0, order=3):
    cutoff = max(0.1, min(cutoff_hz, fs*0.45)) / (fs*0.5)
    b,a = butter(order, cutoff, btype="low")
    return filtfilt(b,a,x,method="gust")

def _winsorize(x, p_low=1.0, p_high=99.0):
    lo, hi = np.nanpercentile(x, [p_low, p_high])
    return np.clip(x, lo, hi)


# -------------------- 실제 데이터 기반 플롯 --------------------
def plot_results(
    data_log,
    title="S-Curve Motion (short)",
    filename="scurve_run.csv",

    pitch_mm=5.0,                # 1 rev 당 mm
    method="butter",             # "butter" | "savgol" | "none"
    pos_cutoff_hz=8.0,           # 위치 필터 컷오프(Hz) - short면 5~10 권장
    vel_cutoff_hz=8.0,           # 속도 필터 컷오프(Hz)
    smooth_ms=40, polyorder=3,   # savgol 쓸 때만 사용
    reverse_sign=False,

    align_to_command=True,       # enc를 com에 시간 정렬
    max_lag_ms=300,              # lag 검색 범위(±ms)
    fit_mode="scale",            # "scale" | "scale+offset"
    winsor_pct=(1.0, 99.0),      # 이상치 클리핑 퍼센타일
    **meta
):
    """
    목표: enc_RPM이 com_RPM과 같은 '추세'로 보이게 정렬/보정.
    - 위치 -> 저역통과 -> 미분 -> 속도 저역통과 -> 이상치 클리핑
    - 교차상관으로 시간 정렬, 선형 보정(scale / scale+offset)
    - 그래프 값 그대로 CSV 저장(첫 줄 메타데이터)
    """
    # --- DataFrame 준비 ---
    df = pd.DataFrame(data_log, columns=[
        "Time_ms", "com_Pos_mm", "enc_Pos_mm", "com_Vel_mm_per_s", "enc_Vel_raw"
    ])
    df["Time_s"] = df["Time_ms"] / 1000.0
    if len(df) < 5:
        raise ValueError("데이터가 너무 적습니다.")

    t = df["Time_s"].to_numpy()
    pos = df["enc_Pos_mm"].to_numpy()
    dt = float(np.median(np.diff(t)))
    if not np.isfinite(dt) or dt <= 0:
        raise ValueError("시간 축이 올바르지 않습니다.")
    fs = 1.0 / dt

    # --- 명령 속도 -> RPM ---
    if pitch_mm:
        df["com_RPM"] = (df["com_Vel_mm_per_s"] / pitch_mm) * 60.0
    else:
        df["com_RPM"] = np.nan
    com_rpm = df["com_RPM"].to_numpy()

    # --- enc_vel 원본을 RPM으로 변환해 추가 ---
    if pitch_mm and "enc_Vel_raw" in df.columns:
        df["enc_RPM_raw"] = (df["enc_Vel_raw"] / pitch_mm) * 60.0
    else:
        df["enc_RPM_raw"] = np.nan

    # --- 위치 필터 + 미분 ---
    if method == "savgol":
        win = max(5, int(round((smooth_ms/1000.0)*fs)) | 1)
        win = min(win, len(pos)-(1-len(pos)%2))
        if win < 5: win = 5
        if win % 2 == 0: win += 1
        vel_mm_s = savgol_filter(pos, window_length=win, polyorder=polyorder, deriv=1, delta=dt)
    elif method == "butter":
        pos_f = _butter_lowpass(pos, fs, cutoff_hz=pos_cutoff_hz, order=3)
        vel_mm_s = np.gradient(pos_f, t)
    else:
        vel_mm_s = np.gradient(pos, t)

    if reverse_sign:
        vel_mm_s = -vel_mm_s

    # --- 속도 저역통과 + 이상치 클리핑 ---
    vel_f = _butter_lowpass(vel_mm_s, fs, cutoff_hz=vel_cutoff_hz, order=3)
    vel_f = _winsorize(vel_f, p_low=winsor_pct[0], p_high=winsor_pct[1])

    # --- mm/s -> RPM ---
    enc_rpm = (vel_f / pitch_mm) * 60.0 if pitch_mm else np.full_like(vel_f, np.nan)

    # --- 시간 정렬 + 선형 보정 ---
    if align_to_command and np.isfinite(enc_rpm).sum() > 5 and np.isfinite(com_rpm).sum() > 5:
        max_lag = int(round((max_lag_ms/1000.0) * fs))
        x = com_rpm - np.nanmean(com_rpm)
        y = enc_rpm - np.nanmean(enc_rpm)
        corr = np.correlate(y, x, mode="full")
        lags = np.arange(-len(x)+1, len(x))
        mask = (lags >= -max_lag) & (lags <= max_lag)
        best_lag = int(lags[mask][np.argmax(corr[mask])])

        if best_lag != 0:
            enc_rpm = np.roll(enc_rpm, -best_lag)
            if best_lag > 0:
                enc_rpm[-best_lag:] = np.nan
            else:
                enc_rpm[:-best_lag] = np.nan

        valid = np.isfinite(enc_rpm) & np.isfinite(com_rpm)
        if valid.sum() > 5:
            X = enc_rpm[valid]
            Y = com_rpm[valid]
            if fit_mode == "scale+offset":
                A = np.vstack([X, np.ones_like(X)]).T
                a,b = np.linalg.lstsq(A, Y, rcond=None)[0]
                enc_rpm = a*enc_rpm + b
            else:  # "scale"
                a = (X @ Y) / (X @ X) if np.any(X) else 1.0
                enc_rpm = a*enc_rpm

    # --- 그래프/CSV용 시리즈 저장 ---
    df["enc_RPM"] = enc_rpm

      # --- Plot ---
    plt.figure(figsize=(8, 6))

    # 속도
    plt.subplot(2,1,1)
    plt.plot(df["Time_ms"], df["com_RPM"], label="Commanded (RPM)", linestyle="--")
    plt.plot(df["Time_ms"], df["enc_RPM_raw"], label="Encoder RPM (raw)", alpha=0.5)
    plt.plot(df["Time_ms"], df["enc_RPM"], label="Encoder RPM (filtered/aligned)", alpha=0.9)
    plt.ylabel("Speed [RPM]")
    plt.legend(); plt.grid(True)

    # 위치
    plt.subplot(2,1,2)
    plt.plot(df["Time_ms"], df["com_Pos_mm"], label="Commanded Position")
    plt.plot(df["Time_ms"], df["enc_Pos_mm"], label="Encoder Position")
    plt.xlabel("Time [ms]"); plt.ylabel("Position [mm]")
    plt.legend(); plt.grid(True)

    plt.suptitle(title)
    plt.tight_layout()
    plt.show()

    # CSV (그래프 값 그대로, 첫 줄 메타데이터)
    save_csv(df, filename=filename, **meta)
    return df
