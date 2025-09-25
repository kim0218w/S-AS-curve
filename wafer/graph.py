import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
try:
    from scipy.signal import savgol_filter, butter, filtfilt
    _HAS_SCIPY = True
except Exception:
    _HAS_SCIPY = False


# -------------------- CSV 저장 --------------------
def save_csv(df_or_list, filename="scurve_run.csv", **meta):
    """
    그래프에 사용된 DataFrame(df)을 그대로 CSV로 저장합니다.
    - list가 들어오면 DataFrame으로 변환
    - motor_id, steps, shape 같은 메타데이터는 CSV의 첫 줄에 기록
    - com_Vel_mm_per_s, enc_Vel_raw 컬럼은 제거
    """
    if isinstance(df_or_list, list):
        df = pd.DataFrame(df_or_list, columns=[
            "Time_ms", "com_Pos_mm", "enc_Pos_mm",
            "com_Vel_mm_per_s", "enc_Vel_raw"
        ])
    else:
        df = df_or_list.copy()

    # 불필요한 컬럼 제거
    drop_cols = [c for c in ["com_Vel_mm_per_s", "enc_Vel_raw"] if c in df.columns]
    if drop_cols:
        df = df.drop(columns=drop_cols)

    os.makedirs("logs", exist_ok=True)
    filepath = os.path.join("logs", filename)

    # 우선순위 컬럼 정리
    preferred_cols = [
        "Time_ms", "Time_s",
        "com_Pos_mm", "enc_Pos_mm",
        "com_RPM", "enc_RPM"
    ]
    cols = [c for c in preferred_cols if c in df.columns]
    other_cols = [c for c in df.columns if c not in cols]
    out_df = df[cols + other_cols]

    # CSV 파일 저장 (첫 줄에 메타데이터 추가)
    with open(filepath, "w", newline="") as f:
        if meta:
            meta_str = ", ".join(f"{k}={v}" for k, v in meta.items())
            f.write(f"# {meta_str}\n")
        out_df.to_csv(f, index=False)

    print(f"-- 실행 완료! CSV 저장: {filepath} --")
    return filepath


# -------------------- 실제 데이터 기반 플롯 --------------------
def plot_results(
    data_log,
    title="S-Curve Motion",
    filename="scurve_run.csv",
    pitch_mm=5.0,
    smooth_method="savgol",
    smooth_ms=30,
    polyorder=3,
    reverse_sign=False,
    scale_enc_to_com=True,
    **meta
):
    """
    실제 데이터 기반 S-curve 그래프 (속도[RPM] + 위치[mm])
    실행 시 그래프 출력 + CSV 자동 저장 (첫 줄에 메타데이터 기록)
    """
    df = pd.DataFrame(data_log, columns=[
        "Time_ms", "com_Pos_mm", "enc_Pos_mm", "com_Vel_mm_per_s", "enc_Vel_raw"
    ])
    df["Time_s"] = df["Time_ms"] / 1000.0

    if len(df) < 3:
        raise ValueError("데이터가 너무 적습니다.")

    # Commanded 속도 → RPM
    if pitch_mm and pitch_mm != 0:
        df["com_RPM"] = (df["com_Vel_mm_per_s"] / pitch_mm) * 60.0
    else:
        df["com_RPM"] = np.nan

    # Encoder 속도 → RPM
    t = df["Time_s"].to_numpy()
    pos = df["enc_Pos_mm"].to_numpy()
    dt = np.median(np.diff(t))
    fs = 1.0 / dt

    if smooth_method == "savgol":
        win_len = max(5, int(round((smooth_ms/1000.0) * fs)) | 1)
        win_len = min(win_len, len(pos) - (1 - len(pos) % 2))
        if win_len < 5: win_len = 5
        if win_len % 2 == 0: win_len += 1
        vel_mm_s = savgol_filter(
            pos, window_length=win_len, polyorder=polyorder, deriv=1, delta=dt
        )
    elif smooth_method == "butter":
        cutoff_hz = 10.0
        b, a = butter(N=3, Wn=cutoff_hz/(fs*0.5), btype='low')
        pos_f = filtfilt(b, a, pos, method="gust")
        vel_mm_s = np.gradient(pos_f, df["Time_s"])
    else:
        vel_mm_s = np.gradient(pos, df["Time_s"])

    if reverse_sign:
        vel_mm_s = -vel_mm_s

    df["enc_RPM"] = (vel_mm_s / pitch_mm) * 60.0 if pitch_mm else np.nan

    if scale_enc_to_com:
        enc_max = np.nanmax(df["enc_RPM"].to_numpy())
        com_max = np.nanmax(df["com_RPM"].to_numpy())
        if np.isfinite(enc_max) and np.isfinite(com_max) and enc_max not in (0, None):
            scale = com_max / enc_max
            df["enc_RPM"] = df["enc_RPM"] * scale

    # ---- Plot ----
    plt.figure(figsize=(8, 6))

    plt.subplot(2, 1, 1)
    plt.plot(df["Time_ms"], df["com_RPM"], label="Commanded (RPM)", linestyle="--")
    plt.plot(df["Time_ms"], df["enc_RPM"], label=f"Encoder (RPM, scaled) [{smooth_method}]")
    plt.ylabel("Speed [RPM]")
    plt.legend()
    plt.grid(True)

    rpm_max = np.nanmax([df["com_RPM"].max(), df["enc_RPM"].max()])
    rpm_min = np.nanmin([df["com_RPM"].min(), df["enc_RPM"].min()])
    plt.ylim(rpm_min * 1.1 if rpm_min < 0 else 0,
             rpm_max * 1.1 if np.isfinite(rpm_max) else None)

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

    # ---- CSV 자동 저장 (첫 줄에 메타데이터 포함, com_Vel_mm_per_s/enc_Vel_raw 제거됨) ----
    save_csv(df, filename=filename, **meta)

    return df
