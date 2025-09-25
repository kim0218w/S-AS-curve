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
    그래프에 사용된 DataFrame을 그대로 CSV로 저장.
    - list가 들어오면 표준 컬럼으로 변환
    - 첫 줄에 메타데이터 주석
    - com_Vel_mm_per_s, enc_Vel_raw은 제거
    """
    if isinstance(df_or_list, list):
        df = pd.DataFrame(df_or_list, columns=[
            "Time_ms", "com_Pos_mm", "enc_Pos_mm",
            "com_Vel_mm_per_s", "enc_Vel_raw"
        ])
    else:
        df = df_or_list.copy()

    # 불필요한 원시 컬럼 제거
    for c in ["com_Vel_mm_per_s", "enc_Vel_raw"]:
        if c in df.columns:
            df.drop(columns=c, inplace=True)

    os.makedirs("logs", exist_ok=True)
    filepath = os.path.join("logs", filename)

    # 내보낼 컬럼 (그래프에 쓰인 것 위주)
    preferred_cols = [
        "Time_ms", "Time_s",
        "com_Pos_mm", "enc_Pos_mm",
        "com_RPM",    # 명령 속도 (그래프 그대로)
        "enc_RPM"     # 보정된 엔코더 속도 (그래프 그대로)
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


# -------------------- 실제 데이터 기반 플롯 --------------------
def plot_results(
    data_log,
    title="S-Curve Motion",
    filename="scurve_run.csv",

    pitch_mm=5.0,                 # 1 rev 당 이송(mm)
    smooth_method="savgol",       # "savgol" | "butter" | "none"
    smooth_ms=30,                 # short: 20~40, mid: 60~100, long: 100~150
    polyorder=3,
    reverse_sign=False,

    # 추세 맞춤 옵션
    align_to_command=True,        # 교차상관으로 enc↔com 시간정렬
    max_lag_ms=200,               # 지연 탐색 범위(±ms)
    fit_mode="scale",             # "scale" | "scale+offset"
    **meta
):
    """
    - 속도는 RPM 단위, com/enc 함께 표시
    - enc_RPM은 (필터→부호옵션) 후 com_RPM에 '지연 정렬 + 선형 보정'으로 추세를 맞춤
    - 그래프에 사용된 com_RPM/enc_RPM 그대로 CSV 저장(첫 줄에 메타데이터)
    """
    # 1) DataFrame 구성
    df = pd.DataFrame(data_log, columns=[
        "Time_ms", "com_Pos_mm", "enc_Pos_mm", "com_Vel_mm_per_s", "enc_Vel_raw"
    ])
    df["Time_s"] = df["Time_ms"] / 1000.0
    if len(df) < 5:
        raise ValueError("데이터가 너무 적습니다.")

    # 2) 명령 속도 -> RPM
    if pitch_mm:
        df["com_RPM"] = (df["com_Vel_mm_per_s"] / pitch_mm) * 60.0
    else:
        df["com_RPM"] = np.nan

    # 3) 엔코더 위치 -> 속도(mm/s)
    t = df["Time_s"].to_numpy()
    pos = df["enc_Pos_mm"].to_numpy()
    dt = float(np.median(np.diff(t)))
    if not np.isfinite(dt) or dt <= 0:
        raise ValueError("시간 축이 올바르지 않습니다.")
    fs = 1.0 / dt

    if smooth_method == "savgol":
        win = max(5, int(round((smooth_ms/1000.0)*fs)) | 1)
        win = min(win, len(pos)-(1-len(pos)%2))
        if win < 5: win = 5
        if win % 2 == 0: win += 1
        vel_mm_s = savgol_filter(pos, window_length=win, polyorder=polyorder, deriv=1, delta=dt)
    elif smooth_method == "butter":
        cutoff_hz = 10.0
        from scipy.signal import butter, filtfilt
        b,a = butter(N=3, Wn=cutoff_hz/(fs*0.5), btype="low")
        pos_f = filtfilt(b,a,pos,method="gust")
        vel_mm_s = np.gradient(pos_f, t)
    else:
        vel_mm_s = np.gradient(pos, t)

    if reverse_sign:
        vel_mm_s = -vel_mm_s

    # 4) mm/s -> RPM (raw)
    enc_rpm_raw = (vel_mm_s / pitch_mm) * 60.0 if pitch_mm else np.full_like(vel_mm_s, np.nan)

    # 5) (선택) 시간 지연 정렬 + 선형 보정으로 추세 맞춤
    enc_rpm = enc_rpm_raw.copy()
    if align_to_command and np.isfinite(enc_rpm).all():
        com = df["com_RPM"].to_numpy()

        # 에지 잡음 영향 줄이기: 앞/뒤 3*win 구간 제외하고 상관 계산
        trim = max(3, int(round((smooth_ms/1000.0)*fs)))
        x = com.copy()
        y = enc_rpm.copy()
        if trim*2 < len(x):
            x[:trim] = x[trim]
            x[-trim:] = x[-trim-1]
            y[:trim] = y[trim]
            y[-trim:] = y[-trim-1]

        # 교차상관으로 지연 추정 (±max_lag_ms 한정)
        max_lag = int(round((max_lag_ms/1000.0) * fs))
        x0 = x - np.nanmean(x)
        y0 = y - np.nanmean(y)
        corr = np.correlate(y0, x0, mode="full")      # y를 x에 정렬시키기
        lags = np.arange(-len(x0)+1, len(x0))
        # 허용 구간 내 최대 상관
        mask = (lags >= -max_lag) & (lags <= max_lag)
        lag_samples = int(lags[mask][np.argmax(corr[mask])])  # >0 이면 y가 x보다 "앞서 있음"
        # y를 x에 맞게 shift (enc를 뒤/앞으로 이동)
        if lag_samples != 0:
            enc_rpm = np.roll(enc_rpm, -lag_samples)
            # 롤된 에지 NaN 처리
            if lag_samples > 0:
                enc_rpm[-lag_samples:] = np.nan
            else:
                enc_rpm[:-lag_samples] = np.nan

        # 선형 보정 (enc ≈ a*enc + b -> com) : 유효 구간에서 계산
        valid = np.isfinite(enc_rpm) & np.isfinite(com)
        if valid.sum() > 5:
            X = enc_rpm[valid]
            Y = com[valid]
            if fit_mode == "scale+offset":
                A = np.vstack([X, np.ones_like(X)]).T
                a,b = np.linalg.lstsq(A, Y, rcond=None)[0]
            else:  # "scale"
                a = (X @ Y) / (X @ X) if np.any(X) else 1.0
                b = 0.0
            enc_rpm = a*enc_rpm + b

    # 6) DataFrame에 그래프용 시리즈 저장
    df["enc_RPM"] = enc_rpm

    # 7) Plot
    plt.figure(figsize=(8, 6))
    # (1) 속도
    plt.subplot(2,1,1)
    plt.plot(df["Time_ms"], df["com_RPM"], label="Commanded (RPM)", linestyle="--")
    plt.plot(df["Time_ms"], df["enc_RPM"], label=f"Encoder (RPM, aligned/{fit_mode})")
    plt.ylabel("Speed [RPM]")
    plt.legend(); plt.grid(True)
    y_max = np.nanmax([df["com_RPM"].max(), df["enc_RPM"].max()])
    y_min = np.nanmin([df["com_RPM"].min(), df["enc_RPM"].min()])
    plt.ylim((y_min*1.1 if y_min < 0 else 0), y_max*1.1 if np.isfinite(y_max) else None)

    # (2) 위치
    plt.subplot(2,1,2)
    plt.plot(df["Time_ms"], df["com_Pos_mm"], label="Commanded Position")
    plt.plot(df["Time_ms"], df["enc_Pos_mm"], label="Encoder Position")
    plt.xlabel("Time [ms]"); plt.ylabel("Position [mm]")
    plt.legend(); plt.grid(True)

    plt.suptitle(title); plt.tight_layout(); plt.show()

    # 8) CSV 자동 저장(그래프에 사용된 com_RPM/enc_RPM 그대로 저장)
    save_csv(df, filename=filename, **meta)
    return df
