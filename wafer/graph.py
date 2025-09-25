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
def save_csv(df: pd.DataFrame, filename="scurve_run.csv", **meta):
    """
    그래프에 사용된 DataFrame(df)을 그대로 CSV로 저장합니다.
    motor_id, steps, shape 등의 추가 키워드 인자도 받아 에러 없이 동작합니다.
    """
    os.makedirs("logs", exist_ok=True)
    filepath = os.path.join("logs", filename)

    # 그래프에 쓰인 주요 컬럼만 우선 정리(존재하는 컬럼만 선택)
    preferred_cols = [
        "Time_ms", "Time_s",
        "com_Pos_mm", "enc_Pos_mm",
        "com_Vel_mm_per_s",
        "com_RPM", "enc_RPM"  # enc_RPM은 스케일 보정된 값
    ]
    cols = [c for c in preferred_cols if c in df.columns]
    # 나머지 컬럼이 더 있으면 보존
    other_cols = [c for c in df.columns if c not in cols]
    out_df = df[cols + other_cols]

    out_df.to_csv(filepath, index=False)
    print(f"-- 실행 완료! CSV 저장: {filepath} --")
    return filepath


# -------------------- 실제 데이터 기반 플롯 --------------------
def plot_results(
    data_log,
    title="S-Curve Motion",
    pitch_mm=5.0,
    smooth_method="savgol",  # "savgol" | "butter" | "none"
    smooth_ms=30,
    polyorder=3,
    reverse_sign=False,      # 필요시 True로 두면 속도 부호 반전(그래프 뒤집기)
    scale_enc_to_com=True    # enc_RPM을 com_RPM 최대값에 맞춰 스케일 보정
):
    """
    실제 데이터 기반 S-curve 그래프 (속도[RPM] + 위치[mm])
    - 그래프에 쓰인 com_RPM, enc_RPM(보정값)을 df에 저장하여 save_csv로 그대로 출력 가능
    - short 모션: smooth_ms=20~40 권장
    """
    # 입력 데이터 → DataFrame
    df = pd.DataFrame(data_log, columns=[
        "Time_ms", "com_Pos_mm", "enc_Pos_mm", "com_Vel_mm_per_s", "enc_Vel_raw"
    ])
    df["Time_s"] = df["Time_ms"] / 1000.0

    if len(df) < 3:
        raise ValueError("데이터가 너무 적습니다.")

    # 샘플 속도
    t = df["Time_s"].to_numpy()
    pos = df["enc_Pos_mm"].to_numpy()
    dt = np.median(np.diff(t))
    if not np.isfinite(dt) or dt <= 0:
        raise ValueError("시간 축이 올바르지 않습니다.")
    fs = 1.0 / dt

    # ----- Commanded 속도 (mm/s → RPM) -----
    if pitch_mm and pitch_mm != 0:
        df["com_RPM"] = (df["com_Vel_mm_per_s"] / pitch_mm) * 60.0
    else:
        df["com_RPM"] = np.nan

    # ----- Encoder 위치 기반 속도 추정 (mm/s) -----
    if smooth_method == "savgol":
        win_len = max(5, int(round((smooth_ms/1000.0) * fs)) | 1)
        win_len = min(win_len, len(pos) - (1 - len(pos) % 2))
        if win_len < 5: win_len = 5
        if win_len % 2 == 0: win_len += 1
        vel_mm_s = savgol_filter(
            pos, window_length=win_len, polyorder=polyorder, deriv=1, delta=dt
        )
    elif smooth_method == "butter":
        cutoff_hz = 10.0  # 필요시 5~10으로 조정
        b, a = butter(N=3, Wn=cutoff_hz/(fs*0.5), btype='low')
        pos_f = filtfilt(b, a, pos, method="gust")
        vel_mm_s = np.gradient(pos_f, df["Time_s"])
    else:
        vel_mm_s = np.gradient(pos, df["Time_s"])

    # 필요시 부호 반전(그래프가 뒤집혀 보일 때)
    if reverse_sign:
        vel_mm_s = -vel_mm_s

    # ----- mm/s → RPM (Encoder) -----
    if pitch_mm and pitch_mm != 0:
        df["enc_RPM"] = (vel_mm_s / pitch_mm) * 60.0
    else:
        df["enc_RPM"] = np.nan

    # ----- 스케일 보정: 추세만 맞추기 위해 enc_RPM을 com_RPM 최대값에 정합 -----
    if scale_enc_to_com:
        enc_max = np.nanmax(df["enc_RPM"].to_numpy())
        com_max = np.nanmax(df["com_RPM"].to_numpy())
        if np.isfinite(enc_max) and np.isfinite(com_max) and enc_max not in (0, None):
            scale = com_max / enc_max
            df["enc_RPM"] = df["enc_RPM"] * scale

    # ----- Plot -----
    plt.figure(figsize=(8, 6))

    # (1) 속도 그래프 (RPM)
    plt.subplot(2, 1, 1)
    plt.plot(df["Time_ms"], df["com_RPM"], label="Commanded (RPM)", linestyle="--")
    plt.plot(df["Time_ms"], df["enc_RPM"], label=f"Encoder (RPM, scaled) [{smooth_method}]")
    plt.ylabel("Speed [RPM]")
    plt.legend()
    plt.grid(True)

    # y축 동기화
    rpm_max = np.nanmax([df["com_RPM"].max(), df["enc_RPM"].max()])
    rpm_min = np.nanmin([df["com_RPM"].min(), df["enc_RPM"].min()])
    # 대부분의 경우 위쪽만 중요하면 아래를 0으로 고정하려면 다음 줄로 교체:
    # rpm_min = 0
    plt.ylim(rpm_min * 1.1 if rpm_min < 0 else 0, rpm_max * 1.1 if np.isfinite(rpm_max) else None)

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
