import os
import csv
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter, butter, filtfilt

# -------------------- CSV 저장 --------------------
def save_csv(data_log, filename="scurve_run.csv", pitch_mm=5.0, **meta):
    """
    enc_vel을 RPS 값으로 저장합니다.
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
            "com_Vel_mm_per_s", "enc_Vel_RPS"  # enc_vel은 RPS로 기록
        ])
        for row in data_log:
            Time_ms, com_Pos_mm, enc_Pos_mm, com_Vel_mm_per_s, enc_Vel_raw = row
            # enc_Vel_raw(mm/s) -> RPS
            enc_Vel_RPS = (enc_Vel_raw / pitch_mm) if (pitch_mm and pitch_mm != 0) else np.nan
            writer.writerow([Time_ms, com_Pos_mm, enc_Pos_mm, com_Vel_mm_per_s, enc_Vel_RPS])
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
    smooth_method="savgol",# "savgol" | "butter" | "regress"
    smooth_ms=80,          # 대역평활 기준 창 길이(ms) ≈ 50~150ms 권장
    polyorder=3            # savgol 다항 차수(2~3)
):
    """
    그래프는 RPS 단위로 그립니다.
    - com_Vel_mm_per_s → com_RPS
    - enc_Pos_mm → 매끄럽게 미분해 enc_RPS
    - 필요시 RPM으로 바꿔 보고 싶으면 RPS*60 쓰면 됩니다.
    """

    # DataFrame 변환
    df = pd.DataFrame(data_log, columns=[
        "Time_ms", "com_Pos_mm", "enc_Pos_mm", "com_Vel_mm_per_s", "enc_Vel_raw"
    ])
    df["Time_s"] = df["Time_ms"] / 1000.0

    # 표본 간격 추정
    if len(df) < 5:
        raise ValueError("데이터가 너무 적습니다.")
    dt = np.median(np.diff(df["Time_s"]))  # seconds/sample
    if not np.isfinite(dt) or dt <= 0:
        raise ValueError("시간 축이 올바르지 않습니다.")
    fs = 1.0 / dt  # Hz

    # 명령 속도(mm/s) -> RPS
    if pitch_mm and pitch_mm != 0:
        df["com_RPS"] = df["com_Vel_mm_per_s"] / pitch_mm
    else:
        df["com_RPS"] = np.nan

    # ----- 엔코더 위치 기반 속도(RPS) 추정 -----
    pos = df["enc_Pos_mm"].to_numpy()

    if smooth_method == "savgol":
        # Savitzky–Golay로 위치를 직접 미분: 1차 미분 = mm/s
        win_len_samples = max(5, int(round((smooth_ms/1000.0) * fs)) | 1)  # 홀수
        # 창 길이가 데이터보다 길면 줄이기
        win_len_samples = min(win_len_samples, len(pos) - (1 - len(pos)%2))
        if win_len_samples < 5: win_len_samples = 5
        if win_len_samples % 2 == 0: win_len_samples += 1
        vel_mm_s = savgol_filter(pos, window_length=win_len_samples, polyorder=polyorder, deriv=1, delta=dt)
    elif smooth_method == "butter":
        # 저역통과 후 미분 (권장 컷오프: 5~15 Hz 근처 시도)
        cutoff_hz = max(2.0, 10.0)  # 필요시 조정
        b, a = butter(N=3, Wn=cutoff_hz/(fs*0.5), btype='low')
        pos_f = filtfilt(b, a, pos, method="gust")
        vel_mm_s = np.gradient(pos_f, df["Time_s"])  # 안정적 수치미분
    elif smooth_method == "regress":
        # 창 회귀(선형회귀)로 기울기(mm/s) 추정
        win_len_samples = max(5, int(round((smooth_ms/1000.0) * fs)))
        if win_len_samples % 2 == 0: win_len_samples += 1
        half = win_len_samples // 2
        vel_mm_s = np.full_like(pos, np.nan, dtype=float)
        t = df["Time_s"].to_numpy()
        for i in range(half, len(pos)-half):
            ti = t[i-half:i+half+1]
            yi = pos[i-half:i+half+1]
            # y = a*t + b → a가 속도(mm/s)
            A = np.vstack([ti, np.ones_like(ti)]).T
            a, b = np.linalg.lstsq(A, yi, rcond=None)[0]
            vel_mm_s[i] = a
        # 양끝 보간
        s = pd.Series(vel_mm_s).interpolate().bfill().ffill().to_numpy()
        vel_mm_s = s
    else:
        raise ValueError("smooth_method는 'savgol' | 'butter' | 'regress' 중 하나여야 합니다.")

    # mm/s -> RPS
    df["enc_RPS"] = (vel_mm_s / pitch_mm) if (pitch_mm and pitch_mm != 0) else np.nan

    # (선택) 펄스 Hz 및 RPM도 보고 싶다면
    steps_per_rev = (360.0/step_angle_deg) * microstep  # 보통 200*microstep
    df["enc_RPM"] = df["enc_RPS"] * 60.0
    df["Pulse_Hz"] = df["enc_RPS"] * steps_per_rev
    df["com_RPM"]  = df["com_RPS"] * 60.0

    # ----- Plot: 고주파 시야 줄이기 위해 표시용 다운샘플(선택) -----
    # 시각화 시 흔들림이 심하면 보기용으로만 다운샘플
    show_every = max(1, int(fs * 0.01))  # ~10ms당 1점 표시
    idx = np.arange(0, len(df), show_every)

    # 그래프 (RPS 기준)
    plt.figure(figsize=(8, 5))
    plt.plot(df["Time_ms"].iloc[idx], df["com_RPS"].iloc[idx], label="Commanded (RPS)", linestyle="--")
    plt.plot(df["Time_ms"].iloc[idx], df["enc_RPS"].iloc[idx], label=f"Encoder (RPS) [{smooth_method}, ~{smooth_ms}ms]")
    plt.xlabel("Time [ms]")
    plt.ylabel("Speed [RPS]")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    return df
