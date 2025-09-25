import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


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

    # 불필요한 원시 컬럼 제거
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
            f.write("# " + ", ".join(f"{k}={v}" for k, v in meta.items()) + "\n")
        out_df.to_csv(f, index=False)

    print(f"-- 실행 완료! CSV 저장: {filepath} --")
    return filepath


# -------------------- 실제 데이터 기반 플롯 --------------------
def plot_results(
    data_log,
    title="S-Curve Motion",
    filename="scurve_run.csv",
    pitch_mm=5.0,   # 1 rev 당 mm
    **meta
):
    """
    Commanded vs Encoder 속도를 직접 비교.
    - enc_RPM은 run loop에서 기록한 enc_Vel_raw(mm/s)를 변환해서 사용
    - 위치 기반 미분/필터링은 제거
    - 그래프와 CSV에는 com_RPM, enc_RPM만 기록
    """
    # --- DataFrame 준비 ---
    df = pd.DataFrame(data_log, columns=[
        "Time_ms", "com_Pos_mm", "enc_Pos_mm", "com_Vel_mm_per_s", "enc_Vel_raw"
    ])
    df["Time_s"] = df["Time_ms"] / 1000.0

    # --- 명령 속도 -> RPM ---
    if pitch_mm:
        df["com_RPM"] = (df["com_Vel_mm_per_s"] / pitch_mm) * 60.0
    else:
        df["com_RPM"] = np.nan

    # --- run loop 속도(enc_Vel_raw) -> RPM ---
    if pitch_mm:
        df["enc_RPM"] = (df["enc_Vel_raw"] / pitch_mm) * 60.0
    else:
        df["enc_RPM"] = np.nan

    # --- Plot ---
    plt.figure(figsize=(8, 6))

    # 속도 비교
    plt.subplot(2, 1, 1)
    plt.plot(df["Time_ms"], df["com_RPM"], label="Commanded (RPM)", linestyle="--")
    plt.plot(df["Time_ms"], df["enc_RPM"], label="Encoder (RPM, from loop)", alpha=0.8)
    plt.ylabel("Speed [RPM]")
    plt.legend()
    plt.grid(True)

    # 위치 비교
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

    # --- CSV 저장 (첫 줄에 메타데이터 포함) ---
    save_csv(df, filename=filename, **meta)
    return df
