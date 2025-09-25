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
    - enc_Vel_raw은 제거하지 않고 Encoder 속도(mm/s)로 저장
    """
    if isinstance(df_or_list, list):
        df = pd.DataFrame(df_or_list, columns=[
            "Time_ms", "com_Pos_mm", "enc_Pos_mm", "com_Vel_mm_per_s", "enc_Vel_raw"
        ])
    else:
        df = df_or_list.copy()

    os.makedirs("logs", exist_ok=True)
    filepath = os.path.join("logs", filename)

    preferred_cols = [
        "Time_ms", "Time_s",
        "com_Pos_mm", "enc_Pos_mm",
        "com_Vel_mm_per_s", "enc_Vel_mm_per_s"
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
def plot_results(csv_path="logs/scurve_run.csv", title="S-curve Motion"):
    """
    run_motor_scurve 에서 기록된 CSV를 불러와 Commanded vs Encoder 비교 그래프 출력.
    - Time_ms : ms 단위 시간
    - com_Vel_mm_per_s : 명령 속도 (mm/s)
    - enc_Vel_mm_per_s : EncoderVelEstimator 기반 추정 속도 (mm/s)
    - com_Pos_mm / enc_Pos_mm : 위치 비교
    """
    df = pd.read_csv(csv_path)

    plt.figure(figsize=(8, 6))

    # --- 속도 비교 ---
    plt.subplot(2, 1, 1)
    plt.plot(df["Time_ms"], df["com_Vel_mm_per_s"],
             label="Commanded (mm/s)", linestyle="--")
    plt.plot(df["Time_ms"], df["enc_Vel_mm_per_s"],
             label="Encoder (mm/s, filtered)", alpha=0.8)
    plt.ylabel("Velocity [mm/s]")
    plt.legend()
    plt.grid(True)

    # --- 위치 비교 ---
    plt.subplot(2, 1, 2)
    plt.plot(df["Time_ms"], df["com_Pos_mm"], label="Commanded Position")
    plt.plot(df["Time_ms"], df["enc_Pos_mm"], label="Encoder Position", alpha=0.8)
    plt.xlabel("Time [ms]")
    plt.ylabel("Position [mm]")
    plt.legend()
    plt.grid(True)

    plt.suptitle(title)
    plt.tight_layout()
    plt.show()
