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
def plot_results(data_or_path, title="S-curve Motion", **meta):
    """
    data_or_path : list (data_log) 또는 str (CSV 경로)
    - list: run_motor_scurve() 반환값
    - str : 저장된 CSV 파일 경로
    """

    if isinstance(data_or_path, str):
        df = pd.read_csv(data_or_path)
    elif isinstance(data_or_path, list):
        df = pd.DataFrame(data_or_path, columns=[
            "Time_ms", "com_Pos_mm", "enc_Pos_mm",
            "com_Vel_mm_per_s", "enc_Vel_mm_per_s"
        ])
    else:
        raise ValueError(f"지원하지 않는 입력 타입: {type(data_or_path)}")

    plt.figure(figsize=(8, 6))

    # --- 속도 비교 ---
    plt.subplot(2, 1, 1)
    plt.plot(df["Time_ms"], df["com_Vel_mm_per_s"],
             label="Commanded Velocity [mm/s]", linestyle="--")
    plt.plot(df["Time_ms"], df["enc_Vel_mm_per_s"],
             label="Encoder Velocity [mm/s]", alpha=0.8)
    plt.ylabel("Velocity [mm/s]")
    plt.legend(); plt.grid(True)

    # --- 위치 비교 ---
    plt.subplot(2, 1, 2)
    plt.plot(df["Time_ms"], df["com_Pos_mm"], label="Commanded Position [mm]")
    plt.plot(df["Time_ms"], df["enc_Pos_mm"], label="Encoder Position [mm]", alpha=0.8)
    plt.xlabel("Time [ms]"); plt.ylabel("Position [mm]")
    plt.legend(); plt.grid(True)

    if meta:
        sub = ", ".join(f"{k}={v}" for k, v in meta.items())
        plt.suptitle(f"{title} | {sub}")
    else:
        plt.suptitle(title)

    plt.tight_layout()
    plt.show()
