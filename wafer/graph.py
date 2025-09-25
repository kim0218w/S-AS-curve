import os
import csv
import pandas as pd
import matplotlib.pyplot as plt

# -------------------- CSV 저장 --------------------
def save_csv(data_log, filename="scurve_run.csv", **meta):
    os.makedirs("logs", exist_ok=True)
    filepath = os.path.join("logs", filename)
    with open(filepath, "w", newline="") as f:
        writer = csv.writer(f)
        # 헤더
        writer.writerow([
            "Time_ms", "com_Pos_mm", "enc_Pos_mm",
            "com_Vel_mm_per_s", "enc_Vel_raw"
        ])
        writer.writerows(data_log)
    print(f"-- 실행 완료! CSV 저장: {filepath} --")
    return filepath


# -------------------- 결과 플로팅 --------------------
def plot_results(data_log, title="S-Curve Motion", motor_id=None, steps=None, shape=None,
                 roll_window=20):
    """
    roll_window: 속도 계산용 이동 평균 윈도우 크기 (샘플 개수 기준)
                 예: 20 → 약 20ms~50ms smoothing 효과
    """

    # DataFrame 변환
    df = pd.DataFrame(data_log, columns=[
        "Time_ms", "com_Pos_mm", "enc_Pos_mm",
        "com_Vel_mm_per_s", "enc_Vel_raw"
    ])

    # 시간 [s]
    df["Time_s"] = df["Time_ms"] / 1000.0

    # --- 윈도우 기반 속도 추정 (encoder position -> velocity) ---
    if roll_window > 1:
        df["enc_Vel_mm_per_s"] = (
            df["enc_Pos_mm"].diff(roll_window) / df["Time_s"].diff(roll_window)
        )
        df["enc_Vel_mm_per_s"] = df["enc_Vel_mm_per_s"].interpolate().fillna(method="bfill").fillna(method="ffill")
    else:
        df["enc_Vel_mm_per_s"] = df["enc_Pos_mm"].diff() / df["Time_s"].diff()

    # --- Figure & Subplot ---
    plt.figure(figsize=(8, 4))

    # (1) 속도 그래프
    plt.subplot(2, 1, 1)
    plt.plot(df["Time_ms"], df["com_Vel_mm_per_s"], label="Commanded Velocity", linestyle="--")
    plt.plot(df["Time_ms"], df["enc_Vel_mm_per_s"], label=f"Encoder Velocity (win={roll_window})", alpha=0.8)
    plt.ylabel("Velocity [mm/s]")
    plt.legend()
    plt.grid()

    # (2) 위치 그래프
    plt.subplot(2, 1, 2)
    plt.plot(df["Time_ms"], df["com_Pos_mm"], label="Commanded Position")
    plt.plot(df["Time_ms"], df["enc_Pos_mm"], label="Encoder Position", alpha=0.8)
    plt.xlabel("Time [ms]")
    plt.ylabel("Position [mm]")
    plt.legend()
    plt.grid()

    plt.suptitle(f"{title} | motor={motor_id}, steps={steps}, shape={shape}")
    plt.tight_layout()
    plt.show()
