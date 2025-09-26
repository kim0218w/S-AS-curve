# graph.py
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
            "Time_ms", "com_Pos_deg", "pul_Pos_deg",
            "com_Vel_deg_per_s", "pul_Vel_deg_per_s"
        ])
        writer.writerows(data_log)
    print(f"-- 실행 완료! CSV 저장: {filepath} --")
    return filepath


# -------------------- 결과 플로팅 --------------------
def plot_results(data_log, title="S/AS-Curve Motion", motor_id=None, steps=None, shape=None,
                 roll_window=1):
    """
    roll_window: PUL 속도 계산용 이동 평균 윈도우 크기
                 (펄스 기반 속도는 이미 매끈하므로 기본=1)
    """

    df = pd.DataFrame(data_log, columns=[
        "Time_ms", "com_Pos_deg", "pul_Pos_deg",
        "com_Vel_deg_per_s", "pul_Vel_deg_per_s"
    ])

    # 시간 [s]
    df["Time_s"] = df["Time_ms"] / 1000.0

    # 이동 평균 smoothing
    if roll_window > 1:
        df["pul_Vel_deg_per_s"] = (
            df["pul_Vel_deg_per_s"].rolling(roll_window, center=True).mean()
        ).fillna(method="bfill").fillna(method="ffill")

    # === 오차 계산 ===
    df["pos_error"] = df["pul_Pos_deg"] - df["com_Pos_deg"]
    df["vel_error"] = df["pul_Vel_deg_per_s"] - df["com_Vel_deg_per_s"]

    # --- 플롯 ---
    plt.figure(figsize=(10, 8))

    # (1) 속도 그래프
    plt.subplot(3, 1, 1)
    plt.plot(df["Time_s"], df["com_Vel_deg_per_s"], "--", label="Commanded Vel [deg/s]")
    plt.plot(df["Time_s"], df["pul_Vel_deg_per_s"], label=f"PUL-based Vel [deg/s] (avg={roll_window})")
    plt.ylabel("Angular Velocity [deg/s]")
    plt.legend()
    plt.grid(True)

    # (2) 위치 그래프
    plt.subplot(3, 1, 2)
    plt.plot(df["Time_s"], df["com_Pos_deg"], label="Commanded Pos [deg]")
    plt.plot(df["Time_s"], df["pul_Pos_deg"], label="PUL-based Pos [deg]")
    plt.ylabel("Angle [deg]")
    plt.legend()
    plt.grid(True)

    # (3) 오차 그래프
    plt.subplot(3, 1, 3)
    plt.plot(df["Time_s"], df["pos_error"], label="Position Error [deg]")
    plt.plot(df["Time_s"], df["vel_error"], label="Velocity Error [deg/s]")
    plt.xlabel("Time [s]")
    plt.ylabel("Error")
    plt.legend()
    plt.grid(True)

    plt.suptitle(f"{title} | motor={motor_id}, steps={steps}, shape={shape}")
    plt.tight_layout()
    plt.show()
