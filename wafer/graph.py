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
            "Time_ms", "com_Pos_deg", "enc_Pos_deg",
            "com_Vel_deg_per_s", "enc_Vel_deg_per_s"
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

    df = pd.DataFrame(data_log, columns=[
        "Time_ms", "com_Pos_deg", "enc_Pos_deg",
        "com_Vel_deg_per_s", "enc_Vel_raw"
    ])

    # 시간 [s]
    df["Time_s"] = df["Time_ms"] / 1000.0

    # --- 윈도우 기반 속도 추정 (encoder position -> velocity) ---
    # enc_Pos_deg를 roll_window 샘플 차분으로 속도로 계산
    if roll_window > 1:
        df["enc_Vel_deg_per_s"] = (
            df["enc_Pos_deg"].diff(roll_window) / df["Time_s"].diff(roll_window)
        )
        # 중간 NaN 보간
        df["enc_Vel_deg_per_s"] = df["enc_Vel_deg_per_s"].interpolate().fillna(method="bfill").fillna(method="ffill")
    else:
        df["enc_Vel_deg_per_s"] = df["enc_Pos_deg"].diff() / df["Time_s"].diff()

    # --- 플롯 ---
    plt.figure(figsize=(10, 5))

    # 속도 곡선
    plt.subplot(2, 1, 1)
    plt.plot(df["Time_s"], df["com_Vel_deg_per_s"], "--", label="com_vel[deg/s]")
    plt.plot(df["Time_s"], df["enc_Vel_deg_per_s"], label=f"enc_vel[deg/s] (avg win={roll_window})")
    plt.ylabel("Angular Velocity [deg/s]")
    plt.legend()
    plt.grid(True)

    # 위치 곡선
    plt.subplot(2, 1, 2)
    plt.plot(df["Time_s"], df["com_Pos_deg"], label="com_pos[deg]")
    plt.plot(df["Time_s"], df["enc_Pos_deg"], label="enc_pos[deg]")
    plt.xlabel("Time [s]")
    plt.ylabel("Angle [deg]")
    plt.legend()
    plt.grid(True)

    plt.suptitle(f"{title} | motor={motor_id}, steps={steps}, shape={shape}")
    plt.tight_layout()
    plt.show()
