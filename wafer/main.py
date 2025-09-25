import os
import csv
import pandas as pd
import matplotlib.pyplot as plt


def save_csv(data_log, filename="scurve_run.csv"):
    os.makedirs("logs", exist_ok=True)
    filepath = os.path.join("logs", filename)
    with open(filepath, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "Time_ms", "com_Angle_deg", "enc_Angle_deg",
            "com_Vel_deg_per_s", "enc_Vel_deg_per_s"
        ])
        writer.writerows(data_log)
    print(f"-- 실행 완료! CSV 저장: {filepath} --")
    return filepath


def plot_results(data_log, save_png=True, title="Stepper Motor Motion (S/AS-Curve)"):
    if not data_log:
        print("[WARN] 데이터가 비어 있어서 그래프를 그릴 수 없습니다.")
        return

    df = pd.DataFrame(data_log, columns=[
        "Time_ms", "com_Angle_deg", "enc_Angle_deg",
        "com_Vel_deg_per_s", "enc_Vel_deg_per_s"
    ])
    df["Time_s"] = df["Time_ms"] / 1000.0

    plt.figure(figsize=(12, 6))
    plt.suptitle(title, fontsize=14, fontweight="bold")

    # -------------------- 속도 --------------------
    plt.subplot(2, 1, 1)
    plt.plot(df["Time_s"], df["com_Vel_deg_per_s"],
             label="Commanded Angular Velocity", linestyle="--", linewidth=2)
    plt.plot(df["Time_s"], df["enc_Vel_deg_per_s"],
             label="Encoder Angular Velocity", alpha=0.8, linewidth=1.5)
    plt.ylabel("Angular Velocity [deg/s]")
    plt.legend(loc="best")
    plt.grid(True, linestyle="--", alpha=0.6)

    # -------------------- 위치 --------------------
    plt.subplot(2, 1, 2)
    plt.plot(df["Time_s"], df["com_Angle_deg"],
             label="Commanded Angle", linewidth=2)
    plt.plot(df["Time_s"], df["enc_Angle_deg"],
             label="Encoder Angle", alpha=0.8, linewidth=1.5)
    plt.xlabel("Time [s]")
    plt.ylabel("Angle [deg]")
    plt.legend(loc="best")
    plt.grid(True, linestyle="--", alpha=0.6)

    plt.tight_layout(rect=[0, 0, 1, 0.96])

    if save_png:
        os.makedirs("logs", exist_ok=True)
        filepath = os.path.join("logs", "last_run.png")
        plt.savefig(filepath)
        print(f"-- 그래프 이미지 저장: {filepath} --")

    plt.show()
