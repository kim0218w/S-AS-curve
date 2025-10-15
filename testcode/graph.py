import os
import csv
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


def save_csv(data_log, filename="scurve_run.csv"):
    os.makedirs("logs", exist_ok=True)
    filepath = os.path.join("logs", filename)
    with open(filepath, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "Time_ms", "com_Pos_deg", "pul_Pos_deg",
            "com_Vel_deg_per_s", "pul_Vel_deg_per_s"
        ])
        writer.writerows(data_log)
    print(f"-- 실행 완료! CSV 저장: {filepath} --")
    return filepath


def moving_average(data, window=10):
    """이동 평균 필터"""
    if len(data) < window:
        return data
    return np.convolve(data, np.ones(window) / window, mode="valid")


def plot_results(data_log):
    df = pd.DataFrame(data_log, columns=[
        "Time_ms", "com_Pos_deg", "pul_Pos_deg",
        "com_Vel_deg_per_s", "pul_Vel_deg_per_s"
    ])

    time = df["Time_ms"].to_numpy() / 1000.0  # 초 단위
    com_vel = df["com_Vel_deg_per_s"].to_numpy()
    pul_vel = df["pul_Vel_deg_per_s"].to_numpy()
    com_pos = df["com_Pos_deg"].to_numpy()
    pul_pos = df["pul_Pos_deg"].to_numpy()

    # 이동 평균 적용 (속도만)
    pul_vel_filtered = moving_average(pul_vel, window=10)
    time_filtered = time[:len(pul_vel_filtered)]

    plt.figure(figsize=(8, 6))

    # ---------------- Velocity ----------------
    plt.subplot(2, 1, 1)
    plt.plot(time, com_vel, "--", label="Commanded Vel [deg/s]")
    plt.plot(time_filtered, pul_vel_filtered, label="PUL-based Vel [deg/s] (MA filtered)", alpha=0.8)
    plt.ylabel("Angular Velocity [deg/s]")
    plt.legend()
    plt.grid()

    # ---------------- Position ----------------
    plt.subplot(2, 1, 2)
    plt.plot(time, com_pos, label="Commanded Pos [deg]")
    plt.plot(time, pul_pos, label="PUL-based Pos [deg]", alpha=0.8)
    plt.xlabel("Time [s]")
    plt.ylabel("Angle [deg]")
    plt.legend()
    plt.grid()

    plt.tight_layout()
    plt.show()
