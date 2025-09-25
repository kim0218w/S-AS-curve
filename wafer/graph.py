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
            "Time_ms", "com_Pos_mm", "enc_Pos_mm",
            "com_Vel_mm_per_s", "enc_Vel_mm_per_s"
        ])
        writer.writerows(data_log)
    print(f"-- 실행 완료! CSV 저장: {filepath} --")
    return filepath


def plot_results(data_log, save_png=True):
    if not data_log:
        print("[WARN] 데이터가 비어 있어서 그래프를 그릴 수 없습니다.")
        return

    df = pd.DataFrame(data_log, columns=[
        "Time_ms", "com_Pos_mm", "enc_Pos_mm",
        "com_Vel_mm_per_s", "enc_Vel_mm_per_s"
    ])

    # ms → 초 단위 변환
    df["Time_s"] = df["Time_ms"] / 1000.0

    plt.figure(figsize=(10, 6))

    # 속도
    plt.subplot(2, 1, 1)
    plt.plot(df["Time_s"], df["com_Vel_mm_per_s"], label="command vel", linestyle="--")
    plt.plot(df["Time_s"], df["enc_Vel_mm_per_s"], label="encoder vel", alpha=0.8)
    plt.ylabel("Velocity [mm/s]")
    plt.legend()
    plt.grid()

    # 위치
    plt.subplot(2, 1, 2)
    plt.plot(df["Time_s"], df["com_Pos_mm"], label="command pos")
    plt.plot(df["Time_s"], df["enc_Pos_mm"], label="encoder pos", alpha=0.8)
    plt.xlabel("Time [s]")
    plt.ylabel("Position [mm]")
    plt.legend()
    plt.grid()

    plt.tight_layout()

    # PNG 저장 옵션
    if save_png:
        os.makedirs("logs", exist_ok=True)
        filepath = os.path.join("logs", "last_run.png")
        plt.savefig(filepath)
        print(f"-- 그래프 이미지 저장: {filepath} --")

    plt.show()
