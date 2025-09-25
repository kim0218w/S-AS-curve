import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

def plot_scurve_profile(total_time, v_max):
    """S-curve 속도 프로파일 그리기"""
    t = np.linspace(0, total_time, 500)
    v = v_max * (np.sin(np.pi * t / total_time))**2

    plt.figure(figsize=(6, 4))
    plt.plot(t, v, label="S-curve velocity")
    plt.xlabel("Time [s]")
    plt.ylabel("Velocity [steps/s]")
    plt.title("S-curve Motion Profile")
    plt.grid()
    plt.legend()
    plt.tight_layout()
    plt.show()


def plot_run_results(data_log):
    """실제 모터 실행 후 로그 데이터(DataFrame 또는 리스트)로 속도/위치 그래프 그리기"""
    if not isinstance(data_log, pd.DataFrame):
        df = pd.DataFrame(data_log, columns=[
            "Time_ms", "com_Pos_mm", "enc_Pos_mm",
            "com_Vel_mm_per_s", "enc_Vel_mm_per_s"
        ])
    else:
        df = data_log

    plt.figure(figsize=(8, 4))

    # 속도 그래프
    plt.subplot(2, 1, 1)
    plt.plot(df["Time_ms"], df["com_Vel_mm_per_s"], label="com_Vel_mm_per_s", linestyle="--")
    plt.plot(df["Time_ms"], df["enc_Vel_mm_per_s"], label="enc_Vel_mm_per_s", alpha=0.8)
    plt.ylabel("Velocity [mm/s]")
    plt.legend()
    plt.grid()

    # 위치 그래프
    plt.subplot(2, 1, 2)
    plt.plot(df["Time_ms"], df["com_Pos_mm"], label="com_Pos_mm")
    plt.plot(df["Time_ms"], df["enc_Pos_mm"], label="enc_Pos_mm", alpha=0.8)
    plt.xlabel("Time [ms]")
    plt.ylabel("Position [mm]")
    plt.legend()
    plt.grid()

    plt.tight_layout()
    plt.show()
