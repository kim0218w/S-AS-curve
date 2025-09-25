import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def plot_scurve_params(result):
    print("[S-curve Parameters]")
    print(result, "\n")
    df = pd.DataFrame([result])
    print(df, "\n")

    total_time = result["total_time"]
    v_max = result["v_max"]
    t = np.linspace(0, total_time, 500)
    v = v_max * (np.sin(np.pi * t / total_time))**2

    plt.figure(figsize=(6, 4))
    plt.plot(t, v, label="S-curve velocity")
    plt.xlabel("Time [s]")
    plt.ylabel("Velocity [steps/s]")
    plt.grid()
    plt.legend()
    plt.show()


def plot_scurve_logs(data_log):
    df = pd.DataFrame(data_log, columns=[
        "Time_ms", "com_Pos_mm", "enc_Pos_mm",
        "com_Vel_mm_per_s", "enc_Vel_mm_per_s"
    ])

    plt.figure(figsize=(8, 4))
    plt.subplot(2, 1, 1)
    plt.plot(df["Time_ms"], df["com_Vel_mm_per_s"], "--", label="com_Vel")
    plt.plot(df["Time_ms"], df["enc_Vel_mm_per_s"], label="enc_Vel")
    plt.ylabel("Velocity [mm/s]")
    plt.legend()
    plt.grid()

    plt.subplot(2, 1, 2)
    plt.plot(df["Time_ms"], df["com_Pos_mm"], label="com_Pos")
    plt.plot(df["Time_ms"], df["enc_Pos_mm"], label="enc_Pos")
    plt.xlabel("Time [ms]")
    plt.ylabel("Position [mm]")
    plt.legend()
    plt.grid()
    plt.tight_layout()
    plt.show()
