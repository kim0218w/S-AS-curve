import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# -------------------- 파라미터 그래프 --------------------
def plot_scurve_params(result: dict):
    """
    result: {"total_steps": int, "v_max": float, "total_time": float}
    """
    print("[S-curve Parameters]")
    print(f"총 스텝 수 : {result['total_steps']} steps")
    print(f"최대 속도  : {result['v_max']:.2f} steps/s")
    print(f"총 이동 시간: {result['total_time']:.3f} s\n")

    df = pd.DataFrame([result])
    print(df, "\n")

    total_time = result["total_time"]
    v_max = result["v_max"]

    # 속도 프로파일
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


# -------------------- 실행 로그 그래프 --------------------
def plot_scurve_logs(data_log: list):
    """
    data_log: [[t_ms, com_pos_mm, enc_pos_mm, com_vel_mm, enc_vel_mm], ...]
    com_pos_mm / com_vel_mm 은 None 이거나 0 이어도 됨
    """
    df = pd.DataFrame(data_log, columns=[
        "Time_ms", "com_Pos_mm", "enc_Pos_mm",
        "com_Vel_mm_per_s", "enc_Vel_mm_per_s"
    ])

    plt.figure(figsize=(8, 4))

    # 속도 그래프
    plt.subplot(2, 1, 1)
    if "com_Vel_mm_per_s" in df:
        plt.plot(df["Time_ms"], df["com_Vel_mm_per_s"], "--", label="com_Vel")
    plt.plot(df["Time_ms"], df["enc_Vel_mm_per_s"], label="enc_Vel")
    plt.ylabel("Velocity [mm/s]")
    plt.legend()
    plt.grid()

    # 위치 그래프
    plt.subplot(2, 1, 2)
    if "com_Pos_mm" in df:
        plt.plot(df["Time_ms"], df["com_Pos_mm"], label="com_Pos")
    plt.plot(df["Time_ms"], df["enc_Pos_mm"], label="enc_Pos")
    plt.xlabel("Time [ms]")
    plt.ylabel("Position [mm]")
    plt.legend()
    plt.grid()

    plt.tight_layout()
    plt.show()
