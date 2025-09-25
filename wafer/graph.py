import os
import csv
import pandas as pd
import matplotlib.pyplot as plt

# -------------------- CSV 저장 --------------------
def save_csv(data_log, filename="scurve_run.csv", step_angle=1.8, microstep=16, pitch_mm=5, **meta):
    """
    enc_vel을 mm/s 대신 RPS 값으로 저장
    step_angle: 스텝 각도 (°/step)
    microstep: 마이크로스텝 설정 값
    pitch_mm: 1회전 당 이동 거리(mm) (리드스크류 pitch 등)
    """
    os.makedirs("logs", exist_ok=True)
    filepath = os.path.join("logs", filename)

    with open(filepath, "w", newline="") as f:
        writer = csv.writer(f)
        # 헤더
        writer.writerow([
            "Time_ms", "com_Pos_mm", "enc_Pos_mm",
            "com_Vel_mm_per_s", "enc_Vel_RPS"
        ])
        
        for row in data_log:
            Time_ms, com_Pos_mm, enc_Pos_mm, com_Vel_mm_per_s, enc_Vel_raw = row

            # enc_Vel_raw(mm/s) → RPS 변환
            enc_Vel_RPS = enc_Vel_raw / pitch_mm  # pitch_mm(mm/회전)으로 나눠 회전수(/s)
            
            writer.writerow([
                Time_ms, com_Pos_mm, enc_Pos_mm,
                com_Vel_mm_per_s, enc_Vel_RPS
            ])

    print(f"-- 실행 완료! CSV 저장: {filepath} --")
    return filepath



# -------------------- 결과 플로팅 --------------------
def plot_results(data_log, title="S-Curve Motion", motor_id=None, steps=None, shape=None,
                 roll_window=20, step_angle=1.8, microstep=16, pitch_mm=5):
    """
    roll_window: 속도 계산용 이동 평균 윈도우 크기
    step_angle: 스텝 각도 (°/step)
    microstep: 마이크로스텝 설정 값 (예: 16)
    pitch_mm: 리드스크류 리드(mm) 또는 1회전 당 이송 거리
    """

    # DataFrame 변환
    df = pd.DataFrame(data_log, columns=[
        "Time_ms", "com_Pos_mm", "enc_Pos_mm",
        "com_Vel_mm_per_s", "enc_Vel_raw"
    ])

    # 시간 [s]
    df["Time_s"] = df["Time_ms"] / 1000.0

    # --- 엔코더 속도 추정 (mm/s) ---
    if roll_window > 1:
        df["enc_Vel_mm_per_s"] = (
            df["enc_Pos_mm"].diff(roll_window) / df["Time_s"].diff(roll_window)
        )
        df["enc_Vel_mm_per_s"] = df["enc_Vel_mm_per_s"].interpolate().fillna(method="bfill").fillna(method="ffill")
    else:
        df["enc_Vel_mm_per_s"] = df["enc_Pos_mm"].diff() / df["Time_s"].diff()

    # --- mm/s → 회전수(RPS) 변환 ---
    df["enc_RPS"] = df["enc_Vel_mm_per_s"] / pitch_mm   # pitch_mm = 1회전 당 mm 이동량
    df["enc_RPM"] = df["enc_RPS"] * 60

    # --- 공식 기반 Motor Speed (RPM) ---
    # Pulse Frequency = RPS * (360 / step_angle) * microstep
    df["Pulse_Hz"] = df["enc_RPS"] * (360.0 / step_angle) * microstep
    df["Motor_RPM"] = (df["Pulse_Hz"] * 60.0) / (step_angle * microstep)

    # --- Figure & Subplot ---
    plt.figure(figsize=(8, 6))

    # (1) 속도 그래프
    plt.subplot(3, 1, 1)
    plt.plot(df["Time_ms"], df["com_Vel_mm_per_s"], label="Commanded Velocity", linestyle="--")
    plt.plot(df["Time_ms"], df["enc_Vel_mm_per_s"], label=f"Encoder Velocity (win={roll_window})", alpha=0.8)
    plt.ylabel("Velocity [mm/s]")
    plt.legend()
    plt.grid()

    # (2) 위치 그래프
    plt.subplot(3, 1, 2)
    plt.plot(df["Time_ms"], df["com_Pos_mm"], label="Commanded Position")
    plt.plot(df["Time_ms"], df["enc_Pos_mm"], label="Encoder Position", alpha=0.8)
    plt.xlabel("Time [ms]")
    plt.ylabel("Position [mm]")
    plt.legend()
    plt.grid()

    # (3) Motor Speed (RPM)
    plt.subplot(3, 1, 3)
    plt.plot(df["Time_ms"], df["Motor_RPM"], label="Motor Speed (RPM)", color="purple")
    plt.ylabel("Speed [RPM]")
    plt.xlabel("Time [ms]")
    plt.legend()
    plt.grid()

    plt.suptitle(f"{title} | motor={motor_id}, steps={steps}, shape={shape}")
    plt.tight_layout()
    plt.show()

    return df
