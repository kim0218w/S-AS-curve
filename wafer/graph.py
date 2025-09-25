import os
import csv
import pandas as pd
import matplotlib.pyplot as plt


def save_csv(data_log, filename_base="scurve_run",
             motor_id=None, steps=None, shape=None, v_max=None):
    os.makedirs("logs", exist_ok=True)

    # 파일 이름에 motor_id, steps, shape 반영
    extra = []
    if motor_id is not None:
        extra.append(f"motor{motor_id}")
    if steps is not None:
        extra.append(f"steps{steps}")
    if shape is not None:
        extra.append(shape)
    extra_str = "_" + "_".join(extra) if extra else ""

    filename = f"{filename_base}{extra_str}.csv"
    filepath = os.path.join("logs", filename)

    with open(filepath, "w", newline="") as f:
        writer = csv.writer(f)

        # -------- 메타데이터 기록 --------
        writer.writerow(["# Metadata"])
        writer.writerow(["motor_id", motor_id if motor_id is not None else "NA"])
        writer.writerow(["steps", steps if steps is not None else "NA"])
        writer.writerow(["shape", shape if shape is not None else "NA"])
        writer.writerow(["v_max [steps/s]", v_max if v_max is not None else "NA"])
        writer.writerow([])

        # -------- 데이터 헤더 및 로그 --------
        writer.writerow([
            "Time_ms", "com_Angle_deg", "enc_Angle_deg",
            "com_Vel_deg_per_s", "enc_Vel_deg_per_s"
        ])
        writer.writerows(data_log)

    print(f"-- 실행 완료! CSV 저장: {filepath} --")
    return filepath


def plot_results(data_log, save_png=True, title="Stepper Motor Motion",
                 motor_id=None, steps=None, shape=None, smooth_alpha=0.2):
    """
    smooth_alpha: EMA 필터 계수 (0~1), 작을수록 더 부드럽게, 기본=0.2
    """
    if not data_log:
        print("[WARN] 데이터가 비어 있어서 그래프를 그릴 수 없습니다.")
        return

    df = pd.DataFrame(data_log, columns=[
        "Time_ms", "com_Angle_deg", "enc_Angle_deg",
        "com_Vel_deg_per_s", "enc_Vel_deg_per_s"
    ])
    df["Time_s"] = df["Time_ms"] / 1000.0

    # -------------------- 속도 smoothing (EMA) --------------------
    if smooth_alpha and 0 < smooth_alpha < 1:
        df["enc_Vel_deg_per_s_smooth"] = df["enc_Vel_deg_per_s"].ewm(alpha=smooth_alpha).mean()
    else:
        df["enc_Vel_deg_per_s_smooth"] = df["enc_Vel_deg_per_s"]

    plt.figure(figsize=(12, 6))
    plt.suptitle(title, fontsize=14, fontweight="bold")

    # -------------------- 속도 --------------------
    plt.subplot(2, 1, 1)
    plt.plot(df["Time_s"], df["com_Vel_deg_per_s"],
             label="Commanded Angular Velocity", linestyle="--", linewidth=2)
    plt.plot(df["Time_s"], df["enc_Vel_deg_per_s_smooth"],
             label=f"Encoder Angular Velocity (EMA α={smooth_alpha})",
             alpha=0.9, linewidth=1.5)
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
        base = title.replace(" ", "_").replace("(", "_").replace(")", "_").lower()
        extra = []
        if motor_id is not None:
            extra.append(f"motor{motor_id}")
        if steps is not None:
            extra.append(f"steps{steps}")
        if shape is not None:
            extra.append(shape)
        extra_str = "_" + "_".join(extra) if extra else ""
        filepath = os.path.join("logs", f"{base}{extra_str}.png")
        plt.savefig(filepath)
        print(f"-- 그래프 이미지 저장: {filepath} --")

    plt.show()
