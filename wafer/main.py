from encoder import (
    GPIOHelper, Encoder,
    MOTOR_STEP_PER_REV, MICROSTEP_SETTING, GEAR_RATIO,
    STEPS_PER_REV, DEG_PER_STEP
)
from scurve import run_motor_scurve, run_motor_ascurve, calc_scurve_params
from graph import save_csv, plot_results


def main():
    # -------------------- 설정값 출력 --------------------
    print("[MOTOR CFG] motor_step/rev =", MOTOR_STEP_PER_REV)
    print("[MOTOR CFG] microstep     =", MICROSTEP_SETTING)
    print("[MOTOR CFG] gear_ratio    =", GEAR_RATIO)
    print("[MOTOR CFG] STEPS_PER_REV =", STEPS_PER_REV)
    print(f"[MOTOR CFG] DEG_PER_STEP  = {DEG_PER_STEP:.6f} deg/step")
    print(f"[MOTOR CFG] 10000 steps   ≈ {10000*DEG_PER_STEP:.2f} deg")

    gpio = GPIOHelper()
    encoder = Encoder(gpio)

    try:
        mode = input("실행 모드 선택 (1: S-curve 실행, 2: AS-curve 실행, 3: 파라미터 계산): ").strip()

        if mode == "1":  # -------------------- S-curve 실행 --------------------
            motor_id = int(input("모터 선택 (17 또는 23): ").strip())
            v_max = float(input("Vmax 입력 [steps/s]: ").strip())
            move_steps = int(input("이동할 스텝 수 입력: ").strip())
            direction = input("모터 방향 입력 (f: forward / b: backward): ").strip().lower()
            shape = input("S-curve 형태 선택 (short/mid/long, 기본=mid): ").strip().lower() or "mid"

            data_log = run_motor_scurve(
                gpio, encoder, motor_id, direction, move_steps, v_max, shape
            )
            filepath = save_csv(data_log, motor_id=motor_id, steps=move_steps, shape=shape, v_max=v_max)
            plot_results(data_log, title=f"S-Curve Motion ({shape})",
                         motor_id=motor_id, steps=move_steps, shape=shape)

        elif mode == "2":  # -------------------- AS-curve 실행 --------------------
            motor_id = int(input("모터 선택 (17 또는 23): ").strip())
            v_max = float(input("Vmax 입력 [steps/s]: ").strip())
            move_steps = int(input("이동할 스텝 수 입력: ").strip())
            direction = input("모터 방향 입력 (f: forward / b: backward): ").strip().lower()
            shape = input("AS-curve 형태 선택 (short/mid/long, 기본=mid): ").strip().lower() or "mid"

            print(f"[INFO] AS-curve 실행 → steps={move_steps}, Vmax={v_max}, "
                  f"motor={motor_id}, dir={direction}, shape={shape}")

            # roll_window, smooth_alpha는 기본값 사용
            data_log = run_motor_ascurve(
                gpio, encoder, motor_id, direction, move_steps, v_max, shape
            )
            filepath = save_csv(data_log, motor_id=motor_id, steps=move_steps, shape=shape, v_max=v_max)
            plot_results(data_log, title=f"AS-Curve Motion ({shape})",
                         motor_id=motor_id, steps=move_steps, shape=shape)

        elif mode == "3":  # -------------------- 파라미터 계산 --------------------
            steps_in = input("총 스텝 수 입력 (또는 Enter): ").strip()
            vmax_in = input("최대 속도 입력 (steps/s, 또는 Enter): ").strip()
            t_in = input("총 이동 시간 입력 (초, 또는 Enter): ").strip()
            total_steps = int(steps_in) if steps_in else None
            v_max = float(vmax_in) if vmax_in else None
            total_time = float(t_in) if t_in else None
            calc_scurve_params(total_steps=total_steps, v_max=v_max, total_time=total_time)

        else:
            print("잘못된 모드 선택")

    except Exception as e:
        print(f"[ERROR] {e}")

    finally:
        try:
            encoder.stop()
        except Exception:
            pass
        try:
            gpio.cleanup()
        except Exception:
            pass


if __name__ == "__main__":
    main()
