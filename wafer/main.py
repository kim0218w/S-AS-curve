from encoder import GPIOHelper, Encoder
from scurve import run_motor_scurve, run_motor_ascurve, calc_scurve_params
from graph import save_csv, plot_results


def main():
    gpio = GPIOHelper()
    encoder = Encoder(gpio)

    try:
        mode = input("실행 모드 선택 (1: S-curve 실행, 2: AS-curve 실행, 3: 파라미터 계산): ").strip()

        if mode == "1":  # S-curve 실행
            motor_id = int(input("모터 선택 (17 또는 23): ").strip())
            v_max = float(input("Vmax 입력 [steps/s]: ").strip())
            move_steps = int(input("이동할 스텝 수 입력 [예: 10000 = 180°]: ").strip())
            direction = input("모터 방향 입력 (f: forward / b: backward): ").strip().lower()
            shape = input("S-curve 형태 선택 (short/mid/long, 기본=mid): ").strip().lower() or "mid"

            print(f"[INFO] S-curve 실행 → steps={move_steps}, Vmax={v_max} [steps/s], "
                  f"motor={motor_id}, dir={direction}, shape={shape}")

            data_log = run_motor_scurve(
                gpio, encoder, motor_id, direction, move_steps, v_max, shape=shape
            )
            filepath = save_csv(data_log,
                                motor_id=motor_id,
                                steps=move_steps,
                                shape=shape,
                                v_max=v_max)
            plot_results(data_log,
                         title=f"S-Curve Motion ({shape})",
                         motor_id=motor_id,
                         steps=move_steps,
                         shape=shape,
                         smooth_alpha=0.1)

        elif mode == "2":  # AS-curve 실행
            motor_id = int(input("모터 선택 (17 또는 23): ").strip())
            v_max = float(input("Vmax 입력 [steps/s]: ").strip())
            move_steps = int(input("이동할 스텝 수 입력 [예: 10000 = 180°]: ").strip())
            direction = input("모터 방향 입력 (f: forward / b: backward): ").strip().lower()
            shape = input("AS-curve 형태 선택 (short/mid/long, 기본=mid): ").strip().lower() or "mid"

            print(f"[INFO] AS-curve 실행 → steps={move_steps}, Vmax={v_max} [steps/s], "
                  f"motor={motor_id}, dir={direction}, shape={shape}")

            data_log = run_motor_ascurve(
                gpio, encoder, motor_id, direction, move_steps, v_max, shape=shape
            )
            filepath = save_csv(data_log,
                                motor_id=motor_id,
                                steps=move_steps,
                                shape=shape,
                                v_max=v_max)
            plot_results(data_log,
                         title=f"AS-Curve Motion ({shape})",
                         motor_id=motor_id,
                         steps=move_steps,
                         shape=shape,
                         smooth_alpha=0.1)

        elif mode == "3":  # 파라미터 계산
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
