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
            total_time = float(input("총 이동 시간 입력 [초, 0.5~30]: ").strip())

            print(f"[INFO] S-curve 실행 → steps={move_steps}, Vmax={v_max} [steps/s], "
                  f"duration={total_time:.2f}s, motor={motor_id}, dir={direction}")

            data_log = run_motor_scurve(
                gpio, encoder, motor_id, direction, move_steps, v_max, total_time
            )
            filepath = save_csv(data_log)
            plot_results(data_log, title="S-Curve Motion")

        elif mode == "2":  # AS-curve 실행
            motor_id = int(input("모터 선택 (17 또는 23): ").strip())
            v_max = float(input("Vmax 입력 [steps/s]: ").strip())
            move_steps = int(input("이동할 스텝 수 입력 [예: 10000 = 180°]: ").strip())
            direction = input("모터 방향 입력 (f: forward / b: backward): ").strip().lower()
            total_time = float(input("총 이동 시간 입력 [초, 0.5~30]: ").strip())

            # 가속 20%, 감속 40%, 나머지 정속
            t_acc = total_time * 0.2
            t_dec = total_time * 0.4
            t_const = total_time - t_acc - t_dec
            if t_const < 0:
                raise ValueError("가속+감속 시간이 전체 시간보다 짧아야 합니다.")

            print(f"[INFO] AS-curve 실행 → steps={move_steps}, Vmax={v_max} [steps/s], "
                  f"duration={total_time:.2f}s, motor={motor_id}, dir={direction}")
            print(f"        가속={t_acc:.2f}s, 정속={t_const:.2f}s, 감속={t_dec:.2f}s")

            data_log = run_motor_ascurve(
                gpio, encoder, motor_id, direction, move_steps, v_max, total_time, t_acc, t_dec
            )
            filepath = save_csv(data_log)
            plot_results(data_log, title="AS-Curve Motion")

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
