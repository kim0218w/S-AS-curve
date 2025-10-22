from encoder import GPIOHelper
from scurve import run_motor_scurve, run_motor_ascurve, compute_total_time_scurve
from graph import save_csv, plot_results


def main():
    gpio = GPIOHelper()

    try:
        mode = input("실행 모드 선택 (1: S-curve 실행, 2: AS-curve 실행, 3: 파라미터 계산): ").strip()

        if mode == "1":
            v_max = float(input("Vmax 입력 [100~5000 steps/s]: ").strip())
            move_steps = int(input("이동할 스텝 수 입력 [100~10000]: ").strip())
            direction = input("모터 방향 입력 (f/b): ").strip().lower()
            motor_id = int(input("모터 ID 입력 (예: 0): ").strip())
            shape = input("프로파일 선택 (short/mid/long) [기본: mid]: ").strip().lower() or "mid"

            data_log = run_motor_scurve(
                gpio=gpio,
                motor_id=motor_id,
                direction=direction,
                total_steps=move_steps,
                v_max_steps=v_max,     
                shape=shape,
            )
            filepath = save_csv(data_log)
            plot_results(data_log)

        elif mode == "2":
            v_max = float(input("Vmax 입력 [100~5000 steps/s]: ").strip())
            move_steps = int(input("이동할 스텝 수 입력 [100~10000]: ").strip())
            direction = input("모터 방향 입력 (f/b): ").strip().lower()
            motor_id = int(input("모터 ID 입력 (예: 0): ").strip())
            shape = input("프로파일 선택 (short/mid/long) [기본: mid]: ").strip().lower() or "mid"

            data_log = run_motor_scurve(
                gpio=gpio,
                motor_id=motor_id,
                direction=direction,
                total_steps=move_steps,
                v_max_steps=v_max,     
                shape=shape,
            )
            filepath = save_csv(data_log)
            plot_results(data_log)

        elif mode == "3":
            steps_in = input("총 스텝 수 입력 (또는 Enter): ").strip()
            vmax_in = input("최대 속도 입력 (steps/s, 또는 Enter): ").strip()
            shape = input("프로파일 선택 (short/mid/long) [기본: mid]: ").strip().lower() or "mid"

            total_steps = int(steps_in) if steps_in else None
            v_max = float(vmax_in) if vmax_in else None

            if total_steps and v_max:
                total_time = compute_total_time_scurve(total_steps, v_max, shape)
                print(f"[S-curve Parameters] total_steps={total_steps}, v_max={v_max}, total_time={total_time:.3f}s")
            else:
                print("총 스텝 수와 최대 속도를 모두 입력해야 합니다.")

        else:
            print("잘못된 모드 선택")

    except Exception as e:
        print(f"[ERROR] {e}")
    finally:
        try:
            gpio.cleanup()
        except:
            pass


if __name__ == "__main__":
    main()
