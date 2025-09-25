

from encoder import GPIOHelper, Encoder
from scurve import run_motor_scurve, calc_scurve_params
from graph import save_csv, plot_results


def main():
    gpio = GPIOHelper()
    encoder = Encoder(gpio)

    try:
        mode = input("실행 모드 선택 (1: 모터 실행, 2: 파라미터 계산): ").strip()
        if mode == "1":
            v_max = float(input("Vmax 입력 [100~5000]: ").strip())
            move_steps = int(input("이동할 스텝 수 입력 [100~10000]: ").strip())
            direction = input("모터 방향 입력 (f/b): ").strip().lower()
            total_time = float(input("총 이동 시간 입력 [초, 0.5~30]: ").strip())

            data_log = run_motor_scurve(gpio, encoder, direction, move_steps, v_max, total_time)
            filepath = save_csv(data_log)
            plot_results(data_log)

        elif mode == "2":
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
        try: encoder.stop()
        except: pass
        try: gpio.cleanup()
        except: pass


if __name__ == "__main__":
    main()
