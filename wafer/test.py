import time
import lgpio
from encoder import Encoder, GPIOHelper   # Encoder는 I2C 버전
from scurve import run_motor_scurve

# === 핀 정의 ===
DIR_PIN_NAME17 = 24
STEP_PIN_NAME17 = 23
ENA_PIN_NAME17 = 25

DIR_PIN_NAME23 = 20
STEP_PIN_NAME23 = 21
ENA_PIN_NAME23 = 16

IN1_PIN = 5
IN2_PIN = 6
PWM_PIN = 13

ACT_TIME = 7  # actuator 동작 시간 (초)


# === actuator 제어 함수 ===
def extend(h, duty=1.0):
    print("[ACT] extend")
    lgpio.gpio_write(h, IN1_PIN, 1)
    lgpio.gpio_write(h, IN2_PIN, 0)


def retract(h, duty=1.0):
    print("[ACT] retract")
    lgpio.gpio_write(h, IN1_PIN, 0)
    lgpio.gpio_write(h, IN2_PIN, 1)


def stop_actuator(h):
    print("[ACT] stop")
    lgpio.gpio_write(h, IN1_PIN, 0)
    lgpio.gpio_write(h, IN2_PIN, 0)


if __name__ == "__main__":
    # === GPIO 초기화 ===
    h = lgpio.gpiochip_open(0)
    for pin in (
        DIR_PIN_NAME17, STEP_PIN_NAME17, ENA_PIN_NAME17,
        DIR_PIN_NAME23, STEP_PIN_NAME23, ENA_PIN_NAME23,
        IN1_PIN, IN2_PIN, PWM_PIN
    ):
        lgpio.gpio_claim_output(h, pin)

    gpio = GPIOHelper()
    encoder = Encoder(bus_num=1)   # ✅ AS5600 I2C 초기화

    try:
        print("Commands:")
        print("  ACT e/r                                -> actuator extend/retract")
        print("  M1 f/b [steps] [vmax] [time]           -> motor1 제어")
        print("  M2 f/b [steps] [vmax] [time]           -> motor2 제어")
        print("  PID kp ki kd                           -> PID 제어 (추가 예정)")
        print("  q                                      -> quit")

        while True:
            cmd = input(">> ").strip().lower().split()
            if not cmd:
                continue
            if cmd[0] == 'q':
                break

            # === actuator 명령 ===
            if cmd[0] == 'act' and len(cmd) >= 2:
                if cmd[1] == 'e':
                    extend(h, 1.0)
                elif cmd[1] == 'r':
                    retract(h, 1.0)
                else:
                    print("Specify e (extend) or r (retract)")
                    continue
                time.sleep(ACT_TIME)
                stop_actuator(h)
                continue

            # === motor1 명령 ===
            if cmd[0] == 'm1' and len(cmd) >= 5:
                direction = cmd[1]
                steps = int(cmd[2])
                vmax = float(cmd[3])
                total_time = float(cmd[4])
                run_motor_scurve(gpio, encoder, direction, steps, vmax, total_time)
                continue

            # === motor2 명령 ===
            if cmd[0] == 'm2' and len(cmd) >= 5:
                direction = cmd[1]
                steps = int(cmd[2])
                vmax = float(cmd[3])
                total_time = float(cmd[4])
                run_motor_scurve(gpio, encoder, direction, steps, vmax, total_time)
                continue

            print(" Unknown command:", cmd)

    finally:
        try:
            encoder.stop()
        except:
            pass
        try:
            gpio.cleanup()
        except:
            pass
        try:
            lgpio.gpiochip_close(h)
        except:
            pass
