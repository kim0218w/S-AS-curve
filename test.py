import smbus
import time
import sys
import os
import math

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR and SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

try:
    import lgpio
except Exception:
    lgpio = None
    print("[ERROR] lgpio is not available. Run on Raspberry Pi with lgpio installed.")

try:
    from gpiozero import DigitalOutputDevice, PWMOutputDevice
except Exception:
    DigitalOutputDevice = None
    PWMOutputDevice = None
    print("[ERROR] gpiozero is not available. Install with: pip install gpiozero")

# 핀 설정
DIR_PIN_NAMA_17 = 24
STEP_PIN_NAMA_17 = 23
ENA_PIN_NAMA_17 = 25

DIR_PIN_NAMA_23 = 20
STEP_PIN_NAMA_23 = 21
ENA_PIN_NAMA_23 = 16

IN1_PIN = 5  # IN1을 GPIO 5에 연결
IN2_PIN = 6  # IN2를 GPIO 6에 연결
PWM_PIN = 13  # PWM을 GPIO 13에 연결

# 속도 변수 설정
a = 0.0004  # NEMA23 최소 딜레이
aa = 0.0005  # NEMA17 최소 딜레이
aaa = 1.5  # 액추에이터 상승 시간 (초)
b = 3.0  # 엑추에이터 하강 시간 (초)

# 스텝 수 설정
steps_per_nama_23 = 10000
steps_per_nama_17 = 1320
steps_per_nama_17_reverse = 1400

# S-curve 파라미터
ACCEL_RATIO = 0.45  # 가속/감속 비율 (전체 스텝의 45%)
GAMMA = 0.3  # S-curve 기울기

# S-curve 상수
MIN_SAFE_DELAY = 0.00025  # 250 us (최소 안전 딜레이)

# GPIO 핀 초기화
# lgpio: 스테퍼 모터용
h = None
if lgpio is not None:
    try:
        h = lgpio.gpiochip_open(0)
        print("[INFO] GPIO chip opened successfully (lgpio)")
        
        # 스테퍼 모터 핀만 설정 (액추에이터는 gpiozero 사용)
        lgpio.gpio_claim_output(h, DIR_PIN_NAMA_17)
        lgpio.gpio_claim_output(h, STEP_PIN_NAMA_17)
        lgpio.gpio_claim_output(h, ENA_PIN_NAMA_17)
        
        lgpio.gpio_claim_output(h, DIR_PIN_NAMA_23)
        lgpio.gpio_claim_output(h, STEP_PIN_NAMA_23)
        lgpio.gpio_claim_output(h, ENA_PIN_NAMA_23)
        
        print("[INFO] Stepper motor pins initialized (lgpio)")
    except Exception as e:
        print(f"[ERROR] GPIO initialization failed: {e}")
        h = None
else:
    print("[ERROR] lgpio library not available")

# gpiozero: 액추에이터용
actuator_in1 = None
actuator_in2 = None
actuator_pwm = None
if DigitalOutputDevice is not None and PWMOutputDevice is not None:
    try:
        actuator_in1 = DigitalOutputDevice(IN1_PIN, initial_value=False)
        actuator_in2 = DigitalOutputDevice(IN2_PIN, initial_value=False)
        actuator_pwm = PWMOutputDevice(PWM_PIN, initial_value=0.0, frequency=1000)
        print(f"[INFO] Actuator initialized (gpiozero): IN1={IN1_PIN}, IN2={IN2_PIN}, PWM={PWM_PIN}")
    except Exception as e:
        print(f"[ERROR] Actuator initialization failed: {e}")
        actuator_in1 = None
        actuator_in2 = None
        actuator_pwm = None
else:
    print("[ERROR] gpiozero library not available")


def smooth_cos_delay(i, n, min_delay, max_delay, gamma=1.0):
    """S-curve 딜레이 계산"""
    if min_delay >= max_delay:
        min_delay, max_delay = sorted([min_delay, max_delay])
    min_delay = max(min_delay, MIN_SAFE_DELAY)
    if n <= 1:
        return min_delay
    t_norm = i / (n - 1)
    if gamma not in (None, 1.0):
        if gamma <= 0:
            gamma = 1.0
        t_norm = min(max(t_norm, 0.0), 1.0) ** gamma
    k = 0.5 * (1 - math.cos(math.pi * t_norm))
    return max_delay - (max_delay - min_delay) * k


def build_delay_profile(total_steps, min_delay, max_delay, accel_ratio, gamma_accel=1.0):
    """S-curve 딜레이 프로파일 생성"""
    if min_delay >= max_delay:
        min_delay, max_delay = sorted([min_delay, max_delay])
    min_delay = max(min_delay, MIN_SAFE_DELAY)
    
    accel_ratio = float(accel_ratio)
    decel_ratio = accel_ratio  # 대칭으로 설정
    
    accel_steps = int(max(0, round(total_steps * accel_ratio)))
    decel_steps = int(max(0, round(total_steps * decel_ratio)))
    
    # 전체 스텝 수를 초과하지 않도록 조정
    total_ad = accel_steps + decel_steps
    if total_ad > total_steps:
        scale = total_steps / float(total_ad)
        accel_steps = int(round(accel_steps * scale))
        decel_steps = max(0, total_steps - accel_steps)
    const_steps = max(0, total_steps - accel_steps - decel_steps)
    
    # 가속 구간 딜레이
    accel_delays = [
        smooth_cos_delay(i, max(accel_steps, 1), min_delay, max_delay, gamma=gamma_accel)
        for i in range(accel_steps)
    ]
    
    # 감속 구간 딜레이 (가속의 역순)
    decel_delays = list(reversed(accel_delays))
    
    # 전체 딜레이 리스트 생성
    delays = []
    delays.extend(accel_delays)
    if const_steps > 0:
        delays.extend([min_delay] * const_steps)
    delays.extend(decel_delays)
    
    return delays


def move_motor_scurve(h, dir_pin, step_pin, total_steps, direction, min_delay, max_delay):
    """S-curve를 사용한 모터 이동"""
    if lgpio is None or h is None:
        return
    
    # 방향 설정
    lgpio.gpio_write(h, dir_pin, direction)
    time.sleep(0.001)  # 방향 신호 안정화
    
    # S-curve 딜레이 프로파일 생성
    delays = build_delay_profile(total_steps, min_delay, max_delay, ACCEL_RATIO, GAMMA)
    
    # 스텝 실행
    for delay in delays:
        lgpio.gpio_write(h, step_pin, 1)
        time.sleep(delay)
        lgpio.gpio_write(h, step_pin, 0)
        time.sleep(delay)


def extend(speed=1.0):
    """액추에이터 확장 (gpiozero)"""
    if actuator_in1 is None or actuator_in2 is None or actuator_pwm is None:
        print("[ERROR] Actuator devices not initialized (gpiozero)")
        return
    try:
        speed = max(0.0, min(1.0, float(speed)))
        actuator_in1.on()   # IN1 = HIGH
        actuator_in2.off()  # IN2 = LOW
        actuator_pwm.value = speed  # PWM duty cycle (0.0 ~ 1.0)
        print(f"[INFO] 리니어 액추에이터 확장, 속도: {speed} (duty: {speed*100:.1f}%)")
    except Exception as e:
        print(f"[ERROR] 액추에이터 확장 실패: {e}")

def retract(speed=1.0):
    """액추에이터 수축 (gpiozero)"""
    if actuator_in1 is None or actuator_in2 is None or actuator_pwm is None:
        print("[ERROR] Actuator devices not initialized (gpiozero)")
        return
    try:
        speed = max(0.0, min(1.0, float(speed)))
        actuator_in1.off()  # IN1 = LOW
        actuator_in2.on()   # IN2 = HIGH
        actuator_pwm.value = speed  # PWM duty cycle (0.0 ~ 1.0)
        print(f"[INFO] 리니어 액추에이터 수축, 속도: {speed} (duty: {speed*100:.1f}%)")
    except Exception as e:
        print(f"[ERROR] 액추에이터 수축 실패: {e}")

def stop_actuator():
    """액추에이터 정지 (gpiozero)"""
    if actuator_in1 is None or actuator_in2 is None or actuator_pwm is None:
        print("[ERROR] Actuator devices not initialized (gpiozero)")
        return
    try:
        actuator_in1.off()  # IN1 = LOW
        actuator_in2.off()  # IN2 = LOW
        actuator_pwm.value = 0.0  # PWM off
        print("[INFO] 리니어 액추에이터 정지")
    except Exception as e:
        print(f"[ERROR] 액추에이터 정지 실패: {e}")

try:
    input("엔터 키를 눌러 다음 단계로 진행하세요...")  # 사용자로부터 엔터 입력받기
    for _ in range(1):
        # ENA 핀 off = 모터 활성화 (lgpio: 0 = 활성화)
        if lgpio is not None and h is not None:
            lgpio.gpio_write(h, ENA_PIN_NAMA_23, 0)
            lgpio.gpio_write(h, ENA_PIN_NAMA_17, 0)
    
    #act 위 ->  23 f:on(시계 : 10000) -> act 아래 -> act 위 -> 17 뒤로 -> 23 b(반시계 : 10000)
    # 네마 17 모터 작동 (S-curve 적용)
        if lgpio is not None and h is not None:
            print("[INFO] NEMA17 모터 작동 (정방향, S-curve)")
            move_motor_scurve(h, DIR_PIN_NAMA_17, STEP_PIN_NAMA_17, 
                            steps_per_nama_17, 0, aa, aa * 4)  # min_delay=aa, max_delay=aa*4
    #3초 딜레이
        time.sleep(3.0)
        
    # 리니어 액추에이터 수축
        retract(1)  # 100% 속도로 수축
        time.sleep(aaa)  # 1.5초 동안 수축
        stop_actuator()
        time.sleep(0.1)  # 0.1초 동안 정지
        
    # # 리니어 액추에이터 확장
    #     extend(1)  # 100% 속도로 확장
    #     time.sleep(aaa)  # 1.5초 동안 확장 
    #     stop_actuator()
    #     time.sleep(0.1)  # 0.1초 동안 정지
    
    # 네마 23 모터 작동 23 f(시계 : 10000) (S-curve 적용)
        if lgpio is not None and h is not None:
            print("[INFO] NEMA23 모터 작동 (정방향, S-curve)")
            move_motor_scurve(h, DIR_PIN_NAMA_23, STEP_PIN_NAMA_23, 
                            steps_per_nama_23, 0, a, a * 4)  # min_delay=a, max_delay=a*4
    
        
    # 리니어 액추에이터 확장
        extend(1)  # 100% 속도로 확장
        time.sleep(aaa)  # 1.5초 동안 확장
        stop_actuator()
        time.sleep(0.1)  # 0.1초 동안 정지

    # 코드 끝나고 5초 정지 딜레이 추가
        time.sleep(3.0)  # 5초 정지

    # 리니어 액추에이터 수축
        retract(1)  # 100% 속도로 수축
        time.sleep(aaa)  # 1.5초 동안 수축
        stop_actuator()
        time.sleep(0.1)  # 0.1초 동안 정지

    # 네마 17 모터 작동 뒤로 (S-curve 적용)
        if lgpio is not None and h is not None:
            print("[INFO] NEMA17 모터 작동 (역방향, S-curve)")
            move_motor_scurve(h, DIR_PIN_NAMA_17, STEP_PIN_NAMA_17, 
                            steps_per_nama_17_reverse, 1, aa, aa * 4)
                
    
    # 네마 23 모터 작동 23 b:on(반시계 : 10000) (S-curve 적용)
        if lgpio is not None and h is not None:
            print("[INFO] NEMA23 모터 작동 (역방향, S-curve)")
            move_motor_scurve(h, DIR_PIN_NAMA_23, STEP_PIN_NAMA_23, 
                            steps_per_nama_23, 1, a, a * 4)
        
    
        
    
except KeyboardInterrupt:
    print("프로그램 종료")

finally:
    # 액추에이터 정지 (gpiozero)
    try:
        stop_actuator()
    except Exception as e:
        print(f"[WARN] Error stopping actuator: {e}")
    
    # gpiozero 디바이스 정리
    try:
        if actuator_in1 is not None:
            actuator_in1.close()
        if actuator_in2 is not None:
            actuator_in2.close()
        if actuator_pwm is not None:
            actuator_pwm.close()
        print("[INFO] Actuator devices closed (gpiozero)")
    except Exception as e:
        print(f"[WARN] Error closing actuator devices: {e}")
    
    # lgpio 정리 (스테퍼 모터)
    if lgpio is not None and h is not None:
        try:
            # ENA 핀 on = 모터 비활성화 (lgpio: 1 = 비활성화)
            lgpio.gpio_write(h, ENA_PIN_NAMA_23, 1)
            lgpio.gpio_write(h, ENA_PIN_NAMA_17, 1)
            lgpio.gpiochip_close(h)
            print("[INFO] GPIO released (lgpio)")
        except Exception as e:
            print(f"[WARN] Error releasing GPIO: {e}")
