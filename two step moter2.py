import smbus
import time
from gpiozero import DigitalOutputDevice, PWMOutputDevice
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
a = 0.0004
aa = 0.0005
aaa = 2.5  # 액추에이터 상승 시간 (초)
b = 3.0 # 엑추에이터 하강 시간 (초)
# 스텝 수 설정
steps_per_nama_23 = 10000
steps_per_nama_17 = 1320
steps_per_nama_17_reverse = 1400

# GPIO 핀 설정
dir_pin_nama_17 = DigitalOutputDevice(DIR_PIN_NAMA_17)
step_pin_nama_17 = DigitalOutputDevice(STEP_PIN_NAMA_17)
ena_pin_nama_17 = DigitalOutputDevice(ENA_PIN_NAMA_17)

dir_pin_nama_23 = DigitalOutputDevice(DIR_PIN_NAMA_23)
step_pin_nama_23 = DigitalOutputDevice(STEP_PIN_NAMA_23)
ena_pin_nama_23 = DigitalOutputDevice(ENA_PIN_NAMA_23)

in1 = DigitalOutputDevice(IN1_PIN)
in2 = DigitalOutputDevice(IN2_PIN)
pwm = PWMOutputDevice(PWM_PIN)


def extend(speed=1.0):
    in1.on()
    in2.off()
    pwm.value = speed
    print(f"리니어 액추에이터 확장, 속도: {speed}")

def retract(speed=1.0):
    in1.off()
    in2.on()
    pwm.value = speed
    print(f"리니어 액추에이터 수축, 속도: {speed}")

def stop_actuator():
    in1.off()
    in2.off()
    pwm.value = 0
    print("리니어 액추에이터 정지")

try:
    input("엔터 키를 눌러 다음 단계로 진행하세요...")  # 사용자로부터 엔터 입력받기
    for _ in range(1):
        ena_pin_nama_23.off()
        ena_pin_nama_17.off()
    
    #17 앞으로  -> act 아래 -> act 위 -> 23 f:on(시계 : 10000) -> act 아래 -> act 위 -> 17 뒤로 -> 23 b(반시계 : 10000)
    # 네마 17 모터 작동
        dir_pin_nama_17.off()
        for _ in range(steps_per_nama_17):
            step_pin_nama_17.on()
            time.sleep(aa)
            step_pin_nama_17.off()
            time.sleep(aa)
        
        
    # 리니어 액추에이터 수축
        extend(1)  # 50% 속도로 수축
        time.sleep(aaa)  # 3.0초 동안 수축
        stop_actuator()
        time.sleep(0.1)  # 2초 동안 정지

    # 리니어 액추에이터 확장
        retract(1)  # 50% 속도로 확장
        time.sleep(b)  # 2.5초 동안 확장 
        stop_actuator()
        time.sleep(0.1)  # 2초 동안 정지
    
    # 네마 23 모터 작동 23 f(시계 : 10000)
        dir_pin_nama_23.off()
        for _ in range(steps_per_nama_23):
            step_pin_nama_23.on()
            time.sleep(a)
            step_pin_nama_23.off()
            time.sleep(a)
    
        
    # 리니어 액추에이터 확장
        extend(1)  # 50% 속도로 확장
        time.sleep(b)  # 2.5초 동안 확장
        stop_actuator()
        time.sleep(0.1)  # 2초 동안 정지

    # 리니어 액추에이터 수축
        retract(1)  # 50% 속도로 수축
        time.sleep(aaa)  # 3.0초 동안 내려가 
        stop_actuator()
        time.sleep(0.1)  # 2초 동안 정지

    # 네마 17 모터 작동 뒤로
        dir_pin_nama_17.on()
        for _ in range(steps_per_nama_17_reverse):
            step_pin_nama_17.on()
            time.sleep(aa)
            step_pin_nama_17.off()
            time.sleep(aa)
                
    
    # 네마 23 모터 작동 23 b:on(반시계 : 10000) 
        dir_pin_nama_23.on()
        for _ in range(steps_per_nama_23):
            step_pin_nama_23.on()
            time.sleep(a)
            step_pin_nama_23.off()
            time.sleep(a)    
        
    
        
    
except KeyboardInterrupt:
    print("프로그램 종료")

finally:
    ena_pin_nama_23.on()
    ena_pin_nama_17.on()
    
    dir_pin_nama_17.close()
    step_pin_nama_17.close()
    ena_pin_nama_17.close()
    
    dir_pin_nama_23.close()
    step_pin_nama_23.close()
    ena_pin_nama_23.close()
    
    stop_actuator()
    in1.close()
    in2.close()
    pwm.close()