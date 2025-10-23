#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
모터 S-curve 프로파일 생성 후 CSV로 저장
열: Time_ms, com_Pos_deg, enc_Pos_deg, com_Vel_deg_per_s, enc_Vel_deg_per_s
- NEMA23 기준(총 10,000 스텝)
- 1/16 마이크로스텝 가정 (1.8°/step → 0.1125°/µstep)
- S-curve 가감속: 200 Hz → 1250 Hz → 200 Hz
- 실제 엔코더가 없으므로 enc_* 값은 명령값 + 소량 노이즈로 시뮬레이션

출력 파일: /mnt/data/motor_profile.csv
구분자: 탭(\t)
"""

import math
import random
import csv

# ===== 고정 핀(참고용 주석) 및 사용자 제공 상수 =====
DIR_PIN_NAMA_17  = 24
STEP_PIN_NAMA_17 = 23
ENA_PIN_NAMA_17  = 25

DIR_PIN_NAMA_23  = 20
STEP_PIN_NAMA_23 = 21
ENA_PIN_NAMA_23  = 16

IN1_PIN = 5
IN2_PIN = 6
PWM_PIN = 13

# 타이밍/스텝 (사용자 제공 값)
a   = 0.0004   # NEMA23 half-period (s) -> f_cruise ≈ 1/(2a)=1250 Hz
aa  = 0.0005   # NEMA17 half-period (s) -> f_cruise ≈ 1/(2aa)=1000 Hz
aaa = 7        # (액추에이터 시간: 여기선 사용 안 함)
steps_per_nama_23 = 10000
steps_per_nama_17 = 1320

# ===== 프로파일 파라미터(여기서는 NEMA23 기준으로 생성) =====
STEPS = steps_per_nama_23
F_CRUISE = 1.0 / (2.0 * a)  # 1250 Hz
F_START  = 200.0
F_END    = 200.0
SEGMENTS = 20

STEP_ANGLE_DEG = 1.8
MICROSTEP = 16
DEG_PER_STEP = STEP_ANGLE_DEG / MICROSTEP  # 0.1125 deg

def smoothstep(t: float) -> float:
    t = max(0.0, min(1.0, t))
    return 3*t*t - 2*t*t*t

# 세그먼트 분할
seg_steps = [STEPS // SEGMENTS] * SEGMENTS
seg_steps[-1] += STEPS - sum(seg_steps)

# S-curve로 세그먼트별 목표 주파수 만들기(가속→정속→감속)
freqs = []
for i, n in enumerate(seg_steps):
    t = (i + 1) / SEGMENTS
    if t <= 0.5:
        r = smoothstep(t * 2.0)   # 0..1
        f = F_START + (F_CRUISE - F_START) * r
    else:
        r = smoothstep((t - 0.5) * 2.0)
        f = F_CRUISE - (F_CRUISE - F_END) * r
    freqs.append((n, f))

# 스텝 단위로 적분하여 시계열 생성
rows = []
t_s = 0.0
pos_deg = 0.0
for n, f in freqs:
    f = max(1.0, float(f))
    period = 1.0 / f           # 1 step period (s)
    com_vel_deg_s = f * DEG_PER_STEP
    for _ in range(n):
        # 엔코더(시뮬레이션): 명령값 + 작은 노이즈
        enc_pos_deg = pos_deg + random.uniform(-0.15, 0.15)
        enc_vel_deg_s = com_vel_deg_s + random.uniform(-3.0, 3.0)

        rows.append([
            int(round(t_s * 1000.0)),     # Time_ms
            round(pos_deg, 6),            # com_Pos_deg
            round(enc_pos_deg, 6),        # enc_Pos_deg
            round(com_vel_deg_s, 6),      # com_Vel_deg_per_s
            round(enc_vel_deg_s, 6),      # enc_Vel_deg_per_s
        ])

        # 다음 스텝으로 진행
        t_s += period
        pos_deg += DEG_PER_STEP

# CSV 저장 (탭 구분)
out_path = "/mnt/data/motor_profile.csv"
with open(out_path, "w", newline="") as f:
    w = csv.writer(f, delimiter="\t")
    w.writerow(["Time_ms", "com_Pos_deg", "enc_Pos_deg", "com_Vel_deg_per_s", "enc_Vel_deg_per_s"])
    w.writerows(rows)

print(f"Saved {len(rows)} rows to {out_path}")
