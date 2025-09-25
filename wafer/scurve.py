def run_motor_scurve(gpio, encoder, direction: str, total_steps: int, v_max: float, total_time: float):
    STEPS_PER_REV = 200
    MICROSTEP = 16
    PITCH_MM = 5.0
    ENC_CPR = 1000

    # 방향 & Enable
    gpio.write(DIR_PIN, 1 if direction == 'f' else 0)
    gpio.write(ENA_PIN, 0)   # Enable LOW = 활성화

    # 제어 주기 (더 짧게)
    dt = 0.001   # 1 ms
    t = 0.0
    moved_steps = 0
    step_accumulator = 0.0
    com_pos = 0.0

    prev_enc = encoder.read()
    enc_init = (prev_enc / ENC_CPR) * PITCH_MM
    data_log = []

    # 속도 필터 (이동평균 + 지수 필터)
    vel_history = []
    vel_est = 0.0
    alpha = 0.2  # LPF 계수

    sample_count = 0

    try:
        while t <= total_time and moved_steps < total_steps:
            # 1) 명령 속도 계산
            com_vel_steps = s_curve_velocity(t, v_max, total_time)
            com_vel_mm = (com_vel_steps / (STEPS_PER_REV * MICROSTEP)) * PITCH_MM
            com_pos += com_vel_mm * dt

            # 2) 스텝 누적 발행
            step_accumulator += com_vel_steps * dt
            while step_accumulator >= 1.0 and moved_steps < total_steps:
                # STEP 펄스 폭을 20µs로 짧게 → 더 부드럽고 고속 가능
                gpio.pulse(STEP_PIN, 0.00002, 0.00002)
                moved_steps += 1
                step_accumulator -= 1.0

            # 3) 엔코더 측정 및 속도 추정
            enc_now = encoder.read()
            delta = enc_now - prev_enc
            prev_enc = enc_now
            enc_pos_mm = (enc_now / ENC_CPR) * PITCH_MM - enc_init

            raw_vel = ((delta / dt) / ENC_CPR) * PITCH_MM if sample_count > 2 else 0.0

            # 이동평균 (최근 5개)
            vel_history.append(raw_vel)
            if len(vel_history) > 5:
                vel_history.pop(0)
            vel_ma = sum(vel_history) / len(vel_history)

            # LPF 적용
            vel_est = alpha * vel_ma + (1 - alpha) * vel_est

            # 4) 로깅
            t_ms = int(round(t * 1000))
            data_log.append([t_ms, com_pos, enc_pos_mm, com_vel_mm, vel_est])

            sample_count += 1
            t += dt
            time.sleep(dt)
    finally:
        gpio.write(ENA_PIN, 1)   # 동작 종료 → Disable

    return data_log
