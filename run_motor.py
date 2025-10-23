import os, time, math, csv, threading
from collections import deque
from typing import Tuple, List, Callable, Optional

# -------------------- 상수/핀 매핑 --------------------
STEP_ANGLE_DEG = 1.8
MICROSTEP      = 16
DEG_PER_STEP   = STEP_ANGLE_DEG / MICROSTEP  # 0.1125°

# 모터 핀(출력)
DIR_PIN_NAMA_17  = 24
STEP_PIN_NAMA_17 = 23
ENA_PIN_NAMA_17  = 25

DIR_PIN_NAMA_23  = 20
STEP_PIN_NAMA_23 = 21
ENA_PIN_NAMA_23  = 16

# 모터 STEP 모니터 입력 핀(사용자 환경에 맞게 수정 가능)
# 예: STEP 핀을 Y-분기해서 아래 입력 핀으로도 연결
STEP_MON_NAMA_17 = 26   # BCM 26 (예시)
STEP_MON_NAMA_23 = 19   # BCM 19 (예시)

ENA_ACTIVE_LOW = True

DEFAULT_HIGH_TIME_MIN = 8e-6
MIN_LOW_TIME = 8e-6
ENABLE_SETTLE_S = 0.001
LPF_ALPHA = 0.15  # enc 속도 저역통과 필터

# -------------------- 유틸 --------------------
def _rt_hint():
    try:
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(20))
        os.sched_setaffinity(0, {1})
    except Exception:
        pass

def _smoothstep(t: float) -> float:
    t = 0.0 if t < 0 else 1.0 if t > 1 else t
    return 3*t*t - 2*t*t*t

def _smootherstep(t: float) -> float:
    t = 0.0 if t < 0 else 1.0 if t > 1 else t
    return t*t*t*(t*(t*6 - 15) + 10)

def _shape_fractions(shape: str):
    s = (shape or "mid").lower()
    if s == "short": return 0.5, 0.0, 0.5
    if s == "long":  return 0.1, 0.8, 0.1
    return 0.25, 0.5, 0.25

def compute_total_time_scurve(total_steps: int, v_max_steps: float, shape: str = "mid") -> float:
    r_acc, r_const, r_dec = _shape_fractions(shape)
    coeff = 0.5 * (r_acc + r_dec) + r_const
    if coeff <= 0 or v_max_steps <= 0:
        raise ValueError("v_max_steps <= 0")
    return total_steps / (v_max_steps * coeff)

def compute_segments(total_steps: int, v_max_steps: float, shape: str = "mid"):
    T = compute_total_time_scurve(total_steps, v_max_steps, shape)
    r_acc, r_const, r_dec = _shape_fractions(shape)
    return T, r_acc*T, r_const*T, r_dec*T

def s_curve_velocity_steps(t, v_max, t_acc, t_const, t_dec, T_total):
    if t < t_acc:
        x = 0.5*math.pi*(t/max(t_acc,1e-12))
        return v_max * (math.sin(x)**2)
    if t < t_acc + t_const:
        return v_max
    if t < T_total:
        tau = t - (t_acc + t_const)
        x = 0.5*math.pi*(tau/max(t_dec,1e-12))
        return v_max * (math.cos(x)**2)
    return 0.0

def as_curve_velocity(t, v_max, t_acc, t_const, t_dec, T_total):
    if t < t_acc:
        return v_max * math.sin((math.pi/2)*(t/max(t_acc,1e-12)))
    if t < t_acc + t_const:
        return v_max
    if t < T_total:
        tau = t - (t_acc + t_const)
        return v_max * math.sin((math.pi/2)*(1 - tau/max(t_dec,1e-12)))
    return 0.0

# -------------------- lgpio 래퍼 --------------------
class LGPIO:
    def __init__(self, chip=0):
        import lgpio
        self.lg = lgpio
        self.h = self.lg.gpiochip_open(chip)
        self.claimed = set()
    def claim_out(self, pin:int):
        self.lg.gpio_claim_output(self.h, pin); self.claimed.add(pin)
    def claim_in(self, pin:int):
        self.lg.gpio_claim_input(self.h, pin); self.claimed.add(pin)
    def write(self, pin:int, v:bool):
        self.lg.gpio_write(self.h, pin, 1 if v else 0)
    def read(self, pin:int)->int:
        return self.lg.gpio_read(self.h, pin)
    def close(self):
        for p in list(self.claimed):
            try: self.lg.gpio_write(self.h, p, 0)
            except: pass
        self.lg.gpiochip_close(self.h)

# -------------------- STEP 모니터(폴링 스레드) --------------------
class StepMonitor:
    """
    모니터 입력 핀에서 상승 에지를 폴링으로 검출해 타임스탬프를 기록.
    - poll_interval_s: 폴링 간격 (기본 50µs)
    """
    def __init__(self, lg: LGPIO, pin_in: int, poll_interval_s: float = 50e-6):
        self.lg = lg; self.pin = pin_in
        self.poll = max(5e-6, float(poll_interval_s))
        self._stop = threading.Event()
        self._thread = None
        self.edge_times = deque(maxlen=100000)  # timestamp(s)
        self.last_state = 0

    def start(self):
        self.lg.claim_in(self.pin)
        self.last_state = self.lg.read(self.pin)
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            s = self.lg.read(self.pin)
            if self.last_state == 0 and s == 1:
                self.edge_times.append(time.perf_counter())
            self.last_state = s
            time.sleep(self.poll)

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    # 측정값 제공
    def get_counts_since(self, t0: float) -> int:
        # t0 이후의 에지 개수
        return sum(1 for ts in self.edge_times if ts >= t0)

    def get_latest_velocity_deg_s(self, window_s: float = 0.02) -> float:
        """최근 window_s 내 평균 속도(deg/s)"""
        if len(self.edge_times) < 2:
            return 0.0
        now = time.perf_counter()
        cutoff = now - window_s
        # 최근 범위의 간격 평균
        times = [ts for ts in self.edge_times if ts >= cutoff]
        if len(times) < 2:
            return 0.0
        dt = times[-1] - times[0]
        steps = len(times) - 1
        if dt <= 0: return 0.0
        steps_per_s = steps / dt
        return steps_per_s * DEG_PER_STEP

# -------------------- 구동 + 로깅 --------------------
class MotorRunner:
    def __init__(self):
        self.lg = LGPIO()
        # 출력 핀 클레임
        for p in (DIR_PIN_NAMA_17, STEP_PIN_NAMA_17, ENA_PIN_NAMA_17,
                  DIR_PIN_NAMA_23, STEP_PIN_NAMA_23, ENA_PIN_NAMA_23):
            self.lg.claim_out(p)

        # 모니터 준비(선택)
        self.monitors = {
            0: StepMonitor(self.lg, STEP_MON_NAMA_17),
            1: StepMonitor(self.lg, STEP_MON_NAMA_23),
        }

    def set_dir(self, motor_id:int, forward:bool):
        pin = DIR_PIN_NAMA_17 if motor_id == 0 else DIR_PIN_NAMA_23
        self.lg.write(pin, 0 if forward else 1)

    def set_enable(self, motor_id:int, on:bool):
        pin = ENA_PIN_NAMA_17 if motor_id == 0 else ENA_PIN_NAMA_23
        if ENA_ACTIVE_LOW:
            self.lg.write(pin, 0 if on else 1)
        else:
            self.lg.write(pin, 1 if on else 0)

    def pulse_step(self, motor_id:int, high_time:float, low_time:float):
        pin = STEP_PIN_NAMA_17 if motor_id == 0 else STEP_PIN_NAMA_23
        self.lg.write(pin, 1); time.sleep(high_time)
        self.lg.write(pin, 0); time.sleep(low_time)

    def run_profile(self, motor_id:int, direction:str, total_steps:int,
                    v_max_steps:float, shape:str, curve:str="s",
                    duty:float=0.5, log_stride:int=10) -> List[List[float]]:
        """
        반환 rows: [Time_ms, com_Pos_deg, enc_Pos_deg, com_Vel_deg_per_s, enc_Vel_deg_per_s]
        """
        T_total, t_acc, t_const, t_dec = compute_segments(total_steps, v_max_steps, shape)
        vel = s_curve_velocity_steps if curve == "s" else as_curve_velocity

        # 준비
        self.set_dir(motor_id, direction == 'f')
        self.set_enable(motor_id, True)
        time.sleep(ENABLE_SETTLE_S)

        # 모니터 시작
        mon = self.monitors.get(motor_id)
        mon.start()

        start = time.perf_counter()
        next_due = start + (1.0 / max(1.0, vel(0.0, v_max_steps, t_acc, t_const, t_dec, T_total)))
        moved = 0
        rows = []
        com_pos_deg = 0.0
        enc_vel_filt = 0.0
        last_loop_abs = start
        log_count = 0

        try:
            while moved < total_steps:
                now = time.perf_counter()
                t = now - start
                if t > T_total:
                    break

                v_steps = vel(t, v_max_steps, t_acc, t_const, t_dec, T_total)
                if v_steps <= 0:
                    time.sleep(0.0005)
                    continue

                period = 1.0 / v_steps
                high_t  = max(DEFAULT_HIGH_TIME_MIN, period * duty)
                low_t   = max(MIN_LOW_TIME, period - high_t)
                if high_t + low_t > period:
                    high_t = max(DEFAULT_HIGH_TIME_MIN, period - MIN_LOW_TIME)
                    low_t  = max(MIN_LOW_TIME, period - high_t)

                # 데드라인 방식 대기
                sleep_time = next_due - now
                if sleep_time > 0.0002:
                    time.sleep(sleep_time - 0.0001)
                    now = time.perf_counter()
                while now < next_due:
                    now = time.perf_counter()

                # 펄스 1회
                self.pulse_step(motor_id, high_t, low_t)
                moved += 1
                next_due = now + period

                # 명령 좌표 적분
                loop_dt = now - last_loop_abs
                com_pos_deg += (v_steps * DEG_PER_STEP) * loop_dt
                last_loop_abs = now

                # enc 속도(최근 창) 측정 + LPF
                enc_vel_inst = mon.get_latest_velocity_deg_s(0.02)
                enc_vel_filt = LPF_ALPHA*enc_vel_inst + (1-LPF_ALPHA)*enc_vel_filt

                # enc 위치: 모니터된 에지 개수 × 각도
                enc_count = mon.get_counts_since(start)
                enc_pos_deg = enc_count * DEG_PER_STEP

                log_count += 1
                if log_count >= log_stride:
                    rows.append([
                        int(round((time.perf_counter()-start)*1000)),
                        round(com_pos_deg,6),
                        round(enc_pos_deg,6),
                        round(v_steps*DEG_PER_STEP,6),
                        round(enc_vel_filt,6),
                    ])
                    log_count = 0

            # 종료 로깅
            rows.append([
                int(round((time.perf_counter()-start)*1000)),
                round(com_pos_deg,6),
                round((mon.get_counts_since(start))*DEG_PER_STEP,6),
                0.0, 0.0
            ])
            return rows
        finally:
            try: mon.stop()
            except: pass
            try: self.set_enable(motor_id, False)
            except: pass

# -------------------- CSV/Plot/CLI --------------------
def save_csv(rows, path="motor_profile_feedback.csv"):
    with open(path, "w", newline="") as f:
        w = csv.writer(f, delimiter="\t")
        w.writerow(["Time_ms","com_Pos_deg","enc_Pos_deg","com_Vel_deg_per_s","enc_Vel_deg_per_s"])
        w.writerows(rows)
    print(f"[CSV] saved: {os.path.abspath(path)}")
    return os.path.abspath(path)

def plot_results(rows):
    try:
        import matplotlib.pyplot as plt
    except Exception:
        print("[plot] matplotlib 미설치 → 그래프 생략")
        return
    if not rows: return
    t = [r[0] for r in rows]
    com_p = [r[1] for r in rows]
    enc_p = [r[2] for r in rows]
    com_v = [r[3] for r in rows]
    enc_v = [r[4] for r in rows]

    plt.figure(); plt.title("Position (deg)"); plt.plot(t, com_p, label="command"); plt.plot(t, enc_p, label="measured"); plt.xlabel("ms"); plt.ylabel("deg"); plt.legend()
    plt.figure(); plt.title("Velocity (deg/s)"); plt.plot(t, com_v, label="command"); plt.plot(t, enc_v, label="measured"); plt.xlabel("ms"); plt.ylabel("deg/s"); plt.legend()
    plt.show()

def main():
    _rt_hint()
    mr = MotorRunner()
    try:
        mode = input("실행 모드 선택 (1: S-curve, 2: AS-curve): ").strip()
        v_max = float(input("Vmax 입력 [steps/s, 예: 2000]: ").strip())
        move_steps = int(input("이동할 스텝 수 [예: 5000]: ").strip())
        direction = input("모터 방향 (f/b): ").strip().lower()
        motor_id = int(input("모터 ID (0=NEMA17, 1=NEMA23): ").strip())
        shape = input("프로파일 선택 (short/mid/long) [기본: mid]: ").strip().lower() or "mid"

        curve = "s" if mode == "1" else "a"
        rows = mr.run_profile(motor_id, direction, move_steps, v_max, shape, curve=curve, duty=0.5, log_stride=10)
        path = save_csv(rows)
        plot_results(rows)
    finally:
        try: mr.lg.close()
        except: pass

if __name__ == "__main__":
    main()
