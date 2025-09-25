import sys
import os
import time
import csv
import numpy as np
import pandas as pd
from graph import plot_run_results, unwrap_deg, finite_diff, cumtrapz, plot_overlay
from scurve import calc_scurve_params, s_curve_velocity, split_motion_time, smooth_cos_delay, rate_to_delay, delay_to_rate, move_stepper_scurve_with_logging, move_stepper_scurve_with_pid
from encoder import Encoder, EncoderVelEstimator, PID, EncoderMux

try:
    import lgpio
except ImportError:
    lgpio = None

# -------------------- Pin Assignments --------------------
DIR_PIN_NAMA_17 = 24
STEP_PIN_NAMA_17 = 23
ENA_PIN_NAMA_17 = 25
ENCODER_A_PIN = 3
ENCODER_B_PIN = 2
ENCODER_INVERT = True
DEG_PER_STEP = 0.018

DIR_PIN_NAMA_23 = 20
STEP_PIN_NAMA_23 = 21
ENA_PIN_NAMA_23 = 16

MOTORS = {
    "M1": {"dir": DIR_PIN_NAMA_17, "step": STEP_PIN_NAMA_17, "ena": ENA_PIN_NAMA_17},
    "M2": {"dir": DIR_PIN_NAMA_23, "step": STEP_PIN_NAMA_23, "ena": ENA_PIN_NAMA_23},
}

# -------------------- GPIO Helper --------------------
class GPIOHelper:
    def __init__(self, extra_output_pins=None):
        self.sim = (lgpio is None)
        self.h = None
        self._claimed = set()
        if not self.sim:
            try:
                self.h = lgpio.gpiochip_open(0)
                if extra_output_pins:
                    for pin in extra_output_pins:
                        self._claim_output(pin)
            except Exception:
                self.sim = True
                self.h = None
                print("[GPIO] 초기화 실패 → 시뮬레이션 모드")

    def _claim_output(self, pin):
        if self.sim or pin in self._claimed:
            return
        try:
            lgpio.gpio_claim_output(self.h, pin)
            self._claimed.add(pin)
        except Exception:
            self.sim = True

    def write(self, pin, val):
        if self.sim: return
        self._claim_output(pin)
        lgpio.gpio_write(self.h, pin, 1 if val else 0)

    def pulse(self, pin, high_time_s, low_time_s):
        if self.sim:
            time.sleep(high_time_s + low_time_s)
            return
        self._claim_output(pin)
        lgpio.gpio_write(self.h, pin, 1)
        time.sleep(high_time_s)
        lgpio.gpio_write(self.h, pin, 0)
        time.sleep(low_time_s)

    def cleanup(self):
        if not self.sim and self.h:
            try:
                lgpio.gpiochip_close(self.h)
            except Exception:
                pass

# -------------------- Initialize --------------------
gpio = GPIOHelper()
encoder = Encoder(gpio=gpio, a_pin=ENCODER_A_PIN, b_pin=ENCODER_B_PIN, invert=ENCODER_INVERT)

# -------------------- Main --------------------
if __name__ == "__main__":
    try:
        print("Commands:")
        print("  M1|M2 f|b [steps] [accel_ratio] ol/cl [pid Kp Ki Kd]")
        print("  q -> quit")

        try:
            encmux = EncoderMux(bus_num=1)
            working = encmux.detect_working_channels([0, 1])
            fb_ch = working[0] if working else None
            def enc_read_deg():
                encmux.select_channel(fb_ch)
                return encmux.read_angle_deg()
        except Exception:
            encmux = None
            enc_read_deg = None
            print("[WARN] I2C encoder not available")

        while True:
            cmd = input(">> ").strip().split()
            if not cmd:
                continue
            if cmd[0].lower() == "q":
                break

            target = cmd[0].upper()
            if target not in MOTORS:
                print("Unknown motor. Use M1 or M2.")
                continue

            forward = (cmd[1].lower() == "f")
            steps = int(cmd[2]) if len(cmd) >= 3 else 2000
            accel_ratio = float(cmd[3]) if len(cmd) >= 4 else 0.2
            mode = cmd[4].lower() if len(cmd) >= 5 else "ol"

            kp, ki, kd = 2.0, 0.2, 0.0
            if len(cmd) >= 6 and cmd[5].lower() == "pid":
                kp, ki, kd = map(float, cmd[6:9])

            pins = MOTORS[target]
            dir_pin, step_pin, ena_pin = pins["dir"], pins["step"], pins["ena"]

            MIN_DELAY = 0.0005
            MAX_DELAY = 0.002

            if mode == "ol":
                logs = move_stepper_scurve_with_logging(
                    gpio, dir_pin, step_pin, ena_pin,
                    total_steps=steps, forward=forward,
                    min_delay=MIN_DELAY, max_delay=MAX_DELAY,
                    accel_ratio=accel_ratio, sample_dt=0.015
                )
            elif mode == "cl":
                if enc_read_deg is None:
                    print("[ERR] Closed-loop requires I2C encoder.")
                    continue
                logs = move_stepper_scurve_with_pid(
                    gpio, dir_pin, step_pin, ena_pin,
                    total_steps=steps, forward=forward,
                    min_delay=MIN_DELAY, max_delay=MAX_DELAY,
                    accel_ratio=accel_ratio, deg_per_step=DEG_PER_STEP,
                    enc_read_deg=enc_read_deg, sample_dt=0.015,
                    pid_gains=(kp, ki, kd)
                )
            else:
                print("Mode must be 'ol' or 'cl'")
                continue

            print(f"[{target}] Done. Logged {len(logs['t'])} samples.")
    except KeyboardInterrupt:
        print("\n[Interrupted]")
    finally:
        try:
            encoder.stop()
        except:
            pass
        try:
            gpio.cleanup()
        except:
            pass
