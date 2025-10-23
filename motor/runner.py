"""High-level motor runner that generates motion profiles and logs feedback."""

from __future__ import annotations

import time
from typing import Dict, List, Optional

from .constants import (
    DEFAULT_HIGH_TIME_MIN,
    DEG_PER_STEP,
    DIR_PIN_NAMA_17,
    DIR_PIN_NAMA_23,
    ENA_ACTIVE_LOW,
    ENA_PIN_NAMA_17,
    ENA_PIN_NAMA_23,
    ENABLE_SETTLE_S,
    LPF_ALPHA,
    MIN_LOW_TIME,
    STEP_MON_NAMA_17,
    STEP_MON_NAMA_23,
    STEP_PIN_NAMA_17,
    STEP_PIN_NAMA_23,
)
from .gpio import LGPIO
from .monitor import StepMonitor
from .profiles import as_curve_velocity, compute_segments, s_curve_velocity_steps


class MotorRunner:
    """Drive stepper motors using lgpio while collecting feedback data."""

    def __init__(self, lg: Optional[LGPIO] = None):
        self.lg = lg or LGPIO()

        for pin in (
            DIR_PIN_NAMA_17,
            STEP_PIN_NAMA_17,
            ENA_PIN_NAMA_17,
            DIR_PIN_NAMA_23,
            STEP_PIN_NAMA_23,
            ENA_PIN_NAMA_23,
        ):
            self.lg.claim_out(pin)

        self.monitors: Dict[int, StepMonitor] = {
            0: StepMonitor(self.lg, STEP_MON_NAMA_17),
            1: StepMonitor(self.lg, STEP_MON_NAMA_23),
        }

    def set_dir(self, motor_id: int, forward: bool) -> None:
        pin = DIR_PIN_NAMA_17 if motor_id == 0 else DIR_PIN_NAMA_23
        self.lg.write(pin, 0 if forward else 1)

    def set_enable(self, motor_id: int, enabled: bool) -> None:
        pin = ENA_PIN_NAMA_17 if motor_id == 0 else ENA_PIN_NAMA_23
        if ENA_ACTIVE_LOW:
            self.lg.write(pin, 0 if enabled else 1)
        else:
            self.lg.write(pin, 1 if enabled else 0)

    def pulse_step(self, motor_id: int, high_time: float, low_time: float) -> None:
        pin = STEP_PIN_NAMA_17 if motor_id == 0 else STEP_PIN_NAMA_23
        self.lg.write(pin, 1)
        time.sleep(high_time)
        self.lg.write(pin, 0)
        time.sleep(low_time)

    def close(self) -> None:
        """Release underlying GPIO resources."""
        try:
            self.lg.close()
        except Exception:
            pass

    def run_profile(
        self,
        motor_id: int,
        direction: str,
        total_steps: int,
        v_max_steps: float,
        shape: str,
        curve: str = "s",
        duty: float = 0.5,
        log_stride: int = 10,
    ) -> List[List[float]]:
        """
        Execute a motion profile and log commanded and measured states.

        Returns rows shaped as:
            [Time_ms, com_Pos_deg, enc_Pos_deg, com_Vel_deg_per_s, enc_Vel_deg_per_s]
        """
        T_total, t_acc, t_const, t_dec = compute_segments(total_steps, v_max_steps, shape)
        vel_fn = s_curve_velocity_steps if curve == "s" else as_curve_velocity

        self.set_dir(motor_id, direction == "f")
        self.set_enable(motor_id, True)
        time.sleep(ENABLE_SETTLE_S)

        mon = self.monitors.get(motor_id)
        if mon:
            mon.start()

        start = time.perf_counter()
        next_due = start + (1.0 / max(1.0, vel_fn(0.0, v_max_steps, t_acc, t_const, t_dec, T_total)))
        moved = 0
        rows: List[List[float]] = []
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

                v_steps = vel_fn(t, v_max_steps, t_acc, t_const, t_dec, T_total)
                if v_steps <= 0:
                    time.sleep(0.0005)
                    continue

                period = 1.0 / v_steps
                high_time = max(DEFAULT_HIGH_TIME_MIN, period * duty)
                low_time = max(MIN_LOW_TIME, period - high_time)
                if high_time + low_time > period:
                    high_time = max(DEFAULT_HIGH_TIME_MIN, period - MIN_LOW_TIME)
                    low_time = max(MIN_LOW_TIME, period - high_time)

                sleep_time = next_due - now
                if sleep_time > 0.0002:
                    time.sleep(sleep_time - 0.0001)
                    now = time.perf_counter()
                while now < next_due:
                    now = time.perf_counter()

                self.pulse_step(motor_id, high_time, low_time)
                moved += 1
                next_due = now + period

                loop_dt = now - last_loop_abs
                com_pos_deg += (v_steps * DEG_PER_STEP) * loop_dt
                last_loop_abs = now

                enc_vel_inst = mon.get_latest_velocity_deg_s(0.02) if mon else 0.0
                enc_vel_filt = LPF_ALPHA * enc_vel_inst + (1 - LPF_ALPHA) * enc_vel_filt

                enc_count = mon.get_counts_since(start) if mon else 0
                enc_pos_deg = enc_count * DEG_PER_STEP

                log_count += 1
                if log_count >= log_stride:
                    rows.append(
                        [
                            int(round((time.perf_counter() - start) * 1000)),
                            round(com_pos_deg, 6),
                            round(enc_pos_deg, 6),
                            round(v_steps * DEG_PER_STEP, 6),
                            round(enc_vel_filt, 6),
                        ]
                    )
                    log_count = 0

            rows.append(
                [
                    int(round((time.perf_counter() - start) * 1000)),
                    round(com_pos_deg, 6),
                    round((mon.get_counts_since(start) if mon else 0) * DEG_PER_STEP, 6),
                    0.0,
                    0.0,
                ]
            )
            return rows
        finally:
            if mon:
                try:
                    mon.stop()
                except Exception:
                    pass
            try:
                self.set_enable(motor_id, False)
            except Exception:
                pass


__all__ = ["MotorRunner"]
