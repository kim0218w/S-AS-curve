"""Motor control package exposing reusable components."""

from .constants import (  # noqa: F401
    DEFAULT_HIGH_TIME_MIN,
    DEG_PER_STEP,
    DIR_PIN_NAMA_17,
    DIR_PIN_NAMA_23,
    ENA_ACTIVE_LOW,
    ENA_PIN_NAMA_17,
    ENA_PIN_NAMA_23,
    ENABLE_SETTLE_S,
    LPF_ALPHA,
    MICROSTEP,
    MIN_LOW_TIME,
    STEP_ANGLE_DEG,
    STEP_MON_NAMA_17,
    STEP_MON_NAMA_23,
    STEP_PIN_NAMA_17,
    STEP_PIN_NAMA_23,
)
from .profiles import (  # noqa: F401
    as_curve_velocity,
    compute_segments,
    compute_total_time_scurve,
    s_curve_velocity_steps,
)
from .gpio import LGPIO  # noqa: F401
from .monitor import StepMonitor  # noqa: F401
from .runner import MotorRunner  # noqa: F401

__all__ = [
    "MotorRunner",
    "LGPIO",
    "StepMonitor",
    "compute_total_time_scurve",
    "compute_segments",
    "s_curve_velocity_steps",
    "as_curve_velocity",
    "STEP_ANGLE_DEG",
    "MICROSTEP",
    "DEG_PER_STEP",
    "DIR_PIN_NAMA_17",
    "STEP_PIN_NAMA_17",
    "ENA_PIN_NAMA_17",
    "DIR_PIN_NAMA_23",
    "STEP_PIN_NAMA_23",
    "ENA_PIN_NAMA_23",
    "STEP_MON_NAMA_17",
    "STEP_MON_NAMA_23",
    "ENA_ACTIVE_LOW",
    "DEFAULT_HIGH_TIME_MIN",
    "MIN_LOW_TIME",
    "ENABLE_SETTLE_S",
    "LPF_ALPHA",
]
