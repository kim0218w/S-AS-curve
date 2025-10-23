"""Motion profile helper functions."""

import math
from typing import Tuple


def _smoothstep(t: float) -> float:
    """Classic smoothstep easing."""
    t = 0.0 if t < 0 else 1.0 if t > 1 else t
    return 3 * t * t - 2 * t * t * t


def _smootherstep(t: float) -> float:
    """Classic smootherstep easing."""
    t = 0.0 if t < 0 else 1.0 if t > 1 else t
    return t * t * t * (t * (t * 6 - 15) + 10)


def _shape_fractions(shape: str) -> Tuple[float, float, float]:
    """Return the acceleration, constant, deceleration fractions for the curve."""
    s = (shape or "mid").lower()
    if s == "short":
        return 0.5, 0.0, 0.5
    if s == "long":
        return 0.1, 0.8, 0.1
    return 0.25, 0.5, 0.25


def compute_total_time_scurve(total_steps: int, v_max_steps: float, shape: str = "mid") -> float:
    """Compute total motion time for the specified S-curve profile."""
    r_acc, r_const, r_dec = _shape_fractions(shape)
    coeff = 0.5 * (r_acc + r_dec) + r_const
    if coeff <= 0 or v_max_steps <= 0:
        raise ValueError("v_max_steps <= 0")
    return total_steps / (v_max_steps * coeff)


def compute_segments(total_steps: int, v_max_steps: float, shape: str = "mid") -> Tuple[float, float, float, float]:
    """Return total time and segment durations for the requested profile."""
    T = compute_total_time_scurve(total_steps, v_max_steps, shape)
    r_acc, r_const, r_dec = _shape_fractions(shape)
    return T, r_acc * T, r_const * T, r_dec * T


def s_curve_velocity_steps(t: float, v_max: float, t_acc: float, t_const: float, t_dec: float, T_total: float) -> float:
    """Sinusoidal-blended S curve velocity profile in steps per second."""
    if t < t_acc:
        x = 0.5 * math.pi * (t / max(t_acc, 1e-12))
        return v_max * (math.sin(x) ** 2)
    if t < t_acc + t_const:
        return v_max
    if t < T_total:
        tau = t - (t_acc + t_const)
        x = 0.5 * math.pi * (tau / max(t_dec, 1e-12))
        return v_max * (math.cos(x) ** 2)
    return 0.0


def as_curve_velocity(t: float, v_max: float, t_acc: float, t_const: float, t_dec: float, T_total: float) -> float:
    """Alternative S curve velocity profile using sine easing."""
    if t < t_acc:
        return v_max * math.sin((math.pi / 2) * (t / max(t_acc, 1e-12)))
    if t < t_acc + t_const:
        return v_max
    if t < T_total:
        tau = t - (t_acc + t_const)
        return v_max * math.sin((math.pi / 2) * (1 - tau / max(t_dec, 1e-12)))
    return 0.0


__all__ = [
    "compute_total_time_scurve",
    "compute_segments",
    "s_curve_velocity_steps",
    "as_curve_velocity",
]
