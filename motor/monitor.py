"""Polling-based step monitor to observe motor feedback."""

from __future__ import annotations

import threading
import time
from collections import deque
from typing import Deque, Optional, TYPE_CHECKING

from .constants import DEG_PER_STEP

if TYPE_CHECKING:
    from .gpio import LGPIO


class StepMonitor:
    """
    Detect rising edges by polling an input pin and record timestamps.

    poll_interval_s controls the sleep between reads (default 50 microseconds).
    """

    def __init__(self, lg: "LGPIO", pin_in: int, poll_interval_s: float = 50e-6):
        self.lg = lg
        self.pin = pin_in
        self.poll = max(5e-6, float(poll_interval_s))
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self.edge_times: Deque[float] = deque(maxlen=100000)
        self.last_state = 0

    def start(self) -> None:
        self.lg.claim_in(self.pin)
        self.last_state = self.lg.read(self.pin)
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            state = self.lg.read(self.pin)
            if self.last_state == 0 and state == 1:
                self.edge_times.append(time.perf_counter())
            self.last_state = state
            time.sleep(self.poll)

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def get_counts_since(self, t0: float) -> int:
        """Return number of detected edges since timestamp t0."""
        return sum(1 for ts in self.edge_times if ts >= t0)

    def get_latest_velocity_deg_s(self, window_s: float = 0.02) -> float:
        """Compute average velocity in degrees per second over the recent window."""
        if len(self.edge_times) < 2:
            return 0.0
        now = time.perf_counter()
        cutoff = now - window_s
        times = [ts for ts in self.edge_times if ts >= cutoff]
        if len(times) < 2:
            return 0.0
        dt = times[-1] - times[0]
        steps = len(times) - 1
        if dt <= 0:
            return 0.0
        steps_per_s = steps / dt
        return steps_per_s * DEG_PER_STEP


__all__ = ["StepMonitor"]
