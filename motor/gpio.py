"""Wrapper around lgpio to simplify setup and teardown."""

from __future__ import annotations

from typing import Set


class LGPIO:
    """Lightweight context for managing lgpio resources."""

    def __init__(self, chip: int = 0):
        import lgpio  # local import to avoid dependency during type checking

        self.lg = lgpio
        self.h = self.lg.gpiochip_open(chip)
        self.claimed: Set[int] = set()

    def claim_out(self, pin: int) -> None:
        self.lg.gpio_claim_output(self.h, pin)
        self.claimed.add(pin)

    def claim_in(self, pin: int) -> None:
        self.lg.gpio_claim_input(self.h, pin)
        self.claimed.add(pin)

    def write(self, pin: int, value: bool) -> None:
        self.lg.gpio_write(self.h, pin, 1 if value else 0)

    def read(self, pin: int) -> int:
        return self.lg.gpio_read(self.h, pin)

    def close(self) -> None:
        for pin in list(self.claimed):
            try:
                self.lg.gpio_write(self.h, pin, 0)
            except Exception:
                pass
        self.lg.gpiochip_close(self.h)


__all__ = ["LGPIO"]
