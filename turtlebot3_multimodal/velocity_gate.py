"""Exclusive source selection and a latched stop at the final velocity output."""

import math

from turtlebot3_multimodal.commands import SafetyLimits
from turtlebot3_multimodal.executor import Velocity


class VelocityGate:
    def __init__(self, limits: SafetyLimits | None = None):
        self.limits = limits or SafetyLimits()
        self.source = "nav"
        self.stopped = False
        self._sample: tuple[Velocity, float] | None = None
        self._last_tick: float | None = None

    def select(self, source: str) -> None:
        if source not in {"nav", "manual"}:
            raise ValueError("source must be nav or manual")
        self.source = source
        self._sample = None

    def stop(self) -> None:
        self.stopped = True
        self._sample = None

    def reset(self) -> None:
        self.stopped = False
        self._sample = None
        self._last_tick = None

    def receive(self, source: str, velocity: Velocity, now: float) -> None:
        if source != self.source or self.stopped:
            return
        if not all(math.isfinite(x) for x in (velocity.linear_x, velocity.angular_z, now)):
            self.stop()
            return
        self._sample = (Velocity(
            max(-self.limits.max_linear_mps, min(self.limits.max_linear_mps, velocity.linear_x)),
            max(-self.limits.max_angular_rps, min(self.limits.max_angular_rps, velocity.angular_z)),
        ), now)

    def tick(self, now: float) -> Velocity:
        if not math.isfinite(now) or (
            self._last_tick is not None and (
                now < self._last_tick
                or now - self._last_tick > self.limits.watchdog_timeout_s
            )
        ):
            self.stop()
        self._last_tick = now
        if self.stopped or self._sample is None:
            return Velocity()
        velocity, received = self._sample
        if not 0 <= now - received <= self.limits.watchdog_timeout_s:
            self.stop()
            return Velocity()
        return velocity
