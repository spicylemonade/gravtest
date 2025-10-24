from __future__ import annotations
from dataclasses import dataclass
from typing import Callable, Optional
from .world import World


@dataclass
class Simulation:
    world: World
    dt: float = 0.01
    integrator: str = 'euler'  # 'euler' or 'rk4'

    def step(self) -> None:
        self.world.step(self.dt, integrator=self.integrator)

    def run(self, steps: int, callback: Optional[Callable[[int, World], None]] = None) -> None:
        for s in range(steps):
            self.step()
            if callback is not None:
                callback(s + 1, self.world)
