from __future__ import annotations
from dataclasses import dataclass, field
from typing import Optional, Tuple
from .vector import Vector2
import math

Color = Tuple[int, int, int]


def _default_radius(mass: float) -> float:
    # Simple radius model (2D): radius scales with sqrt(mass) for constant density area analogy
    return max(0.5, math.sqrt(abs(mass)))


def _default_color(mass: float) -> Color:
    # Blue for light, yellow/orange for heavy
    m = max(0.0, min(1.0, mass / 10.0))
    r = int(200 * m + 30)
    g = int(160 * m + 30)
    b = int(255 - 200 * m)
    return (r, g, b)


@dataclass
class Body:
    mass: float
    pos: Vector2
    vel: Vector2 = field(default_factory=Vector2.zero)
    radius: Optional[float] = None
    color: Optional[Color] = None
    fixed: bool = False
    name: Optional[str] = None

    def __post_init__(self):
        if self.radius is None:
            self.radius = _default_radius(self.mass)
        if self.color is None:
            self.color = _default_color(self.mass)
        if self.name is None:
            self.name = f'Body(m={self.mass:.3g})'

    def copy(self) -> 'Body':
        return Body(
            mass=self.mass,
            pos=Vector2(self.pos.x, self.pos.y),
            vel=Vector2(self.vel.x, self.vel.y),
            radius=self.radius,
            color=self.color,
            fixed=self.fixed,
            name=self.name,
        )

    def momentum(self) -> Vector2:
        return self.vel * self.mass

    def kinetic_energy(self) -> float:
        return 0.5 * self.mass * self.vel.length_sq()
