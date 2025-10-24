from __future__ import annotations
from dataclasses import dataclass
import math

@dataclass(frozen=True)
class Vector2:
    x: float
    y: float

    def __add__(self, other: 'Vector2') -> 'Vector2':
        return Vector2(self.x + other.x, self.y + other.y)

    def __sub__(self, other: 'Vector2') -> 'Vector2':
        return Vector2(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar: float) -> 'Vector2':
        return Vector2(self.x * scalar, self.y * scalar)

    def __rmul__(self, scalar: float) -> 'Vector2':
        return self.__mul__(scalar)

    def __truediv__(self, scalar: float) -> 'Vector2':
        return Vector2(self.x / scalar, self.y / scalar)

    def length(self) -> float:
        return math.hypot(self.x, self.y)

    def length_sq(self) -> float:
        return self.x * self.x + self.y * self.y

    def normalized(self) -> 'Vector2':
        l = self.length()
        if l == 0.0:
            return Vector2(0.0, 0.0)
        return self / l

    def distance_to(self, other: 'Vector2') -> float:
        return (self - other).length()

    def distance_sq_to(self, other: 'Vector2') -> float:
        return (self - other).length_sq()

    def as_tuple(self):
        return (self.x, self.y)

    @staticmethod
    def zero() -> 'Vector2':
        return Vector2(0.0, 0.0)

    @staticmethod
    def from_tuple(t) -> 'Vector2':
        return Vector2(float(t[0]), float(t[1]))
