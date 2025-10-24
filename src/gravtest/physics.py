from __future__ import annotations
from typing import List
import math
from .vector import Vector2
from .body import Body


def compute_accelerations(bodies: List[Body], G: float = 1.0, softening: float = 1e-2) -> List[Vector2]:
    n = len(bodies)
    acc = [Vector2.zero() for _ in range(n)]
    if n <= 1:
        return acc
    eps2 = softening * softening
    for i in range(n):
        bi = bodies[i]
        for j in range(i + 1, n):
            bj = bodies[j]
            rij = bj.pos - bi.pos
            r2 = rij.length_sq() + eps2
            inv_r = 1.0 / math.sqrt(r2)
            inv_r3 = inv_r * inv_r * inv_r
            # Acceleration contribution
            if not bi.fixed:
                acc[i] = acc[i] + (rij * (G * bj.mass * inv_r3))
            if not bj.fixed:
                acc[j] = acc[j] - (rij * (G * bi.mass * inv_r3))
    return acc


def potential_energy(bodies: List[Body], G: float = 1.0, softening: float = 1e-2) -> float:
    n = len(bodies)
    if n <= 1:
        return 0.0
    eps2 = softening * softening
    U = 0.0
    for i in range(n):
        bi = bodies[i]
        for j in range(i + 1, n):
            bj = bodies[j]
            r2 = (bj.pos - bi.pos).length_sq() + eps2
            r = math.sqrt(r2)
            U -= G * bi.mass * bj.mass / r
    return U
