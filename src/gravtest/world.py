from __future__ import annotations
from dataclasses import dataclass, field
from typing import List
from .body import Body
from .vector import Vector2
from .physics import compute_accelerations as compute_acc_direct, potential_energy
from .barnes_hut import compute_accelerations_bh
from .integrators import euler_cromer_step, rk4_step, leapfrog_step


@dataclass
class World:
    bodies: List[Body] = field(default_factory=list)
    G: float = 1.0
    softening: float = 1e-2
    enable_collisions: bool = False
    force_model: str = 'direct'  # 'direct' or 'bh'
    theta: float = 0.5           # Barnes–Hut opening angle parameter
    collision_model: str = 'merge'  # 'merge' or 'bounce'
    restitution: float = 1.0         # coefficient of restitution for 'bounce' collisions

    def add(self, body: Body) -> None:
        self.bodies.append(body)

    def step(self, dt: float, integrator: str = 'euler') -> None:
        if integrator == 'euler':
            euler_cromer_step(self.bodies, dt, G=self.G, softening=self.softening, force_model=self.force_model, theta=self.theta)
        elif integrator == 'rk4':
            rk4_step(self.bodies, dt, G=self.G, softening=self.softening, force_model=self.force_model, theta=self.theta)
        elif integrator in ('leapfrog', 'verlet', 'velocity-verlet'):
            leapfrog_step(self.bodies, dt, G=self.G, softening=self.softening, force_model=self.force_model, theta=self.theta)
        else:
            raise ValueError(f'Unknown integrator: {integrator}')
        if self.enable_collisions:
            self._resolve_collisions()

    def accelerations(self) -> List[Vector2]:
        if self.force_model in ('bh', 'barnes-hut', 'barnes_hut'):
            return compute_accelerations_bh(self.bodies, G=self.G, softening=self.softening, theta=self.theta)
        return compute_acc_direct(self.bodies, G=self.G, softening=self.softening)

    def kinetic_energy(self) -> float:
        return sum(b.kinetic_energy() for b in self.bodies)

    def potential_energy(self) -> float:
        return potential_energy(self.bodies, G=self.G, softening=self.softening)

    def total_energy(self) -> float:
        return self.kinetic_energy() + self.potential_energy()

    def momentum(self) -> Vector2:
        p = Vector2.zero()
        for b in self.bodies:
            p = p + b.momentum()
        return p

    def center_of_mass(self) -> Vector2:
        msum = sum(b.mass for b in self.bodies)
        if msum == 0:
            return Vector2.zero()
        x = sum(b.mass * b.pos.x for b in self.bodies) / msum
        y = sum(b.mass * b.pos.y for b in self.bodies) / msum
        return Vector2(x, y)

    def _resolve_collisions(self) -> None:
        if self.collision_model == 'merge':
            self._resolve_collisions_merge()
        elif self.collision_model == 'bounce':
            self._resolve_collisions_bounce()
        else:
            raise ValueError(f"Unknown collision model: {self.collision_model}")

    def _resolve_collisions_merge(self) -> None:
        # Simple inelastic merge when bodies overlap
        i = 0
        while i < len(self.bodies):
            j = i + 1
            merged = False
            while j < len(self.bodies):
                bi = self.bodies[i]
                bj = self.bodies[j]
                if (bi.pos.distance_to(bj.pos) <= (bi.radius + bj.radius)):
                    m = bi.mass + bj.mass
                    if m == 0:
                        self.bodies.pop(j)
                        self.bodies.pop(i)
                        merged = True
                        break
                    pos = Vector2(
                        (bi.pos.x * bi.mass + bj.pos.x * bj.mass) / m,
                        (bi.pos.y * bi.mass + bj.pos.y * bj.mass) / m,
                    )
                    vel = Vector2(
                        (bi.vel.x * bi.mass + bj.vel.x * bj.mass) / m,
                        (bi.vel.y * bi.mass + bj.vel.y * bj.mass) / m,
                    )
                    radius = (bi.radius ** 2 + bj.radius ** 2) ** 0.5
                    fixed = bi.fixed and bj.fixed
                    name = f'{bi.name}+{bj.name}'
                    color = bi.color if (bi.mass >= bj.mass) else bj.color
                    new_body = Body(mass=m, pos=pos, vel=vel, radius=radius, color=color, fixed=fixed, name=name)
                    self.bodies[i] = new_body
                    self.bodies.pop(j)
                    merged = True
                    break
                else:
                    j += 1
            if not merged:
                i += 1

    def _resolve_collisions_bounce(self) -> None:
        # Elastic/Inelastic collision response using impulse-based resolution for overlapping discs
        e = max(0.0, min(1.0, float(self.restitution)))
        for i in range(len(self.bodies)):
            for j in range(i + 1, len(self.bodies)):
                bi = self.bodies[i]
                bj = self.bodies[j]
                # Skip if both fixed
                if bi.fixed and bj.fixed:
                    continue
                # Check overlap
                dx = bj.pos.x - bi.pos.x
                dy = bj.pos.y - bi.pos.y
                dist2 = dx * dx + dy * dy
                rsum = bi.radius + bj.radius
                if dist2 > rsum * rsum:
                    continue
                # Compute normal
                dist = dist2 ** 0.5
                if dist > 0.0:
                    nx = dx / dist
                    ny = dy / dist
                else:
                    nx, ny = 1.0, 0.0  # arbitrary
                # Positional correction to separate overlapped bodies (proportional to inverse mass)
                inv_mi = 0.0 if bi.fixed or bi.mass == 0 else 1.0 / bi.mass
                inv_mj = 0.0 if bj.fixed or bj.mass == 0 else 1.0 / bj.mass
                inv_sum = inv_mi + inv_mj
                penetration = rsum - dist
                if penetration > 0.0 and inv_sum > 0.0:
                    # Slight slop and bias
                    slop = 1e-6
                    correction_mag = max(0.0, penetration - slop) / inv_sum
                    if not bi.fixed:
                        bi.pos = Vector2(bi.pos.x - nx * correction_mag * inv_mi, bi.pos.y - ny * correction_mag * inv_mi)
                    if not bj.fixed:
                        bj.pos = Vector2(bj.pos.x + nx * correction_mag * inv_mj, bj.pos.y + ny * correction_mag * inv_mj)
                # Relative velocity (vj - vi)
                rvx = bj.vel.x - bi.vel.x
                rvy = bj.vel.y - bi.vel.y
                vn = rvx * nx + rvy * ny
                # If separating, no impulse
                if vn > 0:
                    continue
                denom = inv_sum
                if denom == 0.0:
                    continue
                j_impulse = -(1.0 + e) * vn / denom
                # Apply impulse
                if not bi.fixed:
                    bi.vel = Vector2(bi.vel.x - j_impulse * inv_mi * nx, bi.vel.y - j_impulse * inv_mi * ny)
                if not bj.fixed:
                    bj.vel = Vector2(bj.vel.x + j_impulse * inv_mj * nx, bj.vel.y + j_impulse * inv_mj * ny)
