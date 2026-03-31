from __future__ import annotations
from typing import List
from .vector import Vector2
from .body import Body
from .physics import compute_accelerations as compute_acc_direct
from .barnes_hut import compute_accelerations_bh


def _compute_accel(bodies: List[Body], G: float, softening: float, force_model: str, theta: float) -> List[Vector2]:
    if force_model in ('bh', 'barnes-hut', 'barnes_hut'):
        return compute_accelerations_bh(bodies, G=G, softening=softening, theta=theta)
    return compute_acc_direct(bodies, G=G, softening=softening)


def euler_cromer_step(bodies: List[Body], dt: float, G: float = 1.0, softening: float = 1e-2, force_model: str = 'direct', theta: float = 0.5) -> None:
    acc = _compute_accel(bodies, G=G, softening=softening, force_model=force_model, theta=theta)
    for i, b in enumerate(bodies):
        if b.fixed:
            continue
        a = acc[i]
        new_vel = b.vel + a * dt
        new_pos = b.pos + new_vel * dt
        b.vel = new_vel
        b.pos = new_pos


def _accelerations_at_positions(bodies: List[Body], positions: List[Vector2], G: float, softening: float, force_model: str, theta: float) -> List[Vector2]:
    shadow = []
    for k, b in enumerate(bodies):
        bb = b.copy()
        bb.pos = positions[k]
        shadow.append(bb)
    return _compute_accel(shadow, G=G, softening=softening, force_model=force_model, theta=theta)


def rk4_step(bodies: List[Body], dt: float, G: float = 1.0, softening: float = 1e-2, force_model: str = 'direct', theta: float = 0.5) -> None:
    pos0 = [b.pos for b in bodies]
    vel0 = [b.vel for b in bodies]

    def add_lists(a, b):
        return [ai + bi for ai, bi in zip(a, b)]

    def scale_list(a, s: float):
        return [ai * s for ai in a]

    # k1
    k1_v = _accelerations_at_positions(bodies, pos0, G, softening, force_model, theta)
    k1_x = vel0

    # k2
    pos_k2 = add_lists(pos0, scale_list(k1_x, dt * 0.5))
    vel_k2 = add_lists(vel0, scale_list(k1_v, dt * 0.5))
    k2_v = _accelerations_at_positions(bodies, pos_k2, G, softening, force_model, theta)
    k2_x = vel_k2

    # k3
    pos_k3 = add_lists(pos0, scale_list(k2_x, dt * 0.5))
    vel_k3 = add_lists(vel0, scale_list(k2_v, dt * 0.5))
    k3_v = _accelerations_at_positions(bodies, pos_k3, G, softening, force_model, theta)
    k3_x = vel_k3

    # k4
    pos_k4 = add_lists(pos0, scale_list(k3_x, dt))
    vel_k4 = add_lists(vel0, scale_list(k3_v, dt))
    k4_v = _accelerations_at_positions(bodies, pos_k4, G, softening, force_model, theta)
    k4_x = vel_k4

    for i, b in enumerate(bodies):
        if b.fixed:
            continue
        dv = (k1_v[i] + k2_v[i] * 2.0 + k3_v[i] * 2.0 + k4_v[i]) * (dt / 6.0)
        dx = (k1_x[i] + k2_x[i] * 2.0 + k3_x[i] * 2.0 + k4_x[i]) * (dt / 6.0)
        b.vel = b.vel + dv
        b.pos = b.pos + dx


def leapfrog_step(bodies: List[Body], dt: float, G: float = 1.0, softening: float = 1e-2, force_model: str = 'direct', theta: float = 0.5) -> None:
    a0 = _compute_accel(bodies, G=G, softening=softening, force_model=force_model, theta=theta)
    vhalf = []
    for i, b in enumerate(bodies):
        if b.fixed:
            vhalf.append(b.vel)
        else:
            vhalf.append(b.vel + a0[i] * (0.5 * dt))
    # Drift positions
    for i, b in enumerate(bodies):
        if b.fixed:
            continue
        b.pos = b.pos + vhalf[i] * dt
    # Acceleration at new positions
    a1 = _compute_accel(bodies, G=G, softening=softening, force_model=force_model, theta=theta)
    # Final half-kick
    for i, b in enumerate(bodies):
        if b.fixed:
            continue
        b.vel = vhalf[i] + a1[i] * (0.5 * dt)
