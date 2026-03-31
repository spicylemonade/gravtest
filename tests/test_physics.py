import math
from gravtest.vector import Vector2
from gravtest.body import Body
from gravtest.world import World
from gravtest.simulation import Simulation


def test_gravitational_acceleration_magnitude():
    # Two bodies at unit distance, no softening
    G = 2.0
    w = World(G=G, softening=0.0)
    b0 = Body(mass=1.0, pos=Vector2(0.0, 0.0))
    b1 = Body(mass=2.0, pos=Vector2(1.0, 0.0))
    w.add(b0)
    w.add(b1)
    acc = w.accelerations()
    # |a0| = G*m1/r^2 = 2*2/1 = 4, |a1| = G*m0/r^2 = 2*1/1 = 2
    assert math.isclose(acc[0].length(), 4.0, rel_tol=1e-9, abs_tol=1e-12)
    assert math.isclose(acc[1].length(), 2.0, rel_tol=1e-9, abs_tol=1e-12)


def test_momentum_conservation_two_body():
    # No external forces, internal gravity should conserve total momentum
    w = World(G=1.0, softening=0.0)
    b0 = Body(mass=1.0, pos=Vector2(-0.5, 0.0), vel=Vector2(0.0, 0.0))
    b1 = Body(mass=1.0, pos=Vector2(0.5, 0.0), vel=Vector2(0.0, 0.0))
    w.add(b0)
    w.add(b1)
    sim = Simulation(w, dt=0.001, integrator='euler')
    for _ in range(5000):
        sim.step()
    p = w.momentum()
    assert p.length() < 1e-8


def test_energy_stability_circular_orbit():
    # Satellite orbiting a fixed central mass should have nearly constant total energy with small dt
    G = 1.0
    w = World(G=G, softening=1e-3)
    M = 10.0
    R = 1.0
    v = (G * M / R) ** 0.5
    star = Body(mass=M, pos=Vector2(0.0, 0.0), vel=Vector2(0.0, 0.0), fixed=True, radius=0.2)
    sat = Body(mass=0.1, pos=Vector2(R, 0.0), vel=Vector2(0.0, v), radius=0.07)
    w.add(star)
    w.add(sat)
    sim = Simulation(w, dt=0.002, integrator='euler')
    E0 = w.total_energy()
    for _ in range(3000):
        sim.step()
    E1 = w.total_energy()
    rel_err = abs(E1 - E0) / max(1e-12, abs(E0))
    assert rel_err < 1e-2
