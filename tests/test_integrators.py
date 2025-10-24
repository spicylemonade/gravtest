import math
from gravtest.vector import Vector2
from gravtest.body import Body
from gravtest.world import World
from gravtest.simulation import Simulation


def _circular_world():
    G = 1.0
    w = World(G=G, softening=1e-3)
    M = 10.0
    R = 1.0
    v = (G * M / R) ** 0.5
    star = Body(mass=M, pos=Vector2(0.0, 0.0), fixed=True, radius=0.2)
    sat = Body(mass=0.1, pos=Vector2(R, 0.0), vel=Vector2(0.0, v), radius=0.07)
    w.add(star)
    w.add(sat)
    return w


def test_rk4_energy_drift_less_than_euler():
    steps = 1500
    dt = 0.01
    # Euler-Cromer
    w1 = _circular_world()
    sim1 = Simulation(w1, dt=dt, integrator='euler')
    E0 = w1.total_energy()
    for _ in range(steps):
        sim1.step()
    Ee = w1.total_energy()
    drift_euler = abs(Ee - E0)

    # RK4
    w2 = _circular_world()
    sim2 = Simulation(w2, dt=dt, integrator='rk4')
    E0b = w2.total_energy()
    for _ in range(steps):
        sim2.step()
    Er = w2.total_energy()
    drift_rk4 = abs(Er - E0b)

    # RK4 should be at least as good and typically much better
    assert drift_rk4 <= drift_euler


def test_leapfrog_energy_drift_less_than_euler():
    steps = 1500
    dt = 0.01
    # Euler-Cromer baseline
    w1 = _circular_world()
    sim1 = Simulation(w1, dt=dt, integrator='euler')
    E0 = w1.total_energy()
    for _ in range(steps):
        sim1.step()
    Ee = w1.total_energy()
    drift_euler = abs(Ee - E0)

    # Leapfrog (velocity-Verlet)
    w2 = _circular_world()
    sim2 = Simulation(w2, dt=dt, integrator='leapfrog')
    E0b = w2.total_energy()
    for _ in range(steps):
        sim2.step()
    El = w2.total_energy()
    drift_leap = abs(El - E0b)

    assert drift_leap <= drift_euler
