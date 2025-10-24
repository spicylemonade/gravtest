import math
from gravtest.vector import Vector2
from gravtest.body import Body
from gravtest.world import World
from gravtest.simulation import Simulation


def make_figure_eight_world():
    # Equal masses, figure-eight initial conditions (Simo 2000)
    w = World(G=1.0, softening=1e-3)
    p1 = Vector2(-0.97000436, 0.24308753)
    p2 = Vector2(0.97000436, -0.24308753)
    p3 = Vector2(0.0, 0.0)
    v1 = Vector2(0.4662036850, 0.4323657300)
    v2 = Vector2(0.4662036850, 0.4323657300)
    v3 = Vector2(-0.93240737, -0.86473146)
    w.add(Body(mass=1.0, pos=p1, vel=v1))
    w.add(Body(mass=1.0, pos=p2, vel=v2))
    w.add(Body(mass=1.0, pos=p3, vel=v3))
    return w


def test_figure_eight_momentum_and_energy_stability():
    w = make_figure_eight_world()
    sim = Simulation(w, dt=0.005, integrator='leapfrog')
    p0 = w.momentum()
    E0 = w.total_energy()
    for _ in range(1000):
        sim.step()
    p1 = w.momentum()
    E1 = w.total_energy()
    # Momentum approximately conserved (internal forces only)
    assert p1.length() < 1e-3
    # Energy should remain finite and near initial value
    assert math.isfinite(E1)
    rel_err = abs(E1 - E0) / max(1e-12, abs(E0))
    assert rel_err < 5e-2
