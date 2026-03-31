import math
from gravtest.vector import Vector2
from gravtest.body import Body
from gravtest.world import World


def test_elastic_bounce_equal_mass_head_on():
    w = World(G=0.0, softening=0.0)
    w.enable_collisions = True
    w.collision_model = 'bounce'
    w.restitution = 1.0
    # Equal masses, head-on, overlap to trigger immediate collision handling
    b0 = Body(mass=1.0, pos=Vector2(-0.5, 0.0), vel=Vector2(1.0, 0.0))
    b1 = Body(mass=1.0, pos=Vector2(0.5, 0.0), vel=Vector2(-1.0, 0.0))
    w.add(b0)
    w.add(b1)
    KE0 = w.kinetic_energy()
    w.step(0.0, integrator='euler')  # only collision resolution
    # Velocities should swap (perfectly elastic)
    assert math.isclose(w.bodies[0].vel.x, -1.0, rel_tol=1e-9, abs_tol=1e-12)
    assert math.isclose(w.bodies[1].vel.x, 1.0, rel_tol=1e-9, abs_tol=1e-12)
    KE1 = w.kinetic_energy()
    assert math.isclose(KE1, KE0, rel_tol=1e-9, abs_tol=1e-12)


def test_inelastic_bounce_restitution_scaling():
    e = 0.5
    w = World(G=0.0, softening=0.0)
    w.enable_collisions = True
    w.collision_model = 'bounce'
    w.restitution = e
    b0 = Body(mass=1.0, pos=Vector2(-0.5, 0.0), vel=Vector2(1.0, 0.0))
    b1 = Body(mass=1.0, pos=Vector2(0.5, 0.0), vel=Vector2(-1.0, 0.0))
    w.add(b0)
    w.add(b1)
    KE0 = w.kinetic_energy()
    w.step(0.0, integrator='euler')
    # Final speeds should be +/- e
    assert math.isclose(abs(w.bodies[0].vel.x), e, rel_tol=1e-9, abs_tol=1e-12)
    assert math.isclose(abs(w.bodies[1].vel.x), e, rel_tol=1e-9, abs_tol=1e-12)
    KE1 = w.kinetic_energy()
    assert math.isclose(KE1, (e * e) * KE0, rel_tol=1e-9, abs_tol=1e-12)


def test_bounce_against_fixed_body():
    e = 0.8
    w = World(G=0.0, softening=0.0)
    w.enable_collisions = True
    w.collision_model = 'bounce'
    w.restitution = e
    # Moving body hits a fixed massive body represented as fixed=True
    moving = Body(mass=1.0, pos=Vector2(-0.8, 0.0), vel=Vector2(1.0, 0.0))
    wall = Body(mass=10.0, pos=Vector2(0.0, 0.0), vel=Vector2(0.0, 0.0), fixed=True, radius=1.0)
    w.add(moving)
    w.add(wall)
    w.step(0.0, integrator='euler')
    # Velocity should reverse and scale by e
    assert moving.vel.x < 0.0
    assert math.isclose(abs(moving.vel.x), e * 1.0, rel_tol=1e-9, abs_tol=1e-12)
