import math
from gravtest.vector import Vector2
from gravtest.body import Body
from gravtest.world import World
from gravtest.io import world_to_dict, world_from_dict, save_world_json, load_world_json


def test_collision_merge_properties(tmp_path):
    w = World(G=1.0, softening=1e-2)
    w.enable_collisions = True
    # Equal masses with overlapping radii; zero dt to avoid motion
    b0 = Body(mass=1.0, pos=Vector2(-0.1, 0.0), vel=Vector2(0.5, 0.0))
    b1 = Body(mass=1.0, pos=Vector2(0.1, 0.0), vel=Vector2(-0.5, 0.0))
    w.add(b0)
    w.add(b1)
    p0 = w.momentum()
    w.step(0.0, integrator='euler')
    assert len(w.bodies) == 1
    b = w.bodies[0]
    assert math.isclose(b.mass, 2.0, rel_tol=1e-12, abs_tol=1e-12)
    # Momentum preserved
    p1 = w.momentum()
    assert math.isclose(p0.x, p1.x, rel_tol=1e-9, abs_tol=1e-12)
    assert math.isclose(p0.y, p1.y, rel_tol=1e-9, abs_tol=1e-12)
    # Radius is quadrature of original radii (each ~1 for mass=1)
    assert math.isclose(b.radius, (b0.radius**2 + b1.radius**2) ** 0.5, rel_tol=1e-9, abs_tol=1e-12)


def test_world_json_roundtrip(tmp_path):
    w = World(G=1.23, softening=0.0045)
    w.enable_collisions = True
    w.add(Body(mass=0.5, pos=Vector2(1.0, -2.0), vel=Vector2(0.1, 0.2), radius=0.3, name='A'))
    w.add(Body(mass=2.0, pos=Vector2(-3.0, 4.0), vel=Vector2(-0.2, 0.1), radius=0.7, name='B', fixed=True))

    path = tmp_path / 'world.json'
    save_world_json(w, str(path))
    w2 = load_world_json(str(path))

    assert math.isclose(w.G, w2.G, rel_tol=1e-12)
    assert math.isclose(w.softening, w2.softening, rel_tol=1e-12)
    assert w.enable_collisions == w2.enable_collisions
    assert len(w2.bodies) == 2

    for b, c in zip(w.bodies, w2.bodies):
        assert math.isclose(b.mass, c.mass, rel_tol=1e-12)
        assert math.isclose(b.pos.x, c.pos.x, rel_tol=1e-12)
        assert math.isclose(b.pos.y, c.pos.y, rel_tol=1e-12)
        assert math.isclose(b.vel.x, c.vel.x, rel_tol=1e-12)
        assert math.isclose(b.vel.y, c.vel.y, rel_tol=1e-12)
        assert b.fixed == c.fixed
        assert (b.name or '') == (c.name or '')
