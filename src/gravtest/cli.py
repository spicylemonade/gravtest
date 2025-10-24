from __future__ import annotations
import argparse
import sys
import random
import csv
from typing import List, Optional
from .vector import Vector2
from .body import Body
from .world import World
from .simulation import Simulation
from .io import load_world_json, save_world_json


def scenario_two_body(G: float, softening: float) -> World:
    w = World(G=G, softening=softening)
    b1 = Body(mass=1.0, pos=Vector2(-0.5, 0.0), vel=Vector2(0.0, -0.4))
    b2 = Body(mass=1.0, pos=Vector2(0.5, 0.0), vel=Vector2(0.0, 0.4))
    w.add(b1)
    w.add(b2)
    return w


def scenario_circular_orbit(G: float, softening: float) -> World:
    w = World(G=G, softening=softening)
    M = 10.0
    R = 1.0
    v = (G * M / R) ** 0.5
    star = Body(mass=M, pos=Vector2(0.0, 0.0), vel=Vector2(0.0, 0.0), fixed=True, radius=0.2, name="Star")
    sat = Body(mass=0.1, pos=Vector2(R, 0.0), vel=Vector2(0.0, v), radius=0.07, name="Satellite")
    w.add(star)
    w.add(sat)
    return w


def scenario_three_body(G: float, softening: float) -> World:
    w = World(G=G, softening=softening)
    w.add(Body(mass=1.0, pos=Vector2(-1.0, 0.0), vel=Vector2(0.0, 0.3)))
    w.add(Body(mass=1.0, pos=Vector2(1.0, 0.0), vel=Vector2(0.0, -0.3)))
    w.add(Body(mass=0.5, pos=Vector2(0.0, 0.8), vel=Vector2(-0.35, 0.0)))
    return w


def scenario_figure_eight(G: float, softening: float) -> World:
    w = World(G=G, softening=softening)
    p1 = Vector2(-0.97000436, 0.24308753)
    p2 = Vector2(0.97000436, -0.24308753)
    p3 = Vector2(0.0, 0.0)
    v1 = Vector2(0.4662036850, 0.4323657300)
    v2 = Vector2(0.4662036850, 0.4323657300)
    v3 = Vector2(-0.93240737, -0.86473146)
    w.add(Body(mass=1.0, pos=p1, vel=v1, name="A"))
    w.add(Body(mass=1.0, pos=p2, vel=v2, name="B"))
    w.add(Body(mass=1.0, pos=p3, vel=v3, name="C"))
    return w


def scenario_random(n: int, G: float, softening: float, seed: Optional[int] = None) -> World:
    rng = random.Random(seed)
    w = World(G=G, softening=softening)
    for _ in range(n):
        mass = 0.2 + rng.random() * 1.5
        pos = Vector2(rng.uniform(-2, 2), rng.uniform(-2, 2))
        vel = Vector2(rng.uniform(-0.5, 0.5), rng.uniform(-0.5, 0.5))
        w.add(Body(mass=mass, pos=pos, vel=vel))
    return w


def parse_args(argv: List[str]) -> argparse.Namespace:
    ap = argparse.ArgumentParser(prog='gravtest', description='2D gravity simulator')
    ap.add_argument('--scenario', choices=['two-body', 'circular-orbit', 'three-body', 'figure-eight', 'random'], default='circular-orbit')
    ap.add_argument('--scenario-file', type=str, default=None, help='Path to scenario JSON file (overrides --scenario)')
    ap.add_argument('--n', type=int, default=6, help='number of bodies for random scenario')
    ap.add_argument('--steps', type=int, default=5000)
    ap.add_argument('--dt', type=float, default=0.01)
    ap.add_argument('--integrator', choices=['euler', 'rk4', 'leapfrog'], default='euler')
    ap.add_argument('--G', type=float, default=1.0)
    ap.add_argument('--softening', type=float, default=1e-2)
    ap.add_argument('--collisions', action='store_true', help='enable collision handling')
    ap.add_argument('--collision-model', choices=['merge', 'bounce'], default='merge', help='collision handling model')
    ap.add_argument('--restitution', type=float, default=1.0, help='coefficient of restitution for bounce model (0..1)')
    ap.add_argument('--headless', action='store_true', help='run without GUI (no pygame required)')
    ap.add_argument('--steps-per-frame', type=int, default=1, help='GUI: number of physics steps per rendered frame')
    ap.add_argument('--seed', type=int, default=None)
    ap.add_argument('--output-json', type=str, default=None, help='Write final world snapshot to JSON file')
    ap.add_argument('--save-trajectory', type=str, default=None, help='Write CSV trajectory: step,index,name,mass,x,y,vx,vy')
    ap.add_argument('--save-energies', type=str, default=None, help='Write CSV energies: step,K,U,E')
    ap.add_argument('--save-interval', type=int, default=1, help='Record trajectory/energies every N steps')
    ap.add_argument('--force-model', choices=['direct', 'bh'], default='direct', help='Force calculation model')
    ap.add_argument('--theta', type=float, default=0.5, help='Barnes–Hut opening angle parameter (smaller = more accurate)')
    return ap.parse_args(argv)


def build_world(opts: argparse.Namespace) -> World:
    if opts.scenario_file:
        w = load_world_json(opts.scenario_file)
        w.G = opts.G if opts.G is not None else w.G
        w.softening = opts.softening if opts.softening is not None else w.softening
    else:
        if opts.scenario == 'two-body':
            w = scenario_two_body(G=opts.G, softening=opts.softening)
        elif opts.scenario == 'circular-orbit':
            w = scenario_circular_orbit(G=opts.G, softening=opts.softening)
        elif opts.scenario == 'three-body':
            w = scenario_three_body(G=opts.G, softening=opts.softening)
        elif opts.scenario == 'figure-eight':
            w = scenario_figure_eight(G=opts.G, softening=opts.softening)
        elif opts.scenario == 'random':
            w = scenario_random(n=opts.n, G=opts.G, softening=opts.softening, seed=opts.seed)
        else:
            raise ValueError('Unknown scenario')
    w.enable_collisions = bool(opts.collisions) or bool(getattr(w, 'enable_collisions', False))
    w.force_model = str(opts.force_model)
    w.theta = float(opts.theta)
    w.collision_model = str(opts.collision_model)
    w.restitution = float(opts.restitution)
    return w


def headless_run(opts: argparse.Namespace) -> int:
    w = build_world(opts)
    sim = Simulation(w, dt=opts.dt, integrator=opts.integrator)

    csv_file = None
    csv_writer = None
    energies_file = None
    energies_writer = None

    if opts.save_trajectory:
        csv_file = open(opts.save_trajectory, 'w', newline='', encoding='utf-8')
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(['step', 'index', 'name', 'mass', 'x', 'y', 'vx', 'vy'])

    if opts.save_energies:
        energies_file = open(opts.save_energies, 'w', newline='', encoding='utf-8')
        energies_writer = csv.writer(energies_file)
        energies_writer.writerow(['step', 'K', 'U', 'E'])

    interval = max(1, int(opts.save_interval))

    def cb(step_idx: int, world: World):
        if energies_writer is not None and (step_idx % interval == 0):
            energies_writer.writerow([step_idx, world.kinetic_energy(), world.potential_energy(), world.total_energy()])
        if csv_writer is not None and (step_idx % interval == 0):
            for i, b in enumerate(world.bodies):
                csv_writer.writerow([step_idx, i, b.name or '', b.mass, b.pos.x, b.pos.y, b.vel.x, b.vel.y])

    try:
        sim.run(opts.steps, callback=cb if (csv_writer is not None or energies_writer is not None) else None)
    finally:
        if csv_file is not None:
            csv_file.close()
        if energies_file is not None:
            energies_file.close()

    if opts.output_json:
        save_world_json(w, opts.output_json)

    print(f"Final energy: K={w.kinetic_energy():.6g}, U={w.potential_energy():.6g}, E={w.total_energy():.6g}")
    for i, b in enumerate(w.bodies):
        print(f"Body {i}: m={b.mass:.3g}, pos=({b.pos.x:.4g},{b.pos.y:.4g}), vel=({b.vel.x:.4g},{b.vel.y:.4g})")
    return 0


def gui_run(opts: argparse.Namespace) -> int:
    try:
        from .renderer import PygameRenderer
    except Exception as e:
        print(f"Renderer unavailable: {e}. Falling back to headless.")
        return headless_run(opts)
    w = build_world(opts)
    r = PygameRenderer(w)
    r.loop(sim_dt=opts.dt, integrator=opts.integrator, steps_per_frame=max(1, int(opts.steps_per_frame)))
    return 0


def main(argv: List[str] | None = None) -> int:
    opts = parse_args(sys.argv[1:] if argv is None else argv)
    if opts.headless:
        return headless_run(opts)
    else:
        return gui_run(opts)


if __name__ == '__main__':  # pragma: no cover
    raise SystemExit(main())
