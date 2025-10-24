from __future__ import annotations
from typing import Dict, Any, List
import json
import csv
from .world import World
from .body import Body
from .vector import Vector2


def body_to_dict(b: Body) -> Dict[str, Any]:
    return {
        "name": b.name,
        "mass": b.mass,
        "pos": [b.pos.x, b.pos.y],
        "vel": [b.vel.x, b.vel.y],
        "radius": b.radius,
        "color": list(b.color) if b.color is not None else None,
        "fixed": bool(b.fixed),
    }


def world_to_dict(w: World) -> Dict[str, Any]:
    return {
        "G": w.G,
        "softening": w.softening,
        "collisions": bool(w.enable_collisions),
        "collision_model": getattr(w, 'collision_model', 'merge'),
        "restitution": getattr(w, 'restitution', 1.0),
        "force_model": getattr(w, 'force_model', 'direct'),
        "theta": getattr(w, 'theta', 0.5),
        "bodies": [body_to_dict(b) for b in w.bodies],
    }


def body_from_dict(d: Dict[str, Any]) -> Body:
    name = d.get("name")
    mass = float(d["mass"])  # required
    pos = Vector2(float(d["pos"][0]), float(d["pos"][1]))
    vel = Vector2(float(d.get("vel", [0.0, 0.0])[0]), float(d.get("vel", [0.0, 0.0])[1]))
    radius = d.get("radius")
    color = tuple(d["color"]) if d.get("color") is not None else None
    fixed = bool(d.get("fixed", False))
    return Body(mass=mass, pos=pos, vel=vel, radius=radius, color=color, fixed=fixed, name=name)


def world_from_dict(d: Dict[str, Any]) -> World:
    w = World(G=float(d.get("G", 1.0)), softening=float(d.get("softening", 1e-2)))
    w.enable_collisions = bool(d.get("collisions", False))
    # Optional extended parameters
    w.force_model = str(d.get("force_model", getattr(w, 'force_model', 'direct')))
    w.theta = float(d.get("theta", getattr(w, 'theta', 0.5)))
    w.collision_model = str(d.get("collision_model", getattr(w, 'collision_model', 'merge')))
    w.restitution = float(d.get("restitution", getattr(w, 'restitution', 1.0)))
    for bd in d.get("bodies", []):
        w.add(body_from_dict(bd))
    return w


def save_world_json(w: World, path: str) -> None:
    with open(path, "w", encoding="utf-8") as f:
        json.dump(world_to_dict(w), f, indent=2)


def load_world_json(path: str) -> World:
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    return world_from_dict(data)


def save_trajectory_csv(path: str, rows: List[Dict[str, Any]]) -> None:
    # rows: dicts with keys step,index,name,mass,x,y,vx,vy
    fieldnames = ["step", "index", "name", "mass", "x", "y", "vx", "vy"]
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for r in rows:
            writer.writerow(r)
