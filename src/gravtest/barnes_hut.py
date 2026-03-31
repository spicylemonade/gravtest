from __future__ import annotations
from dataclasses import dataclass
from typing import List, Optional
import math
from .vector import Vector2
from .body import Body


@dataclass
class _QuadNode:
    cx: float
    cy: float
    half: float
    mass: float = 0.0
    comx: float = 0.0
    comy: float = 0.0
    body_index: Optional[int] = None
    nw: Optional['_QuadNode'] = None
    ne: Optional['_QuadNode'] = None
    sw: Optional['_QuadNode'] = None
    se: Optional['_QuadNode'] = None

    def is_leaf(self) -> bool:
        return self.nw is None and self.ne is None and self.sw is None and self.se is None

    def _child_for(self, x: float, y: float) -> str:
        # Quadrants: nw (x<cx, y>=cy), ne (x>=cx, y>=cy), sw (x<cx, y<cy), se (x>=cx, y<cy)
        if x < self.cx:
            return 'nw' if y >= self.cy else 'sw'
        else:
            return 'ne' if y >= self.cy else 'se'

    def _ensure_children(self):
        if not self.is_leaf():
            return
        h = self.half * 0.5
        self.nw = _QuadNode(self.cx - h, self.cy + h, h)
        self.ne = _QuadNode(self.cx + h, self.cy + h, h)
        self.sw = _QuadNode(self.cx - h, self.cy - h, h)
        self.se = _QuadNode(self.cx + h, self.cy - h, h)

    def _get_child(self, quad: str) -> '_QuadNode':
        if quad == 'nw':
            return self.nw  # type: ignore[return-value]
        if quad == 'ne':
            return self.ne  # type: ignore[return-value]
        if quad == 'sw':
            return self.sw  # type: ignore[return-value]
        return self.se  # type: ignore[return-value]

    def insert(self, bodies: List[Body], idx: int, max_depth: int = 64, depth: int = 0):
        b = bodies[idx]
        x = b.pos.x
        y = b.pos.y
        m_old = self.mass
        m_new = m_old + b.mass
        if m_new != 0.0:
            if m_old == 0.0:
                self.comx = x
                self.comy = y
            else:
                self.comx = (self.comx * m_old + x * b.mass) / m_new
                self.comy = (self.comy * m_old + y * b.mass) / m_new
        self.mass = m_new

        if self.is_leaf():
            if self.body_index is None:
                self.body_index = idx
                return
            # Already contains a body; need to subdivide unless depth exhausted
            if depth >= max_depth or self.half < 1e-12:
                # Aggregate bodies at this node; treat as internal node with no children
                # Clear body_index to avoid self-force subtraction ambiguity
                self.body_index = None
                return
            old_idx = self.body_index
            self.body_index = None
            self._ensure_children()
            # Reinsert the old body
            quad_old = self._child_for(bodies[old_idx].pos.x, bodies[old_idx].pos.y)
            self._get_child(quad_old).insert(bodies, old_idx, max_depth, depth + 1)
            # Insert the new body
            quad_new = self._child_for(x, y)
            self._get_child(quad_new).insert(bodies, idx, max_depth, depth + 1)
        else:
            quad = self._child_for(x, y)
            self._get_child(quad).insert(bodies, idx, max_depth, depth + 1)


def _build_root(bodies: List[Body]) -> _QuadNode:
    if not bodies:
        return _QuadNode(0.0, 0.0, 1.0)
    min_x = min(b.pos.x for b in bodies)
    max_x = max(b.pos.x for b in bodies)
    min_y = min(b.pos.y for b in bodies)
    max_y = max(b.pos.y for b in bodies)
    width = max(max_x - min_x, max_y - min_y)
    half = max(width * 0.5, 1e-3)
    cx = (min_x + max_x) * 0.5
    cy = (min_y + max_y) * 0.5
    return _QuadNode(cx, cy, half)


def _accumulate_from(node: _QuadNode, bodies: List[Body], i: int, G: float, softening: float, theta: float) -> Vector2:
    bi = bodies[i]
    if node.mass == 0.0:
        return Vector2.zero()
    dx = node.comx - bi.pos.x
    dy = node.comy - bi.pos.y
    d2 = dx * dx + dy * dy
    # Leaf handling
    if node.is_leaf():
        if node.body_index is None:
            return Vector2.zero()
        if node.body_index == i:
            return Vector2.zero()
        r2 = d2 + softening * softening
        inv_r = 1.0 / math.sqrt(r2) if r2 > 0.0 else 0.0
        inv_r3 = inv_r * inv_r * inv_r
        return Vector2(dx * (G * node.mass * inv_r3), dy * (G * node.mass * inv_r3))
    # Internal node: check opening criterion
    d = math.sqrt(d2) if d2 > 0.0 else 0.0
    s = node.half * 2.0
    if d > 0.0 and (s / d) < theta:
        r2 = d2 + softening * softening
        inv_r = 1.0 / math.sqrt(r2)
        inv_r3 = inv_r * inv_r * inv_r
        return Vector2(dx * (G * node.mass * inv_r3), dy * (G * node.mass * inv_r3))
    # Else, recurse to children
    acc = Vector2.zero()
    if node.nw is not None:
        acc = acc + _accumulate_from(node.nw, bodies, i, G, softening, theta)
    if node.ne is not None:
        acc = acc + _accumulate_from(node.ne, bodies, i, G, softening, theta)
    if node.sw is not None:
        acc = acc + _accumulate_from(node.sw, bodies, i, G, softening, theta)
    if node.se is not None:
        acc = acc + _accumulate_from(node.se, bodies, i, G, softening, theta)
    return acc


def compute_accelerations_bh(bodies: List[Body], G: float = 1.0, softening: float = 1e-2, theta: float = 0.5) -> List[Vector2]:
    n = len(bodies)
    acc = [Vector2.zero() for _ in range(n)]
    if n <= 1:
        return acc
    root = _build_root(bodies)
    # Insert all bodies into the quadtree
    for idx in range(n):
        root.insert(bodies, idx)
    # Compute acceleration for each body (fixed bodies: accel=0 but they still contribute)
    for i in range(n):
        if bodies[i].fixed:
            acc[i] = Vector2.zero()
        else:
            acc[i] = _accumulate_from(root, bodies, i, G, softening, theta)
    return acc
