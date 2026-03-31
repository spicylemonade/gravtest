import math
import random
from gravtest.vector import Vector2
from gravtest.body import Body
from gravtest.physics import compute_accelerations as compute_acc_direct
from gravtest.barnes_hut import compute_accelerations_bh


def test_bh_two_body_matches_direct():
    G = 1.0
    soft = 1e-3
    b0 = Body(mass=1.0, pos=Vector2(0.0, 0.0))
    b1 = Body(mass=2.0, pos=Vector2(1.0, 0.0))
    bodies = [b0, b1]
    a_dir = compute_acc_direct(bodies, G=G, softening=soft)
    a_bh = compute_accelerations_bh(bodies, G=G, softening=soft, theta=0.3)
    # Exact within tight tolerance
    assert math.isclose(a_dir[0].x, a_bh[0].x, rel_tol=1e-9, abs_tol=1e-12)
    assert math.isclose(a_dir[0].y, a_bh[0].y, rel_tol=1e-9, abs_tol=1e-12)
    assert math.isclose(a_dir[1].x, a_bh[1].x, rel_tol=1e-9, abs_tol=1e-12)
    assert math.isclose(a_dir[1].y, a_bh[1].y, rel_tol=1e-9, abs_tol=1e-12)


def test_bh_cluster_reasonable_accuracy():
    rng = random.Random(42)
    G = 1.0
    soft = 1e-2
    bodies = []
    for _ in range(25):
        m = 0.2 + rng.random() * 2.0
        x = rng.uniform(-1.0, 1.0)
        y = rng.uniform(-1.0, 1.0)
        bodies.append(Body(mass=m, pos=Vector2(x, y)))
    a_dir = compute_acc_direct(bodies, G=G, softening=soft)
    a_bh = compute_accelerations_bh(bodies, G=G, softening=soft, theta=0.5)
    # Compute RMS relative error across nonzero accelerations
    num = 0
    err2_sum = 0.0
    for ad, ab in zip(a_dir, a_bh):
        mag = (ad.x * ad.x + ad.y * ad.y) ** 0.5
        diffx = ad.x - ab.x
        diffy = ad.y - ab.y
        diff = (diffx * diffx + diffy * diffy) ** 0.5
        denom = max(1e-8, mag)
        rel = diff / denom
        err2_sum += rel * rel
        num += 1
    rms_rel = (err2_sum / max(1, num)) ** 0.5
    assert rms_rel < 0.1
