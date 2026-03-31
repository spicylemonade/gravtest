GravTest Specification

Overview
- A minimal, headless-first 2D N-body gravity simulator with optional pygame renderer.
- Deterministic physics engine with configurable gravitational constant (G), softening, and integrators.
- Integrators: Euler-Cromer (symplectic), Leapfrog/Velocity-Verlet (symplectic), and RK4 (accurate baseline).
- Force models: Direct O(N²) pairwise summation (default) and Barnes–Hut O(N log N) approximation with opening angle parameter (theta).
- Collisions: Merge (inelastic coalescence) and Bounce (elastic/inelastic with coefficient of restitution).
- Command-line interface for scenarios (including figure-eight), headless or GUI runs, and data export to JSON/CSV.
- Unit tests cover force law correctness, momentum conservation, energy stability, integrator comparisons, collision merging and bounce restitution, Barnes–Hut accuracy, and JSON I/O roundtrip.

Collision Models
- Merge: When discs overlap, replace them with a single body (mass, momentum conserved; radius via quadratic area sum).
- Bounce: Impulse-based resolution with positional correction. For overlapping discs with normal n and relative normal velocity v_n, apply impulse j = -(1+e) v_n / (1/m1 + 1/m2), where e is restitution (0..1). Update velocities accordingly, and separate positions proportional to inverse masses.

Acceptance Criteria
- All unit tests pass (pytest), including bounce collision tests.
- CLI supports --collision-model and --restitution; default remains merge.
- JSON serialization includes force model/theta and collision model/restitution when present.
