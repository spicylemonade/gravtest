# GravTest

A minimal, testable 2D N-body gravity simulator. Core physics runs headless; an optional pygame renderer provides interactive visualization with pan/zoom, trails, and COM centering.

Features
- Newtonian gravity with Plummer-like softening.
- Integrators: Symplectic Euler (Euler–Cromer), Symplectic Leapfrog (Velocity–Verlet), and RK4.
- Force models: Direct O(N²) summation (default) or Barnes–Hut O(N log N) approximation with configurable opening angle (theta).
- Collisions: Merge (inelastic coalescence) or Bounce (elastic/inelastic with coefficient of restitution).
- CLI with built-in scenarios (including figure-eight), JSON scenario loading, trajectory and energy CSV export, and final snapshot export.
- Unit tests for force law, momentum conservation, energy stability, integrator accuracy (RK4/Leapfrog vs Euler), collision merging and bounce restitution, JSON I/O, and Barnes–Hut accuracy.

Quickstart
- Headless run (direct):
  python run.py --headless --scenario circular-orbit --steps 2000 --dt 0.005 --integrator leapfrog
- Headless run (Barnes–Hut):
  python run.py --headless --scenario random --n 200 --dt 0.005 --steps 5000 --integrator leapfrog --force-model bh --theta 0.5
- Headless run (bounce collisions):
  python run.py --headless --scenario random --n 20 --dt 0.01 --steps 500 --collisions --collision-model bounce --restitution 0.8
- GUI (requires pygame):
  python run.py --scenario circular-orbit --dt 0.01 --integrator leapfrog --steps-per-frame 3

Install as a package (editable) with console script:
- python -m venv .venv && source .venv/bin/activate
- pip install -e .
- gravtest --headless --scenario circular-orbit --steps 1000 --dt 0.01 --integrator leapfrog --force-model bh --theta 0.5

CLI usage
- gravtest --help

Key options
- --scenario {two-body,circular-orbit,three-body,figure-eight,random}
- --scenario-file PATH.json (overrides --scenario)
- --steps, --dt, --integrator {euler,leapfrog,rk4}
- --G, --softening
- --force-model {direct,bh}, --theta FLOAT
- --collisions [--collision-model {merge,bounce}] [--restitution FLOAT]
- --output-json PATH.json (final snapshot)
- --save-trajectory PATH.csv [--save-interval N]
- --save-energies PATH.csv [--save-interval N]
- --headless to run without GUI
- --steps-per-frame N (GUI: physics steps per frame)

Tests
- make test

Paper (LaTeX)
- make paper

License
- MIT
