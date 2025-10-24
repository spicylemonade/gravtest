from __future__ import annotations
from typing import Tuple, List
from .world import World

try:
    import pygame  # type: ignore
except Exception:  # pragma: no cover - optional dependency
    pygame = None


class PygameRenderer:
    def __init__(self, world: World, width: int = 900, height: int = 700, scale: float = 100.0):
        if pygame is None:
            raise RuntimeError('pygame is not installed. Install it to use the renderer.')
        self.world = world
        self.width = width
        self.height = height
        self.scale = scale  # pixels per simulation unit
        self.center_x = 0.0
        self.center_y = 0.0
        self.bg = (10, 10, 20)
        self.trails_enabled = True
        self.max_trail_len = 200
        self.trails: List[List[Tuple[int, int]]] = []
        self._step_counter = 0
        self._dt = 0.0
        self._integrator = 'euler'
        pygame.init()
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption('GravTest')
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont('consolas', 16)

    def world_to_screen(self, x: float, y: float) -> Tuple[int, int]:
        sx = int(self.width / 2 + (x - self.center_x) * self.scale)
        sy = int(self.height / 2 - (y - self.center_y) * self.scale)
        return (sx, sy)

    def _ensure_trails_size(self):
        # Match number of bodies
        n = len(self.world.bodies)
        while len(self.trails) < n:
            self.trails.append([])
        if len(self.trails) > n:
            self.trails = self.trails[:n]

    def _update_trails(self):
        if not self.trails_enabled:
            return
        self._ensure_trails_size()
        for i, b in enumerate(self.world.bodies):
            sx, sy = self.world_to_screen(b.pos.x, b.pos.y)
            self.trails[i].append((sx, sy))
            if len(self.trails[i]) > self.max_trail_len:
                self.trails[i].pop(0)

    def draw(self):
        self.screen.fill(self.bg)
        # Draw trails first
        if self.trails_enabled:
            for i, trail in enumerate(self.trails):
                if len(trail) > 1:
                    color = (120, 120, 180)
                    pygame.draw.lines(self.screen, color, False, trail, 1)
        # Draw bodies
        for b in self.world.bodies:
            sx, sy = self.world_to_screen(b.pos.x, b.pos.y)
            r = max(1, int(b.radius * self.scale))
            color = b.color or (200, 200, 255)
            pygame.draw.circle(self.screen, color, (sx, sy), r)
        # HUD
        text = f'Bodies: {len(self.world.bodies)}  E={self.world.total_energy():.6g}  dt={self._dt}  int={self._integrator}  step={self._step_counter}'
        surf = self.font.render(text, True, (200, 200, 200))
        self.screen.blit(surf, (10, 10))
        pygame.display.flip()

    def loop(self, sim_dt: float = 0.01, integrator: str = 'euler', steps_per_frame: int = 1):
        self._dt = sim_dt
        self._integrator = integrator
        running = True
        paused = False
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_SPACE:
                        paused = not paused
                    elif event.key in (pygame.K_PLUS, pygame.K_EQUALS):
                        self.scale *= 1.1
                    elif event.key == pygame.K_MINUS:
                        self.scale /= 1.1
                    elif event.key == pygame.K_t:
                        self.trails_enabled = not self.trails_enabled
                    elif event.key == pygame.K_x:
                        self.trails = [[] for _ in self.world.bodies]
                    elif event.key == pygame.K_c:
                        com = self.world.center_of_mass()
                        self.center_x = com.x
                        self.center_y = com.y
                    elif event.key == pygame.K_r:
                        self.center_x = 0.0
                        self.center_y = 0.0
                        self.scale = 100.0
            keys = pygame.key.get_pressed()
            pan_speed = 20.0 / self.scale
            if keys[pygame.K_LEFT]:
                self.center_x -= pan_speed
            if keys[pygame.K_RIGHT]:
                self.center_x += pan_speed
            if keys[pygame.K_UP]:
                self.center_y += pan_speed
            if keys[pygame.K_DOWN]:
                self.center_y -= pan_speed

            if not paused:
                # Fixed timestep, optionally multiple physics updates per frame
                for _ in range(max(1, int(steps_per_frame))):
                    self.world.step(sim_dt, integrator=integrator)
                    self._step_counter += 1
                    self._update_trails()
            self.draw()
            self.clock.tick(60)
        pygame.quit()
