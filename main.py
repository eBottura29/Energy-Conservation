from pg_extensions import *
import json

class Settings:
    g = 0
    BOUNDS = Vector2(0, 0)
    PARTICLE_COUNT = 0
    DENSITY = 0
    ENERGY_LOSS_PARTICLES = 0
    ENERGY_LOSS_BOUNDS = 0
    DRAG_COEFFICIENT = 0

class Particle:
    def __init__(self, position, velocity, radius, mass, color):
        self.position = position
        self.velocity = velocity
        self.radius = radius
        self.mass = mass
        self.color = color

    def handle_boundary_collisions(self):
        half_bounds_size = Settings.BOUNDS / 2 - Vector2(1, 1) * self.radius

        if abs(self.position.x) >= half_bounds_size.x:
            self.position.x = half_bounds_size.x * sign(self.position.x)
            self.velocity.x *= -1 * Settings.ENERGY_LOSS_BOUNDS
        if abs(self.position.y) >= half_bounds_size.y:
            self.position.y = half_bounds_size.y * sign(self.position.y)
            self.velocity.y *= -1 * Settings.ENERGY_LOSS_BOUNDS

    def handle_particle_collisions(self, other):
        dst = other.position - self.position
        dir = dst.normalize()
        overlap = self.radius + other.radius - dst.magnitude()
        self.position -= dir * overlap / 2
        other.position += dir * overlap / 2

        v1 = -(other.mass * other.velocity.magnitude()**2 / self.mass)**0.5  # Derived from T1 = T2, m1 v1^2 = m2 v2^2
        v2 = (self.mass * self.velocity.magnitude()**2 / other.mass)**0.5  # Derived from T1 = T2, m1 v1^2 = m2 v2^2

        if dst.magnitude() == 0:
            dir = Vector2.random_polar()

        self.velocity = dir * v1 * Settings.ENERGY_LOSS_PARTICLES
        other.velocity = dir * v2 * Settings.ENERGY_LOSS_PARTICLES

    def detect_particle_collisions(self, particles):
        for other in particles:
            if self == other:
                continue

            dst = other.position - self.position

            if dst.magnitude() < self.radius + other.radius:
                self.handle_particle_collisions(other)

    def update(self, particles):
        # drag
        drag = 0.5 * Settings.DRAG_COEFFICIENT * math.pi * self.radius * self.velocity.sqr_magnitude()
        dir = self.velocity.normalize() * -1

        # update position and velocity using RK4
        self.acceleration = Vector2(0, -Settings.g) + dir * drag / self.mass
        self.position, self.velocity = runge_kutta_4(self.position, self.velocity, self.acceleration, window.delta_time)

        # collisions
        self.handle_boundary_collisions()
        self.detect_particle_collisions(particles)

        self.render()

    def render(self):
        draw_circle(window.SURFACE, self.color, self.position, self.radius)

def update_ui():
    fps_text = Text(f"FPS: {window.clock.get_fps():.2f}",
        Text.arial_24,
        Vector2(-window.WIDTH // 2 + 32, window.HEIGHT // 2 - 32 * 1),
        Text.top_left,
        WHITE,
        BLACK,
    )
    fps_text.render()

    T = 0
    for p in particles:
        T += 0.5 * p.mass * p.velocity.sqr_magnitude()  # T = (1/2)mv² (kinetic energy)
    T /= Settings.PARTICLE_COUNT
    kinetic_energy_text = Text(f"T: {T:.2e}",
        Text.arial_24,
        Vector2(-window.WIDTH // 2 + 32, window.HEIGHT // 2 - 32 * 2),
        Text.top_left,
        WHITE,
        BLACK,
    )
    kinetic_energy_text.render()

    V = 0
    for p in particles:
        h = abs(p.position.y - Settings.BOUNDS.y / 2)
        V += p.mass * Settings.g * h  # V = mgh (potential energy)
    V /= Settings.PARTICLE_COUNT
    potential_energy_text = Text(f"V: {V:.2e}",
        Text.arial_24,
        Vector2(-window.WIDTH // 2 + 32, window.HEIGHT // 2 - 32 * 3),
        Text.top_left,
        WHITE,
        BLACK,
    )
    potential_energy_text.render()

    total_energy_text = Text(f"E: {T + V:.2e}",
        Text.arial_24,
        Vector2(-window.WIDTH // 2 + 32, window.HEIGHT // 2 - 32 * 4),
        Text.top_left,
        WHITE,
        BLACK,
    )
    total_energy_text.render()

def start():
    global particles

    with open("config.json", "r") as f:
        contents = json.load(f)
        Settings.g = contents["g"]
        Settings.BOUNDS = Vector2(contents["bounds"]["width"], contents["bounds"]["height"])
        Settings.PARTICLE_COUNT = contents["particle_count"]
        Settings.DENSITY = contents["density"]
        Settings.ENERGY_LOSS_PARTICLES = contents["energy_loss_particles"]
        Settings.ENERGY_LOSS_BOUNDS = contents["energy_loss_bounds"]
        Settings.DRAG_COEFFICIENT = contents["drag_coefficient"]

    particles = []
    for _ in range(Settings.PARTICLE_COUNT):
        r = random_float(20, 40)
        pos = Vector2.random(-Settings.BOUNDS/2, Settings.BOUNDS/2)
        vel = Vector2.random_polar(0, math.tau, 50, 150)
        particles.append(Particle(pos, vel, r, math.pi * r**2 * Settings.DENSITY, WHITE))

def update():
    global window
    window = get_window()
    window.clear(BLACK)

    if input_manager.get_key_down(pygame.K_ESCAPE):
        window.running = False

    for p in particles:
        p.update(particles)

    draw_rectangle(window.SURFACE, GREEN, Vector2(-Settings.BOUNDS.x / 2, Settings.BOUNDS.y / 2), Settings.BOUNDS, 2)

    update_ui()

    set_window(window)

if __name__ == '__main__':
    run(start, update, 1280, 720, False, 'Energy Conservation', 999)
