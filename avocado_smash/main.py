"""Avocado Smash: Magnetic Edition.

Run with no hardware for keyboard play, or add --sensor-port after calibration.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import random
import time

from live_sensor import LiveGestureClient

try:
    import pygame
except ImportError:
    print("Install dependencies with: python3 -m pip install -r requirements.txt")
    raise


W, H, FPS = 1280, 720, 60
ROUND_SECONDS = 60.0
ZONE = pygame.Rect(75, 170, 810, 270)
BEST_SCORE = Path(__file__).resolve().parent / ".avocado_smash_score.json"

BG = (15, 23, 42)
PANEL = (30, 41, 59)
PANEL_EDGE = (71, 85, 105)
WHITE = (241, 245, 249)
MUTED = (148, 163, 184)
FRESH, FLESH, PIT = (132, 204, 22), (220, 252, 231), (120, 53, 15)
STEEL, ROTTEN, ORANGE, CYAN = (148, 163, 184), (91, 65, 38), (251, 146, 60), (34, 211, 238)


@dataclass(frozen=True)
class Spec:
    kind: str
    action: str
    label: str
    cue: str
    key: int
    key_name: str
    color: tuple[int, int, int]


SPECS = (
    Spec("fresh", "spread", "FRESH", "SLICE", pygame.K_j, "J", FRESH),
    Spec("armored", "fist", "ARMORED", "SMASH", pygame.K_k, "K", STEEL),
    Spec("rotten", "wrist_up", "ROTTEN", "DEFLECT", pygame.K_f, "F", ORANGE),
)
BY_ACTION = {spec.action: spec for spec in SPECS}


@dataclass
class Target:
    spec: Spec
    opportunity: int
    x: float
    y: float
    vx: float
    vy: float
    rotation: float
    spin: float
    resolved: bool = False
    wrong_penalized: bool = False

    def eligible(self) -> bool:
        return ZONE.collidepoint(int(self.x), int(self.y))


@dataclass
class Particle:
    x: float
    y: float
    vx: float
    vy: float
    life: float
    full_life: float
    color: tuple[int, int, int]


class Controls:
    def __init__(self) -> None:
        self.down: set[str] = set()
        self.scores = {"rest": 0.0, "spread": 0.0, "fist": 0.0, "wrist_up": 0.0}

    def press(self, key: int) -> str | None:
        for spec in SPECS:
            if key == spec.key and spec.action not in self.down:
                self.down.add(spec.action)
                return spec.action
        return None

    def release(self, key: int) -> None:
        for spec in SPECS:
            if spec.key == key:
                self.down.discard(spec.action)

    def set_scores(self, values: dict[str, float]) -> None:
        for key in self.scores:
            self.scores[key] = max(0.0, min(1.0, float(values.get(key, 0.0))))

    def clear_scores(self) -> None:
        self.set_scores({})


class Game:
    def __init__(self, speed: float, sensor_port: str | None, profile: Path | None) -> None:
        self.speed = speed
        self.client = LiveGestureClient(sensor_port, profile) if sensor_port and profile else None
        if self.client:
            self.client.start()
        pygame.init()
        pygame.display.set_caption("Avocado Smash: Magnetic Edition")
        self.window = pygame.display.set_mode((W, H), pygame.RESIZABLE)
        self.canvas = pygame.Surface((W, H))
        self.clock = pygame.time.Clock()
        pygame.key.set_repeat(0)
        self.xl = pygame.font.SysFont("arialblack", 44)
        self.lg = pygame.font.SysFont("arialblack", 27)
        self.md = pygame.font.SysFont("arial", 20, bold=True)
        self.sm = pygame.font.SysFont("arial", 15, bold=True)
        self.tiny = pygame.font.SysFont("arial", 12)
        self.sprites = {spec.kind: self.make_sprite(spec) for spec in SPECS}
        self.controls = Controls()
        self.best = self.read_best()
        self.running, self.state, self.sensor_disabled = True, "menu", False
        self.last_reading = 0.0
        self.reset()

    def reset(self) -> None:
        self.targets: list[Target] = []
        self.particles: list[Particle] = []
        self.score = self.combo = self.longest_combo = self.cleared = 0
        self.elapsed, self.next_spawn, self.countdown = 0.0, 0.85, 0.0
        self.opportunity = 0
        self.message, self.message_color, self.message_time = "", WHITE, 0.0

    def run(self) -> int:
        try:
            while self.running:
                dt = min(0.05, self.clock.tick(FPS) / 1000)
                self.events()
                self.read_sensor()
                if self.state == "playing":
                    self.update_round(dt)
                elif self.state == "countdown":
                    self.countdown -= dt
                    if self.countdown <= 0:
                        self.state = "playing"
                        self.controls.down.clear()
                self.update_particles(dt)
                self.draw()
            return 0
        finally:
            if self.client:
                self.client.stop()
            pygame.quit()

    def events(self) -> None:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYUP:
                self.controls.release(event.key)
            elif event.type == pygame.KEYDOWN:
                self.key(event.key)

    def key(self, key: int) -> None:
        if key == pygame.K_ESCAPE:
            self.running = False
        elif key in (pygame.K_SPACE, pygame.K_RETURN):
            if self.state in ("menu", "results"):
                self.reset()
                self.begin()
            elif self.state == "paused":
                self.start_countdown()
            elif self.state == "sensor_wait" and self.sensor_failed():
                self.sensor_disabled = True
                self.start_countdown()
        elif key == pygame.K_p:
            if self.state == "playing":
                self.state = "paused"
            elif self.state == "paused":
                self.start_countdown()
        else:
            action = self.controls.press(key)
            if action and self.state == "playing":
                self.act(action, "keyboard")

    def begin(self) -> None:
        self.state = "sensor_wait" if self.client and not self.sensor_disabled and not self.sensor_ready() else "countdown"
        if self.state == "countdown":
            self.countdown = 3.0

    def start_countdown(self) -> None:
        self.state, self.countdown = "countdown", 3.0
        self.controls.down.clear()

    def read_sensor(self) -> None:
        if not self.client or self.sensor_disabled:
            return
        for label, onset, scores in self.client.poll():
            self.controls.set_scores(scores)
            self.last_reading = time.monotonic()
            # Polling here consumes actions while menu, pause, and countdown are up.
            if onset and self.state == "playing" and label in BY_ACTION:
                self.act(label, "sensor")
        if self.last_reading and time.monotonic() - self.last_reading > 1.0:
            self.controls.clear_scores()
        if self.state == "sensor_wait" and self.sensor_ready():
            self.start_countdown()

    def update_round(self, dt: float) -> None:
        self.elapsed += dt
        self.message_time = max(0.0, self.message_time - dt)
        if self.elapsed >= ROUND_SECONDS:
            self.finish()
            return
        self.next_spawn -= dt
        if self.next_spawn <= 0:
            self.spawn()
            # A target needs to clear the zone before the next type arrives;
            # this preserves an 0.8-second re-arm gap for the live onset detector.
            self.next_spawn = max(1.45 / self.speed + 0.82, 2.8 - self.elapsed * 0.009)
        for target in self.targets:
            target.x += target.vx * dt * self.speed
            target.y += target.vy * dt * self.speed
            target.vy += 860 * dt * self.speed
            target.rotation += target.spin * dt * self.speed
        self.misses()
        self.targets = [t for t in self.targets if not t.resolved and t.y < H + 120]

    def spawn(self) -> None:
        spec = random.choices(SPECS, weights=(0.52, 0.25, 0.23))[0]
        self.opportunity += 1
        count = random.randint(2, 3) if spec.kind == "fresh" else 1
        # Every launch starts in the lower half and arcs toward a different
        # point in the action zone, so targets enter from distinct angles.
        start_x = random.uniform(90, 865)
        start_y = random.uniform(H * 0.66, H + 95)
        apex_y = random.uniform(ZONE.top + 45, ZONE.top + 125)
        launch_speed = math.sqrt(2 * 860 * (start_y - apex_y))
        apex_time = launch_speed / 860
        center = random.uniform(ZONE.left + 145, ZONE.right - 145)
        for index in range(count):
            offset = (index - (count - 1) / 2) * 76
            apex_x = center + offset
            vx = (apex_x - start_x) / apex_time + random.uniform(-18, 18)
            self.targets.append(Target(spec, self.opportunity, start_x + offset * 0.25, start_y + abs(offset) * 0.12, vx, -launch_speed, random.uniform(-20, 20), random.uniform(-105, 105)))

    def act(self, action: str, source: str) -> None:
        eligible = [target for target in self.targets if not target.resolved and target.eligible()]
        if not eligible:
            return
        correct = [target for target in eligible if target.spec.action == action]
        if not correct:
            if not all(target.wrong_penalized for target in eligible):
                for target in eligible:
                    target.wrong_penalized = True
                self.score, self.combo = max(0, self.score - 5), 0
                self.flash("WRONG MOVE  -5", ORANGE)
            return
        points = 10 * len(correct) * self.multiplier()
        for target in correct:
            target.resolved = True
            self.cleared += 1
            self.burst(target, action)
        self.score += points
        self.combo += 1
        self.longest_combo = max(self.longest_combo, self.combo)
        self.flash(f"{source.upper()} +{points}  x{self.multiplier()}", FRESH)

    def misses(self) -> None:
        gone = [target for target in self.targets if not target.resolved and target.y >= H + 80]
        processed: set[int] = set()
        for target in gone:
            if target.opportunity in processed:
                continue
            processed.add(target.opportunity)
            group = [item for item in self.targets if item.opportunity == target.opportunity and not item.resolved]
            if target.spec.kind == "rotten":
                self.score = max(0, self.score - 10)
                self.flash("ROTTEN HIT  -10", ORANGE)
            else:
                self.combo = 0
                self.flash("MISSED", MUTED)
            for item in group:
                item.resolved = True
                self.add_particles(item.x, min(item.y, H - 20), item.spec.color, 7)

    def multiplier(self) -> int:
        return min(4, 1 + self.combo // 5)

    def flash(self, text: str, color: tuple[int, int, int]) -> None:
        self.message, self.message_color, self.message_time = text, color, 0.9

    def burst(self, target: Target, action: str) -> None:
        if action == "spread":
            self.add_particles(target.x - 15, target.y, FLESH, 12, -95)
            self.add_particles(target.x + 15, target.y, FRESH, 12, 95)
        elif action == "fist":
            self.add_particles(target.x, target.y, STEEL, 20)
        else:
            self.add_particles(target.x, target.y, CYAN, 18, 150)

    def add_particles(self, x: float, y: float, color: tuple[int, int, int], count: int, bias: float = 0) -> None:
        for _ in range(count):
            life = random.uniform(.28, .64)
            self.particles.append(Particle(x, y, random.uniform(-180, 180) + bias, random.uniform(-260, -45), life, life, color))
        self.particles = self.particles[-180:]

    def update_particles(self, dt: float) -> None:
        next_particles = []
        for particle in self.particles:
            particle.life -= dt
            particle.vy += 620 * dt
            particle.x += particle.vx * dt
            particle.y += particle.vy * dt
            if particle.life > 0:
                next_particles.append(particle)
        self.particles = next_particles

    def finish(self) -> None:
        self.state = "results"
        self.targets.clear()
        if self.score > self.best:
            self.best = self.score
            try:
                BEST_SCORE.write_text(json.dumps({"best_score": self.best}) + "\n")
            except OSError:
                pass

    def read_best(self) -> int:
        try:
            return max(0, int(json.loads(BEST_SCORE.read_text()).get("best_score", 0)))
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            return 0

    def sensor_ready(self) -> bool:
        return bool(self.client and self.client.status.startswith("live"))

    def sensor_failed(self) -> bool:
        return bool(self.client and "error" in self.client.status.lower())

    def draw(self) -> None:
        self.canvas.fill(BG)
        self.header()
        self.playfield()
        self.cards()
        self.sensor_panel()
        for particle in self.particles:
            radius = max(1, int(5 * particle.life / particle.full_life))
            pygame.draw.circle(self.canvas, particle.color, (int(particle.x), int(particle.y)), radius)
        if self.state != "playing":
            self.overlay()
        width, height = self.window.get_size()
        scale = min(width / W, height / H)
        size = (int(W * scale), int(H * scale))
        image = pygame.transform.smoothscale(self.canvas, size) if size != (W, H) else self.canvas
        self.window.fill((0, 0, 0))
        self.window.blit(image, ((width - size[0]) // 2, (height - size[1]) // 2))
        pygame.display.flip()

    def header(self) -> None:
        self.text("AVOCADO SMASH", self.xl, WHITE, (48, 18))
        self.text("MAGNETIC EDITION", self.sm, CYAN, (52, 69))
        time_left = max(0, math.ceil(ROUND_SECONDS - self.elapsed))
        self.text(f"SCORE {self.score:04d}     TIME {time_left:02d}s     COMBO {self.combo}     x{self.multiplier()}", self.md, WHITE, (430, 25))
        if self.message_time:
            self.text(self.message, self.sm, self.message_color, (430, 66))

    def playfield(self) -> None:
        board = pygame.Rect(48, 105, 860, 430)
        pygame.draw.rect(self.canvas, PANEL, board, border_radius=18)
        pygame.draw.rect(self.canvas, PANEL_EDGE, board, width=2, border_radius=18)
        pygame.draw.rect(self.canvas, (17, 47, 63), ZONE, border_radius=18)
        pygame.draw.rect(self.canvas, CYAN, ZONE, width=4, border_radius=18)
        self.text("ACT HERE", self.lg, CYAN, (ZONE.centerx, ZONE.top + 17), center=True)
        self.text("Match the target cue while it is inside the zone", self.sm, WHITE, (ZONE.centerx, ZONE.top + 52), center=True)
        for target in self.targets:
            sprite = pygame.transform.rotozoom(self.sprites[target.spec.kind], target.rotation, 1)
            rect = sprite.get_rect(center=(target.x, target.y))
            if target.eligible():
                pygame.draw.ellipse(self.canvas, target.spec.color, rect.inflate(18, 18), width=4)
                self.text(target.spec.cue, self.sm, target.spec.color, (target.x, target.y - 58), center=True)
            self.canvas.blit(sprite, rect)

    def cards(self) -> None:
        for index, spec in enumerate((SPECS[0], SPECS[1], SPECS[2])):
            rect = pygame.Rect(55 + index * 286, 558, 264, 130)
            pygame.draw.rect(self.canvas, PANEL, rect, border_radius=14)
            pygame.draw.rect(self.canvas, spec.color, rect, width=3, border_radius=14)
            key = pygame.Rect(rect.left + 17, rect.top + 18, 42, 36)
            pygame.draw.rect(self.canvas, spec.color, key, border_radius=7)
            self.text(spec.key_name, self.md, BG, key.center, center=True)
            self.text({"spread": "SPREAD", "fist": "FIST", "wrist_up": "WRIST UP"}[spec.action], self.md, WHITE, (rect.left + 72, rect.top + 18))
            self.text(f"{spec.cue} {spec.label}", self.sm, spec.color, (rect.left + 18, rect.top + 73))

    def sensor_panel(self) -> None:
        rect = pygame.Rect(930, 105, 300, 583)
        pygame.draw.rect(self.canvas, PANEL, rect, border_radius=18)
        pygame.draw.rect(self.canvas, PANEL_EDGE, rect, width=2, border_radius=18)
        self.text("LIVE SENSOR", self.lg, WHITE, (952, 127))
        if self.client and not self.sensor_disabled:
            mode, status = ("LIVE", self.client.status) if self.sensor_ready() else ("PREPARING", self.client.status)
        elif self.client:
            mode, status = "KEYBOARD", "keyboard continuation"
        else:
            mode, status = "KEYBOARD", "keyboard controls ready"
        self.text(mode, self.sm, CYAN if mode == "LIVE" else ORANGE, (952, 164))
        self.text(status[:36], self.tiny, MUTED, (952, 185))
        self.text("Relative gesture scores", self.md, WHITE, (952, 226))
        self.text("KNN proximity, not probability", self.tiny, MUTED, (952, 250))
        colors = {"rest": MUTED, "spread": FRESH, "fist": STEEL, "wrist_up": ORANGE}
        labels = (("rest", "Rest"), ("spread", "Spread"), ("fist", "Fist"), ("wrist_up", "Wrist Up"))
        for index, (name, label) in enumerate(labels):
            y, value = 291 + index * 68, self.controls.scores[name]
            self.text(label, self.sm, WHITE, (952, y))
            self.text(f"{value * 100:3.0f}%", self.sm, WHITE, (1205, y), right=True)
            bar = pygame.Rect(952, y + 25, 252, 14)
            pygame.draw.rect(self.canvas, PANEL_EDGE, bar, border_radius=7)
            pygame.draw.rect(self.canvas, colors[name], (bar.left, bar.top, int(bar.width * value), bar.height), border_radius=7)
        self.text("Relax between gestures", self.md, CYAN, (952, 612))
        self.text("Stable gesture onsets trigger actions.", self.tiny, MUTED, (952, 638))

    def overlay(self) -> None:
        layer = pygame.Surface((W, H), pygame.SRCALPHA)
        layer.fill((3, 7, 18, 190)); self.canvas.blit(layer, (0, 0))
        box = pygame.Rect(210, 132, 700, 410)
        pygame.draw.rect(self.canvas, (24, 36, 54), box, border_radius=22)
        pygame.draw.rect(self.canvas, CYAN, box, width=3, border_radius=22)
        if self.state == "menu":
            title, lines = "AVOCADO SMASH", ("SPREAD slices fresh avocados", "FIST smashes armored avocados", "WRIST UP deflects rotten avocados", "Relax between gestures. Press SPACE or ENTER.")
        elif self.state == "sensor_wait":
            title = "SENSOR PREPARING"
            lines = ("The sensor connection failed. Press SPACE or ENTER for keyboard play.",) if self.sensor_failed() else ("Keep your hand relaxed while the baseline is collected.", "The round begins when live readings are ready.")
        elif self.state == "countdown":
            title, lines = str(max(1, math.ceil(self.countdown))), ("Get ready.", "Use F, J, K or your calibrated gestures.")
        elif self.state == "paused":
            title, lines = "PAUSED", ("Press P, SPACE, or ENTER to resume.", "The countdown clears queued gestures before play resumes.")
        else:
            title, lines = "ROUND COMPLETE", (f"Score {self.score}    Best {self.best}", f"Targets cleared {self.cleared}    Longest combo {self.longest_combo}", "Press SPACE or ENTER to play again.")
        self.text(title, self.xl, WHITE, (box.centerx, box.top + 54), center=True)
        for index, line in enumerate(lines):
            self.text(line, self.md, WHITE, (box.centerx, box.top + 150 + index * 48), center=True)

    def text(self, text: str, font: pygame.font.Font, color: tuple[int, int, int], pos: tuple[float, float], center: bool = False, right: bool = False) -> None:
        surface = font.render(text, True, color)
        rect = surface.get_rect()
        if center: rect.center = pos
        elif right: rect.topright = pos
        else: rect.topleft = pos
        self.canvas.blit(surface, rect)

    def make_sprite(self, spec: Spec) -> pygame.Surface:
        surface = pygame.Surface((92, 92), pygame.SRCALPHA)
        outer = pygame.Rect(10, 5, 72, 82)
        shell = ROTTEN if spec.kind == "rotten" else STEEL if spec.kind == "armored" else (55, 100, 30)
        flesh = (101, 84, 47) if spec.kind == "rotten" else FLESH
        pygame.draw.ellipse(surface, shell, outer); pygame.draw.ellipse(surface, flesh, outer.inflate(-12, -12))
        pygame.draw.circle(surface, PIT if spec.kind != "rotten" else (55, 42, 25), (46, 55), 15)
        if spec.kind == "armored":
            pygame.draw.rect(surface, (71, 85, 105), (18, 42, 56, 10), border_radius=4)
            for point in ((27, 30), (65, 30), (25, 65), (67, 65)): pygame.draw.circle(surface, WHITE, point, 4)
        elif spec.kind == "rotten":
            for point in ((29, 34), (63, 40), (37, 68), (58, 62)): pygame.draw.circle(surface, (48, 52, 24), point, 5)
            pygame.draw.arc(surface, ORANGE, (28, 11, 36, 24), math.pi, math.tau, 3)
        else:
            pygame.draw.line(surface, WHITE, (22, 23), (68, 70), 3)
        pygame.draw.circle(surface, spec.color, (46, 55), 10)
        label = self.tiny.render(spec.cue[0], True, BG); surface.blit(label, label.get_rect(center=(46, 55)))
        return surface


def positive_speed(value: str) -> float:
    try: speed = float(value)
    except ValueError as exc: raise argparse.ArgumentTypeError("speed must be a number") from exc
    if not math.isfinite(speed) or speed <= 0: raise argparse.ArgumentTypeError("speed must be greater than zero")
    return speed


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Play Avocado Smash: Magnetic Edition.")
    parser.add_argument("--speed", type=positive_speed, default=1.0)
    parser.add_argument("--sensor-port", help="Optional ESP32 serial port; F, J, and K remain active.")
    parser.add_argument("--profile", type=Path, default=Path(__file__).resolve().parents[1] / "calibration_pipeline" / "profile.json")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    raise SystemExit(Game(args.speed, args.sensor_port, args.profile if args.sensor_port else None).run())
