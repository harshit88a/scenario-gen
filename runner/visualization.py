"""
Pygame display: RGB camera views, LiDAR bird's-eye view, and HUD overlay.

Display modes (cycled with TAB)
--------------------------------
behind  – chase camera from behind the ego
top     – elevated chase camera
lidar   – top-down LiDAR point-cloud (BEV)
"""

import math
import time

import carla
import numpy as np
import pygame

from .utils import get_speed, compute_ttc
from .termination import TerminationChecker


# ─────────────────────────────────────────────────────────────────────────────
# Constants
# ─────────────────────────────────────────────────────────────────────────────

W, H = 800, 600            # pygame window resolution
BEV_RANGE = 40.0           # metres shown in each direction for the BEV display
BEV_SCALE = min(W, H) / (2 * BEV_RANGE)   # pixels per metre

CAMERA_TRANSFORMS = {
    "behind": carla.Transform(
        carla.Location(x=-6, y=0, z=3.0), carla.Rotation(pitch=-10)
    ),
    # Elevated chase cam: above and behind, tilted forward so you see the road ahead
    "top": carla.Transform(
        carla.Location(x=-4, y=0, z=6.0), carla.Rotation(pitch=-25)
    ),
    "front": carla.Transform(
        carla.Location(x=0, y=0, z=2.0), carla.Rotation(pitch=0)
    ),
}

# Ordered cycle for TAB key
DISPLAY_MODES = ["behind", "top", "lidar"]


# ─────────────────────────────────────────────────────────────────────────────
# Sensor attachment
# ─────────────────────────────────────────────────────────────────────────────

def attach_display_camera(world, blueprint_library, vehicle, view: str) -> carla.Actor:
    """Attach an RGB camera for the given *view* position."""
    bp = blueprint_library.find("sensor.camera.rgb")
    bp.set_attribute("image_size_x", str(W))
    bp.set_attribute("image_size_y", str(H))
    bp.set_attribute("fov", "100")
    tf = CAMERA_TRANSFORMS.get(view, CAMERA_TRANSFORMS["behind"])
    return world.spawn_actor(bp, tf, attach_to=vehicle)


def attach_bev_lidar(world, blueprint_library, vehicle) -> carla.Actor:
    """Attach a dedicated LiDAR sensor for the bird's-eye view display."""
    bp = blueprint_library.find("sensor.lidar.ray_cast")
    bp.set_attribute("range",              "50")
    bp.set_attribute("channels",           "32")
    bp.set_attribute("points_per_second",  "100000")
    bp.set_attribute("rotation_frequency", "20")
    bp.set_attribute("upper_fov",          "2")
    bp.set_attribute("lower_fov",          "-24")
    tf = carla.Transform(carla.Location(x=0, y=0, z=2.0))
    return world.spawn_actor(bp, tf, attach_to=vehicle)


# ─────────────────────────────────────────────────────────────────────────────
# Render functions
# ─────────────────────────────────────────────────────────────────────────────

def render_bev(
    surface: pygame.Surface,
    points: np.ndarray,
    ego,
    adversaries: list,
) -> None:
    """
    Render a top-down LiDAR view onto *surface*.

    CARLA sensor frame: x=forward, y=right, z=up
    Screen mapping:     x → up,    y → right
    """
    surface.fill((15, 15, 20))
    cx, cy = W // 2, H // 2

    # Grid rings
    for r_m in (10, 20, 30, 40):
        r_px = int(r_m * BEV_SCALE)
        pygame.draw.circle(surface, (40, 40, 50), (cx, cy), r_px, 1)

    # Axis lines
    pygame.draw.line(surface, (40, 40, 50), (cx, 0), (cx, H), 1)
    pygame.draw.line(surface, (40, 40, 50), (0, cy), (W, cy), 1)

    # LiDAR points — coloured by height
    if points is not None and len(points) > 0:
        xs = points[:, 0]   # forward
        ys = points[:, 1]   # right
        zs = points[:, 2]   # up
        mask = (
            (zs > -2.0) & (zs < 3.0)
            & (np.abs(xs) < BEV_RANGE)
            & (np.abs(ys) < BEV_RANGE)
        )
        xs, ys, zs = xs[mask], ys[mask], zs[mask]

        if len(xs):
            # Map z → colour  (low=blue, mid=green, high=red)
            z_norm = np.clip((zs + 0.5) / 3.5, 0, 1)
            r = (z_norm * 220).astype(np.uint8)
            g = ((1 - np.abs(z_norm - 0.5) * 2) * 200).astype(np.uint8)
            b = ((1 - z_norm) * 200).astype(np.uint8)
            colours = np.stack([r, g, b], axis=1)

            sx = (cx + ys * BEV_SCALE).astype(int)   # y (right) → screen x
            sy = (cy - xs * BEV_SCALE).astype(int)   # x (fwd)   → screen y (up)

            for i in range(len(sx)):
                if 0 <= sx[i] < W and 0 <= sy[i] < H:
                    surface.set_at(
                        (sx[i], sy[i]),
                        (int(colours[i, 0]), int(colours[i, 1]), int(colours[i, 2])),
                    )

    # Ego marker (white rectangle + arrow)
    pygame.draw.rect(surface, (240, 240, 240), (cx - 4, cy - 8, 8, 14), 0)
    pygame.draw.polygon(
        surface, (240, 240, 240),
        [(cx, cy - 14), (cx - 5, cy - 8), (cx + 5, cy - 8)],
    )

    # Adversary markers in ego-relative coords
    ego_tf      = ego.get_transform()
    ego_yaw_rad = math.radians(ego_tf.rotation.yaw)
    cos_y       = math.cos(-ego_yaw_rad)
    sin_y       = math.sin(-ego_yaw_rad)

    for adv in adversaries:
        adv_loc = adv.get_location()
        dx = adv_loc.x - ego_tf.location.x
        dy = adv_loc.y - ego_tf.location.y
        rx =  cos_y * dx - sin_y * dy   # forward in ego frame
        ry =  sin_y * dx + cos_y * dy   # right in ego frame
        sx_adv = int(cx + ry * BEV_SCALE)
        sy_adv = int(cy - rx * BEV_SCALE)
        if 0 <= sx_adv < W and 0 <= sy_adv < H:
            pygame.draw.rect(
                surface, (255, 80, 80), (sx_adv - 4, sy_adv - 7, 8, 12), 0
            )

    # Range label
    font_small = pygame.font.SysFont("monospace", 11)
    surface.blit(
        font_small.render(f"±{int(BEV_RANGE)}m", True, (80, 80, 100)),
        (W - 50, H - 18),
    )


def render_hud(
    surface: pygame.Surface,
    font: pygame.font.Font,
    ego,
    adversaries: list,
    cm,
    tc: TerminationChecker,
    display_mode: str,
    episode_num: int = 1,
) -> None:
    """Draw the telemetry overlay onto *surface*."""
    speed_kmh = get_speed(ego) * 3.6
    ttc_str   = f"{compute_ttc(ego, adversaries[0]):.2f}s" if adversaries else "N/A"
    dist      = ego.get_location().distance(tc.start_loc)
    elapsed   = tc.elapsed()
    min_left  = max(0.0, float(tc.cfg.get("min_duration_seconds", 0)) - elapsed)

    lines = [
        f"Episode:  {episode_num}",
        f"Speed:    {speed_kmh:5.1f} km/h",
        f"TTC:      {ttc_str}",
        f"Ego dist: {dist:5.1f} m",
        f"Time:     {elapsed:5.1f} s",
        f"Collision:{'YES ⚠' if cm.collision_occurred else ' No'}",
    ]
    if min_left > 0:
        lines.append(f"Min left: {min_left:.0f} s")

    for i, adv in enumerate(adversaries):
        adv_d   = ego.get_location().distance(adv.get_location())
        adv_spd = get_speed(adv) * 3.6
        lines.append(f"Adv-{i}: {adv_d:5.1f} m  {adv_spd:.1f}km/h")

    lines += ["", f"[{display_mode.upper()}] TAB=cycle N=skip"]

    overlay = pygame.Surface((210, len(lines) * 20 + 12), pygame.SRCALPHA)
    overlay.fill((0, 0, 0, 155))
    surface.blit(overlay, (5, 5))
    for i, line in enumerate(lines):
        col = (255, 80, 80) if ("YES" in line or "Adv" in line) else (210, 210, 60)
        surface.blit(font.render(line, True, col), (10, 10 + i * 20))


def show_intermission(
    screen: pygame.Surface,
    font: pygame.font.Font,
    episode_num: int,
    duration: float = 1.0,
) -> bool:
    """
    Show a brief between-episode countdown screen.
    Returns True to continue, False if the user pressed ESC.
    N key skips the countdown immediately.
    """
    t0    = time.time()
    clock = pygame.time.Clock()
    while True:
        remaining = duration - (time.time() - t0)
        if remaining <= 0:
            return True

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    return False
                if event.key == pygame.K_n:
                    return True  # skip remaining countdown

        screen.fill((10, 10, 18))
        big   = pygame.font.SysFont("monospace", 36, bold=True)
        small = pygame.font.SysFont("monospace", 20)
        screen.blit(
            big.render(f"Episode {episode_num} complete", True, (180, 180, 60)),
            (W // 2 - 210, H // 2 - 60),
        )
        screen.blit(
            small.render(f"Next in {remaining:.1f}s  (N = skip now)", True, (120, 200, 120)),
            (W // 2 - 210, H // 2),
        )
        screen.blit(
            small.render("ESC = quit", True, (160, 80, 80)),
            (W // 2 - 60, H // 2 + 50),
        )
        pygame.display.flip()
        clock.tick(30)
