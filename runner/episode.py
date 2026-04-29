"""
Single-episode runner and top-level scenario loop.
"""

import random
from queue import Queue, Empty

import numpy as np
import pygame

from .config import load_config
from .connection import connect, restore_async
from .world_setup import set_weather, configure_traffic_lights
from .spawn import get_transform_for_spawn, safe_spawn_vehicle, safe_spawn_walker
from .behavior import VehicleBehaviorController, WalkerBehaviorController
from .sensors import attach_sensors, CollisionMonitor
from .termination import TerminationChecker, evaluate_postconditions
from .visualization import (
    W, H, DISPLAY_MODES,
    attach_display_camera, attach_bev_lidar,
    render_bev, render_hud, show_intermission,
)


# ─────────────────────────────────────────────────────────────────────────────
# Internal helpers
# ─────────────────────────────────────────────────────────────────────────────

def _resolve_spawn(spawn_cfg, world, cmap, sps, ego_transform=None, named_actors=None):
    """
    Route to get_transform_for_spawn, forwarding optional context
    (ego_transform, named_actors) needed by reference-based spawn modes.
    """
    return get_transform_for_spawn(
        spawn_cfg, world, cmap, sps, ego_transform, named_actors=named_actors
    )


def _cleanup_episode(vehicle_actors, walker_actors, sensor_actors,
                     walker_ctrls, cm, world) -> None:
    """Destroy all per-episode actors.  Safe to call even if partially built."""
    for c in walker_ctrls:
        try:
            c.destroy()
        except Exception:
            pass
    if cm:
        try:
            cm.destroy()
        except Exception:
            pass
    for s in sensor_actors:
        try:
            s.stop()
            s.destroy()
        except Exception:
            pass
    for a in reversed(vehicle_actors + walker_actors):
        try:
            a.destroy()
        except Exception:
            pass
    # Give the server a few ticks to process the deletions
    for _ in range(5):
        try:
            world.tick()
        except Exception:
            pass


# ─────────────────────────────────────────────────────────────────────────────
# Single-episode runner
# ─────────────────────────────────────────────────────────────────────────────

def _run_one_episode(
    world, client, bpl, cmap, sps, config,
    episode_num: int,
    screen: pygame.Surface,
    clock: pygame.time.Clock,
    font: pygame.font.Font,
    display_mode_holder: list,   # [str] — persists across episodes
    bev_surf: pygame.Surface,
) -> str:
    """
    Spawn actors, run the simulation loop until a termination condition is
    met or the user intervenes, then clean up.

    Returns
    -------
    "quit"  – user pressed ESC / closed the window
    "next"  – user pressed N (skip to next episode)
    "done"  – episode ended normally
    """
    vehicle_actors: list = []
    walker_actors:  list = []
    sensor_actors:  list = []
    walker_ctrls:   list = []
    cm = None

    # Traffic Manager — synchronous mode, same fixed_delta as world
    tm = client.get_trafficmanager(8000)
    tm.set_synchronous_mode(True)
    tm.set_global_distance_to_leading_vehicle(2.0)

    try:
        # ── Environment ──────────────────────────────────────────────────
        env = config.get("environment", {})
        if "weather" in env:
            set_weather(world, env["weather"])
        configure_traffic_lights(world, env.get("traffic_lights", "normal"))

        # ── Ego ──────────────────────────────────────────────────────────
        ego_cfg = config["ego"]
        ego_tf  = _resolve_spawn(
            ego_cfg.get("spawn", {"mode": "random"}), world, cmap, sps
        )
        ego = safe_spawn_vehicle(world, bpl, ego_cfg["blueprint"], ego_tf)
        vehicle_actors.append(ego)
        world.tick()
        print(f"  [Ego]  {ego.get_location()}")

        cm = CollisionMonitor(world, bpl, ego)
        for s in attach_sensors(world, bpl, ego, ego_cfg.get("sensors", [])):
            sensor_actors.append(s)

        # ── Adversaries ──────────────────────────────────────────────────
        # Two-pass spawn: vehicles/props first so walkers that use
        # relative_to_actor can always resolve their reference regardless
        # of the order entries appear in the config.
        adversaries:  list = []
        adv_ctrls:    list = []
        named_actors: dict = {}   # id → actor, for reference-based spawns

        all_adv_cfgs = config.get("adversaries", [])
        non_walkers  = [c for c in all_adv_cfgs if c.get("type") != "walker"]
        walkers      = [c for c in all_adv_cfgs if c.get("type") == "walker"]

        for adv_cfg in non_walkers + walkers:
            adv_tf = _resolve_spawn(
                adv_cfg.get("spawn", {"mode": "nearby_random"}),
                world, cmap, sps,
                ego_transform=ego.get_transform(),
                named_actors=named_actors,
            )

            if adv_cfg.get("type") == "walker":
                adv  = safe_spawn_walker(
                    world, bpl,
                    adv_cfg.get("blueprint", "walker.pedestrian.0001"),
                    adv_tf,
                )
                walker_actors.append(adv)
                ctrl = WalkerBehaviorController(adv, adv_cfg.get("behavior", {}), world, target_ego=ego)
                walker_ctrls.append(ctrl)
            elif adv_cfg.get("type") == "prop":
                adv = safe_spawn_vehicle(world, bpl, adv_cfg["blueprint"], adv_tf)
                vehicle_actors.append(adv)
                beh = adv_cfg.get("behavior", {})
                if "physics" in beh:
                    adv.set_simulate_physics(beh["physics"])
                # Static props have no behavior controller
            else:
                adv  = safe_spawn_vehicle(world, bpl, adv_cfg["blueprint"], adv_tf)
                vehicle_actors.append(adv)
                adv_beh       = adv_cfg.get("behavior", {})
                adv_target_id = adv_beh.get("target_id")
                adv_target    = (
                    named_actors.get(adv_target_id)
                    if adv_target_id and adv_target_id in named_actors
                    else ego
                )
                ctrl = VehicleBehaviorController(
                    adv, adv_beh,
                    cmap,
                    target_actor=adv_target,
                    tm=tm,
                )
                adv_ctrls.append(ctrl)

            adversaries.append(adv)
            adv_id = adv_cfg.get("id")
            if adv_id:
                named_actors[adv_id] = adv

        # In synchronous mode actor positions are only valid after a tick
        world.tick()

        ordered_adv_cfgs = non_walkers + walkers
        for i, (adv_cfg, adv) in enumerate(
            zip(ordered_adv_cfgs, adversaries)
        ):
            dist = ego.get_location().distance(adv.get_location())
            loc  = adv.get_location()
            print(f"  [Adv '{adv_cfg.get('id', '?')}']  {loc}  dist={dist:.1f}m")
            if loc.x == 0.0 and loc.y == 0.0:
                print(
                    f"  [Warning] Adv-{i} at world origin — "
                    f"spawn likely failed or collided"
                )

        # ── Ego behavior ─────────────────────────────────────────────────
        ego_beh       = ego_cfg.get("behavior", {"type": "ttc_brake", "target_speed": 10})
        ego_target_id = ego_beh.get("target_id")
        ego_target    = (
            named_actors.get(ego_target_id)
            if ego_target_id and ego_target_id in named_actors
            else (adversaries[0] if adversaries else None)
        )
        ego_ctrl = VehicleBehaviorController(
            ego,
            ego_beh,
            cmap,
            target_actor=ego_target,
            tm=tm,
        )

        # ── Termination config ────────────────────────────────────────────
        term_cfg = dict(config.get("termination", {}))
        ep_dur   = float(
            config.get("loop", {}).get(
                "episode_duration_seconds",
                term_cfg.get("max_duration_seconds", 20.0),
            )
        )
        term_cfg["max_duration_seconds"] = ep_dur
        tc = TerminationChecker(term_cfg, ego, cm)

        # ── Display cameras (behind + top always attached) ─────────────────
        queues:  dict = {}
        cameras: dict = {}
        for view in ("behind", "top"):
            cam = attach_display_camera(world, bpl, ego, view)
            sensor_actors.append(cam)
            cameras[view] = cam
            q: Queue = Queue()
            queues[view] = q
            cam.listen(
                lambda img, _q=q: _q.put(
                    np.frombuffer(img.raw_data, dtype=np.uint8)
                    .reshape((img.height, img.width, 4))[:, :, :3][:, :, ::-1]
                )
            )

        # BEV LiDAR
        lidar_pts = [None]
        bev_lidar = attach_bev_lidar(world, bpl, ego)
        sensor_actors.append(bev_lidar)
        bev_lidar.listen(
            lambda data, lp=lidar_pts: lp.__setitem__(
                0,
                np.frombuffer(data.raw_data, dtype=np.float32)
                .reshape((-1, 4))[:, :3],
            )
        )

        # ── Main simulation loop ──────────────────────────────────────────
        rgb_frames = {
            "behind": np.zeros((H, W, 3), dtype=np.uint8),
            "top":    np.zeros((H, W, 3), dtype=np.uint8),
        }

        while True:
            world.tick()

            # Events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return "quit"
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        return "quit"
                    if event.key == pygame.K_n:
                        print("  [Skip] N pressed → next episode")
                        return "next"
                    if event.key == pygame.K_TAB:
                        idx = DISPLAY_MODES.index(display_mode_holder[0])
                        display_mode_holder[0] = DISPLAY_MODES[
                            (idx + 1) % len(DISPLAY_MODES)
                        ]
                        print(f"  [Display] → {display_mode_holder[0]}")

            # Draw
            mode = display_mode_holder[0]
            if mode in ("behind", "top"):
                try:
                    while True:
                        rgb_frames[mode] = queues[mode].get_nowait()
                except Empty:
                    pass
                screen.blit(
                    pygame.surfarray.make_surface(rgb_frames[mode].swapaxes(0, 1)),
                    (0, 0),
                )
            else:  # lidar BEV
                render_bev(bev_surf, lidar_pts[0], ego, adversaries)
                screen.blit(bev_surf, (0, 0))

            render_hud(screen, font, ego, adversaries, cm, tc, mode, episode_num)
            pygame.display.flip()
            clock.tick(30)

            # Behaviors
            ego_ctrl.tick()
            for c in adv_ctrls:
                c.tick()
            for c in walker_ctrls:
                c.tick()

            if tc.should_terminate():
                break

        # Postconditions
        pcfg = config.get("postconditions", [])
        if pcfg:
            print("  [Postconditions]")
            evaluate_postconditions(pcfg, ego, cm)

        return "done"

    finally:
        _cleanup_episode(
            vehicle_actors, walker_actors, sensor_actors, walker_ctrls, cm, world
        )


# ─────────────────────────────────────────────────────────────────────────────
# Top-level entry point
# ─────────────────────────────────────────────────────────────────────────────

def run(config_path: str, host: str = "localhost", port: int = 2000) -> None:
    """Load *config_path* and run the scenario episode loop."""
    config        = load_config(config_path)
    scenario_name = config.get("scenario", {}).get("type", "scenario")
    loop_cfg      = config.get("loop", {})
    max_episodes  = loop_cfg.get("max_episodes", 999999)
    intermission  = float(loop_cfg.get("intermission_seconds", 1.0))

    print(f"\n{'=' * 60}")
    print(f"Scenario  : {scenario_name}")
    print(f"Map       : {config['map']}")
    print(f"Max eps   : {max_episodes}")
    print(f"Ep dur    : {loop_cfg.get('episode_duration_seconds', 20)} s")
    print(f"ESC=quit  N=skip episode  TAB=cycle view  (behind/top/lidar)")
    print(f"{'=' * 60}\n")

    client, world = connect(config["map"], host, port)
    bpl  = world.get_blueprint_library()
    cmap = world.get_map()
    sps  = cmap.get_spawn_points()
    print(f"[Setup] {len(sps)} spawn points on {config['map']}\n")

    # Shuffle spawn points once — each episode rotates to a new region
    random.shuffle(sps)

    # Init pygame once — stays alive for all episodes
    pygame.init()
    screen   = pygame.display.set_mode((W, H))
    pygame.display.set_caption(scenario_name)
    clock    = pygame.time.Clock()
    font     = pygame.font.SysFont("monospace", 15)
    bev_surf = pygame.Surface((W, H))

    display_mode_holder = ["behind"]   # persists across episodes

    try:
        for episode in range(1, max_episodes + 1):
            print(f"── Episode {episode} ──────────────────────────────────────")

            # Rotate spawn pool so each episode starts in a different area
            sps_rotated = sps[episode % len(sps):] + sps[:episode % len(sps)]

            result = _run_one_episode(
                world, client, bpl, cmap, sps_rotated, config,
                episode, screen, clock, font,
                display_mode_holder, bev_surf,
            )

            if result == "quit":
                print("\n[Loop] User quit.")
                break

            if episode >= max_episodes:
                print(f"\n[Loop] Reached max_episodes={max_episodes}.")
                break

            cont = show_intermission(screen, font, episode, intermission)
            if not cont:
                print("\n[Loop] User quit during intermission.")
                break

    finally:
        restore_async(world)
        pygame.quit()
        print("[Done]\n")
