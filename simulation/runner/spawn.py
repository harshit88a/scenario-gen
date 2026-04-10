"""
Actor spawn utilities.

Spawn modes
-----------
random            – random spawn point
index             – spawn_points[index]
coordinates       – explicit x/y/z + yaw
relative_to_ego   – directional offset from ego, road-snapped
near_intersection – spawn point near a junction
junction_cross    – spawn on a road that CROSSES the ego's upcoming junction
nearby_random     – map spawn point within a distance band from ego
"""

import math
import random

import carla


# Compass-direction → yaw offset (degrees) relative to ego heading
DIRECTION_ANGLES = {
    "front":       0,
    "rear":        180,
    "left":       -90,
    "right":       90,
    "front_left": -45,
    "front_right":  45,
    "rear_left":  -135,
    "rear_right":  135,
}


# ─────────────────────────────────────────────────────────────────────────────
# Internal helpers
# ─────────────────────────────────────────────────────────────────────────────

def _junction_cross_spawn(
    spawn_cfg: dict,
    carla_map,
    spawn_points: list,
    ego_transform,
) -> "carla.Transform | None":
    """
    Find the first junction ahead of ego (stepping 3 m at a time so we never
    jump over it), then return a spawn transform on a CROSSING road — i.e. a
    road whose heading is at least 45° different from ego's heading.

    Two bugs that plagued the old code, now fixed:
      1. waypoint.next(big_dist) jumps in one shot, skipping junctions.
         → We now walk 3 m per iteration.
      2. Inside a junction entry_wp.road_id is an internal junction lane ID,
         not the external approach road ID. Using it to filter always failed.
         → We now walk previous(5 m) back to the approach road and compare
           heading angle instead of road_id.
    """
    if ego_transform is None:
        print("[JunctionCross] No ego_transform provided — fallback")
        return None

    search_dist   = float(spawn_cfg.get("search_distance",  60.0))
    approach_dist = float(spawn_cfg.get("approach_distance", 30.0))
    step_size     = 3.0

    ego_wp  = carla_map.get_waypoint(ego_transform.location, project_to_road=True)
    ego_yaw = ego_transform.rotation.yaw

    # ── Step 1: walk forward to find the first upcoming junction ─────────
    current         = ego_wp
    target_junction = None
    dist_walked     = 0.0

    while dist_walked < search_dist:
        nexts = current.next(step_size)
        if not nexts:
            break
        current      = nexts[0]
        dist_walked += step_size

        if current.is_junction:
            target_junction = current.get_junction()
            print(
                f"  [JunctionCross] Junction found at ~{dist_walked:.0f} m ahead "
                f"(junction_id={current.junction_id})"
            )
            break

    if target_junction is None:
        print(
            f"  [JunctionCross] No junction within {search_dist} m — "
            f"falling back to heading-based spawn"
        )
        return _heading_perpendicular_spawn(
            carla_map, spawn_points, ego_yaw, approach_dist
        )

    # ── Step 2: examine every lane pair through the junction ─────────────
    try:
        wp_pairs = target_junction.get_waypoints(carla.LaneType.Driving)
    except Exception as e:
        print(f"  [JunctionCross] get_waypoints failed ({e}) — fallback")
        return None

    print(f"  [JunctionCross] Junction has {len(wp_pairs)} lane pair(s)")

    candidates = []
    for i, (entry_wp, _) in enumerate(wp_pairs):
        # Walk 5 m back from the junction entry to reach the external approach road
        prev_wps = entry_wp.previous(5.0)
        if not prev_wps:
            continue
        outside_wp   = prev_wps[0]
        approach_yaw = outside_wp.transform.rotation.yaw
        yaw_diff     = abs((approach_yaw - ego_yaw + 180) % 360 - 180)

        print(
            f"    pair {i:02d}: approach_yaw={approach_yaw:6.1f}°  "
            f"ego_yaw={ego_yaw:6.1f}°  diff={yaw_diff:5.1f}°  "
            f"loc={outside_wp.transform.location}"
        )

        # Keep roads that are at least 45° different from ego's heading.
        # This filters out ego's own road (≈ 0°) and the road directly
        # opposite (≈ 180°), keeping only genuinely crossing roads.
        if 45 < yaw_diff < 135:
            spawn_wps = outside_wp.previous(approach_dist)
            spawn_wp  = spawn_wps[0] if spawn_wps else outside_wp
            print(f"    → ✓ accepted  spawn at {spawn_wp.transform.location}")
            candidates.append(spawn_wp)
        else:
            print("    → ✗ skipped  (same/opposite road)")

    if not candidates:
        # Softer fallback: accept any road that is > 30° different
        print("  [JunctionCross] No 45–135° candidates; relaxing threshold to 30°")
        for entry_wp, _ in wp_pairs:
            prev_wps = entry_wp.previous(5.0)
            if not prev_wps:
                continue
            outside_wp   = prev_wps[0]
            approach_yaw = outside_wp.transform.rotation.yaw
            yaw_diff     = abs((approach_yaw - ego_yaw + 180) % 360 - 180)
            if 30 < yaw_diff < 150:
                spawn_wps = outside_wp.previous(approach_dist)
                candidates.append(spawn_wps[0] if spawn_wps else outside_wp)

    if not candidates:
        print("  [JunctionCross] Still no candidates — heading-based fallback")
        return _heading_perpendicular_spawn(
            carla_map, spawn_points, ego_yaw, approach_dist
        )

    chosen = random.choice(candidates)
    print(
        f"  [JunctionCross] ★ Final spawn: {chosen.transform.location}  "
        f"yaw={chosen.transform.rotation.yaw:.1f}°"
    )
    return carla.Transform(
        carla.Location(
            x=chosen.transform.location.x,
            y=chosen.transform.location.y,
            z=chosen.transform.location.z + 0.3,
        ),
        chosen.transform.rotation,
    )


def _heading_perpendicular_spawn(
    carla_map,
    spawn_points: list,
    ego_yaw: float,
    min_dist: float = 15.0,
) -> carla.Transform:
    """
    Last-resort fallback: pick a spawn point whose road heading is
    45–135° different from *ego_yaw* (perpendicular road).
    """
    candidates = []
    for sp in spawn_points:
        wp = carla_map.get_waypoint(sp.location, project_to_road=True)
        if wp is None:
            continue
        yaw_diff = abs((wp.transform.rotation.yaw - ego_yaw + 180) % 360 - 180)
        if 45 < yaw_diff < 135:
            candidates.append(sp)
    if candidates:
        chosen = random.choice(candidates)
        print(f"  [HeadingFallback] Using spawn point at {chosen.location}")
        return chosen
    print("  [HeadingFallback] No perpendicular spawn point found — random")
    return random.choice(spawn_points)


def _nearby_random_spawn(
    spawn_cfg: dict,
    spawn_points: list,
    ego_transform,
) -> carla.Transform:
    """
    Pick a real map spawn point whose distance from ego is within
    [min_dist, max_dist].  Guaranteed to be on a valid road surface.
    Falls back to any spawn point if no candidate matches.
    """
    min_d   = float(spawn_cfg.get("min_dist", 15.0))
    max_d   = float(spawn_cfg.get("max_dist", 50.0))
    ego_loc = ego_transform.location

    candidates = [
        sp for sp in spawn_points
        if min_d <= ego_loc.distance(sp.location) <= max_d
    ]

    if candidates:
        chosen = random.choice(candidates)
        print(
            f"  [NearbyRandom] {len(candidates)} candidates  "
            f"({min_d:.0f}–{max_d:.0f} m)  → {chosen.location}"
        )
        return chosen

    # Expand search if nothing found in the requested band
    print(
        f"  [NearbyRandom] No point in {min_d}–{max_d} m range; "
        f"expanding to 10–100 m"
    )
    candidates = [
        sp for sp in spawn_points
        if 10.0 <= ego_loc.distance(sp.location) <= 100.0
    ]
    return random.choice(candidates) if candidates else random.choice(spawn_points)


# ─────────────────────────────────────────────────────────────────────────────
# Public API
# ─────────────────────────────────────────────────────────────────────────────

def get_transform_for_spawn(
    spawn_cfg: dict,
    world,
    carla_map,
    spawn_points: list,
    ego_transform=None,
) -> carla.Transform:
    """Resolve a spawn config dict into a concrete carla.Transform."""
    mode = spawn_cfg.get("mode", "random")

    if mode == "random":
        return random.choice(spawn_points)

    if mode == "index":
        return spawn_points[int(spawn_cfg.get("index", 0)) % len(spawn_points)]

    if mode == "coordinates":
        loc = spawn_cfg["location"]
        rot = spawn_cfg.get("rotation", {})
        return carla.Transform(
            carla.Location(x=loc["x"], y=loc["y"], z=loc.get("z", 0.5)),
            carla.Rotation(
                yaw=rot.get("yaw", 0.0),
                pitch=rot.get("pitch", 0.0),
                roll=rot.get("roll", 0.0),
            ),
        )

    if mode == "relative_to_ego":
        if ego_transform is None:
            raise ValueError("relative_to_ego requires ego_transform")
        direction    = spawn_cfg.get("direction", "front")
        distance     = float(spawn_cfg.get("distance", 20.0))
        lat_offset   = float(spawn_cfg.get("lateral_offset", 0.0))
        ego_yaw      = ego_transform.rotation.yaw
        angle_offset = DIRECTION_ANGLES.get(direction, 0)
        total_rad    = math.radians(ego_yaw + angle_offset)
        dx = distance * math.cos(total_rad)
        dy = distance * math.sin(total_rad)
        perp_rad = math.radians(ego_yaw + angle_offset + 90)
        dx += lat_offset * math.cos(perp_rad)
        dy += lat_offset * math.sin(perp_rad)
        raw_loc = carla.Location(
            x=ego_transform.location.x + dx,
            y=ego_transform.location.y + dy,
            z=max(ego_transform.location.z, 0.5),
        )
        wp = carla_map.get_waypoint(
            raw_loc, project_to_road=True, lane_type=carla.LaneType.Driving
        )
        if wp:
            return carla.Transform(
                carla.Location(
                    x=wp.transform.location.x,
                    y=wp.transform.location.y,
                    z=wp.transform.location.z + 0.2,
                ),
                wp.transform.rotation,
            )
        return carla.Transform(raw_loc)

    if mode == "near_intersection":
        search_dist = float(spawn_cfg.get("search_distance", 30.0))
        candidates  = []
        for sp in spawn_points:
            wp = carla_map.get_waypoint(sp.location)
            for ahead in wp.next(search_dist):
                if ahead.is_junction:
                    candidates.append(sp)
                    break
        return random.choice(candidates) if candidates else random.choice(spawn_points)

    if mode == "junction_cross":
        result = _junction_cross_spawn(
            spawn_cfg, carla_map, spawn_points, ego_transform
        )
        return result if result else random.choice(spawn_points)

    if mode == "nearby_random":
        if ego_transform is None:
            return random.choice(spawn_points)
        return _nearby_random_spawn(spawn_cfg, spawn_points, ego_transform)

    print(f"[Warning] Unknown spawn mode '{mode}' — using random")
    return random.choice(spawn_points)


def safe_spawn_vehicle(
    world,
    blueprint_library,
    blueprint_name: str,
    transform: carla.Transform,
    retries: int = 6,
) -> carla.Actor:
    """
    Try to spawn a vehicle blueprint at *transform*.  Jitters the position on
    each retry and discards ghost actors that land at world origin.
    """
    bp = blueprint_library.find(blueprint_name)
    if bp is None:
        raise ValueError(f"Blueprint not found: {blueprint_name}")
    if bp.has_attribute("color"):
        bp.set_attribute(
            "color",
            random.choice(bp.get_attribute("color").recommended_values),
        )
    for attempt in range(retries):
        jitter = attempt * 1.0
        t = carla.Transform(
            carla.Location(
                x=transform.location.x + random.uniform(-jitter, jitter),
                y=transform.location.y + random.uniform(-jitter, jitter),
                z=max(transform.location.z, 0.3),
            ),
            transform.rotation,
        )
        actor = world.try_spawn_actor(bp, t)
        if actor:
            # In sync mode, tick once so the position is actually registered.
            # If the actor sits at world origin afterwards, the spawn point was
            # occupied — destroy and try the next jittered position.
            try:
                world.tick()
            except Exception:
                pass
            loc = actor.get_location()
            if not (loc.x == 0.0 and loc.y == 0.0 and loc.z == 0.0):
                return actor
            actor.destroy()
            print(f"  [Spawn] Attempt {attempt + 1}: got (0,0,0) ghost, retrying...")
    raise RuntimeError(
        f"Could not spawn '{blueprint_name}' after {retries} attempts"
    )


def safe_spawn_walker(
    world,
    blueprint_library,
    blueprint_name: str,
    transform: carla.Transform,
    retries: int = 5,
) -> carla.Actor:
    """Try to spawn a pedestrian, falling back to a random pedestrian blueprint."""
    bp = blueprint_library.find(blueprint_name)
    if bp is None:
        walkers = list(blueprint_library.filter("walker.pedestrian.*"))
        bp      = random.choice(walkers)
    if bp.has_attribute("is_invincible"):
        bp.set_attribute("is_invincible", "false")
    for attempt in range(retries):
        jitter = attempt * 0.5
        t = carla.Transform(
            carla.Location(
                x=transform.location.x + random.uniform(-jitter, jitter),
                y=transform.location.y + random.uniform(-jitter, jitter),
                z=max(transform.location.z, 0.3),
            ),
            transform.rotation,
        )
        actor = world.try_spawn_actor(bp, t)
        if actor:
            return actor
    raise RuntimeError(f"Walker spawn failed after {retries} attempts")
