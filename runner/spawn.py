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
                    (perpendicular, 45–135° heading diff).  Use for right-turn
                    or side-road adversaries.
oncoming_junction – spawn on the ONCOMING road of ego's upcoming junction
                    (opposing approach, >135° heading diff).  Use for left-turn
                    oncoming adversaries.
exit_lane_ahead   – walk ego's path THROUGH the upcoming junction following the
                    given maneuver, then place adversary forward_distance metres
                    past the exit.  Use for vehicles already in ego's exit lane.
nearby_random     – map spawn point within a distance band from ego
lane_ahead        – walk forward_distance metres along road from ego (or reference)
random_lane       – random spawn point on any driving lane
"""

import math
import random

import carla

from .utils import walk_waypoints, lateral_offset_location


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

def _lane_ahead_spawn(
    spawn_cfg: dict,
    carla_map,
    ego_transform,
    named_actors: dict,
) -> "carla.Transform | None":
    """
    Walk *forward_distance* metres along the road from a reference point
    and return a transform on the lane centre (+ optional lateral offset).

    Reference priority:
      1. spawn_cfg["reference"] — ID of an already-spawned adversary
      2. ego_transform          — ego vehicle position
    """
    ref_name = spawn_cfg.get("reference")
    if ref_name and named_actors and ref_name in named_actors:
        ref_loc = named_actors[ref_name].get_location()
    elif ego_transform is not None:
        ref_loc = ego_transform.location
    else:
        return None

    forward_dist   = float(spawn_cfg.get("forward_distance", 20.0))
    lateral_offset = float(spawn_cfg.get("lateral_offset",   0.0))

    wp = carla_map.get_waypoint(
        ref_loc, project_to_road=True, lane_type=carla.LaneType.Driving
    )
    if wp is None:
        return None

    current = walk_waypoints(wp, forward_dist, step=3.0)
    loc = current.transform.location
    if lateral_offset != 0.0:
        loc = lateral_offset_location(loc, current.transform.rotation.yaw, lateral_offset)
    loc = carla.Location(x=loc.x, y=loc.y, z=loc.z + 0.3)
    return carla.Transform(loc, current.transform.rotation)


def _oncoming_junction_spawn(
    spawn_cfg: dict,
    carla_map,
    ego_transform,
) -> "carla.Transform | None":
    """
    Walk forward from ego to the first upcoming junction, then find the road
    that approaches that same junction from the OPPOSITE direction (yaw_diff > 135°).
    This is the oncoming lane for unprotected-left-turn scenarios.
    """
    if ego_transform is None:
        return None

    search_dist   = float(spawn_cfg.get("search_distance",  60.0))
    approach_dist = float(spawn_cfg.get("approach_distance", 35.0))
    step          = 3.0

    ego_wp  = carla_map.get_waypoint(ego_transform.location, project_to_road=True)
    ego_yaw = ego_transform.rotation.yaw

    # Walk to find junction
    current, walked = ego_wp, 0.0
    target_junction = None
    while walked < search_dist:
        nexts = current.next(step)
        if not nexts:
            break
        current  = nexts[0]
        walked  += step
        if current.is_junction:
            target_junction = current.get_junction()
            print(
                f"  [OncomingJunction] Junction found ~{walked:.0f} m ahead "
                f"(id={current.junction_id})"
            )
            break

    if target_junction is None:
        print(f"  [OncomingJunction] No junction within {search_dist} m — fallback")
        return None

    try:
        wp_pairs = target_junction.get_waypoints(carla.LaneType.Driving)
    except Exception as e:
        print(f"  [OncomingJunction] get_waypoints failed ({e})")
        return None

    candidates = []
    for entry_wp, _ in wp_pairs:
        prev_wps = entry_wp.previous(5.0)
        if not prev_wps:
            continue
        outside_wp   = prev_wps[0]
        approach_yaw = outside_wp.transform.rotation.yaw
        yaw_diff     = abs((approach_yaw - ego_yaw + 180) % 360 - 180)
        # Oncoming lane: vehicle travelling roughly TOWARD ego (yaw_diff > 135°)
        if yaw_diff > 135:
            spawn_wps = outside_wp.previous(approach_dist)
            spawn_wp  = spawn_wps[0] if spawn_wps else outside_wp
            candidates.append(spawn_wp)

    if not candidates:
        print("  [OncomingJunction] No oncoming candidates found")
        return None

    chosen = random.choice(candidates)
    print(
        f"  [OncomingJunction] ★ {chosen.transform.location}  "
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


def _exit_lane_ahead_spawn(
    spawn_cfg: dict,
    carla_map,
    ego_transform,
) -> "carla.Transform | None":
    """
    Place a vehicle in the lane ego will enter after completing its turn.

    Algorithm:
      1. Walk step-by-step from ego to find the first junction entry waypoint.
      2. Query get_junction().get_waypoints() for all lane pairs through the junction.
      3. Filter entry waypoints matching ego's approach direction (yaw within 45°).
      4. Among matching pairs, pick the exit with the highest yaw delta matching
         the requested maneuver (most positive for right_turn, most negative for left_turn).
      5. Walk out of the junction from that exit waypoint until is_junction=False.
      6. Walk forward_distance metres further along that exit lane.
      7. Return the resulting transform.

    This uses the same junction API as _junction_supports in near_intersection,
    ensuring the adversary always lands in the same exit lane ego will navigate into.
    """
    if ego_transform is None:
        return None

    maneuver      = spawn_cfg.get("maneuver", "right_turn")
    forward_dist  = float(spawn_cfg.get("forward_distance", 8.0))
    search_dist   = float(spawn_cfg.get("search_distance",  60.0))
    step          = 3.0

    ego_wp = carla_map.get_waypoint(ego_transform.location, project_to_road=True)
    if ego_wp is None:
        return None

    # If ego is spawned inside the junction, back up to the last valid approach lane.
    while ego_wp.is_junction:
        prev = ego_wp.previous(5.0)
        if not prev:
            return None
        ego_wp = prev[0]

    # ── Step 1: Walk step-by-step to find junction entry ─────────────────────
    current     = ego_wp
    walked      = 0.0
    junction_wp = None   # first waypoint inside the junction

    while walked < search_dist:
        nexts = current.next(step)
        if not nexts:
            break
        nxt = nexts[0]
        if nxt.is_junction:
            junction_wp = nxt
            print(
                f"  [ExitLaneAhead] Junction found ~{walked:.0f} m ahead "
                f"(junction_id={nxt.junction_id})"
            )
            break
        current  = nxt
        walked  += step

    if junction_wp is None:
        print("  [ExitLaneAhead] No junction within search range — fallback")
        return None

    # current is now the last pre-junction waypoint; its yaw is the approach heading
    approach_yaw = current.transform.rotation.yaw

    # ── Step 2: Query junction lane pairs ────────────────────────────────────
    try:
        wp_pairs = junction_wp.get_junction().get_waypoints(carla.LaneType.Driving)
    except Exception as e:
        print(f"  [ExitLaneAhead] get_waypoints failed ({e}) — fallback")
        return None

    print(f"  [ExitLaneAhead] Junction has {len(wp_pairs)} lane pair(s); approach_yaw={approach_yaw:.1f}°")

    # ── Step 3 + 4: Find the best exit lane matching the requested maneuver ──
    best_exit_wp  = None
    best_score    = None   # maximised: positive yaw delta for right, negative for left

    for ent_wp, ext_wp in wp_pairs:
        # Back up from the junction entry to the external approach road
        prev = ent_wp.previous(5.0)
        ent_yaw = prev[0].transform.rotation.yaw if prev else ent_wp.transform.rotation.yaw

        # Only consider paths that approach from the same direction as ego (within 45°)
        approach_diff = abs((ent_yaw - approach_yaw + 180) % 360 - 180)
        if approach_diff > 45:
            continue

        # Yaw change entry→exit (positive = clockwise = right in CARLA)
        d = (ext_wp.transform.rotation.yaw - ent_wp.transform.rotation.yaw) % 360
        if d > 180:
            d -= 360

        print(
            f"    pair: ent_yaw={ent_yaw:.1f}°  ext_yaw={ext_wp.transform.rotation.yaw:.1f}°  "
            f"delta={d:.1f}°  approach_diff={approach_diff:.1f}°"
        )

        if maneuver == "right_turn":
            # Want most positive delta (sharpest right turn)
            if d > 20 and (best_score is None or d > best_score):
                best_score   = d
                best_exit_wp = ext_wp
        elif maneuver == "left_turn":
            # Want most negative delta (sharpest left turn)
            if d < -20 and (best_score is None or d < best_score):
                best_score   = d
                best_exit_wp = ext_wp
        else:
            # straight — minimise |delta|
            if best_score is None or abs(d) < abs(best_score):
                best_score   = d
                best_exit_wp = ext_wp

    if best_exit_wp is None:
        print(f"  [ExitLaneAhead] No matching '{maneuver}' exit found — fallback")
        return None

    print(
        f"  [ExitLaneAhead] Best exit: yaw={best_exit_wp.transform.rotation.yaw:.1f}°  "
        f"delta={best_score:.1f}°"
    )

    # ── Step 5: Walk out of the junction from the exit waypoint ──────────────
    exit_cur = best_exit_wp
    for _ in range(30):   # safety cap: max 30 steps inside junction
        if not exit_cur.is_junction:
            break
        nxt = exit_cur.next(step)
        if not nxt:
            break
        exit_cur = nxt[0]

    # ── Step 6: Walk forward_dist metres past the junction exit ──────────────
    exit_cur = walk_waypoints(exit_cur, forward_dist, step=step)

    loc = exit_cur.transform.location
    yaw = exit_cur.transform.rotation.yaw

    # Optional lateral offset
    lateral_offset = float(spawn_cfg.get("lateral_offset", 0.0))
    if lateral_offset != 0.0:
        loc = lateral_offset_location(loc, yaw, lateral_offset)

    print(f"  [ExitLaneAhead] ★ {loc}  yaw={yaw:.1f}°")
    return carla.Transform(
        carla.Location(x=loc.x, y=loc.y, z=max(loc.z + 0.3, 0.5)),
        carla.Rotation(yaw=yaw),
    )


def get_transform_for_spawn(
    spawn_cfg: dict,
    world,
    carla_map,
    spawn_points: list,
    ego_transform=None,
    named_actors: dict = None,
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
        # lateral offset reuses the same perpendicular formula as lateral_offset_location
        perp_rad = math.radians(ego_yaw + angle_offset + 90)
        dx += lat_offset * math.cos(perp_rad)
        dy += lat_offset * math.sin(perp_rad)
        raw_loc = carla.Location(
            x=ego_transform.location.x + dx,
            y=ego_transform.location.y + dy,
            z=max(ego_transform.location.z, 0.5),
        )
        # project_to_road=false: use raw sidewalk/shoulder position but still
        # get the road direction yaw so _compute_destination works correctly.
        if spawn_cfg.get("project_to_road", True):
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
        else:
            # Off-road spawn (sidewalk / shoulder): keep raw position,
            # borrow road-direction yaw so walker faces along the road.
            road_wp = carla_map.get_waypoint(
                raw_loc, project_to_road=True, lane_type=carla.LaneType.Driving
            )
            road_rot = road_wp.transform.rotation if road_wp else carla.Rotation(yaw=ego_yaw)
            return carla.Transform(
                carla.Location(x=raw_loc.x, y=raw_loc.y, z=max(raw_loc.z, 0.5)),
                road_rot,
            )
        return carla.Transform(raw_loc)

    if mode == "near_intersection":
        search_dist = float(spawn_cfg.get("search_distance", 30.0))
        maneuver    = spawn_cfg.get("maneuver")  # optional: "right_turn" / "left_turn"
        step        = 3.0

        def _junction_supports(sp_wp, turn):
            """
            Walk step-by-step from sp_wp to the first junction and check whether
            that junction has an exit matching *turn* from this approach direction.
            Returns True when *turn* is None/"straight" or when a matching exit is found.
            """
            if turn not in ("right_turn", "left_turn"):
                return True
            if sp_wp.is_junction:
                return False
            approach_yaw = sp_wp.transform.rotation.yaw
            cur = sp_wp
            for _ in range(int(search_dist / step) + 1):
                nexts = cur.next(step)
                if not nexts:
                    return False
                cur = nexts[0]
                if not cur.is_junction:
                    continue
                try:
                    pairs = cur.get_junction().get_waypoints(carla.LaneType.Driving)
                    for ent_wp, ext_wp in pairs:
                        # Back up 5 m from the junction entry to get the approach yaw
                        prev = ent_wp.previous(5.0)
                        ent_yaw = (prev[0].transform.rotation.yaw if prev
                                   else ent_wp.transform.rotation.yaw)
                        if abs((ent_yaw - approach_yaw + 180) % 360 - 180) > 45:
                            continue   # different approach road, skip
                        # Yaw change from junction entry to exit
                        d = (ext_wp.transform.rotation.yaw
                             - ent_wp.transform.rotation.yaw) % 360
                        if d > 180:
                            d -= 360
                        if turn == "right_turn" and d > 30:
                            return True
                        if turn == "left_turn" and d < -30:
                            return True
                except Exception:
                    return True   # can't inspect junction — allow it
                return False   # junction found but no matching exit
            return False

        candidates = []
        for sp in spawn_points:
            wp = carla_map.get_waypoint(sp.location, project_to_road=True)
            if wp is None or wp.is_junction:
                continue
            # Walk step-by-step so we never jump over a close junction
            cur, has_junction = wp, False
            for _ in range(int(search_dist / step) + 1):
                nexts = cur.next(step)
                if not nexts:
                    break
                cur = nexts[0]
                if cur.is_junction:
                    has_junction = True
                    break
            if not has_junction:
                continue
            if _junction_supports(wp, maneuver):
                candidates.append(sp)

        if not candidates:
            # Soft fallback: any spawn near a junction (no maneuver filter)
            print(f"  [NearIntersection] No '{maneuver}' junction found — "
                  f"relaxing to any junction")
            for sp in spawn_points:
                wp = carla_map.get_waypoint(sp.location, project_to_road=True)
                if wp is None:
                    continue
                cur = wp
                for _ in range(int(search_dist / step) + 1):
                    nexts = cur.next(step)
                    if not nexts:
                        break
                    cur = nexts[0]
                    if cur.is_junction:
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

    if mode == "random_lane":
        # Pick a random spawn point that lands on a driving lane
        driving = [
            sp for sp in spawn_points
            if carla_map.get_waypoint(
                sp.location, project_to_road=True, lane_type=carla.LaneType.Driving
            ) is not None
        ]
        return random.choice(driving) if driving else random.choice(spawn_points)

    if mode == "lane_ahead":
        result = _lane_ahead_spawn(spawn_cfg, carla_map, ego_transform, named_actors or {})
        if result:
            return result
        print("[Warning] lane_ahead: could not compute transform — using random")
        return random.choice(spawn_points)

    if mode == "oncoming_junction":
        result = _oncoming_junction_spawn(spawn_cfg, carla_map, ego_transform)
        if result:
            return result
        print("[Warning] oncoming_junction: no candidate found — using random")
        return random.choice(spawn_points)

    if mode == "exit_lane_ahead":
        result = _exit_lane_ahead_spawn(spawn_cfg, carla_map, ego_transform)
        if result:
            return result
        print("[Warning] exit_lane_ahead: could not trace exit — using random")
        return random.choice(spawn_points)

    if mode == "relative_to_actor":
        ref_id = spawn_cfg.get("reference")
        if not ref_id or named_actors is None or ref_id not in named_actors:
            print(f"[Warning] relative_to_actor: '{ref_id}' not in named_actors — using random")
            return random.choice(spawn_points)
        ref_actor = named_actors[ref_id]
        ref_tf  = ref_actor.get_transform()
        yaw     = ref_tf.rotation.yaw
        forward = float(spawn_cfg.get("forward", 0.0))
        right   = float(spawn_cfg.get("right",   0.0))   # positive = right of ref heading
        fwd_rad = math.radians(yaw)
        rgt_rad = math.radians(yaw + 90)                 # +90 = right in CARLA convention
        x = ref_tf.location.x + forward * math.cos(fwd_rad) + right * math.cos(rgt_rad)
        y = ref_tf.location.y + forward * math.sin(fwd_rad) + right * math.sin(rgt_rad)
        z = max(ref_tf.location.z, 0.5)
        raw_loc = carla.Location(x=x, y=y, z=z)
        # Borrow road-direction yaw so walkers face along the road; keep raw (off-road) position.
        road_wp  = carla_map.get_waypoint(raw_loc, project_to_road=True, lane_type=carla.LaneType.Driving)
        road_rot = road_wp.transform.rotation if road_wp else ref_tf.rotation
        print(f"  [RelativeToActor] ref='{ref_id}'  forward={forward}  right={right}  → {raw_loc}  yaw={road_rot.yaw:.1f}°")
        return carla.Transform(raw_loc, road_rot)

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
            # Tick once so the walker is registered in synchronous mode
            # before the caller tries to attach an AI controller.
            try:
                world.tick()
            except Exception:
                pass
            return actor
    raise RuntimeError(f"Walker spawn failed after {retries} attempts")
