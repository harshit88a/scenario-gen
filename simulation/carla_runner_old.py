"""
CARLA Adversarial Scenario Runner
==================================

Entry point for running adversarial driving scenarios in CARLA.

This module provides a clean command-line interface for executing scenario
configurations. The actual simulation logic is delegated to the `runner` package
which implements OOP principles for modularity and extensibility.

Supported scenarios include:
  - Junction crossing conflicts
  - Cut-in maneuvers
  - Intersection collisions
  - And more...

Usage:
    python carla_runner.py --config config/scenario_intersection_conflict.json
    python carla_runner.py --config config/scenario_cut_in.json --host localhost --port 2000

Example config.json:
    {
        "map": "Town05",
        "scenario": {
            "type": "intersection_conflict",
            "ego_maneuver": "straight",
            "adversary_maneuver": "left_turn"
        },
        ...
    }
"""

import argparse
import sys

from runner import run


def main():
    """
    Main entry point for the CARLA adversarial scenario runner.
    
    Parses command-line arguments and delegates to the runner module.
    """
    parser = argparse.ArgumentParser(
        description="Run adversarial driving scenarios in CARLA",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --config config/scenario_intersection_conflict.json
  %(prog)s --config config/scenario_cut_in.json --host localhost --port 2000
        """
    )
    
    parser.add_argument(
        "--config",
        required=True,
        type=str,
        help="Path to the scenario configuration JSON file (required)"
    )
    
    parser.add_argument(
        "--host",
        default="localhost",
        type=str,
        help="CARLA server hostname (default: localhost)"
    )
    
    parser.add_argument(
        "--port",
        default=2000,
        type=int,
        help="CARLA server port (default: 2000)"
    )
    
    args = parser.parse_args()
    
    try:
        run(args.config, host=args.host, port=args.port)
    except KeyboardInterrupt:
        print("\n[Interrupted] User terminated the simulation.")
        sys.exit(0)
    except FileNotFoundError as e:
        print(f"[Error] Configuration file not found: {args.config}")
        sys.exit(1)
    except ConnectionError as e:
        print(f"[Error] Failed to connect to CARLA server at {args.host}:{args.port}")
        print("       Make sure CARLA is running: ./CarlaUE4.sh")
        sys.exit(1)
    except Exception as e:
        print(f"[Error] An unexpected error occurred: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()


# ─────────────────────────────────────────────────────────────────────────────
# Spawn
# ─────────────────────────────────────────────────────────────────────────────

DIRECTION_ANGLES = {
    "front": 0, "rear": 180, "left": -90, "right": 90,
    "front_left": -45, "front_right": 45,
    "rear_left": -135, "rear_right": 135,
}

def _junction_cross_spawn(spawn_cfg: dict, carla_map,
                           spawn_points: list,
                           ego_transform) -> "carla.Transform | None":
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

    ego_wp  = carla_map.get_waypoint(ego_transform.location,
                                      project_to_road=True)
    ego_yaw = ego_transform.rotation.yaw

    # ── Step 1: walk forward to find the first upcoming junction ─────────
    current          = ego_wp
    target_junction  = None
    dist_walked      = 0.0

    while dist_walked < search_dist:
        nexts = current.next(step_size)
        if not nexts:
            break
        current      = nexts[0]
        dist_walked += step_size

        if current.is_junction:
            target_junction = current.get_junction()
            print(f"  [JunctionCross] Junction found at ~{dist_walked:.0f} m ahead "
                  f"(junction_id={current.junction_id})")
            break

    if target_junction is None:
        print(f"  [JunctionCross] No junction within {search_dist} m — "
              f"falling back to heading-based spawn")
        return _heading_perpendicular_spawn(carla_map, spawn_points, ego_yaw,
                                             approach_dist)

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
        outside_wp      = prev_wps[0]
        approach_yaw    = outside_wp.transform.rotation.yaw
        yaw_diff        = abs((approach_yaw - ego_yaw + 180) % 360 - 180)

        print(f"    pair {i:02d}: approach_yaw={approach_yaw:6.1f}°  "
              f"ego_yaw={ego_yaw:6.1f}°  diff={yaw_diff:5.1f}°  "
              f"loc={outside_wp.transform.location}")

        # Keep roads that are at least 45° different from ego's heading.
        # This filters out ego's own road (≈ 0°) and the road directly
        # opposite (≈ 180°), keeping only genuinely crossing roads.
        if 45 < yaw_diff < 135:
            # Walk back further so the adversary starts well before the junction
            spawn_wps = outside_wp.previous(approach_dist)
            spawn_wp  = spawn_wps[0] if spawn_wps else outside_wp
            print(f"    → ✓ accepted  spawn at {spawn_wp.transform.location}")
            candidates.append(spawn_wp)
        else:
            print(f"    → ✗ skipped  (same/opposite road)")

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
        return _heading_perpendicular_spawn(carla_map, spawn_points, ego_yaw,
                                             approach_dist)

    chosen = random.choice(candidates)
    print(f"  [JunctionCross] ★ Final spawn: {chosen.transform.location}  "
          f"yaw={chosen.transform.rotation.yaw:.1f}°")
    return carla.Transform(
        carla.Location(x=chosen.transform.location.x,
                       y=chosen.transform.location.y,
                       z=chosen.transform.location.z + 0.3),
        chosen.transform.rotation,
    )


def _heading_perpendicular_spawn(carla_map, spawn_points: list,
                                  ego_yaw: float,
                                  min_dist: float = 15.0):
    """
    Last-resort fallback: pick a spawn point whose road heading is
    45–135° different from ego_yaw (perpendicular road).
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


def get_transform_for_spawn(spawn_cfg: dict, world, carla_map,
                             spawn_points: list,
                             ego_transform=None) -> carla.Transform:
    """
    Resolve spawn config into a carla.Transform.

    Modes:
      random            – random spawn point
      index             – spawn_points[index]
      coordinates       – explicit x/y/z + yaw
      relative_to_ego   – directional offset from ego, road-snapped
      near_intersection – spawn point near a junction
      junction_cross    – spawn on a road that CROSSES the ego's upcoming
                          junction (the correct mode for intersection conflicts)
    """
    mode = spawn_cfg.get("mode", "random")

    if mode == "random":
        return random.choice(spawn_points)

    elif mode == "index":
        return spawn_points[int(spawn_cfg.get("index", 0)) % len(spawn_points)]

    elif mode == "coordinates":
        loc = spawn_cfg["location"]
        rot = spawn_cfg.get("rotation", {})
        return carla.Transform(
            carla.Location(x=loc["x"], y=loc["y"], z=loc.get("z", 0.5)),
            carla.Rotation(yaw=rot.get("yaw", 0.0),
                           pitch=rot.get("pitch", 0.0),
                           roll=rot.get("roll", 0.0)),
        )

    elif mode == "relative_to_ego":
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
        wp = carla_map.get_waypoint(raw_loc, project_to_road=True,
                                    lane_type=carla.LaneType.Driving)
        if wp:
            return carla.Transform(
                carla.Location(x=wp.transform.location.x,
                               y=wp.transform.location.y,
                               z=wp.transform.location.z + 0.2),
                wp.transform.rotation)
        return carla.Transform(raw_loc)

    elif mode == "near_intersection":
        search_dist = float(spawn_cfg.get("search_distance", 30.0))
        candidates = []
        for sp in spawn_points:
            wp = carla_map.get_waypoint(sp.location)
            for ahead in wp.next(search_dist):
                if ahead.is_junction:
                    candidates.append(sp)
                    break
        return random.choice(candidates) if candidates else random.choice(spawn_points)

    elif mode == "junction_cross":
        result = _junction_cross_spawn(spawn_cfg, carla_map, spawn_points,
                                        ego_transform)
        return result if result else random.choice(spawn_points)

    else:
        print(f"[Warning] Unknown spawn mode '{mode}' — using random")
        return random.choice(spawn_points)


def safe_spawn_vehicle(world, blueprint_library, blueprint_name: str,
                        transform: carla.Transform, retries: int = 6):
    bp = blueprint_library.find(blueprint_name)
    if bp is None:
        raise ValueError(f"Blueprint not found: {blueprint_name}")
    if bp.has_attribute("color"):
        bp.set_attribute("color",
            random.choice(bp.get_attribute("color").recommended_values))
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
            # Ghost actor at origin — discard and retry
            actor.destroy()
            print(f"  [Spawn] Attempt {attempt+1}: got (0,0,0) ghost, retrying...")
    raise RuntimeError(f"Could not spawn '{blueprint_name}' after {retries} attempts")


def safe_spawn_walker(world, blueprint_library, blueprint_name: str,
                       transform: carla.Transform, retries: int = 5):
    bp = blueprint_library.find(blueprint_name)
    if bp is None:
        walkers = list(blueprint_library.filter("walker.pedestrian.*"))
        bp = random.choice(walkers)
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


# ─────────────────────────────────────────────────────────────────────────────
# Behavior
# ─────────────────────────────────────────────────────────────────────────────

def _speed(actor) -> float:
    v = actor.get_velocity()
    return math.sqrt(v.x ** 2 + v.y ** 2)


def compute_ttc(a, b) -> float:
    dist = a.get_location().distance(b.get_location())
    return dist / max(_speed(a) - _speed(b), 0.01)


def _steer_toward(actor, target: carla.Location):
    loc = actor.get_location()
    dx, dy = target.x - loc.x, target.y - loc.y
    d = math.sqrt(dx ** 2 + dy ** 2)
    if d < 2.0:
        return 0.0, True
    dx /= d; dy /= d
    yaw = actor.get_transform().rotation.yaw
    fx = math.cos(math.radians(yaw))
    fy = math.sin(math.radians(yaw))
    return max(-1.0, min(1.0, (fx * dy - fy * dx) * 0.9)), False


def _throttle(actor, target_speed: float) -> float:
    return max(0.0, min(1.0, (target_speed - _speed(actor)) * 0.4 + 0.3))


class WaypointPlanner:
    def __init__(self, actor, carla_map, maneuver="straight",
                 lookahead=120.0, step=3.0):
        self.wps = self._plan(actor, carla_map, maneuver, lookahead, step)
        self.idx = 0

    def _plan(self, actor, carla_map, maneuver, lookahead, step):
        wp = carla_map.get_waypoint(actor.get_location(), project_to_road=True)
        result, current, n = [wp], wp, int(lookahead / step)
        for i in range(n):
            nexts = current.next(step)
            if not nexts:
                break
            if maneuver == "straight":
                chosen = min(nexts, key=lambda w: abs(
                    (w.transform.rotation.yaw - current.transform.rotation.yaw + 180) % 360 - 180))
            elif maneuver in ("left_turn", "right_turn"):
                straight = min(nexts, key=lambda w: abs(
                    (w.transform.rotation.yaw - current.transform.rotation.yaw + 180) % 360 - 180))
                if i < n // 4:
                    chosen = straight
                else:
                    sign = -1 if maneuver == "left_turn" else 1
                    def yaw_score(w):
                        d = (w.transform.rotation.yaw - current.transform.rotation.yaw) % 360
                        if d > 180: d -= 360
                        return sign * d
                    chosen = min(nexts, key=yaw_score)
            else:
                chosen = nexts[0]
            result.append(chosen)
            current = chosen
        return result

    def target(self, actor) -> carla.Location:
        loc = actor.get_location()
        while self.idx < len(self.wps) - 1:
            if loc.distance(self.wps[self.idx].transform.location) < 5.0:
                self.idx += 1
            else:
                break
        return self.wps[min(self.idx, len(self.wps) - 1)].transform.location


class VehicleBehaviorController:
    """
    Behavior types:
      traffic_manager – CARLA Traffic Manager (natural road following).
      intersection_conflict – TM autopilot + forces adversary straight through
                              the same junction as ego, T-boning it. Uses
                              junction_cross spawn for perpendicular approach.
      ttc_brake       – custom waypoint follower; brake when TTC < threshold
      constant_speed  – custom waypoint follower at fixed speed
      cut_in          – follow path; steer into ego lane when close
      sudden_brake    – drive then brake after N seconds
      stopped         – hold brake
      autopilot       – basic CARLA autopilot (no TM config)
    """
    def __init__(self, actor, cfg: dict, carla_map,
                 target_actor=None, tm=None):
        self.actor        = actor
        self.cfg          = cfg
        self.btype        = cfg.get("type", "constant_speed")
        self.target_speed = float(cfg.get("target_speed", 10.0))
        self.brake_int    = float(cfg.get("brake_intensity", 1.0))
        self.ttc_thresh   = float(cfg.get("ttc_threshold", 4.0))
        self.target_actor = target_actor
        self._t0          = time.time()
        self._tm          = tm
        self._cmap        = carla_map

        if self.btype in ("traffic_manager", "intersection_conflict"):
            if tm is None:
                print("  [Behavior] TM not provided — falling back to autopilot")
                actor.set_autopilot(True)
            else:
                tm_port = tm.get_port()
                actor.set_autopilot(True, tm_port)
                speed_pct = float(cfg.get("speed_pct", -20.0))
                tm.vehicle_percentage_speed_difference(actor, speed_pct)
                if cfg.get("ignore_lights", True):
                    tm.ignore_lights_percentage(actor, 100.0)
                    tm.ignore_signs_percentage(actor, 100.0)
                tm.auto_lane_change(actor, cfg.get("auto_lane_change", False))
                if target_actor is not None:
                    tm.collision_detection(actor, target_actor, False)

                if self.btype == "intersection_conflict" and target_actor is not None:
                    self._setup_conflict_path(actor, target_actor, carla_map, tm)
                else:
                    print(f"  [Behavior] traffic_manager: port={tm_port}, "
                          f"speed_pct={speed_pct}")

        elif self.btype == "autopilot":
            actor.set_autopilot(True)

        else:
            self.planner = WaypointPlanner(actor, carla_map,
                                           maneuver=cfg.get("maneuver", "straight"))

    # ── Intersection conflict path builder ────────────────────────────────────
    @staticmethod
    def _find_junction_ahead(wp, search_dist: float = 80.0):
        """Walk forward 3 m at a time; return first Junction found."""
        current, step = wp, 3.0
        for _ in range(int(search_dist / step)):
            nexts = current.next(step)
            if not nexts:
                break
            current = nexts[0]
            if current.is_junction:
                return current.get_junction(), current
        return None, None

    def _setup_conflict_path(self, actor, ego, carla_map, tm):
        """
        Build a waypoint path that sends the adversary STRAIGHT THROUGH the
        junction that ego is heading toward.  Because the adversary approaches
        from a perpendicular road (junction_cross spawn), going straight
        creates a T-bone intersection conflict with ego.

        Falls back silently to plain TM autopilot if anything fails.
        """
        try:
            ego_wp = carla_map.get_waypoint(ego.get_location(),
                                             project_to_road=True)
            junction, _ = self._find_junction_ahead(ego_wp)
            if junction is None:
                print("  [ConflictPath] No junction ahead of ego — plain TM")
                return

            adv_wp = carla_map.get_waypoint(actor.get_location(),
                                             project_to_road=True)
            path: list = []
            step = 3.0

            # ── Phase 1: walk from adversary up to the junction ───────────
            current = adv_wp
            for _ in range(int(100 / step)):   # up to 100 m approach
                nexts = current.next(step)
                if not nexts:
                    break
                current = nexts[0]
                path.append(current.transform.location)
                if current.is_junction:
                    break

            # ── Phase 2: traverse the junction going STRAIGHT ─────────────
            #    "Straight" = pick the next wp whose yaw changes least.
            for _ in range(15):
                nexts = current.next(step)
                if not nexts:
                    break
                current = min(nexts, key=lambda w: abs(
                    (w.transform.rotation.yaw
                     - current.transform.rotation.yaw + 180) % 360 - 180))
                path.append(current.transform.location)
                if not current.is_junction:
                    break

            # ── Phase 3: continue 30 m past the junction ──────────────────
            for _ in range(int(30 / step)):
                nexts = current.next(step)
                if not nexts:
                    break
                current = nexts[0]
                path.append(current.transform.location)

            if len(path) < 3:
                print("  [ConflictPath] Path too short — plain TM")
                return

            tm.set_path(actor, path)
            print(f"  [ConflictPath] ✓ {len(path)} waypoints through junction")

        except Exception as e:
            print(f"  [ConflictPath] Setup failed ({e}) — plain TM autopilot")

    def tick(self):
        if self.btype in ("traffic_manager", "intersection_conflict", "autopilot"):
            return   # TM/autopilot handles control every tick automatically

        if self.btype == "stopped":
            self.actor.apply_control(carla.VehicleControl(brake=1.0))
            return

        tgt      = self.planner.target(self.actor)
        steer, _ = _steer_toward(self.actor, tgt)
        throttle = _throttle(self.actor, self.target_speed)

        if self.btype == "ttc_brake":
            if self.target_actor and compute_ttc(self.actor, self.target_actor) < self.ttc_thresh:
                self.actor.apply_control(
                    carla.VehicleControl(brake=self.brake_int, steer=0.0))
                return

        elif self.btype == "cut_in":
            if self.target_actor:
                d = self.actor.get_location().distance(self.target_actor.get_location())
                if d < float(self.cfg.get("cut_in_distance", 18.0)):
                    s, _ = _steer_toward(self.actor, self.target_actor.get_location())
                    steer = max(-0.6, min(0.6, s * 1.5))

        elif self.btype == "sudden_brake":
            if time.time() - self._t0 >= float(self.cfg.get("brake_after_seconds", 3.0)):
                self.actor.apply_control(
                    carla.VehicleControl(brake=self.brake_int, steer=0.0))
                return

        self.actor.apply_control(
            carla.VehicleControl(throttle=throttle, steer=steer))


class WalkerBehaviorController:
    def __init__(self, actor, cfg: dict, world):
        self.actor = actor
        self._ctrl = self._spawn_ctrl(actor, cfg, world)

    @staticmethod
    def _spawn_ctrl(actor, cfg, world):
        bp   = world.get_blueprint_library().find("controller.ai.walker")
        ctrl = world.spawn_actor(bp, carla.Transform(), attach_to=actor)
        ctrl.start()
        direction = cfg.get("direction", "right")
        angle_map = {"right": 90, "left": -90, "forward": 0, "backward": 180}
        rad = math.radians(actor.get_transform().rotation.yaw
                           + angle_map.get(direction, 90))
        tgt = carla.Location(
            x=actor.get_location().x + math.cos(rad) * 20,
            y=actor.get_location().y + math.sin(rad) * 20,
            z=actor.get_location().z,
        )
        ctrl.go_to_location(tgt)
        ctrl.set_max_speed(float(cfg.get("target_speed", 1.4)))
        return ctrl

    def tick(self): pass

    def destroy(self):
        self._ctrl.stop()
        self._ctrl.destroy()


# ─────────────────────────────────────────────────────────────────────────────
# Sensors
# ─────────────────────────────────────────────────────────────────────────────

def attach_sensors(world, blueprint_library, vehicle, sensors_cfg: list) -> list:
    attached = []
    for s in sensors_cfg:
        bp = blueprint_library.find(s["type"])
        if bp is None:
            print(f"[Warning] Sensor not found: {s['type']}")
            continue
        for k, v in s.get("attributes", {}).items():
            if bp.has_attribute(k):
                bp.set_attribute(k, str(v))
        t = s.get("transform", {})
        tf = carla.Transform(
            carla.Location(x=t.get("x", 0), y=t.get("y", 0), z=t.get("z", 0)),
            carla.Rotation(pitch=t.get("pitch", 0), yaw=t.get("yaw", 0),
                           roll=t.get("roll", 0)),
        )
        actor = world.spawn_actor(bp, tf, attach_to=vehicle)
        attached.append(actor)
        print(f"  [Sensor] {s.get('id', s['type'])} attached")
    return attached


# ─────────────────────────────────────────────────────────────────────────────
# Collision Monitor
# ─────────────────────────────────────────────────────────────────────────────

class CollisionMonitor:
    def __init__(self, world, blueprint_library, vehicle):
        self.collision_occurred = False
        self.events = []
        bp = blueprint_library.find("sensor.other.collision")
        self._sensor = world.spawn_actor(bp, carla.Transform(), attach_to=vehicle)
        self._sensor.listen(self._cb)

    def _cb(self, e):
        self.collision_occurred = True
        self.events.append(e)
        print(f"[Collision] ego ← {e.other_actor.type_id}")

    def destroy(self):
        self._sensor.stop()
        self._sensor.destroy()


# ─────────────────────────────────────────────────────────────────────────────
# Termination
# ─────────────────────────────────────────────────────────────────────────────

class TerminationChecker:
    def __init__(self, cfg: dict, ego, cm: CollisionMonitor):
        self.cfg       = cfg
        self.ego       = ego
        self.cm        = cm
        self.start_loc = ego.get_location()
        self.t0        = time.time()
        self._stopped  = None

    def elapsed(self) -> float:
        return time.time() - self.t0

    def should_terminate(self) -> bool:
        # ── Never terminate before min_duration ──────────────────────────
        min_dur = float(self.cfg.get("min_duration_seconds", 0.0))
        if self.elapsed() < min_dur:
            return False

        if (self.cfg.get("end_on_collision") and self.cm.collision_occurred):
            print("[Termination] collision"); return True

        max_dist = self.cfg.get("max_distance_from_start")
        if max_dist and self.ego.get_location().distance(self.start_loc) > max_dist:
            print("[Termination] max_distance"); return True

        max_dur = self.cfg.get("max_duration_seconds")
        if max_dur and self.elapsed() > max_dur:
            print("[Termination] max_duration"); return True

        if self.cfg.get("end_on_ego_stopped"):
            if _speed(self.ego) < 0.1:
                self._stopped = self._stopped or time.time()
                if time.time() - self._stopped > 2.0:
                    print("[Termination] ego stopped"); return True
            else:
                self._stopped = None

        return False


# ─────────────────────────────────────────────────────────────────────────────
# Postconditions
# ─────────────────────────────────────────────────────────────────────────────

def evaluate_postconditions(cfg: list, ego, cm: CollisionMonitor):
    results = {}
    for c in cfg:
        ctype = c["type"]
        if ctype == "no_collision":
            ok = not cm.collision_occurred
            results["no_collision"] = ok
        elif ctype == "ego_speed_below":
            ok = _speed(ego) * 3.6 < float(c.get("value", 5.0))
            results[f"ego_speed_below_{c.get('value')}kmh"] = ok
        elif ctype == "ego_speed_above":
            ok = _speed(ego) * 3.6 > float(c.get("value", 0.0))
            results[f"ego_speed_above_{c.get('value')}kmh"] = ok
        else:
            print(f"  [Post] Unknown: {ctype}"); continue
        lbl = list(results)[-1]
        print(f"  [Post] {lbl}: {'PASS ✓' if ok else 'FAIL ✗'}")
    overall = all(results.values()) if results else True
    print(f"  Overall: {'PASS ✓' if overall else 'FAIL ✗'}")
    return results


# ─────────────────────────────────────────────────────────────────────────────
# Visualization  (RGB camera  +  LiDAR bird's-eye view)
# ─────────────────────────────────────────────────────────────────────────────

W, H = 800, 600          # pygame window size
BEV_RANGE = 40.0         # metres shown in each direction for BEV
BEV_SCALE = min(W, H) / (2 * BEV_RANGE)   # px/m

CAMERA_TRANSFORMS = {
    "behind": carla.Transform(carla.Location(x=-6, y=0, z=3.0),
                               carla.Rotation(pitch=-10)),
    # Elevated chase cam: above and behind, tilted forward so you see the road ahead
    "top":    carla.Transform(carla.Location(x=-4, y=0, z=6.0),
                               carla.Rotation(pitch=-25)),
    "front":  carla.Transform(carla.Location(x=0,  y=0, z=2.0),
                               carla.Rotation(pitch=0)),
}

# Ordered cycle for TAB key
DISPLAY_MODES = ["behind", "top", "lidar"]


def attach_display_camera(world, blueprint_library, vehicle, view: str) -> carla.Actor:
    bp = blueprint_library.find("sensor.camera.rgb")
    bp.set_attribute("image_size_x", str(W))
    bp.set_attribute("image_size_y", str(H))
    bp.set_attribute("fov", "100")
    tf = CAMERA_TRANSFORMS.get(view, CAMERA_TRANSFORMS["behind"])
    return world.spawn_actor(bp, tf, attach_to=vehicle)


def attach_bev_lidar(world, blueprint_library, vehicle):
    """Attach a dedicated LiDAR for the bird's-eye view display."""
    bp = blueprint_library.find("sensor.lidar.ray_cast")
    bp.set_attribute("range",               "50")
    bp.set_attribute("channels",            "32")
    bp.set_attribute("points_per_second",   "100000")
    bp.set_attribute("rotation_frequency",  "20")
    bp.set_attribute("upper_fov",           "2")
    bp.set_attribute("lower_fov",           "-24")
    tf = carla.Transform(carla.Location(x=0, y=0, z=2.0))
    return world.spawn_actor(bp, tf, attach_to=vehicle)


def render_bev(surface: pygame.Surface, points: np.ndarray,
               ego, adversaries: list):
    """
    Render a top-down LiDAR view.
    CARLA sensor frame: x=forward, y=right, z=up
    Screen mapping:  x→up, y→right
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

    # LiDAR points — colour by height
    if points is not None and len(points) > 0:
        xs = points[:, 0]           # forward
        ys = points[:, 1]           # right
        zs = points[:, 2]           # up
        # Filter: keep only points above ground and within display range
        mask = (zs > -2.0) & (zs < 3.0) & (np.abs(xs) < BEV_RANGE) & (np.abs(ys) < BEV_RANGE)
        xs, ys, zs = xs[mask], ys[mask], zs[mask]

        if len(xs):
            # Map z → colour  (low=blue, mid=green, high=red)
            z_norm = np.clip((zs + 0.5) / 3.5, 0, 1)
            r = (z_norm * 220).astype(np.uint8)
            g = ((1 - np.abs(z_norm - 0.5) * 2) * 200).astype(np.uint8)
            b = ((1 - z_norm) * 200).astype(np.uint8)
            colours = np.stack([r, g, b], axis=1)

            # Screen coords
            sx = (cx + ys * BEV_SCALE).astype(int)    # y (right) → screen x
            sy = (cy - xs * BEV_SCALE).astype(int)    # x (fwd)   → screen y (up)

            for i in range(len(sx)):
                if 0 <= sx[i] < W and 0 <= sy[i] < H:
                    surface.set_at((sx[i], sy[i]),
                                   (int(colours[i, 0]),
                                    int(colours[i, 1]),
                                    int(colours[i, 2])))

    # Ego marker (white rectangle)
    pygame.draw.rect(surface, (240, 240, 240), (cx - 4, cy - 8, 8, 14), 0)
    pygame.draw.polygon(surface, (240, 240, 240),
                        [(cx, cy - 14), (cx - 5, cy - 8), (cx + 5, cy - 8)])

    # Adversary markers in ego-relative coords
    ego_tf = ego.get_transform()
    ego_yaw_rad = math.radians(ego_tf.rotation.yaw)
    cos_y, sin_y = math.cos(-ego_yaw_rad), math.sin(-ego_yaw_rad)

    for adv in adversaries:
        adv_loc = adv.get_location()
        dx = adv_loc.x - ego_tf.location.x
        dy = adv_loc.y - ego_tf.location.y
        # Rotate into ego frame
        rx =  cos_y * dx - sin_y * dy   # forward
        ry =  sin_y * dx + cos_y * dy   # right
        sx_adv = int(cx + ry * BEV_SCALE)
        sy_adv = int(cy - rx * BEV_SCALE)
        if 0 <= sx_adv < W and 0 <= sy_adv < H:
            pygame.draw.rect(surface, (255, 80, 80),
                             (sx_adv - 4, sy_adv - 7, 8, 12), 0)

    # Range label
    font_small = pygame.font.SysFont("monospace", 11)
    surface.blit(font_small.render(f"±{int(BEV_RANGE)}m", True, (80, 80, 100)),
                 (W - 50, H - 18))


def render_hud(surface, font, ego, adversaries, cm, tc, display_mode,
               episode_num: int = 1):
    speed_kmh = _speed(ego) * 3.6
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

    # One line per adversary showing distance + speed
    for i, adv in enumerate(adversaries):
        adv_d   = ego.get_location().distance(adv.get_location())
        adv_spd = _speed(adv) * 3.6
        lines.append(f"Adv-{i}: {adv_d:5.1f} m  {adv_spd:.1f}km/h")

    lines += ["", f"[{display_mode.upper()}] TAB=cycle N=skip"]

    overlay = pygame.Surface((210, len(lines) * 20 + 12), pygame.SRCALPHA)
    overlay.fill((0, 0, 0, 155))
    surface.blit(overlay, (5, 5))
    for i, line in enumerate(lines):
        col = (255, 80, 80) if ("YES" in line or "Adv" in line) else (210, 210, 60)
        surface.blit(font.render(line, True, col), (10, 10 + i * 20))


# ─────────────────────────────────────────────────────────────────────────────
# nearby_random spawn helper  (most reliable for visibility)
# ─────────────────────────────────────────────────────────────────────────────

def _nearby_random_spawn(spawn_cfg: dict, spawn_points: list,
                          ego_transform) -> carla.Transform:
    """
    Pick a real map spawn point whose distance from ego is within
    [min_dist, max_dist].  Guaranteed to be on a valid road surface.
    Falls back to any spawn point if no candidate matches.
    """
    min_d = float(spawn_cfg.get("min_dist", 15.0))
    max_d = float(spawn_cfg.get("max_dist", 50.0))
    ego_loc = ego_transform.location

    candidates = [sp for sp in spawn_points
                  if min_d <= ego_loc.distance(sp.location) <= max_d]

    if candidates:
        chosen = random.choice(candidates)
        print(f"  [NearbyRandom] {len(candidates)} candidates  "
              f"({min_d:.0f}–{max_d:.0f} m)  → {chosen.location}")
        return chosen

    # Expand search if nothing found
    print(f"  [NearbyRandom] No point in {min_d}–{max_d} m range; "
          f"expanding to 10–100 m")
    candidates = [sp for sp in spawn_points
                  if 10.0 <= ego_loc.distance(sp.location) <= 100.0]
    return random.choice(candidates) if candidates else random.choice(spawn_points)


# ─────────────────────────────────────────────────────────────────────────────
# Episode helpers
# ─────────────────────────────────────────────────────────────────────────────

def _cleanup_episode(vehicle_actors, walker_actors, sensor_actors,
                      walker_ctrls, cm, world):
    """Destroy all per-episode actors.  Safe to call even if partially built."""
    for c in walker_ctrls:
        try: c.destroy()
        except: pass
    if cm:
        try: cm.destroy()
        except: pass
    for s in sensor_actors:
        try: s.stop(); s.destroy()
        except: pass
    for a in reversed(vehicle_actors + walker_actors):
        try: a.destroy()
        except: pass
    # Let the world process the deletions
    for _ in range(5):
        try: world.tick()
        except: pass


def _spawn_resolve(spawn_cfg, world, cmap, sps, ego_transform=None):
    """Route to the right spawn helper including the new nearby_random mode."""
    if spawn_cfg.get("mode") == "nearby_random":
        if ego_transform is None:
            return random.choice(sps)
        return _nearby_random_spawn(spawn_cfg, sps, ego_transform)
    return get_transform_for_spawn(spawn_cfg, world, cmap, sps, ego_transform)


def _intermission(screen, font, episode_num, duration: float = 1.0) -> bool:
    """
    Show a brief between-episode screen with countdown.
    Returns True to continue, False if ESC pressed.
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
                    return True   # skip remaining countdown

        screen.fill((10, 10, 18))
        big   = pygame.font.SysFont("monospace", 36, bold=True)
        small = pygame.font.SysFont("monospace", 20)

        screen.blit(big.render(f"Episode {episode_num} complete", True, (180, 180, 60)),
                    (W // 2 - 210, H // 2 - 60))
        screen.blit(small.render(f"Next in {remaining:.1f}s  (N = skip now)",
                                  True, (120, 200, 120)),
                    (W // 2 - 210, H // 2))
        screen.blit(small.render("ESC = quit",  True, (160, 80, 80)),
                    (W // 2 - 60, H // 2 + 50))
        pygame.display.flip()
        clock.tick(30)


# ─────────────────────────────────────────────────────────────────────────────
# Single-episode runner
# ─────────────────────────────────────────────────────────────────────────────

def _run_one_episode(world, client, bpl, cmap, sps, config,
                      episode_num: int,
                      screen, clock, font,
                      display_mode_holder: list,   # [str]  persists across episodes
                      bev_surf: pygame.Surface) -> str:
    """
    Spawn actors, run until termination, clean up.
    Returns:  "quit"  – user pressed ESC / closed window
              "next"  – user pressed N (skip to next episode)
              "done"  – episode ended normally
    """
    vehicle_actors = []
    walker_actors  = []
    sensor_actors  = []
    walker_ctrls   = []
    cm             = None

    # Traffic Manager — synchronous mode, same fixed_delta as world
    tm = client.get_trafficmanager(8000)
    tm.set_synchronous_mode(True)
    tm.set_global_distance_to_leading_vehicle(2.0)

    try:
        env = config.get("environment", {})
        if "weather" in env:
            set_weather(world, env["weather"])
        configure_traffic_lights(world, env.get("traffic_lights", "normal"))

        # ── Ego ──────────────────────────────────────────────────────────
        ego_cfg = config["ego"]
        ego_tf  = _spawn_resolve(ego_cfg.get("spawn", {"mode": "random"}),
                                  world, cmap, sps)
        ego = safe_spawn_vehicle(world, bpl, ego_cfg["blueprint"], ego_tf)
        vehicle_actors.append(ego)
        world.tick()
        print(f"  [Ego]  {ego.get_location()}")

        cm = CollisionMonitor(world, bpl, ego)
        for s in attach_sensors(world, bpl, ego, ego_cfg.get("sensors", [])):
            sensor_actors.append(s)

        # ── Adversaries ──────────────────────────────────────────────────
        adversaries = []
        adv_ctrls   = []
        for adv_cfg in config.get("adversaries", []):
            adv_tf = _spawn_resolve(
                adv_cfg.get("spawn", {"mode": "nearby_random"}),
                world, cmap, sps, ego_transform=ego.get_transform())

            if adv_cfg.get("type") == "walker":
                adv  = safe_spawn_walker(world, bpl,
                           adv_cfg.get("blueprint","walker.pedestrian.0001"), adv_tf)
                walker_actors.append(adv)
                ctrl = WalkerBehaviorController(adv, adv_cfg.get("behavior",{}), world)
                walker_ctrls.append(ctrl)
            else:
                adv  = safe_spawn_vehicle(world, bpl, adv_cfg["blueprint"], adv_tf)
                vehicle_actors.append(adv)
                ctrl = VehicleBehaviorController(
                    adv, adv_cfg.get("behavior", {}),
                    cmap,
                    target_actor=ego,
                    tm=tm)                          # pass TM so it can use it
                adv_ctrls.append(ctrl)
            adversaries.append(adv)

        # In synchronous mode actor positions are only valid after a tick
        world.tick()

        for i, (adv_cfg, adv) in enumerate(zip(config.get("adversaries", []), adversaries)):
            dist = ego.get_location().distance(adv.get_location())
            loc  = adv.get_location()
            print(f"  [Adv '{adv_cfg.get('id','?')}']  {loc}  dist={dist:.1f}m")
            if loc.x == 0.0 and loc.y == 0.0:
                print(f"  [Warning] Adv-{i} at world origin — spawn likely failed or collided")

        # ── Ego behavior (custom controller, not TM) ──────────────────────
        ego_ctrl = VehicleBehaviorController(
            ego,
            ego_cfg.get("behavior", {"type": "ttc_brake", "target_speed": 10}),
            cmap,
            target_actor=adversaries[0] if adversaries else None,
            tm=tm)

        # ── Termination ───────────────────────────────────────────────────
        term_cfg = dict(config.get("termination", {}))
        ep_dur   = float(config.get("loop", {}).get(
                         "episode_duration_seconds",
                         term_cfg.get("max_duration_seconds", 20.0)))
        term_cfg["max_duration_seconds"] = ep_dur
        tc = TerminationChecker(term_cfg, ego, cm)

        # ── Dual cameras (behind + top) always attached ───────────────────
        queues   = {}
        cameras  = {}
        for view in ("behind", "top"):
            cam = attach_display_camera(world, bpl, ego, view)
            sensor_actors.append(cam)
            cameras[view] = cam
            q: Queue = Queue()
            queues[view] = q
            cam.listen(lambda img, _q=q: _q.put(
                np.frombuffer(img.raw_data, dtype=np.uint8)
                .reshape((img.height, img.width, 4))[:, :, :3][:, :, ::-1]))

        # BEV LiDAR
        lidar_pts = [None]
        bev_lidar = attach_bev_lidar(world, bpl, ego)
        sensor_actors.append(bev_lidar)
        bev_lidar.listen(lambda data, lp=lidar_pts: lp.__setitem__(
            0, np.frombuffer(data.raw_data, dtype=np.float32)
                .reshape((-1, 4))[:, :3]))

        # ── Main simulation loop ──────────────────────────────────────────
        rgb_frames = {"behind": np.zeros((H, W, 3), dtype=np.uint8),
                      "top":    np.zeros((H, W, 3), dtype=np.uint8)}

        while True:
            world.tick()

            # ── Events ───────────────────────────────────────────────────
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
                        display_mode_holder[0] = DISPLAY_MODES[(idx + 1) % len(DISPLAY_MODES)]
                        print(f"  [Display] → {display_mode_holder[0]}")

            # ── Draw ─────────────────────────────────────────────────────
            mode = display_mode_holder[0]

            if mode in ("behind", "top"):
                try:
                    while True:
                        rgb_frames[mode] = queues[mode].get_nowait()
                except Empty:
                    pass
                screen.blit(
                    pygame.surfarray.make_surface(rgb_frames[mode].swapaxes(0, 1)),
                    (0, 0))
            else:   # lidar BEV
                render_bev(bev_surf, lidar_pts[0], ego, adversaries)
                screen.blit(bev_surf, (0, 0))

            render_hud(screen, font, ego, adversaries, cm, tc, mode, episode_num)
            pygame.display.flip()
            clock.tick(30)

            # ── Behaviors ────────────────────────────────────────────────
            ego_ctrl.tick()
            for c in adv_ctrls:   c.tick()
            for c in walker_ctrls: c.tick()

            if tc.should_terminate():
                break

        # Postconditions
        pcfg = config.get("postconditions", [])
        if pcfg:
            print("  [Postconditions]")
            evaluate_postconditions(pcfg, ego, cm)

        return "done"

    finally:
        _cleanup_episode(vehicle_actors, walker_actors, sensor_actors,
                          walker_ctrls, cm, world)


# ─────────────────────────────────────────────────────────────────────────────
# Main entry – episode loop
# ─────────────────────────────────────────────────────────────────────────────

def run(config_path: str, host: str = "localhost", port: int = 2000):
    config = load_config(config_path)
    scenario_name = config.get("scenario", {}).get("type", "scenario")
    loop_cfg      = config.get("loop", {})
    max_episodes  = loop_cfg.get("max_episodes", 999999)     # effectively infinite
    intermission  = float(loop_cfg.get("intermission_seconds", 1.0))

    print(f"\n{'='*60}")
    print(f"Scenario  : {scenario_name}")
    print(f"Map       : {config['map']}")
    print(f"Max eps   : {max_episodes}")
    print(f"Ep dur    : {loop_cfg.get('episode_duration_seconds', 20)} s")
    print(f"ESC=quit  N=skip episode  TAB=cycle view  (behind/top/lidar)")
    print(f"{'='*60}\n")

    client, world = connect(config["map"], host, port)
    bpl  = world.get_blueprint_library()
    cmap = world.get_map()
    sps  = cmap.get_spawn_points()
    print(f"[Setup] {len(sps)} spawn points on {config['map']}\n")

    # Shuffle spawn points once – each episode cycles to a new region
    random.shuffle(sps)

    # Init pygame once – stays alive for all episodes
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
                display_mode_holder, bev_surf)

            if result == "quit":
                print("\n[Loop] User quit.")
                break

            if episode >= max_episodes:
                print(f"\n[Loop] Reached max_episodes={max_episodes}.")
                break

            # Intermission between episodes
            cont = _intermission(screen, font, episode, intermission)
            if not cont:
                print("\n[Loop] User quit during intermission.")
                break

    finally:
        restore_async(world)
        pygame.quit()
        print("[Done]\n")


# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--host",   default="localhost")
    ap.add_argument("--port",   type=int, default=2000)
    args = ap.parse_args()
    run(args.config, host=args.host, port=args.port)