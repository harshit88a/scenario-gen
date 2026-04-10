"""
Vehicle and walker behavior controllers.

VehicleBehaviorController behavior types
-----------------------------------------
traffic_manager      – CARLA Traffic Manager (natural road following).
intersection_conflict – TM autopilot + forces adversary straight through
                        the same junction as ego, T-boning it.  Uses
                        junction_cross spawn for perpendicular approach.
ttc_brake            – custom waypoint follower; brake when TTC < threshold.
constant_speed       – custom waypoint follower at fixed speed.
cut_in               – follow path; steer into ego lane when close.
sudden_brake         – drive then brake after N seconds.
stopped              – hold brake.
autopilot            – basic CARLA autopilot (no TM config).
"""

import math
import time

import carla

from .utils import get_speed, compute_ttc, steer_toward, throttle_for_speed


class WaypointPlanner:
    """Pre-compute a road-following waypoint path for an actor."""

    def __init__(self, actor, carla_map, maneuver: str = "straight",
                 lookahead: float = 120.0, step: float = 3.0):
        self.wps = self._plan(actor, carla_map, maneuver, lookahead, step)
        self.idx = 0

    def _plan(self, actor, carla_map, maneuver, lookahead, step):
        wp      = carla_map.get_waypoint(actor.get_location(), project_to_road=True)
        result  = [wp]
        current = wp
        n       = int(lookahead / step)

        for i in range(n):
            nexts = current.next(step)
            if not nexts:
                break

            if maneuver == "straight":
                chosen = min(
                    nexts,
                    key=lambda w: abs(
                        (w.transform.rotation.yaw
                         - current.transform.rotation.yaw + 180) % 360 - 180
                    ),
                )
            elif maneuver in ("left_turn", "right_turn"):
                straight = min(
                    nexts,
                    key=lambda w: abs(
                        (w.transform.rotation.yaw
                         - current.transform.rotation.yaw + 180) % 360 - 180
                    ),
                )
                if i < n // 4:
                    chosen = straight
                else:
                    sign = -1 if maneuver == "left_turn" else 1

                    def yaw_score(w, _cur=current, _sign=sign):
                        d = (w.transform.rotation.yaw
                             - _cur.transform.rotation.yaw) % 360
                        if d > 180:
                            d -= 360
                        return _sign * d

                    chosen = min(nexts, key=yaw_score)
            else:
                chosen = nexts[0]

            result.append(chosen)
            current = chosen

        return result

    def target(self, actor) -> carla.Location:
        """Return the next lookahead waypoint location for *actor*."""
        loc = actor.get_location()
        while self.idx < len(self.wps) - 1:
            if loc.distance(self.wps[self.idx].transform.location) < 5.0:
                self.idx += 1
            else:
                break
        return self.wps[min(self.idx, len(self.wps) - 1)].transform.location


class VehicleBehaviorController:
    """Tick-based vehicle controller supporting multiple behavior strategies."""

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
                    print(
                        f"  [Behavior] traffic_manager: port={tm_port}, "
                        f"speed_pct={speed_pct}"
                    )

        elif self.btype == "autopilot":
            actor.set_autopilot(True)

        else:
            self.planner = WaypointPlanner(
                actor, carla_map, maneuver=cfg.get("maneuver", "straight")
            )

    # ── Intersection conflict path builder ────────────────────────────────

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
            ego_wp   = carla_map.get_waypoint(ego.get_location(), project_to_road=True)
            junction, _ = self._find_junction_ahead(ego_wp)
            if junction is None:
                print("  [ConflictPath] No junction ahead of ego — plain TM")
                return

            adv_wp = carla_map.get_waypoint(actor.get_location(), project_to_road=True)
            path: list = []
            step = 3.0

            # Phase 1: walk from adversary up to the junction
            current = adv_wp
            for _ in range(int(100 / step)):
                nexts = current.next(step)
                if not nexts:
                    break
                current = nexts[0]
                path.append(current.transform.location)
                if current.is_junction:
                    break

            # Phase 2: traverse the junction going STRAIGHT
            # "Straight" = pick the next wp whose yaw changes least.
            for _ in range(15):
                nexts = current.next(step)
                if not nexts:
                    break
                current = min(
                    nexts,
                    key=lambda w: abs(
                        (w.transform.rotation.yaw
                         - current.transform.rotation.yaw + 180) % 360 - 180
                    ),
                )
                path.append(current.transform.location)
                if not current.is_junction:
                    break

            # Phase 3: continue 30 m past the junction
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

    # ── Per-tick control ──────────────────────────────────────────────────

    def tick(self) -> None:
        if self.btype in ("traffic_manager", "intersection_conflict", "autopilot"):
            return  # TM / autopilot handles control every tick automatically

        if self.btype == "stopped":
            self.actor.apply_control(carla.VehicleControl(brake=1.0))
            return

        tgt      = self.planner.target(self.actor)
        steer, _ = steer_toward(self.actor, tgt)
        throttle = throttle_for_speed(self.actor, self.target_speed)

        if self.btype == "ttc_brake":
            if (self.target_actor
                    and compute_ttc(self.actor, self.target_actor) < self.ttc_thresh):
                self.actor.apply_control(
                    carla.VehicleControl(brake=self.brake_int, steer=0.0)
                )
                return

        elif self.btype == "cut_in":
            if self.target_actor:
                d = self.actor.get_location().distance(
                    self.target_actor.get_location()
                )
                if d < float(self.cfg.get("cut_in_distance", 18.0)):
                    s, _ = steer_toward(self.actor, self.target_actor.get_location())
                    steer = max(-0.6, min(0.6, s * 1.5))

        elif self.btype == "sudden_brake":
            if time.time() - self._t0 >= float(self.cfg.get("brake_after_seconds", 3.0)):
                self.actor.apply_control(
                    carla.VehicleControl(brake=self.brake_int, steer=0.0)
                )
                return

        self.actor.apply_control(
            carla.VehicleControl(throttle=throttle, steer=steer)
        )


class WalkerBehaviorController:
    """AI walker controller — spawns a controller.ai.walker and sets a target."""

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
        rad = math.radians(
            actor.get_transform().rotation.yaw + angle_map.get(direction, 90)
        )
        tgt = carla.Location(
            x=actor.get_location().x + math.cos(rad) * 20,
            y=actor.get_location().y + math.sin(rad) * 20,
            z=actor.get_location().z,
        )
        ctrl.go_to_location(tgt)
        ctrl.set_max_speed(float(cfg.get("target_speed", 1.4)))
        return ctrl

    def tick(self) -> None:
        pass  # AI controller updates automatically

    def destroy(self) -> None:
        self._ctrl.stop()
        self._ctrl.destroy()
