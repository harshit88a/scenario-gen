"""
Vehicle and walker behavior controllers.

VehicleBehaviorController behavior types
-----------------------------------------
traffic_manager      – CARLA Traffic Manager (natural road following).
intersection_conflict – TM autopilot + forces adversary straight through
                        the same junction as ego, T-boning it.  Uses
                        junction_cross spawn for perpendicular approach.
ttc_brake            – custom waypoint follower; brake when TTC < threshold.
                        Keys: target_speed, brake_intensity, ttc_threshold, maneuver.
follow_lane          – waypoint follower at fixed speed, no braking logic.
                        Keys: target_speed, maneuver.
constant_speed       – alias for follow_lane.
cut_in               – follow path; steer into ego lane when close.
                        Keys: target_speed, cut_in_distance, maneuver.
sudden_brake         – drive then full brake after N seconds.
                        Keys: target_speed, brake_after_seconds, brake_intensity.
lane_change          – follow current lane then steer into adjacent lane at trigger.
                        Keys: target_speed, direction (left|right),
                        trigger_seconds (float) OR trigger_dist (float, requires
                        target_actor=ego), maneuver.
sudden_accelerate    – drive at target_speed; surge to surge_speed when ego within
                        surge_trigger_dist.  Models throttle-malfunction / aggressive
                        gap-close.  Keys: target_speed, surge_speed, surge_trigger_dist.
sudden_turn          – drive straight at target_speed; rebuild planner with
                        turn_maneuver when ego within trigger_dist.  Models a
                        last-second direction change at a junction.
                        Keys: target_speed, turn_maneuver (left_turn|right_turn),
                        trigger_dist.
stopped              – hold brake every tick (freshly spawned, engine off).
parked               – apply hand_brake once on spawn, no further control.
                        Keys: (none required).
autopilot            – basic CARLA autopilot (no TM config).

sudden_brake extended key:
  trigger_dist       – if set, brake is triggered when ego is within this distance
                        instead of after brake_after_seconds.

WalkerBehaviorController behavior types
-----------------------------------------
walk_across          – Walker crosses perpendicular to the road via direct
                        WalkerControl (no AI/navmesh). Matches Scenic's
                        CrossingBehavior / SetWalkingDirectionAction.
                        Keys: target_speed (m/s, default 1.4), direction
                        (right|left|forward|backward), trigger_dist (optional).
walk_to              – Walk straight toward explicit world coordinates.
                        Keys: destination {x,y,z}, target_speed, trigger_dist.
stationary           – Actor stands still (zero-speed WalkerControl each tick).
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

    def _plan(self, actor, carla_map, maneuver, lookahead, step, start_wp=None):
        if start_wp is not None:
            wp = start_wp
        else:
            wp = carla_map.get_waypoint(actor.get_location(), project_to_road=True)
        if wp is None:
            return []
        # If starting inside a junction, back up to the last valid approach lane.
        while wp.is_junction:
            prev = wp.previous(5.0)
            if not prev:
                break
            wp = prev[0]
        result  = [wp]
        current = wp
        n       = int(lookahead / step)

        for i in range(n):
            nexts = current.next(step)
            if not nexts:
                break

            straight = min(
                nexts,
                key=lambda w, _c=current: abs(
                    (w.transform.rotation.yaw
                     - _c.transform.rotation.yaw + 180) % 360 - 180
                ),
            )

            if maneuver == "straight":
                chosen = straight
            elif maneuver in ("left_turn", "right_turn"):
                # The turning choice must be made at the junction ENTRY point —
                # i.e., when current is still on the approach road but nexts are
                # already inside the junction (different lanes per maneuver).
                # Once inside the junction we just follow the selected lane
                # (usually only one next waypoint), then continue straight on exit.
                at_entry = (
                    not current.is_junction
                    and any(nw.is_junction for nw in nexts)
                )
                if at_entry:
                    # Use the junction lane API to find the correct turning lane.
                    # wp.next() from the approach road often returns only the direct
                    # lane continuation (straight), missing turning alternatives.
                    # get_waypoints() enumerates every entry→exit pair so we can
                    # pick the one matching the requested maneuver.
                    approach_yaw = current.transform.rotation.yaw
                    chosen = None

                    junction_nw = next(
                        (nw for nw in nexts if nw.is_junction), None
                    )
                    if junction_nw is not None:
                        try:
                            pairs = junction_nw.get_junction().get_waypoints(
                                carla.LaneType.Driving
                            )
                            best_score = None
                            for ent_wp, ext_wp in pairs:
                                prev = ent_wp.previous(5.0)
                                ent_yaw = (prev[0].transform.rotation.yaw
                                           if prev else ent_wp.transform.rotation.yaw)
                                # Only consider paths that approach from our direction
                                if abs((ent_yaw - approach_yaw + 180) % 360 - 180) > 45:
                                    continue
                                d = (ext_wp.transform.rotation.yaw
                                     - ent_wp.transform.rotation.yaw) % 360
                                if d > 180:
                                    d -= 360
                                if maneuver == "right_turn" and d > 20:
                                    if best_score is None or d > best_score:
                                        best_score = d
                                        chosen = ent_wp
                                elif maneuver == "left_turn" and d < -20:
                                    if best_score is None or d < best_score:
                                        best_score = d
                                        chosen = ent_wp
                        except Exception:
                            pass

                    # Fallback: yaw-score nexts directly (original approach)
                    if chosen is None:
                        sign = 1 if maneuver == "left_turn" else -1

                        def yaw_score(w, _cur=current, _sign=sign):
                            d = (w.transform.rotation.yaw
                                 - _cur.transform.rotation.yaw) % 360
                            if d > 180:
                                d -= 360
                            return _sign * d

                        best = min(nexts, key=yaw_score)
                        best_d = (best.transform.rotation.yaw
                                  - current.transform.rotation.yaw) % 360
                        if best_d > 180:
                            best_d -= 360
                        # Require at least 5° deviation to avoid committing to a
                        # slight kink at T-junctions that lack a matching exit.
                        chosen = best if abs(best_d) > 5 else straight
                else:
                    chosen = straight
            else:
                chosen = nexts[0]

            result.append(chosen)
            current = chosen

        return result

    @classmethod
    def from_wp(cls, start_wp, maneuver: str = "straight",
                lookahead: float = 120.0, step: float = 3.0):
        """Build a planner starting from an already-resolved waypoint."""
        obj = cls.__new__(cls)
        obj.wps = obj._plan(None, None, maneuver, lookahead, step, start_wp=start_wp)
        obj.idx = 0
        return obj

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

        elif self.btype == "parked":
            actor.apply_control(carla.VehicleControl(hand_brake=True))

        else:
            if self.btype == "lane_change":
                self._lc_direction      = cfg.get("direction", "left")
                self._lc_trigger_sec    = cfg.get("trigger_seconds")
                self._lc_trigger_dist   = cfg.get("trigger_dist")
                self._lc_done           = False
            elif self.btype == "sudden_accelerate":
                self._surge_speed       = float(cfg.get("surge_speed", 30.0))
                self._surge_dist        = float(cfg.get("surge_trigger_dist", 30.0))
                self._surged            = False
            elif self.btype == "sudden_turn":
                self._st_maneuver       = cfg.get("turn_maneuver", "right_turn")
                self._st_trigger_dist   = float(cfg.get("trigger_dist", 22.0))
                self._st_done           = False
            self.planner = WaypointPlanner(
                actor,
                carla_map,
                maneuver=cfg.get("maneuver", "straight"),
                lookahead=float(cfg.get("lookahead", 120.0)),
                step=float(cfg.get("planner_step", 3.0)),
            )

    # ── Lane-change helper ────────────────────────────────────────────────

    def _exec_lane_change(self) -> None:
        """Rebuild the waypoint planner starting from the adjacent lane."""
        wp  = self._cmap.get_waypoint(self.actor.get_location(), project_to_road=True)
        adj = wp.get_left_lane() if self._lc_direction == "left" else wp.get_right_lane()
        if adj is not None and adj.lane_type == carla.LaneType.Driving:
            self.planner = WaypointPlanner.from_wp(adj, maneuver="straight")
        self._lc_done = True

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

        if self.btype == "parked":
            return  # hand_brake set once in __init__; nothing to do each tick

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
            trig_dist = self.cfg.get("trigger_dist")
            if trig_dist is not None and self.target_actor is not None:
                should_brake = (
                    self.actor.get_location().distance(
                        self.target_actor.get_location()
                    ) <= float(trig_dist)
                )
            else:
                should_brake = (
                    time.time() - self._t0
                    >= float(self.cfg.get("brake_after_seconds", 3.0))
                )
            if should_brake:
                self.actor.apply_control(
                    carla.VehicleControl(brake=self.brake_int, steer=0.0)
                )
                return

        elif self.btype == "sudden_accelerate":
            if not self._surged and self.target_actor is not None:
                dist = self.actor.get_location().distance(
                    self.target_actor.get_location()
                )
                if dist <= self._surge_dist:
                    self.target_speed = self._surge_speed
                    self._surged      = True

        elif self.btype == "sudden_turn":
            if not self._st_done and self.target_actor is not None:
                dist = self.actor.get_location().distance(
                    self.target_actor.get_location()
                )
                if dist <= self._st_trigger_dist:
                    self.planner = WaypointPlanner(
                        self.actor, self._cmap, maneuver=self._st_maneuver
                    )
                    self._st_done = True

        elif self.btype == "lane_change":
            if not self._lc_done:
                triggered = False
                if self._lc_trigger_sec is not None:
                    triggered = (time.time() - self._t0) >= float(self._lc_trigger_sec)
                if not triggered and self._lc_trigger_dist is not None and self.target_actor is not None:
                    triggered = (
                        self.actor.get_location().distance(self.target_actor.get_location())
                        <= float(self._lc_trigger_dist)
                    )
                if triggered:
                    self._exec_lane_change()

        self.actor.apply_control(
            carla.VehicleControl(throttle=throttle, steer=steer)
        )


class WalkerBehaviorController:
    """Direct-control walker — applies WalkerControl each tick (no AI controller).

    This mirrors Scenic's CrossingBehavior / SetWalkingDirectionAction approach:
    compute a fixed direction vector once, then push it via apply_control() every
    tick.  CARLA's AI walker (go_to_location) relies on navmesh pathfinding; off-road
    spawn positions are often not on the navmesh, so pedestrians either freeze or
    take detour paths.  Direct control bypasses navmesh entirely.

    Behavior types
    --------------
    walk_across  – walk perpendicular to the road in *direction*.
                   Keys: direction (right|left|forward|backward),
                         target_speed (m/s, default 1.4),
                         trigger_dist (optional float).
    walk_to      – walk straight toward explicit world coordinates.
                   Keys: destination {x,y,z}, target_speed, trigger_dist (optional).
    stationary   – stand still (zero-speed control applied each tick).

    trigger_dist : float (optional)
        Walker holds position until ego is within this distance, then starts.
        Requires target_ego to be passed in.
    """

    def __init__(self, actor, cfg: dict, world, target_ego=None):
        self.actor          = actor
        self.btype          = cfg.get("type", "walk_across")
        self._cfg           = cfg
        self._target_ego    = target_ego
        self._trigger_dist  = cfg.get("trigger_dist")
        self._triggered     = self._trigger_dist is None  # True = start immediately
        self._world         = world
        self._speed         = float(cfg.get("target_speed", 1.4))
        self._walk_seconds  = cfg.get("walk_seconds")   # optional: stop after N seconds
        self._trigger_time  = None                       # set when walking starts
        self._walk_dir      = None                       # carla.Vector3D, set when triggered

        if self._triggered and self.btype != "stationary":
            self._walk_dir = self._compute_walk_dir()

    def _compute_walk_dir(self) -> carla.Vector3D:
        """
        Return a normalised horizontal direction vector for the walker.

        For walk_to  : points from current position toward the destination.
        For walk_across: perpendicular to the road in the requested direction.
                         Uses ego's road waypoint so the axis is always correct
                         even when the walker is on a sidewalk far from the lane.
        """
        if self.btype == "walk_to" and "destination" in self._cfg:
            dst = self._cfg["destination"]
            loc = self.actor.get_location()
            dx  = dst["x"] - loc.x
            dy  = dst["y"] - loc.y
            d   = math.sqrt(dx ** 2 + dy ** 2)
            if d < 0.01:
                return carla.Vector3D(x=1.0, y=0.0, z=0.0)
            return carla.Vector3D(x=dx / d, y=dy / d, z=0.0)

        direction = self._cfg.get("direction", "right")
        angle_map = {"right": 90, "left": -90, "forward": 0, "backward": 180}

        # Derive road axis from ego's VELOCITY when available.
        # get_waypoint(ego_location) often snaps to the opposite-direction lane
        # on two-way roads (the opposing lane can be geometrically closer),
        # returning a yaw 180° off — which flips the crossing direction entirely.
        # A moving vehicle's velocity vector is always correct; it cannot be 180°
        # off by accident.  This mirrors Scenic: CrossingBehavior uses the ego's
        # known heading, not a waypoint lookup.
        base_yaw = self.actor.get_transform().rotation.yaw  # last-resort fallback
        if self._target_ego is not None:
            try:
                vel = self._target_ego.get_velocity()
                spd = math.sqrt(vel.x ** 2 + vel.y ** 2)
                if spd > 0.5:
                    # Moving ego: velocity direction is the most accurate road axis
                    base_yaw = math.degrees(math.atan2(vel.y, vel.x))
                else:
                    # Stationary/just-spawned ego: fall back to transform yaw
                    base_yaw = self._target_ego.get_transform().rotation.yaw
            except Exception:
                base_yaw = self._target_ego.get_transform().rotation.yaw

        rad = math.radians(base_yaw + angle_map.get(direction, 90))
        return carla.Vector3D(x=math.cos(rad), y=math.sin(rad), z=0.0)

    def tick(self) -> None:
        if self.btype == "stationary":
            self.actor.apply_control(carla.WalkerControl(speed=0.0))
            return

        if not self._triggered:
            # Hold position until ego enters trigger radius
            self.actor.apply_control(carla.WalkerControl(speed=0.0))
            if self._target_ego is None:
                return
            dist = self.actor.get_location().distance(self._target_ego.get_location())
            if dist <= float(self._trigger_dist):
                self._triggered    = True
                self._trigger_time = time.time()
                self._walk_dir     = self._compute_walk_dir()

        if self._walk_dir is not None:
            # Optional: stop after walk_seconds (Scenic's CrossAndFreeze pattern)
            if (self._walk_seconds is not None
                    and self._trigger_time is not None
                    and time.time() - self._trigger_time >= float(self._walk_seconds)):
                self.actor.apply_control(carla.WalkerControl(speed=0.0))
                return

            self.actor.apply_control(
                carla.WalkerControl(direction=self._walk_dir, speed=self._speed)
            )

    def destroy(self) -> None:
        # No AI controller actor was spawned — nothing to destroy.
        pass
