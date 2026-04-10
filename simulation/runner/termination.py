"""
Episode termination logic and post-condition evaluation.
"""

import time

from .utils import get_speed
from .sensors import CollisionMonitor


class TerminationChecker:
    """
    Decides when the current episode should end based on the termination
    config block from the scenario JSON.

    Supported termination conditions
    ----------------------------------
    end_on_collision       – stop as soon as a collision is detected
    max_distance_from_start – stop once ego travels beyond this distance (m)
    max_duration_seconds   – hard timeout
    end_on_ego_stopped     – stop if ego stays below 0.1 m/s for > 2 s
    min_duration_seconds   – never terminate before this time has elapsed
    """

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
        min_dur = float(self.cfg.get("min_duration_seconds", 0.0))
        if self.elapsed() < min_dur:
            return False

        if self.cfg.get("end_on_collision") and self.cm.collision_occurred:
            print("[Termination] collision")
            return True

        max_dist = self.cfg.get("max_distance_from_start")
        if max_dist and self.ego.get_location().distance(self.start_loc) > max_dist:
            print("[Termination] max_distance")
            return True

        max_dur = self.cfg.get("max_duration_seconds")
        if max_dur and self.elapsed() > max_dur:
            print("[Termination] max_duration")
            return True

        if self.cfg.get("end_on_ego_stopped"):
            if get_speed(self.ego) < 0.1:
                self._stopped = self._stopped or time.time()
                if time.time() - self._stopped > 2.0:
                    print("[Termination] ego stopped")
                    return True
            else:
                self._stopped = None

        return False


def evaluate_postconditions(cfg: list, ego, cm: CollisionMonitor) -> dict:
    """
    Evaluate each postcondition in *cfg* and print a PASS/FAIL summary.
    Returns a dict mapping condition label → bool.
    """
    results = {}
    for c in cfg:
        ctype = c["type"]
        if ctype == "no_collision":
            ok  = not cm.collision_occurred
            lbl = "no_collision"
        elif ctype == "ego_speed_below":
            ok  = get_speed(ego) * 3.6 < float(c.get("value", 5.0))
            lbl = f"ego_speed_below_{c.get('value')}kmh"
        elif ctype == "ego_speed_above":
            ok  = get_speed(ego) * 3.6 > float(c.get("value", 0.0))
            lbl = f"ego_speed_above_{c.get('value')}kmh"
        else:
            print(f"  [Post] Unknown: {ctype}")
            continue

        results[lbl] = ok
        print(f"  [Post] {lbl}: {'PASS ✓' if ok else 'FAIL ✗'}")

    overall = all(results.values()) if results else True
    print(f"  Overall: {'PASS ✓' if overall else 'FAIL ✗'}")
    return results
