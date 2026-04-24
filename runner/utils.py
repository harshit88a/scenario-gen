"""
Low-level math/physics helpers shared across modules.
"""

import math
import carla


def get_speed(actor) -> float:
    """Return the planar speed of an actor in m/s."""
    v = actor.get_velocity()
    return math.sqrt(v.x ** 2 + v.y ** 2)


def compute_ttc(a, b) -> float:
    """Estimate time-to-collision between two actors (seconds)."""
    dist = a.get_location().distance(b.get_location())
    return dist / max(get_speed(a) - get_speed(b), 0.01)


def steer_toward(actor, target: carla.Location):
    """
    Compute a steering value [-1, 1] to turn the actor toward *target*.
    Returns (steer, reached) where *reached* is True when the target is
    within 2 m.
    """
    loc = actor.get_location()
    dx, dy = target.x - loc.x, target.y - loc.y
    d = math.sqrt(dx ** 2 + dy ** 2)
    if d < 2.0:
        return 0.0, True
    dx /= d
    dy /= d
    yaw = actor.get_transform().rotation.yaw
    fx = math.cos(math.radians(yaw))
    fy = math.sin(math.radians(yaw))
    return max(-1.0, min(1.0, (fx * dy - fy * dx) * 0.9)), False


def throttle_for_speed(actor, target_speed: float) -> float:
    """Simple P-controller throttle to reach *target_speed* (m/s)."""
    return max(0.0, min(1.0, (target_speed - get_speed(actor)) * 0.4 + 0.3))


def walk_waypoints(wp, distance: float, step: float = 3.0):
    """Walk *distance* metres along the road from *wp* and return the final waypoint."""
    current = wp
    walked = 0.0
    while walked < distance:
        nexts = current.next(step)
        if not nexts:
            break
        current = nexts[0]
        walked += step
    return current


def lateral_offset_location(loc: carla.Location, yaw: float, offset: float) -> carla.Location:
    """Return *loc* shifted *offset* metres perpendicular to *yaw* (positive = left)."""
    perp_rad = math.radians(yaw + 90)
    return carla.Location(
        x=loc.x + offset * math.cos(perp_rad),
        y=loc.y + offset * math.sin(perp_rad),
        z=loc.z,
    )
