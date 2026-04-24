"""
Sensor attachment and collision monitoring.
"""

import carla


def attach_sensors(world, blueprint_library, vehicle, sensors_cfg: list) -> list:
    """
    Attach each sensor described in *sensors_cfg* to *vehicle*.
    Returns a list of spawned sensor actors.
    """
    attached = []
    for s in sensors_cfg:
        bp = blueprint_library.find(s["type"])
        if bp is None:
            print(f"[Warning] Sensor not found: {s['type']}")
            continue
        for k, v in s.get("attributes", {}).items():
            if bp.has_attribute(k):
                bp.set_attribute(k, str(v))
        t  = s.get("transform", {})
        tf = carla.Transform(
            carla.Location(x=t.get("x", 0), y=t.get("y", 0), z=t.get("z", 0)),
            carla.Rotation(
                pitch=t.get("pitch", 0),
                yaw=t.get("yaw", 0),
                roll=t.get("roll", 0),
            ),
        )
        actor = world.spawn_actor(bp, tf, attach_to=vehicle)
        attached.append(actor)
        print(f"  [Sensor] {s.get('id', s['type'])} attached")
    return attached


class CollisionMonitor:
    """Attach a collision sensor to *vehicle* and record any collision events."""

    def __init__(self, world, blueprint_library, vehicle):
        self.collision_occurred = False
        self.events             = []
        bp                      = blueprint_library.find("sensor.other.collision")
        self._sensor            = world.spawn_actor(
            bp, carla.Transform(), attach_to=vehicle
        )
        self._sensor.listen(self._cb)

    def _cb(self, e) -> None:
        self.collision_occurred = True
        self.events.append(e)
        print(f"[Collision] ego ← {e.other_actor.type_id}")

    def destroy(self) -> None:
        self._sensor.stop()
        self._sensor.destroy()
