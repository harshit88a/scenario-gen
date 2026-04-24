"""
CARLA client / world connection helpers.
"""

import carla


def connect(map_name: str, host: str = "localhost", port: int = 2000):
    """
    Connect to a CARLA server, load *map_name*, and enable synchronous mode
    at 30 Hz.  Returns (client, world).
    """
    client = carla.Client(host, port)
    client.set_timeout(20.0)
    world = client.load_world(map_name)
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 1.0 / 30.0
    world.apply_settings(settings)
    return client, world


def restore_async(world) -> None:
    """Switch the world back to asynchronous mode (best-effort)."""
    try:
        settings = world.get_settings()
        settings.synchronous_mode = False
        world.apply_settings(settings)
    except Exception:
        pass
