"""
validator.py — Lightweight structural validation for generated CARLA
scenario configs.

Design choice: we validate *structure and enums* only — not every numeric
range or cross-field invariant. Over-strict validation causes the LLM to
burn retries on perfectly runnable configs that merely differ in style,
and the real authority on "will this run" is your carla_runner.py anyway.

The valid values below are taken directly from the `_modes` / `_types`
lists in your boilerplate.json.
"""
from __future__ import annotations

import json
from typing import Any


REQUIRED_TOP_LEVEL = {"map", "scenario", "ego", "adversaries"}

VALID_MAPS = {
    "Town01", "Town02", "Town03", "Town04", "Town05",
    "Town06", "Town07", "Town10HD", "Town11", "Town12",
}

VALID_SPAWN_MODES = {
    "random", "random_lane", "index", "coordinates", "near_intersection",
    "nearby_random", "relative_to_ego", "lane_ahead", "junction_cross",
    "relative_to_actor",
}

VALID_EGO_BEHAVIORS = {
    "follow_lane", "ttc_brake", "constant_speed", "traffic_manager",
    "autopilot", "stopped", "parked",
}

VALID_VEHICLE_BEHAVIORS = {
    "traffic_manager", "intersection_conflict", "follow_lane",
    "constant_speed", "ttc_brake", "cut_in", "sudden_brake",
    "stopped", "parked", "autopilot", "lane_change",
}

VALID_WALKER_BEHAVIORS = {"walk_across", "walk_to", "stationary"}
VALID_PROP_BEHAVIORS = {"static"}

VALID_ADV_TYPES = {"vehicle", "walker", "prop"}

VALID_WEATHER = {
    "ClearNoon", "CloudyNoon", "WetNoon", "WetCloudyNoon",
    "SoftRainNoon", "MidRainyNoon", "HardRainNoon",
    "ClearSunset", "CloudySunset", "WetSunset", "WetCloudySunset",
    "SoftRainSunset", "MidRainSunset", "HardRainSunset",
    "ClearNight", "CloudyNight", "WetNight", "WetCloudyNight",
    "SoftRainNight", "MidRainyNight", "HardRainNight",
    "FoggyNoon", "FoggySunset", "FoggyNight",
}

VALID_TRAFFIC_LIGHTS = {"normal", "force_green", "default"}
VALID_CAMERAS = {"behind", "top", "lidar"}


def _req(obj: dict, key: str, kind: type, path: str, errs: list[str]) -> bool:
    if key not in obj:
        errs.append(f"{path}.{key} is required")
        return False
    if not isinstance(obj[key], kind):
        errs.append(f"{path}.{key} must be {kind.__name__}, got {type(obj[key]).__name__}")
        return False
    return True


def _enum(val: Any, allowed: set, path: str, errs: list[str]) -> None:
    if val not in allowed:
        # Keep the displayed set short if it's big.
        shown = sorted(allowed)
        errs.append(f"{path}={val!r} not in {shown}")


def _validate_spawn(spawn: dict, path: str, errs: list[str]) -> None:
    if _req(spawn, "mode", str, path, errs):
        _enum(spawn["mode"], VALID_SPAWN_MODES, f"{path}.mode", errs)


def _validate_adversary(adv: dict, i: int, errs: list[str]) -> None:
    path = f"adversaries[{i}]"
    if not isinstance(adv, dict):
        errs.append(f"{path} is not an object")
        return

    # Type
    if not _req(adv, "type", str, path, errs):
        return
    _enum(adv["type"], VALID_ADV_TYPES, f"{path}.type", errs)
    adv_type = adv["type"]

    _req(adv, "blueprint", str, path, errs)

    # Spawn
    if _req(adv, "spawn", dict, path, errs):
        _validate_spawn(adv["spawn"], f"{path}.spawn", errs)

    # Behavior
    if _req(adv, "behavior", dict, path, errs):
        beh = adv["behavior"]
        if _req(beh, "type", str, f"{path}.behavior", errs):
            valid_behaviors = {
                "vehicle": VALID_VEHICLE_BEHAVIORS,
                "walker":  VALID_WALKER_BEHAVIORS,
                "prop":    VALID_PROP_BEHAVIORS,
            }.get(adv_type)
            if valid_behaviors is not None:
                _enum(beh["type"], valid_behaviors,
                      f"{path}.behavior.type (for type={adv_type!r})", errs)


def validate_config(cfg: Any) -> tuple[bool, list[str]]:
    """
    Return (ok, error_messages). When ok is True, the list is empty.
    """
    errs: list[str] = []

    if not isinstance(cfg, dict):
        return False, ["top-level value must be an object"]

    missing = REQUIRED_TOP_LEVEL - set(cfg)
    if missing:
        errs.append(f"missing top-level keys: {sorted(missing)}")

    # map
    if "map" in cfg:
        if not isinstance(cfg["map"], str):
            errs.append("`map` must be a string")
        else:
            _enum(cfg["map"], VALID_MAPS, "map", errs)

    # scenario
    if "scenario" in cfg:
        sc = cfg["scenario"]
        if not isinstance(sc, dict):
            errs.append("`scenario` must be an object")
        else:
            _req(sc, "type", str, "scenario", errs)

    # loop
    if "loop" in cfg and not isinstance(cfg["loop"], dict):
        errs.append("`loop` must be an object")

    # ego
    if "ego" in cfg:
        ego = cfg["ego"]
        if not isinstance(ego, dict):
            errs.append("`ego` must be an object")
        else:
            _req(ego, "blueprint", str, "ego", errs)
            if _req(ego, "spawn", dict, "ego", errs):
                _validate_spawn(ego["spawn"], "ego.spawn", errs)
            if _req(ego, "behavior", dict, "ego", errs):
                beh = ego["behavior"]
                if _req(beh, "type", str, "ego.behavior", errs):
                    _enum(beh["type"], VALID_EGO_BEHAVIORS,
                          "ego.behavior.type", errs)

    # adversaries
    if "adversaries" in cfg:
        advs = cfg["adversaries"]
        if not isinstance(advs, list):
            errs.append("`adversaries` must be a list")
        else:
            for i, adv in enumerate(advs):
                _validate_adversary(adv, i, errs)

    # environment (optional but common)
    if "environment" in cfg:
        env = cfg["environment"]
        if not isinstance(env, dict):
            errs.append("`environment` must be an object")
        else:
            if "weather" in env and env["weather"] not in VALID_WEATHER:
                errs.append(f"environment.weather={env['weather']!r} not a known preset")
            if "traffic_lights" in env and env["traffic_lights"] not in VALID_TRAFFIC_LIGHTS:
                errs.append(f"environment.traffic_lights={env['traffic_lights']!r} "
                            f"not in {sorted(VALID_TRAFFIC_LIGHTS)}")

    # display (optional)
    if "display" in cfg and isinstance(cfg["display"], dict):
        cam = cfg["display"].get("camera")
        if cam is not None and cam not in VALID_CAMERAS:
            errs.append(f"display.camera={cam!r} not in {sorted(VALID_CAMERAS)}")

    return len(errs) == 0, errs


def main() -> int:
    import sys
    if len(sys.argv) != 2:
        print("usage: validator.py <config.json>", file=sys.stderr)
        return 2
    with open(sys.argv[1], "r", encoding="utf-8") as f:
        cfg = json.load(f)
    ok, errs = validate_config(cfg)
    if ok:
        print(f"OK — {sys.argv[1]} passes schema validation")
        return 0
    print(f"INVALID — {sys.argv[1]} has {len(errs)} issues:")
    for e in errs:
        print(f"  - {e}")
    return 1


if __name__ == "__main__":
    import sys
    sys.exit(main())
