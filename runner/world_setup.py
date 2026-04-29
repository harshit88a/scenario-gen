"""
World environment setup: weather conditions and traffic-light states.
"""

import carla


WEATHER_PRESETS = {
    # Noon
    "ClearNoon":       carla.WeatherParameters.ClearNoon,
    "CloudyNoon":      carla.WeatherParameters.CloudyNoon,
    "WetNoon":         carla.WeatherParameters.WetNoon,
    "WetCloudyNoon":   carla.WeatherParameters.WetCloudyNoon,
    "SoftRainNoon":    carla.WeatherParameters.SoftRainNoon,
    "MidRainyNoon":    carla.WeatherParameters.MidRainyNoon,
    "HardRainNoon":    carla.WeatherParameters.HardRainNoon,
    # Sunset
    "ClearSunset":     carla.WeatherParameters.ClearSunset,
    "CloudySunset":    carla.WeatherParameters.CloudySunset,
    "WetSunset":       carla.WeatherParameters.WetSunset,
    "WetCloudySunset": carla.WeatherParameters.WetCloudySunset,
    "SoftRainSunset":  carla.WeatherParameters.SoftRainSunset,
    "MidRainSunset":   carla.WeatherParameters.MidRainSunset,
    "HardRainSunset":  carla.WeatherParameters.HardRainSunset,
    # Night
    "ClearNight":      carla.WeatherParameters.ClearNight,
    "CloudyNight":     carla.WeatherParameters.CloudyNight,
    "WetNight":        carla.WeatherParameters.WetNight,
    "WetCloudyNight":  carla.WeatherParameters.WetCloudyNight,
    "SoftRainNight":   carla.WeatherParameters.SoftRainNight,
    "MidRainyNight":   carla.WeatherParameters.MidRainyNight,
    "HardRainNight":   carla.WeatherParameters.HardRainNight,
    # Foggy — no named CARLA preset; approximated with custom parameters
    "FoggyNoon": carla.WeatherParameters(
        cloudiness=10.0, sun_altitude_angle=70.0,
        fog_density=60.0, fog_distance=0.0,
    ),
    "FoggySunset": carla.WeatherParameters(
        cloudiness=20.0, sun_altitude_angle=15.0,
        fog_density=60.0, fog_distance=0.0,
    ),
    "FoggyNight": carla.WeatherParameters(
        cloudiness=50.0, sun_altitude_angle=-90.0,
        fog_density=60.0, fog_distance=0.0,
    ),
}


def set_weather(world, weather_cfg) -> None:
    """
    Apply weather from a preset name (str) or a parameter dict.
    Falls back to ClearNoon for unknown preset names.
    """
    if isinstance(weather_cfg, str):
        world.set_weather(
            WEATHER_PRESETS.get(weather_cfg, carla.WeatherParameters.ClearNoon)
        )
    elif isinstance(weather_cfg, dict):
        world.set_weather(carla.WeatherParameters(
            cloudiness=weather_cfg.get("cloudiness", 0.0),
            precipitation=weather_cfg.get("precipitation", 0.0),
            precipitation_deposits=weather_cfg.get("precipitation_deposits", 0.0),
            wind_intensity=weather_cfg.get("wind_intensity", 0.0),
            sun_azimuth_angle=weather_cfg.get("sun_azimuth_angle", 0.0),
            sun_altitude_angle=weather_cfg.get("sun_altitude_angle", 70.0),
            fog_density=weather_cfg.get("fog_density", 0.0),
            fog_distance=weather_cfg.get("fog_distance", 0.0),
            wetness=weather_cfg.get("wetness", 0.0),
        ))


def configure_traffic_lights(world, mode: str) -> None:
    """
    Set traffic-light state for the entire map.
    Modes: "normal"/"default" (do nothing), "force_green", "force_red".
    """
    if mode in ("normal", "default"):
        return
    state = (
        carla.TrafficLightState.Green
        if mode == "force_green"
        else carla.TrafficLightState.Red
    )
    for actor in world.get_actors():
        if "traffic_light" in actor.type_id:
            actor.set_state(state)
            actor.freeze(True)
