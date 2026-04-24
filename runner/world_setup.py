"""
World environment setup: weather conditions and traffic-light states.
"""

import carla


WEATHER_PRESETS = {
    "ClearNoon":      carla.WeatherParameters.ClearNoon,
    "ClearSunset":    carla.WeatherParameters.ClearSunset,
    "CloudyNoon":     carla.WeatherParameters.CloudyNoon,
    "CloudySunset":   carla.WeatherParameters.CloudySunset,
    "WetNoon":        carla.WeatherParameters.WetNoon,
    "WetCloudyNoon":  carla.WeatherParameters.WetCloudyNoon,
    "MidRainyNoon":   carla.WeatherParameters.MidRainyNoon,
    "HardRainNoon":   carla.WeatherParameters.HardRainNoon,
    "SoftRainNoon":   carla.WeatherParameters.SoftRainNoon,
    "ClearNight":     carla.WeatherParameters.ClearNight,
    "WetNight":       carla.WeatherParameters.WetNight,
    "HardRainNight":  carla.WeatherParameters.HardRainNight,
    "SoftRainNight":  carla.WeatherParameters.SoftRainNight,
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
    Modes: "normal" (do nothing), "force_green", "force_red".
    """
    if mode == "normal":
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
