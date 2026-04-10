# CARLA Adversarial Scenario Runner - Refactored Architecture

## Overview

The `carla_runner.py` has been refactored to serve as a clean entry point that delegates all simulation logic to the modular `runner` package. This follows Object-Oriented Programming (OOP) principles for maintainability, extensibility, and testability.

## Architecture

### Entry Point: `carla_runner.py`

A lightweight script that:
- Parses command-line arguments (`--config`, `--host`, `--port`)
- Validates input files
- Delegates to the `runner` package
- Handles exceptions gracefully
- Provides helpful error messages

```bash
python carla_runner.py --config config/scenario_intersection_conflict.json
python carla_runner.py --config config/scenario_cut_in.json --host localhost --port 2000
```

### Core Modules: `runner/` Package

The package is organized by responsibility using OOP principles:

#### 1. **`config.py`** – Configuration Management
- `load_config(path)` – Parse JSON scenario configuration
- Type validation and defaults handling
- Single responsibility: configuration I/O

#### 2. **`connection.py`** – CARLA Connection
- `connect()` – Establish client-world connection
- `restore_async()` – Clean up world state
- Encapsulates all CARLA server communication
- Configures synchronous mode and fixed timesteps

#### 3. **`world_setup.py`** – World Environment
- `set_weather()` – Configure weather conditions
- `configure_traffic_lights()` – Set traffic light behavior
- Pre-configured weather presets (ClearNoon, WetNight, etc.)

#### 4. **`spawn.py`** – Actor Spawning
- `safe_spawn_vehicle()` – Spawn vehicles with retry logic
- `safe_spawn_walker()` – Spawn pedestrians
- `get_transform_for_spawn()` – Calculate spawn positions
- Supports multiple spawn modes: `random`, `junction_cross`, `nearby_random`
- Encapsulates spawn point validation and retry strategies

#### 5. **`behavior.py`** – Actor Control (OOP Design)
Classes:
- **`WaypointPlanner`** – Pre-computes road-following paths
  - Supports maneuvers: "straight", "left_turn", "right_turn", "u_turn"
  - Configurable lookahead distance and step size
  
- **`VehicleBehaviorController`** – Abstract vehicle controller
  - Subclasses for different behaviors:
    - `TrafficManagerController` – CARLA Traffic Manager
    - `IntersectionConflictController` – T-bone collision scenarios
    - `TTCBrakeController` – Brake on Time-to-Collision
    - `ConstantSpeedController` – Fixed-speed waypoint following
    - `CutInController` – Lane-change maneuvers
    - `SuddenBrakeController` – Brake after delay
  
- **`WalkerBehaviorController`** – Pedestrian controller
  - Navigation and target reaching

#### 6. **`sensors.py`** – Sensor Management (OOP Design)
Classes:
- **`CollisionMonitor`** – Tracks collision events
- **`attach_sensors()`** – Attaches RGB camera, LiDAR
- Each sensor has its own listener queue

#### 7. **`termination.py`** – Scenario Termination
- **`TerminationChecker`** class – Evaluates termination conditions
  - Time-based: max duration, min duration
  - Event-based: collision, off-road, reached target
  - Custom: postconditions
- `evaluate_postconditions()` – Post-simulation analysis

#### 8. **`visualization.py`** – Rendering
- **Camera modes**: "behind", "top", "front" RGB views
- **LiDAR BEV** (Bird's-Eye-View) mode
- `render_bev()` – Point cloud rendering
- `render_hud()` – On-screen telemetry display
- Multi-threaded image queues for thread-safe rendering

#### 9. **`utils.py`** – Utilities
- `get_speed()` – Current vehicle speed
- `compute_ttc()` – Time-to-Collision calculation
- `steer_toward()` – Steering calculation toward target
- `throttle_for_speed()` – Throttle calculation for target speed

#### 10. **`episode.py`** – Simulation Loop (Main Orchestrator)
- `run()` – Top-level entry point (called by `carla_runner.py`)
- `_run_one_episode()` – Single episode execution
  - Spawns actors
  - Runs simulation loop
  - Handles user input (ESC, N, TAB keys)
  - Enforces termination conditions
  - Cleans up resources
- Episode loop management with configurable max_episodes
- Inter-episode intermission screen

## Configuration Schema

Example `config.json`:

```json
{
  "map": "Town05",
  
  "scenario": {
    "type": "intersection_conflict",
    "ego_maneuver": "straight",
    "adversary_maneuver": "left_turn"
  },
  
  "ego": {
    "blueprint": "vehicle.tesla.model3",
    "spawn": {
      "mode": "random"
    },
    "behavior": {
      "type": "traffic_manager"
    }
  },
  
  "adversaries": [
    {
      "blueprint": "vehicle.tesla.model3",
      "spawn": {
        "mode": "junction_cross",
        "direction": "left"
      },
      "behavior": {
        "type": "intersection_conflict"
      }
    }
  ],
  
  "environment": {
    "weather": "ClearNoon",
    "traffic_lights": "force_green"
  },
  
  "termination": {
    "max_duration_seconds": 20,
    "min_duration_seconds": 2,
    "on_collision": "stop",
    "on_off_road": "stop"
  },
  
  "loop": {
    "max_episodes": 10,
    "episode_duration_seconds": 20,
    "intermission_seconds": 2
  }
}
```

## OOP Design Principles Applied

### 1. **Single Responsibility Principle (SRP)**
Each module has a single, well-defined responsibility:
- `config.py` – Configuration loading
- `connection.py` – CARLA connection management
- `spawn.py` – Actor spawning logic
- `behavior.py` – Actor control logic
- `sensors.py` – Sensor management
- `termination.py` – Termination logic
- `visualization.py` – Rendering

### 2. **Open/Closed Principle (OCP)**
- Extensible behavior controllers via inheritance
- New behaviors can be added without modifying existing code
- Weather presets dictionary for easy addition

### 3. **Liskov Substitution Principle (LSP)**
- All behavior controllers inherit from common base
- Can be swapped without breaking episode logic

### 4. **Dependency Injection**
- Components receive dependencies in `__init__`
- Episode passes necessary objects to controllers
- Decouples components

### 5. **Composition Over Inheritance**
- `CollisionMonitor` and sensors are composed into episode
- `WaypointPlanner` is composed into behavior controllers

### 6. **Encapsulation**
- Public methods for external use
- Private methods (prefixed with `_`) for internal logic
- Clear interfaces between modules

## Usage Workflow

```
User runs:
  $ python carla_runner.py --config config.json

↓

carla_runner.py (entry point)
  ├─ Parse CLI args
  └─ Call runner.run()

↓

runner/episode.py (orchestrator)
  ├─ Load config (runner/config.py)
  ├─ Connect to CARLA (runner/connection.py)
  ├─ Setup world (runner/world_setup.py)
  ├─ For each episode:
  │  ├─ Spawn actors (runner/spawn.py)
  │  ├─ Attach sensors (runner/sensors.py)
  │  ├─ Initialize behaviors (runner/behavior.py)
  │  ├─ Simulation loop:
  │  │  ├─ Tick world
  │  │  ├─ Update behaviors (runner/behavior.py)
  │  │  ├─ Check termination (runner/termination.py)
  │  │  └─ Render (runner/visualization.py)
  │  └─ Cleanup
  └─ Restore world async mode

↓

Simulation complete, window closes
```

## Extending the System

### Adding a New Behavior Controller

```python
# In runner/behavior.py
class MyCustomController(VehicleBehaviorController):
    def __init__(self, vehicle, config, carla_map):
        super().__init__(vehicle, config, carla_map)
        # Custom initialization
    
    def tick(self):
        # Implement your custom behavior
        control = carla.VehicleControl(throttle=0.5)
        self.vehicle.apply_control(control)
```

Then reference in config:
```json
{
  "ego": {
    "behavior": {
      "type": "my_custom_controller"
    }
  }
}
```

### Adding a New Scenario Type

Create configuration file and behavior combinations:
```json
{
  "scenario": {
    "type": "my_scenario",
    "description": "My custom scenario"
  }
}
```

## Benefits of This Architecture

✅ **Maintainability** – Clear module boundaries, easy to find and modify code
✅ **Testability** – Each module can be tested independently
✅ **Extensibility** – Add new behaviors, scenarios without touching core code
✅ **Reusability** – Components can be used in other projects
✅ **Readability** – Clean entry point, self-documenting code
✅ **Scalability** – Easy to add new features (walkers, traffic, etc.)

## Files Summary

| File | Lines | Purpose |
|------|-------|---------|
| `carla_runner.py` | 100 | Entry point, CLI parsing |
| `runner/__init__.py` | 5 | Package initialization |
| `runner/config.py` | 50 | Config loading |
| `runner/connection.py` | 25 | CARLA connection |
| `runner/world_setup.py` | 50 | World configuration |
| `runner/spawn.py` | 200 | Spawn logic |
| `runner/behavior.py` | 300+ | Behavior controllers (OOP) |
| `runner/sensors.py` | 150 | Sensor management |
| `runner/termination.py` | 100 | Termination logic |
| `runner/visualization.py` | 200+ | Rendering |
| `runner/utils.py` | 100 | Utility functions |
| `runner/episode.py` | 350+ | Episode orchestration |

**Total: ~1,500 lines of well-organized, OOP-compliant code**
