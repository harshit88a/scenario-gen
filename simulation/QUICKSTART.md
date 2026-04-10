# Quick Start Guide

## Prerequisites

1. **CARLA Server Running**
   ```bash
   cd ~/CARLA_0.9.15
   ./CarlaUE4.sh
   ```

2. **Python Dependencies**
   ```bash
   pip install carla pygame numpy
   ```

## Running a Scenario

### Basic Usage
```bash
cd ~/Documents/dev/scenario-gen/simulation

# Run with default server (localhost:2000)
python carla_runner.py --config config/scenario_intersection_conflict.json

# Run with custom server
python carla_runner.py --config config/scenario_cut_in.json --host 192.168.1.100 --port 2000
```

### Available Commands
```bash
# Show help
python carla_runner.py --help

# List configurations (if you have multiple scenarios)
ls config/
```

## In-Simulation Controls

| Key | Action |
|-----|--------|
| **ESC** | Quit simulation |
| **N** | Skip to next episode |
| **TAB** | Cycle camera view (behind/top/lidar) |
| **L** | Toggle LiDAR view |
| **C** | Toggle RGB camera view |

## Configuration Files

### Create New Scenario

Create `config/my_scenario.json`:
```json
{
  "map": "Town05",
  
  "scenario": {
    "type": "my_scenario",
    "description": "Custom scenario"
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
    "on_collision": "stop"
  },
  
  "loop": {
    "max_episodes": 5,
    "episode_duration_seconds": 20,
    "intermission_seconds": 2
  }
}
```

Run it:
```bash
python carla_runner.py --config config/my_scenario.json
```

## Understanding the Output

### Console Output
```
============================================================
Scenario  : intersection_conflict
Map       : Town05
Max eps   : 10
Ep dur    : 20 s
ESC=quit  N=skip episode  TAB=cycle view (behind/top/lidar)
============================================================

[Setup] 302 spawn points on Town05

── Episode 1 ──────────────────────────────────────────────
  [Spawn] Ego at Location(...)
  [Spawn] Adversary at Location(...)
  [Sensors] Attached RGB camera, LiDAR
  [Behavior] Ego: traffic_manager | Adversary: intersection_conflict
  [Running] min_dur=2s, max_dur=20s
  [Terminated] Collision detected at T=8.5s
  [Cleanup] Destroyed 2 vehicles, 2 sensors

...
```

### HUD Display (On-Screen)
```
Episode: 1/10
Ego: 12.3 m/s | Adversary: 14.1 m/s
TTC: 2.5s | Distance: 15.4m
Collisions: 1 | Damage: 45%
Camera: Behind (TAB to cycle)
Time: 8.5s / 20.0s
```

## Troubleshooting

### Error: "Failed to connect to CARLA server"
```
→ Make sure CARLA is running: ./CarlaUE4.sh
→ Check if server is on localhost:2000 (or specify --host/--port)
```

### Error: "Configuration file not found"
```
→ Verify config path: ls config/scenario_name.json
→ Use absolute path if needed
```

### Pygame display issues (GL context error)
```
→ Usually fixed in latest version
→ Try running with: SDL_VIDEODRIVER=windowed python carla_runner.py ...
```

### No adversary vehicle visible
```
→ Check config spawn mode and direction
→ Verify adversary blueprint exists in CARLA
→ Try "random" spawn mode instead of "junction_cross"
```

## Performance Tips

1. **Reduce episodes**: Set `max_episodes: 1` in config for testing
2. **Skip intermission**: Set `intermission_seconds: 0`
3. **Disable LiDAR**: Use RGB camera only for faster rendering
4. **Simple map**: Use smaller maps like Town01 or Town02

## Output Files

Scenarios generate data in the `/simulation` directory:
- **config/** – Scenario configurations
- **screenshots/** – Frame captures from simulations
- **report/** – Summary statistics

## Example Scenarios Included

✅ **intersection_conflict.json** – T-bone collision at junction
✅ **cut_in.json** – Vehicle cutting into ego lane
✅ **sudden_brake.json** – Leading vehicle brakes suddenly

## Next Steps

1. Read [ARCHITECTURE.md](./ARCHITECTURE.md) for detailed module breakdown
2. Explore `runner/` package to understand behavior controllers
3. Create custom scenarios by modifying existing configs
4. Extend with new behaviors by inheriting from controller classes
