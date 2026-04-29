# CARLA Adversarial Scenario Generator

A fine-tuned Qwen2.5-Coder-7B model that converts natural-language descriptions into validated CARLA scenario JSON configurations. Describe a driving scenario in plain English and the pipeline produces a ready-to-run config file, starts CARLA, and launches the simulation — all from a single command.

---

## Table of Contents

1. [Hardware & Software Requirements](#requirements)
2. [Installing CARLA 0.9.15](#installing-carla-0915)
3. [Python Dependencies](#python-dependencies)
4. [One-Time Setup](#one-time-setup)
5. [Project Layout](#project-layout)
6. [Quick Start — Single Pipeline Command](#quick-start--single-pipeline-command)
7. [Manual Workflow — Three Terminals](#manual-workflow--three-terminals)
8. [Running a Saved Config Directly](#running-a-saved-config-directly)
9. [pipeline.py Reference](#pipelinepy-reference)
10. [generate_config.py Reference](#generate_configpy-reference)
11. [carla_runner.py Reference](#carla_runnerpy-reference)
12. [How Validation Works](#how-validation-works)
13. [Simulator Controls](#simulator-controls)
14. [Supported Config Values](#supported-config-values)
15. [JSON Config Structure](#json-config-structure)
16. [Troubleshooting](#troubleshooting)

---

## Requirements

### Hardware

- GPU with at least 8 GB VRAM (tested on RTX 3070 8 GB)
- CUDA 11.8 or 12.1

### Software

- Ubuntu 20.04 or 22.04
- Python 3.10 or newer
- CARLA 0.9.15

---

## Installing CARLA 0.9.15

Download the prebuilt package from the CARLA releases page and extract it:

```bash
wget https://carla-releases.s3.us-east-005.backblazeb2.com/Linux/CARLA_0.9.15.tar.gz
tar -xzf CARLA_0.9.15.tar.gz -C CARLA_0.9.15/
```

Install the CARLA Python client library into your environment:

```bash
pip install CARLA_0.9.15/PythonAPI/carla/dist/carla-0.9.15-cp3*-linux-x86_64.whl
```

Verify the server starts without errors:

```bash
cd CARLA_0.9.15
./CarlaUE4.sh -prefernvidia
```

You should see the Unreal Engine window open and stay stable. First boot can take 30–60 seconds. Press `Ctrl-C` to stop it once confirmed working.

---

## Python Dependencies

Install PyTorch first, matching your CUDA version:

```bash
# CUDA 12.1
pip install torch==2.4.0 --index-url https://download.pytorch.org/whl/cu121

# CUDA 11.8
pip install torch==2.4.0 --index-url https://download.pytorch.org/whl/cu118
```

Then install the remaining dependencies:

```bash
pip install -r requirements.txt
pip install pygame numpy
```

| Package | Purpose |
|---|---|
| `transformers>=4.45.0` | Model loading and tokenizer |
| `peft>=0.12.0` | LoRA adapter support |
| `bitsandbytes>=0.46.1` | 4-bit NF4 quantization |
| `accelerate>=0.33.0` | Required by `device_map="auto"` |
| `pygame` | Simulation display window and HUD |
| `numpy` | Camera/LiDAR frame processing |

---

## One-Time Setup

Download and unzip the LoRA adapter into the project directory:

```bash
unzip qwen7b_json_lora_final.zip
```

After unzipping, `./qwen7b_json_lora_final/adapter_model.safetensors` must exist.

The first run also downloads the Qwen2.5-Coder-7B base model weights (~15 GB) into `./hf_cache/`. Subsequent runs load from cache and start in about 30 seconds.

---

## Project Layout

```
scenario-gen/
├── pipeline.py                 # End-to-end runner: LLM → CARLA → simulation
├── generate_config.py          # LLM inference — reads a prompt, writes a JSON
├── validator.py                # Schema validation for generated configs
├── carla_runner.py             # CARLA client — runs a config against the simulator
├── requirements.txt            # Python dependencies
├── runner/                     # Simulation internals (OOP package)
│   ├── episode.py              # Episode loop and actor lifecycle
│   ├── behavior.py             # Vehicle and walker behavior controllers
│   ├── spawn.py                # Spawn mode implementations
│   ├── sensors.py              # Sensor attachment and collision monitor
│   ├── termination.py          # Episode termination conditions
│   ├── visualization.py        # Pygame display, HUD, BEV LiDAR view
│   ├── world_setup.py          # Weather and traffic-light configuration
│   ├── connection.py           # CARLA client/world connection helpers
│   ├── config.py               # JSON config loader
│   └── utils.py                # Math helpers (TTC, steering, throttle)
├── qwen7b_json_lora_final/     # LoRA adapter (unzip here before first run)
├── hf_cache/                   # HuggingFace model cache (auto-created)
├── generated_config/           # Output directory for generated JSON configs
└── config-examples/            # Hand-crafted reference configs
```

---

## Quick Start — Single Pipeline Command

The recommended way to run everything from one terminal:

```bash
python3 pipeline.py
```

You will be prompted to describe the scenario interactively. Or pass the description directly:

```bash
python3 pipeline.py --prompt "A vehicle in the adjacent lane cuts sharply into ego's lane at 50 km/h, forcing an emergency brake."
```

**What happens automatically:**

1. The LLM generates and validates a JSON config (retrying up to 3 times if needed)
2. The LLM process exits, fully releasing all VRAM
3. A 2-second settle pause ensures the GPU driver has flushed LLM allocations
4. `CARLA_0.9.15/CarlaUE4.sh -prefernvidia` is launched as a background process
5. The pipeline polls `localhost:2000` every 2 seconds until CARLA accepts connections (up to 120 s)
6. `carla_runner.py` is launched with the generated config
7. When the runner exits (or you press `Ctrl-C`), the CARLA server is gracefully shut down

### Key pipeline flags

| Flag | Default | Purpose |
|------|---------|---------|
| `--prompt "..."` | stdin | Scenario description (skip interactive input) |
| `--no-run` | off | Generate JSON only; skip CARLA entirely |
| `--no-carla-start` | off | Skip steps 3–5; assume CARLA is already running |
| `--carla-dir PATH` | `./CARLA_0.9.15` | Path to the CARLA installation |
| `--carla-flags FLAGS` | `-prefernvidia` | Extra flags for `CarlaUE4.sh` |
| `--carla-timeout N` | `120` | Seconds to wait for CARLA to become ready |
| `--config-dir DIR` | `generated_config` | Where to write the generated JSON |
| `--max-retries N` | `3` | LLM retry attempts on validation failure |
| `--host HOST` | `localhost` | CARLA server host |
| `--port PORT` | `2000` | CARLA server port |

### Example: headless rendering (no display)

```bash
python3 pipeline.py \
  --prompt "Lead car drives ahead then brakes suddenly" \
  --carla-flags "-prefernvidia -RenderOffScreen"
```

### Example: CARLA already running

If you started CARLA separately and want to skip the auto-start:

```bash
python3 pipeline.py --no-carla-start --prompt "Cyclist cuts in from sidewalk"
```

---

## Manual Workflow — Three Terminals

Use this if you want to keep CARLA running across multiple scenario generations (avoids the slow UE4 boot each time).

### Terminal 1 — Start CARLA (keep this running)

```bash
cd CARLA_0.9.15
./CarlaUE4.sh -prefernvidia
```

### Terminal 2 — Generate the JSON

```bash
python3 generate_config.py
```

Type the scenario description (multi-line OK), then press `Ctrl-D`. The JSON is written to `generated_config/config_<timestamp>.json` and the process exits, fully releasing VRAM.

You can also pass the prompt directly:

```bash
python3 generate_config.py --prompt "A pedestrian runs across a crosswalk on a red signal."
```

### Terminal 3 — Run the scenario

```bash
python3 carla_runner.py --config generated_config/config_<timestamp>.json
```

If CARLA crashes mid-episode, re-run this command against the same JSON — no need to re-generate.

---

## Running a Saved Config Directly

Any config from `generated_config/` or `config-examples/` can be run directly without the LLM:

```bash
python3 carla_runner.py --config config-examples/cut_in_junction_01.json
```

Useful flags:

| Flag | Default | Purpose |
|------|---------|---------|
| `--config PATH` | required | Path to scenario JSON |
| `--host HOST` | `localhost` | CARLA server host |
| `--port PORT` | `2000` | CARLA server port |

---

## pipeline.py Reference

```
python3 pipeline.py [--prompt TEXT] [--no-run] [--no-carla-start]
                    [--carla-dir PATH] [--carla-flags FLAGS]
                    [--carla-timeout SECS] [--config-dir DIR]
                    [--adapter-dir DIR] [--cache-dir DIR]
                    [--max-retries N] [--host HOST] [--port PORT]
```

Exits with code `0` on success, non-zero on any failure. Safe to `Ctrl-C` at any point — the CARLA server is always cleaned up in the `finally` block.

---

## generate_config.py Reference

```
python3 generate_config.py [--prompt TEXT] [--out PATH]
                           [--adapter-dir DIR] [--cache-dir DIR]
                           [--max-retries N] [--temperature F]
                           [--max-new-tokens N]
```

| Flag | Default | Purpose |
|------|---------|---------|
| `--prompt TEXT` | stdin | Scenario description |
| `--out PATH` | `generated_config/config_<ts>.json` | Output file path |
| `--adapter-dir DIR` | `./qwen7b_json_lora_final` | LoRA adapter folder |
| `--cache-dir DIR` | `./hf_cache` | HuggingFace cache |
| `--max-retries N` | `3` | Re-sample attempts on failure |
| `--temperature F` | `0.2` | Sampling temperature (bumped +0.15 per retry) |
| `--max-new-tokens N` | `1500` | Token budget |

The script handles validation and retries internally. When it exits with code `0`, the output file is guaranteed to have passed schema validation.

---

## carla_runner.py Reference

```
python3 carla_runner.py --config PATH [--host HOST] [--port PORT]
```

Keyboard controls during simulation:

| Key | Action |
|-----|--------|
| `ESC` | Quit |
| `N` | Skip to next episode |
| `TAB` | Cycle display mode: behind camera → top camera → LiDAR BEV |

The runner loops episodes according to `loop.max_episodes` in the JSON. Each episode spawns all actors fresh and destroys them on completion.

---

## How Validation Works

`generate_config.py` calls `validator.py` automatically after each generation attempt. If validation fails, it re-samples with a slightly higher temperature (up to `--max-retries` times). By the time a file is written to disk, it has passed:

- Required top-level keys (`map`, `scenario`, `ego`, `adversaries`)
- Valid CARLA map names
- Valid spawn modes, behavior types, and adversary types
- Behavior/type consistency (e.g. `walk_across` only valid on walkers)
- Correct JSON types (`adversaries` must be a list, not an object)

Validate any config by hand:

```bash
python3 validator.py generated_config/config_20260101_120000.json
```

---

## Simulator Controls

| Key | Effect |
|-----|--------|
| `ESC` | Quit simulation |
| `N` | Skip current episode and start the next one |
| `TAB` | Cycle view: `behind` → `top` → `lidar` (BEV point cloud) |

---

## Supported Config Values

**Maps:** `Town01`, `Town02`, `Town03`, `Town04`, `Town05`, `Town06`, `Town07`, `Town10HD`, `Town11`, `Town12`

**Adversary types:** `vehicle`, `walker`, `prop`

**Spawn modes:** `random`, `random_lane`, `index`, `coordinates`, `near_intersection`, `nearby_random`, `relative_to_ego`, `relative_to_actor`, `lane_ahead`, `junction_cross`, `oncoming_junction`, `exit_lane_ahead`

**Ego behaviors:** `follow_lane`, `constant_speed`, `ttc_brake`, `traffic_manager`, `autopilot`, `stopped`, `parked`

**Vehicle behaviors:** `follow_lane`, `lane_change`, `cut_in`, `sudden_brake`, `sudden_accelerate`, `sudden_turn`, `constant_speed`, `ttc_brake`, `intersection_conflict`, `traffic_manager`, `autopilot`, `stopped`, `parked`

**Walker behaviors:** `walk_across`, `walk_to`, `stationary`

**Weather presets:** `ClearNoon`, `CloudyNoon`, `WetNoon`, `WetCloudyNoon`, `SoftRainNoon`, `MidRainyNoon`, `HardRainNoon`, `ClearSunset`, `CloudySunset`, `WetSunset`, `WetCloudySunset`, `SoftRainSunset`, `MidRainSunset`, `HardRainSunset`, `ClearNight`, `CloudyNight`, `WetNight`, `WetCloudyNight`, `SoftRainNight`, `MidRainyNight`, `HardRainNight`, `FoggyNoon`, `FoggySunset`, `FoggyNight`

**Traffic lights:** `normal`, `force_green`, `force_red`

**Cameras:** `behind`, `top`, `lidar`

---

## JSON Config Structure

A complete annotated example:

```jsonc
{
  "map": "Town05",                     // CARLA map to load
  "scenario": {
    "type": "cut_in_junction",         // human label, shown in window title
    "description": "Adversary cuts in from adjacent lane before a junction."
  },
  "loop": {
    "max_episodes": 10,                // number of times to repeat
    "episode_duration_seconds": 20,    // hard timeout per episode
    "intermission_seconds": 1          // pause between episodes
  },
  "ego": {
    "blueprint": "vehicle.lincoln.mkz_2017",
    "spawn": { "mode": "near_intersection", "search_distance": 40 },
    "behavior": {
      "type": "follow_lane",
      "target_speed": 10.0,
      "maneuver": "straight"
      // optional: "target_id": "adv1"  — resolve TTC relative to named adversary
    },
    "sensors": []                       // optional extra sensors
  },
  "adversaries": [
    {
      "id": "adv1",                     // used for relative_to_actor / target_id refs
      "type": "vehicle",                // vehicle | walker | prop
      "blueprint": "vehicle.mercedes.coupe",
      "spawn": {
        "mode": "lane_ahead",
        "forward_distance": 18.0,
        "lateral_offset": 3.5
      },
      "behavior": {
        "type": "cut_in",
        "target_speed": 11.0,
        "cut_in_distance": 16.0,
        "maneuver": "straight"
        // optional: "target_id": "ego"  — measure distances relative to ego
      }
    }
  ],
  "environment": {
    "weather": "ClearNoon",
    "traffic_lights": "normal"          // normal | force_green | force_red
  },
  "display": {
    "enabled": true,
    "camera": "behind"                  // behind | top | lidar
  },
  "termination": {
    "max_duration_seconds": 20,
    "max_distance_from_start": 70,
    "end_on_collision": false,
    "end_on_ego_stopped": false,
    "min_duration_seconds": 2
  },
  "postconditions": [
    { "type": "no_collision" },
    { "type": "ego_speed_below", "value": 5 }
  ]
}
```

---

## Troubleshooting

**`[gen] ERROR: adapter directory not found`**
Unzip `qwen7b_json_lora_final.zip` in the project root so that `./qwen7b_json_lora_final/adapter_model.safetensors` exists.

**`[Error] Failed to connect to CARLA server`**
CARLA is not running or not yet ready. Start it with `cd CARLA_0.9.15 && ./CarlaUE4.sh -prefernvidia` and wait for the UE4 window to appear before running the runner. When using `pipeline.py`, increase `--carla-timeout` if your machine is slow to boot.

**`[Collision] ego ← static.fence` (repeated)**
The ego spawned inside or adjacent to a fence prop. This is a map collision during spawn — the episode will run but immediately trigger the collision sensor. Try a different spawn mode (`random_lane` instead of `near_intersection`) or add `"end_on_collision": false` in the termination block to let the episode play out regardless.

**VRAM out-of-memory when running pipeline**
The LLM and CARLA both need significant VRAM. `pipeline.py` automatically waits for the LLM subprocess to exit (fully releasing VRAM) before starting CARLA. If you see OOM errors with CARLA, make sure you are not running the generator and CARLA simultaneously. Use `pipeline.py` rather than two manual terminals to guarantee the handoff.

**`Import "peft" could not be resolved` in the editor**
This is a linter warning only — `peft` is installed at runtime. Run `pip install peft` if you see it at runtime too.

**Scenario runs but adversary is spawned at world origin (0, 0, 0)**
The spawn mode could not find a valid location. Check the terminal output for `[Warning]` lines. Try a less constrained spawn mode (`nearby_random` or `random_lane`) or increase `search_distance`.

**`No junction within N m`**
The current spawn point has no junction nearby within `search_distance`. Increase `search_distance` in the spawn config or switch to `near_intersection` spawn mode for the ego so it always starts near a junction.

