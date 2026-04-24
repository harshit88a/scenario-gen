# CARLA Adversarial Scenario Generator

A fine-tuned Qwen2.5-Coder-7B model that converts natural-language descriptions into validated CARLA scenario JSON configurations. Describe a driving scenario in plain English, and the pipeline produces a ready-to-run config file for your CARLA simulation.

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
# Download (adjust the URL to the correct release asset)
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

You should see the Unreal Engine window open and stay stable. This first boot can take 30–60 seconds.

---

## Python Dependencies

Install PyTorch first, matching your CUDA version. Only one of these lines is needed:

```bash
# CUDA 12.1
pip install torch==2.4.0 --index-url https://download.pytorch.org/whl/cu121

# CUDA 11.8
pip install torch==2.4.0 --index-url https://download.pytorch.org/whl/cu118
```

Then install the remaining dependencies:

```bash
pip install -r requirements.txt
```

The `requirements.txt` installs:

| Package | Purpose |
|---|---|
| `transformers>=4.45.0` | Model loading and tokenizer |
| `peft>=0.12.0` | LoRA adapter support |
| `bitsandbytes>=0.46.1` | 4-bit NF4 quantization |
| `accelerate>=0.33.0` | Required by `device_map="auto"` in transformers |

---

## Project Layout

```
scenario-gen/
├── generate_config.py          # LLM inference — reads a prompt, writes a JSON
├── validator.py                # Schema validation for generated configs
├── carla_runner.py             # CARLA client — runs a config against the simulator
├── requirements.txt            # Python dependencies (see above)
├── qwen7b_json_lora_final/     # LoRA adapter (unzip here before first run)
├── hf_cache/                   # HuggingFace model cache (auto-created on first run)
├── generated_config/           # Output directory for generated JSON configs
└── config-examples/            # Reference configs used for training
```

---

## One-Time Setup

Download and unzip the LoRA adapter into the project directory:

```bash
unzip qwen7b_json_lora_final.zip
```

After unzipping, `./qwen7b_json_lora_final/adapter_model.safetensors` must exist.

The first run will also download the Qwen2.5-Coder-7B base model weights (~15 GB) into `./hf_cache/`. Subsequent runs load from cache and start in about 30 seconds.

---

## Workflow

The pipeline has three independent pieces with different lifetimes. Run them in separate terminals.

### Terminal 1 — Generate the JSON

```bash
python3 generate_config.py
```

The script prompts you to describe the scenario. Type the description (multi-line is fine), then press `Ctrl-D` to submit. The JSON is written to `generated_config/config_<timestamp>.json` and the process exits immediately, fully releasing VRAM.

Example input:

```
A vehicle in the left adjacent lane abruptly cuts into the ego's lane at
highway speed with minimal gap, forcing an emergency brake from the ego.
```

Optional flags:

```
--adapter-dir   Path to the LoRA adapter folder   (default: ./qwen7b_json_lora_final)
--cache-dir     HuggingFace cache directory        (default: ./hf_cache)
--max-retries   Re-sample attempts on failure      (default: 3)
--temperature   Sampling temperature               (default: 0.2)
--max-new-tokens  Token budget for generation      (default: 1500)
```

### Terminal 2 — Start CARLA

You can start CARLA in parallel with Terminal 1. The UE4 boot takes time and you do not want it on the critical path.

```bash
cd CARLA_0.9.15
./CarlaUE4.sh -prefernvidia
```

### Terminal 3 — Run the Scenario

```bash
python3 carla_runner.py --config generated_config/config_20260101_120000.json
```

If CARLA crashes mid-episode, re-run this command against the same JSON without regenerating.

---

## How Validation Works

`generate_config.py` calls `validator.py` automatically after each generation attempt. If validation fails, the generator re-samples with a slightly higher temperature (up to `--max-retries` times). By the time a file is written to disk, it has already passed:

- Required top-level keys (`map`, `scenario`, `ego`, `adversaries`)
- Valid CARLA map names
- Valid spawn modes, behavior types, and adversary types
- Behavior/type consistency (`walk_across` is only valid on a walker, never a vehicle)
- Correct JSON types (`adversaries` must be a list, not an object)

The validator does not check numeric ranges or cross-field semantics. The real authority on whether a config will run is `carla_runner.py`.

You can also validate any config file by hand:

```bash
python3 validator.py generated_config/config_20260101_120000.json
```

---

## Why Three Terminals Instead of One Pipeline Script

The three components have very different lifetimes:

- **Generator** is one-shot. Load model, generate, exit. Roughly 30–60 seconds total.
- **Simulator** is long-lived. UE4 boot is slow; you want to keep it running across many scenarios.
- **Runner** is per-scenario. When CARLA OOM-crashes (it happens on 8 GB cards), you want to re-run the runner against the same JSON, not regenerate everything from scratch.

Coupling them means a CARLA crash kills an LLM session you no longer need to redo, and you serialize the slow UE4 boot behind the LLM that could have run in parallel. Decoupling also means `del model; torch.cuda.empty_cache()` is not enough to free VRAM — the CUDA context and PyTorch allocator stay resident until process exit. Process exit is the only guarantee on 8 GB cards.

To verify VRAM is being released, run `watch -n 0.5 nvidia-smi` in a side terminal. You should see VRAM climb to ~5.5 GB during generation, then drop to near zero when the script exits, then climb again as CARLA loads.

---

## Supported Config Values

These are the values the validator accepts. The model has been trained to use them.

**Maps:** `Town01`, `Town02`, `Town03`, `Town04`, `Town05`, `Town06`, `Town07`, `Town10HD`, `Town11`, `Town12`

**Adversary types:** `vehicle`, `walker`, `prop`

**Spawn modes:** `random`, `random_lane`, `index`, `coordinates`, `near_intersection`, `nearby_random`, `relative_to_ego`, `relative_to_actor`, `lane_ahead`, `junction_cross`

**Vehicle behaviors:** `follow_lane`, `lane_change`, `cut_in`, `sudden_brake`, `constant_speed`, `ttc_brake`, `intersection_conflict`, `traffic_manager`, `autopilot`, `stopped`, `parked`

**Walker behaviors:** `walk_across`, `walk_to`, `stationary`

**Weather presets:** `ClearNoon`, `CloudyNoon`, `WetNoon`, `WetCloudyNoon`, `SoftRainNoon`, `MidRainyNoon`, `HardRainNoon`, `ClearSunset`, `CloudySunset`, `WetSunset`, `WetCloudySunset`, `SoftRainSunset`, `MidRainSunset`, `HardRainSunset`, `ClearNight`, `CloudyNight`, `WetNight`, `WetCloudyNight`, `SoftRainNight`, `MidRainyNight`, `HardRainNight`, `FoggyNoon`, `FoggySunset`, `FoggyNight`

**Traffic lights:** `normal`, `force_green`, `default`

**Cameras:** `behind`, `top`, `lidar`
