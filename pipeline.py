#!/usr/bin/env python3
"""
pipeline.py — CARLA adversarial scenario pipeline.

Flow:
    1. Ask user for a scenario description.
    2. Run generate_config.py in a SUBPROCESS.
       When the subprocess exits, the OS reclaims 100% of the VRAM used
       by the LLM. This is the key reason we don't do everything in one
       process — see the note in generate_config.py.
    3. Validate the generated JSON against the structural schema.
    4. Retry on validation failure (runs the subprocess again).
    5. Hand off the validated config to your carla_runner.py.

Before running this:
    - The CARLA simulator must already be up in another terminal:
          cd CARLA_0.9.15 && ./CarlaUE4.sh -prefernvidia
    - qwen7b_json_lora_final/ must exist in the current directory
      (unzip the adapter bundle you downloaded from Colab).

Usage:
    # Interactive (default)
    python3 pipeline.py

    # Pass the prompt on the command line
    python3 pipeline.py --prompt "A pedestrian runs across a crosswalk..."

    # Generate only, don't launch CARLA
    python3 pipeline.py --no-run
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from datetime import datetime

from validator import validate_config


def generate_with_llm(prompt: str,
                      out_path: str,
                      adapter_dir: str,
                      cache_dir: str) -> int:
    """Spawn generate_config.py. Return its exit code."""
    cmd = [
        sys.executable, "generate_config.py",
        "--prompt", prompt,
        "--out", out_path,
        "--adapter-dir", adapter_dir,
        "--cache-dir", cache_dir,
    ]
    print("\n[pipeline] Running LLM in a subprocess "
          "(VRAM fully released when it exits)")
    print(f"[pipeline]   -> {out_path}")
    # subprocess.call streams the child's stdout/stderr to our terminal
    # and blocks until exit — exactly what we want.
    return subprocess.call(cmd)


def launch_carla_runner(runner: str, config_path: str) -> int:
    cmd = [sys.executable, runner, "--config", config_path]
    print(f"\n[pipeline] Launching CARLA runner: {' '.join(cmd)}\n")
    return subprocess.call(cmd)


def read_prompt_from_stdin() -> str:
    print("=" * 72)
    print(" CARLA Adversarial Scenario Generator ")
    print("=" * 72)
    print("Describe the adversarial scenario (multi-line OK).")
    print("Finish with Ctrl-D (Linux/macOS) or Ctrl-Z then Enter (Windows).\n")
    try:
        return sys.stdin.read().strip()
    except KeyboardInterrupt:
        return ""


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--prompt", default=None,
                    help="Scenario description. If omitted, read from stdin.")
    ap.add_argument("--config-dir", default="config",
                    help="Directory to write generated configs into.")
    ap.add_argument("--runner", default="carla_runner.py",
                    help="Path to your CARLA runner script.")
    ap.add_argument("--adapter-dir", default="./qwen7b_json_lora_final",
                    help="Path to the unzipped LoRA adapter folder.")
    ap.add_argument("--cache-dir", default="./hf_cache",
                    help="HuggingFace cache (base model lives here).")
    ap.add_argument("--max-retries", type=int, default=3,
                    help="Retry generation when validation fails.")
    ap.add_argument("--no-run", action="store_true",
                    help="Stop after validation; do not launch CARLA.")
    args = ap.parse_args()

    # 1. Prompt
    prompt = args.prompt if args.prompt else read_prompt_from_stdin()
    if not prompt:
        print("Empty prompt — aborting.", file=sys.stderr)
        return 1

    os.makedirs(args.config_dir, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    config_path = os.path.join(args.config_dir, f"generated_{stamp}.json")

    # 2 + 3 + 4. Generate and validate, retrying as a single unit.
    cfg: dict | None = None
    for attempt in range(1, args.max_retries + 1):
        rc = generate_with_llm(prompt, config_path,
                               args.adapter_dir, args.cache_dir)
        if rc != 0:
            print(f"[pipeline] Generator subprocess failed (rc={rc}).")
            if attempt < args.max_retries:
                print(f"[pipeline] Retrying ({attempt + 1}/{args.max_retries})...")
                continue
            return rc

        try:
            with open(config_path, "r", encoding="utf-8") as f:
                cfg = json.load(f)
        except json.JSONDecodeError as e:
            # generate_config.py already retries json.loads internally, so
            # this path is unlikely — but guard anyway.
            print(f"[pipeline] Output file is not valid JSON: {e}")
            if attempt < args.max_retries:
                continue
            return 1

        ok, errs = validate_config(cfg)
        if ok:
            print(f"[pipeline] Schema valid. Config at: {config_path}")
            break

        print(f"[pipeline] Schema invalid ({len(errs)} issue(s)):")
        for e in errs:
            print(f"           - {e}")
        if attempt < args.max_retries:
            print(f"[pipeline] Retrying ({attempt + 1}/{args.max_retries})...")
            cfg = None
        else:
            print("[pipeline] Max retries reached; keeping the last output "
                  f"at {config_path} for inspection.")
            return 1

    assert cfg is not None

    # Show a short preview so the user sees what's about to run.
    preview = json.dumps(cfg, indent=2)
    print("\n[pipeline] Config preview (first ~600 chars):")
    print(preview[:600] + ("\n..." if len(preview) > 600 else ""))

    if args.no_run:
        print("\n[pipeline] --no-run set; skipping CARLA launch.")
        return 0

    if not os.path.isfile(args.runner):
        print(f"\n[pipeline] Runner {args.runner} not found. "
              "Use --runner to point at your carla_runner.py.",
              file=sys.stderr)
        return 1

    # Small pause so the GPU driver has fully freed the LLM's blocks
    # before CARLA starts grabbing VRAM. Not strictly required but cheap.
    time.sleep(1.0)

    # 5. Hand off.
    return launch_carla_runner(args.runner, config_path)


if __name__ == "__main__":
    sys.exit(main())
