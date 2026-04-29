#!/usr/bin/env python3
"""
pipeline.py — End-to-end CARLA adversarial scenario pipeline.

Flow:
    1. Ask the user for a scenario description (or pass via --prompt).
    2. Run generate_config.py in a SUBPROCESS.
       The subprocess owns the LLM weights and handles generation, internal
       retries, and schema validation.  When it exits the OS fully reclaims
       all LLM VRAM before CARLA is started.
    3. Start the CARLA server (CARLA_0.9.15/CarlaUE4.sh) as a background
       process and poll port 2000 until the server accepts connections.
    4. Run carla_runner.py with the generated config.
    5. On exit (normal or Ctrl-C) gracefully terminate the CARLA server.

Prerequisites:
    - qwen7b_json_lora_final/ must exist (LoRA adapter bundle).
    - CARLA_0.9.15/ must exist (the simulator binaries).

Usage:
    python3 pipeline.py                      # interactive prompt
    python3 pipeline.py --prompt "..."       # non-interactive
    python3 pipeline.py --no-run             # generate JSON only, skip CARLA
    python3 pipeline.py --no-carla-start     # assume CARLA is already running
"""
from __future__ import annotations

import argparse
import os
import socket
import subprocess
import sys
import time
from datetime import datetime


# ─────────────────────────────────────────────────────────────────────────────
# Step helpers
# ─────────────────────────────────────────────────────────────────────────────

def read_prompt_from_stdin() -> str:
    print("=" * 72)
    print(" CARLA Adversarial Scenario Generator")
    print("=" * 72)
    print("Describe the adversarial scenario (multi-line OK).")
    print("Finish with Ctrl-D (Linux/macOS) or Ctrl-Z then Enter (Windows).\n")
    try:
        return sys.stdin.read().strip()
    except KeyboardInterrupt:
        return ""


def run_generator(prompt: str, out_path: str,
                  adapter_dir: str, cache_dir: str,
                  max_retries: int) -> int:
    """
    Spawn generate_config.py as a subprocess.
    The subprocess owns all LLM weights; when it exits the OS fully reclaims
    the VRAM before CARLA is started.
    generate_config.py handles internal retries and schema validation itself.
    """
    cmd = [
        sys.executable, "generate_config.py",
        "--prompt",      prompt,
        "--out",         out_path,
        "--adapter-dir", adapter_dir,
        "--cache-dir",   cache_dir,
        "--max-retries", str(max_retries),
    ]
    print("\n[pipeline] Starting LLM generator subprocess …")
    print(f"[pipeline]   output → {out_path}")
    # subprocess.call streams child stdout/stderr to our terminal and blocks.
    return subprocess.call(cmd)


def start_carla(carla_dir: str, extra_flags: list[str]) -> subprocess.Popen:
    """Launch CarlaUE4.sh as a background process and return its Popen handle."""
    cmd = ["./CarlaUE4.sh"] + extra_flags
    print(f"\n[pipeline] Starting CARLA server: {' '.join(cmd)}")
    print(f"[pipeline]   cwd = {carla_dir}\n")
    # Inherit stdout/stderr so CARLA's own log appears in the terminal.
    return subprocess.Popen(cmd, cwd=carla_dir)


def wait_for_carla(host: str = "localhost", port: int = 2000,
                   timeout: float = 120.0, interval: float = 2.0) -> bool:
    """
    Poll host:port until a TCP connection succeeds or *timeout* seconds elapse.
    Returns True when CARLA is ready, False on timeout.
    """
    print(f"[pipeline] Waiting for CARLA on {host}:{port} "
          f"(timeout={timeout:.0f} s) …", flush=True)
    deadline = time.time() + timeout
    attempt  = 0
    while time.time() < deadline:
        attempt += 1
        try:
            with socket.create_connection((host, port), timeout=2.0):
                print(f"[pipeline] CARLA is ready  (attempt {attempt})")
                return True
        except OSError:
            print(f"[pipeline]   … not ready yet (attempt {attempt})", flush=True)
            time.sleep(interval)
    return False


def run_scenario(runner: str, config_path: str,
                 host: str, port: int) -> int:
    """Run carla_runner.py and stream its output to the terminal."""
    cmd = [
        sys.executable, runner,
        "--config", config_path,
        "--host",   host,
        "--port",   str(port),
    ]
    print(f"\n[pipeline] Launching runner: {' '.join(cmd)}\n")
    return subprocess.call(cmd)


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main() -> int:
    ap = argparse.ArgumentParser(
        description="End-to-end pipeline: generate config → start CARLA → run scenario",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--prompt", default=None,
                    help="Scenario description. If omitted, read interactively from stdin.")
    ap.add_argument("--config-dir", default="generated_config",
                    help="Directory for generated JSON configs (default: generated_config).")
    ap.add_argument("--runner", default="carla_runner.py",
                    help="Path to carla_runner.py (default: carla_runner.py).")
    ap.add_argument("--adapter-dir", default="./qwen7b_json_lora_final",
                    help="Path to the LoRA adapter folder.")
    ap.add_argument("--cache-dir", default="./hf_cache",
                    help="HuggingFace model cache directory.")
    ap.add_argument("--max-retries", type=int, default=3,
                    help="Max LLM generation retries on schema failure (default: 3).")
    ap.add_argument("--carla-dir", default="./CARLA_0.9.15",
                    help="CARLA installation directory (default: ./CARLA_0.9.15).")
    ap.add_argument("--carla-flags", default="-prefernvidia",
                    help="Extra flags passed to CarlaUE4.sh (default: -prefernvidia).")
    ap.add_argument("--host", default="localhost",
                    help="CARLA server host (default: localhost).")
    ap.add_argument("--port", type=int, default=2000,
                    help="CARLA server port (default: 2000).")
    ap.add_argument("--carla-timeout", type=float, default=120.0,
                    help="Seconds to wait for CARLA to become ready (default: 120).")
    ap.add_argument("--no-carla-start", action="store_true",
                    help="Skip launching CARLA; assume it is already running.")
    ap.add_argument("--no-run", action="store_true",
                    help="Stop after JSON generation; do not start CARLA or the runner.")
    args = ap.parse_args()

    # ── 1. Prompt ─────────────────────────────────────────────────────────────
    prompt = args.prompt or read_prompt_from_stdin()
    if not prompt:
        print("[pipeline] Empty prompt — aborting.", file=sys.stderr)
        return 1

    # ── 2. Generate config (LLM subprocess — releases VRAM on exit) ───────────
    if not os.path.isdir(args.adapter_dir):
        print(f"[pipeline] ERROR: adapter directory not found: {args.adapter_dir}",
              file=sys.stderr)
        return 2

    os.makedirs(args.config_dir, exist_ok=True)
    stamp       = datetime.now().strftime("%Y%m%d_%H%M%S")
    config_path = os.path.join(args.config_dir, f"config_{stamp}.json")

    rc = run_generator(prompt, config_path,
                       args.adapter_dir, args.cache_dir,
                       args.max_retries)
    if rc != 0:
        print(f"[pipeline] Generator exited with code {rc} — aborting.",
              file=sys.stderr)
        return rc

    if not os.path.isfile(config_path):
        print(f"[pipeline] Expected config not found at {config_path}",
              file=sys.stderr)
        return 1

    print(f"\n[pipeline] Config ready: {config_path}")

    if args.no_run:
        print("[pipeline] --no-run set; stopping before CARLA launch.")
        return 0

    # ── 3. Start CARLA server ─────────────────────────────────────────────────
    carla_proc: subprocess.Popen | None = None
    if not args.no_carla_start:
        carla_dir = os.path.abspath(args.carla_dir)
        if not os.path.isfile(os.path.join(carla_dir, "CarlaUE4.sh")):
            print(f"[pipeline] ERROR: CarlaUE4.sh not found in {carla_dir}",
                  file=sys.stderr)
            return 1

        # Brief pause: lets the GPU driver flush any remaining LLM allocations
        # before CARLA claims VRAM.
        print("[pipeline] Waiting 2 s for VRAM to settle after LLM exit …")
        time.sleep(2.0)

        extra_flags = [f for f in args.carla_flags.split() if f]
        carla_proc  = start_carla(carla_dir, extra_flags)

    # ── 4. Wait for CARLA to accept connections, then run the scenario ─────────
    try:
        if not args.no_carla_start:
            ready = wait_for_carla(args.host, args.port, args.carla_timeout)
            if not ready:
                print(
                    f"[pipeline] CARLA did not become ready within "
                    f"{args.carla_timeout:.0f} s — aborting.",
                    file=sys.stderr,
                )
                return 1
        else:
            print(f"[pipeline] --no-carla-start: assuming CARLA is up on "
                  f"{args.host}:{args.port}")

        # ── 5. Run the scenario ────────────────────────────────────────────────
        rc = run_scenario(args.runner, config_path, args.host, args.port)

    except KeyboardInterrupt:
        print("\n[pipeline] Interrupted by user.")
        rc = 0

    finally:
        # ── 6. Gracefully shut down the CARLA server we started ───────────────
        if carla_proc is not None and carla_proc.poll() is None:
            print("\n[pipeline] Stopping CARLA server …")
            try:
                carla_proc.terminate()
                carla_proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                carla_proc.kill()
            print("[pipeline] CARLA stopped.")

    print("[pipeline] Done.")
    return rc


if __name__ == "__main__":
    sys.exit(main())

