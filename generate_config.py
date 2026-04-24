#!/usr/bin/env python3
"""
generate_config.py — Generate a CARLA scenario JSON from a natural-language
description using the fine-tuned Qwen2.5-Coder-7B model.

Workflow (recommended):
    Terminal 1:  python3 generate_config.py
                 # type scenario description, Ctrl-D to submit
                 # exits when done — VRAM fully released by the OS
    Terminal 2:  cd CARLA_0.9.15 && ./CarlaUE4.sh -prefernvidia
                 # (can be started in parallel with terminal 1)
    Terminal 3:  python3 carla_runner.py --config generated_config/config_<timestamp>.json

The generator, simulator, and runner are deliberately decoupled:
  - Generator is one-shot; exits the instant the JSON is written.
  - Simulator is long-lived; start it once and reuse it for many scenarios.
  - Runner is per-scenario; re-run without regenerating if CARLA crashes.

The system prompt, chat template, pad-token handling, and sampling params
here MUST match fine-tuning time (Colab notebook, cells 8 and 28).
"""
from __future__ import annotations

import argparse
import json
import os
import re
import sys
from datetime import datetime

import torch
from transformers import AutoModelForCausalLM, AutoTokenizer, BitsAndBytesConfig
from peft import PeftModel

from validator import validate_config


MODEL_NAME = "Qwen/Qwen2.5-Coder-7B-Instruct"

# Must match the SYSTEM_PROMPT used during fine-tuning (notebook cell 8).
SYSTEM_PROMPT = (
    "You are an expert at generating CARLA autonomous driving scenario JSON "
    "configuration files. Given a natural language description of a driving "
    "scenario, generate a complete, valid JSON configuration that accurately "
    "implements the described scenario. The JSON must include map, scenario, "
    "loop, ego, adversaries, environment, display, termination, and "
    "postconditions fields as appropriate. Output ONLY the JSON object — no "
    "explanation, no markdown fences, no commentary before or after."
)


def extract_json(text: str) -> str:
    """
    Pull the first balanced {...} block out of a string.
    Tolerates the (rare) case where the model emits ```json fences
    despite the system prompt telling it not to.
    """
    text = re.sub(r"^```(?:json)?\s*|\s*```\s*$", "", text.strip(),
                  flags=re.MULTILINE)
    start = text.find("{")
    if start < 0:
        return ""
    depth = 0
    in_str = False
    esc = False
    for i in range(start, len(text)):
        c = text[i]
        if esc:
            esc = False
            continue
        if c == "\\":
            esc = True
            continue
        if c == '"':
            in_str = not in_str
            continue
        if in_str:
            continue
        if c == "{":
            depth += 1
        elif c == "}":
            depth -= 1
            if depth == 0:
                return text[start : i + 1]
    return text[start:]  # unterminated — caller will get a JSON error


def load_model(adapter_dir: str, cache_dir: str):
    """Load Qwen2.5-Coder-7B in 4-bit NF4 and apply the LoRA adapter."""
    bnb = BitsAndBytesConfig(
        load_in_4bit=True,
        bnb_4bit_quant_type="nf4",
        bnb_4bit_compute_dtype=torch.float16,  # RTX 3070 (Ampere) — fp16 is fine
        bnb_4bit_use_double_quant=True,
    )

    print(f"[gen] Loading tokenizer (cache: {cache_dir})...", flush=True)
    tok = AutoTokenizer.from_pretrained(MODEL_NAME, cache_dir=cache_dir)
    if tok.pad_token is None:
        # Defensive: Qwen2.5 ships with a pad token, but if someone swapped
        # tokenizer versions we fall back to the training convention.
        tok.add_special_tokens({"pad_token": "<|pad|>"})
    tok.padding_side = "right"

    print(f"[gen] Loading base model {MODEL_NAME} in 4-bit "
          "(first run downloads ~15 GB to the HF cache)...", flush=True)
    base = AutoModelForCausalLM.from_pretrained(
        MODEL_NAME,
        quantization_config=bnb,
        device_map="auto",
        cache_dir=cache_dir,
        torch_dtype=torch.float16,
    )

    print(f"[gen] Applying LoRA adapter from {adapter_dir}...", flush=True)
    model = PeftModel.from_pretrained(base, adapter_dir)
    model.eval()

    if torch.cuda.is_available():
        used = torch.cuda.memory_allocated() / 1e9
        print(f"[gen] Model ready. VRAM used: {used:.2f} GB", flush=True)
    return tok, model


def sample_once(tok, model, description: str,
                max_new_tokens: int, temperature: float, top_p: float) -> str:
    msgs = [
        {"role": "system", "content": SYSTEM_PROMPT},
        {"role": "user", "content": description},
    ]
    input_ids = tok.apply_chat_template(
        msgs,
        tokenize=True,
        add_generation_prompt=True,
        return_tensors="pt",
    ).to(model.device)

    with torch.no_grad():
        out = model.generate(
            input_ids,
            max_new_tokens=max_new_tokens,
            temperature=temperature,
            do_sample=True,
            top_p=top_p,
            pad_token_id=tok.pad_token_id,
            eos_token_id=tok.eos_token_id,
        )
    return tok.decode(out[0][input_ids.shape[1]:], skip_special_tokens=True)


def generate_and_validate(tok, model, prompt: str,
                          max_retries: int,
                          base_temperature: float,
                          max_new_tokens: int) -> tuple[dict | None, str | None]:
    """
    Sample, parse, and schema-validate, retrying up to max_retries times.
    Returns (config_dict, None) on success or (None, last_error) on failure.
    """
    temperature = base_temperature
    last_err: str | None = None

    for attempt in range(1, max_retries + 1):
        print(f"[gen] Attempt {attempt}/{max_retries} "
              f"(temperature={temperature:.2f})", flush=True)
        raw = sample_once(tok, model, prompt,
                          max_new_tokens=max_new_tokens,
                          temperature=temperature,
                          top_p=0.9)

        body = extract_json(raw)
        if not body:
            last_err = "no JSON object found in model output"
            print(f"[gen] {last_err}", flush=True)
        else:
            try:
                cfg = json.loads(body)
            except json.JSONDecodeError as e:
                last_err = f"json.loads failed: {e}"
                print(f"[gen] {last_err}", flush=True)
            else:
                ok, errs = validate_config(cfg)
                if ok:
                    return cfg, None
                last_err = f"schema validation failed ({len(errs)} issue(s)):\n        " \
                           + "\n        ".join(f"- {e}" for e in errs)
                print(f"[gen] {last_err}", flush=True)

        # Bump temperature slightly on each retry to escape the same local mode.
        temperature = min(0.6, temperature + 0.15)

    return None, last_err


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--adapter-dir", default="./qwen7b_json_lora_final",
                    help="Path to the unzipped LoRA adapter folder.")
    ap.add_argument("--cache-dir", default="./hf_cache",
                    help="HuggingFace cache (base model goes here).")
    ap.add_argument("--max-retries", type=int, default=3,
                    help="Re-sample if the output fails JSON parse or schema validation.")
    ap.add_argument("--temperature", type=float, default=0.2)
    ap.add_argument("--max-new-tokens", type=int, default=1500)
    args = ap.parse_args()

    # Resolve the prompt from stdin.
    print("=" * 72)
    print(" CARLA Adversarial Scenario Generator")
    print("=" * 72)
    print("Describe the adversarial scenario (multi-line OK).")
    print("Finish with Ctrl-D (Linux/macOS) or Ctrl-Z then Enter (Windows).\n",
          flush=True)
    try:
        prompt = sys.stdin.read().strip()
    except KeyboardInterrupt:
        print("\nAborted.", file=sys.stderr)
        return 1

    if not prompt:
        print("Empty prompt — aborting.", file=sys.stderr)
        return 1

    if not os.path.isdir(args.adapter_dir):
        print(f"[gen] ERROR: adapter directory not found: {args.adapter_dir}\n"
              f"      Unzip qwen7b_json_lora_final.zip here first.",
              file=sys.stderr)
        return 2

    os.makedirs(args.cache_dir, exist_ok=True)
    out_dir = "generated_config"
    os.makedirs(out_dir, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = os.path.join(out_dir, f"config_{timestamp}.json")

    tok, model = load_model(args.adapter_dir, args.cache_dir)

    cfg, err = generate_and_validate(
        tok, model, prompt,
        max_retries=args.max_retries,
        base_temperature=args.temperature,
        max_new_tokens=args.max_new_tokens,
    )

    if cfg is None:
        print(f"[gen] FAILED after {args.max_retries} attempts. "
              f"Last error: {err}", file=sys.stderr)
        return 1

    # Write with indent=2 — same format the model was trained on.
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=2)

    print(f"\n[gen] Wrote {out_path}")
    print(f"[gen] Next: python3 carla_runner.py --config {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
