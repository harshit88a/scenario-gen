"""
Fine-tune Data Preparer
────────────────────────
Converts augmented.jsonl into train.jsonl and val.jsonl in ChatML
messages format, ready for QLoRA fine-tuning with trl/transformers.

Each scenic record expands into N training pairs:
    1 original description  → code
    + up to 3 paraphrases   → same code

The validation set uses ONLY original descriptions (no paraphrases),
so it tests true generalization to unseen phrasings.

Usage:
    python prepare_finetune.py --input augmented.jsonl
    python prepare_finetune.py --input augmented.jsonl --val_split 0.15 --seed 42

Output:
    train.jsonl   — training pairs (original + paraphrases)
    val.jsonl     — validation pairs (originals only)
    split_report.txt — human-readable summary
"""

import json
import random
import argparse
import pathlib
from collections import defaultdict


# ─────────────────────────────────────────────
# SYSTEM PROMPT
# ─────────────────────────────────────────────

SYSTEM_PROMPT = (
    "You are an expert in the Scenic 3.0 probabilistic programming language "
    "for autonomous driving simulation in CARLA. "
    "Given a natural language description of a driving scenario, generate "
    "complete, valid Scenic 3.0 code that accurately implements the described scenario. "
    "The code must include all necessary parameters, behaviors, road geometry setup, "
    "and actor spawn positions. Do not include any explanation — output only the code."
)


# ─────────────────────────────────────────────
# HELPERS
# ─────────────────────────────────────────────

def make_pair(description: str, code: str) -> dict:
    """Create a single ChatML training pair."""
    return {
        "messages": [
            {"role": "system",    "content": SYSTEM_PROMPT},
            {"role": "user",      "content": description.strip()},
            {"role": "assistant", "content": code.strip()},
        ]
    }


def load_augmented(input_path: str) -> list[dict]:
    """Load and validate records from augmented.jsonl."""
    records = []
    skipped = 0
    with open(input_path, "r", encoding="utf-8") as f:
        for i, line in enumerate(f, start=1):
            line = line.strip()
            if not line:
                continue
            try:
                r = json.loads(line)
            except json.JSONDecodeError as e:
                print(f"  WARNING: Skipping malformed line {i}: {e}")
                skipped += 1
                continue

            # Must have both description and code
            if not r.get("scenario_description") or not r.get("code"):
                print(f"  WARNING: Skipping {r.get('id', f'line {i}')} — missing description or code")
                skipped += 1
                continue

            records.append(r)

    print(f"Loaded {len(records)} valid records ({skipped} skipped)")
    return records


def write_jsonl(pairs: list[dict], path: str):
    with open(path, "w", encoding="utf-8") as f:
        for pair in pairs:
            f.write(json.dumps(pair, ensure_ascii=False) + "\n")


# ─────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Prepare fine-tuning data from augmented.jsonl")
    parser.add_argument("--input",     type=str, default="augmented.jsonl",
                        help="Path to augmented JSONL file (default: augmented.jsonl)")
    parser.add_argument("--output_dir", type=str, default=".",
                        help="Directory to write train.jsonl and val.jsonl (default: current dir)")
    parser.add_argument("--val_split", type=float, default=0.1,
                        help="Fraction of RECORDS (not pairs) to reserve for validation (default: 0.1)")
    parser.add_argument("--seed",      type=int, default=42,
                        help="Random seed for reproducible split (default: 42)")
    args = parser.parse_args()

    output_dir = pathlib.Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # ── Load ──────────────────────────────────────────────────────────────
    records = load_augmented(args.input)
    if not records:
        print("No valid records found. Exiting.")
        return

    # ── Split records (not pairs) into train / val ────────────────────────
    # Shuffling records before splitting ensures the val set is a random
    # sample of scenarios, not just the last N files alphabetically.
    random.seed(args.seed)
    shuffled = records.copy()
    random.shuffle(shuffled)

    n_val = max(1, int(len(shuffled) * args.val_split))
    val_records   = shuffled[:n_val]
    train_records = shuffled[n_val:]

    # ── Build training pairs ──────────────────────────────────────────────
    # Training: original + all available paraphrases
    train_pairs = []
    para_stats = defaultdict(int)   # track paraphrase count distribution

    for r in train_records:
        code = r["code"]
        n_para = len(r.get("paraphrases", []))
        para_stats[n_para] += 1

        # Original description
        train_pairs.append(make_pair(r["scenario_description"], code))

        # Paraphrases (however many were generated — 0, 1, 2, or 3)
        for p in r.get("paraphrases", []):
            if p.get("description"):
                train_pairs.append(make_pair(p["description"], code))

    # ── Build validation pairs ────────────────────────────────────────────
    # Validation: ONLY original descriptions — no paraphrases.
    # This tests whether the model generalizes to unseen phrasings,
    # not just memorized paraphrase variants.
    val_pairs = [make_pair(r["scenario_description"], r["code"]) for r in val_records]

    # ── Write outputs ─────────────────────────────────────────────────────
    train_path = output_dir / "train.jsonl"
    val_path   = output_dir / "val.jsonl"

    write_jsonl(train_pairs, str(train_path))
    write_jsonl(val_pairs,   str(val_path))

    # ── Report ────────────────────────────────────────────────────────────
    report_lines = [
        "─" * 54,
        "  FINE-TUNE DATA SUMMARY",
        "─" * 54,
        f"  Input records total         : {len(records)}",
        f"  Train records (scenarios)   : {len(train_records)}",
        f"  Val records   (scenarios)   : {len(val_records)}  ({args.val_split*100:.0f}% of total)",
        "",
        "  Training pairs breakdown:",
        f"    Records with 0 paraphrases: {para_stats[0]:>4}  →  {para_stats[0] * 1:>4} pairs",
        f"    Records with 1 paraphrase : {para_stats[1]:>4}  →  {para_stats[1] * 2:>4} pairs",
        f"    Records with 2 paraphrases: {para_stats[2]:>4}  →  {para_stats[2] * 3:>4} pairs",
        f"    Records with 3 paraphrases: {para_stats[3]:>4}  →  {para_stats[3] * 4:>4} pairs",
        "─" * 54,
        f"  Total TRAIN pairs           : {len(train_pairs)}",
        f"  Total VAL   pairs           : {len(val_pairs)}",
        "─" * 54,
        "",
        "  Output files:",
        f"    {train_path}",
        f"    {val_path}",
        "─" * 54,
    ]

    report = "\n".join(report_lines)
    print("\n" + report)

    # Save report to file for reference
    report_path = output_dir / "split_report.txt"
    report_path.write_text(report + "\n")
    print(f"\n  Report saved to: {report_path}")


if __name__ == "__main__":
    main()