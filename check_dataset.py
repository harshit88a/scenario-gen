"""
Dataset Checker
───────────────
Analyzes augmented.jsonl and reports how many training examples
are available (original + paraphrases), broken down by paraphrase count.

Usage:
    python check_dataset.py --input augmented.jsonl
"""

import json
import argparse
from collections import Counter


def check_dataset(input_path: str):
    records = []
    with open(input_path, "r", encoding="utf-8") as f:
        for i, line in enumerate(f, start=1):
            line = line.strip()
            if not line:
                continue
            try:
                records.append(json.loads(line))
            except json.JSONDecodeError as e:
                print(f"  WARNING: Skipping malformed line {i}: {e}")

    total_records = len(records)
    if total_records == 0:
        print("No records found in file.")
        return

    # Count paraphrases per record
    para_counts = Counter()
    total_training_examples = 0
    failed = []         # 0 paraphrases
    partial = []        # 1 or 2 paraphrases

    for r in records:
        n = len(r.get("paraphrases", []))
        para_counts[n] += 1
        total_training_examples += 1 + n   # original + however many paraphrases

        if n == 0:
            failed.append(r["source_file"])
        elif n < 3:
            partial.append((r["source_file"], n))

    # ── Report ────────────────────────────────────────────────────────────
    print("\n" + "─" * 52)
    print("  DATASET SUMMARY")
    print("─" * 52)
    print(f"  Total .scenic files parsed      : {total_records}")
    print(f"  Total training examples         : {total_training_examples}")
    print()
    print("  Breakdown by paraphrase count:")
    print(f"    0 paraphrases (original only) : {para_counts[0]:>4}  →  {para_counts[0] * 1:>4} examples")
    print(f"    1 paraphrase                  : {para_counts[1]:>4}  →  {para_counts[1] * 2:>4} examples")
    print(f"    2 paraphrases                 : {para_counts[2]:>4}  →  {para_counts[2] * 3:>4} examples")
    print(f"    3 paraphrases (full)          : {para_counts[3]:>4}  →  {para_counts[3] * 4:>4} examples")
    print("─" * 52)
    print(f"  Files with full 3 paraphrases   : {para_counts[3]} / {total_records}")
    print(f"  Files needing retry (< 3)       : {para_counts[0] + para_counts[1] + para_counts[2]}")
    print("─" * 52)

    if failed:
        print(f"\n  Files with NO paraphrases ({len(failed)}):")
        for f in failed:
            print(f"    - {f}")

    if partial:
        print(f"\n  Files with PARTIAL paraphrases ({len(partial)}):")
        for f, n in partial:
            print(f"    - {f}  ({n}/3 generated)")

    print()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Check augmented.jsonl dataset statistics.")
    parser.add_argument("--input", type=str, default="augmented.jsonl",
                        help="Path to augmented JSONL file (default: augmented.jsonl)")
    args = parser.parse_args()
    check_dataset(args.input)