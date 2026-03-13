"""
Scenic 3.0 Dataset Generator
─────────────────────────────
Parses .scenic files, extracts descriptions + code, then uses Gemini 2.5 Flash
to generate 3 paraphrases per example. Saves results to a .jsonl file.

Usage:
    export GEMINI_API_KEY="your_key_here"
    python generate_scenic_dataset.py --input_dir ./scenic_examples --output augmented.jsonl

Requirements:
    pip install google-genai tqdm
"""

import os
import re
import json
import time
import argparse
import pathlib
from tqdm import tqdm
from google import genai
from google.genai import types


# ─────────────────────────────────────────────
# CONFIG
# ─────────────────────────────────────────────

MODEL = "gemini-2.5-flash"

SYSTEM_PROMPT = """You are a technical writer helping augment an autonomous driving simulation dataset.
The scenarios are written using the Scenic 3.0 probabilistic programming language for CARLA simulator.

Given a Scenic 3.0 scenario description, generate exactly 3 paraphrases.

Rules:
- Preserve ALL technical meaning: actor types (ego, car, prop), behaviors, distances, speed values, and whether the scenario is deterministic or random (uses Range/Uniform)
- Do NOT reference, quote, or alter any code
- Do NOT add information that is not in the original description
- Style variation must follow this order:
  1. Technical/formal — precise terminology, may use passive voice, suitable for a research paper
  2. Plain English narrative — explain it like describing a scene to a colleague unfamiliar with Scenic
  3. Concise — a single sentence capturing only the core scenario

Output format rules:
- Respond with ONLY 3 lines, nothing else — no preamble, no explanation, no blank lines
- Each paraphrase MUST fit on a single line — do NOT wrap or break onto the next line
- Each line MUST start with its number and a period: "1.", "2.", "3."

1. <technical paraphrase on one single line>
2. <narrative paraphrase on one single line>
3. <concise paraphrase on one single line>"""


# ─────────────────────────────────────────────
# PARSER — extract fields from a .scenic file
# ─────────────────────────────────────────────

# Section header pattern: "# 2. ADV BEHAVIOR ..."
SECTION_HEADER = re.compile(r"^#\s*\d+\.\s+(.+)$")

# Section numbers to skip (section 1 = map/model config, same in every file)
SKIP_SECTIONS = {1}


def parse_scenic_file(filepath: str, index: int = 0) -> dict | None:
    """
    Parse a single .scenic file into a structured dict.

    Returns None if no scenario description is found (malformed file).

    Fields extracted:
        id                  : unique UUID
        source_file         : relative path to the .scenic file
        scenario_description: top-level comment block (the main description)
        section_comments    : dict of section_name -> comment text (sections 2-4)
        code                : full raw file content
        paraphrases         : empty list (filled later by Gemini)
    """
    path = pathlib.Path(filepath)
    text = path.read_text(encoding="utf-8")
    lines = text.splitlines()

    # ── 1. Extract top-level scenario description ──────────────────────────
    # Defined as the leading contiguous comment block before any code.
    # Handles both single-line and multi-line comment blocks.
    # Example:
    #   # Scenario: Ego drives straight ...
    #   # continuation of description
    #   param map = ...   ← code starts here

    scenario_lines = []
    first_code_line = 0

    for i, line in enumerate(lines):
        stripped = line.strip()
        if stripped.startswith("#"):
            # Strip the leading "# " and accumulate
            cleaned = stripped.lstrip("#").strip()
            # Skip lines that are just separator bars like "# ─────────"
            if cleaned and not re.match(r"^[-─=*]+$", cleaned):
                scenario_lines.append(cleaned)
        elif stripped == "":
            # Allow blank lines within the header block
            continue
        else:
            first_code_line = i
            break

    scenario_description = " ".join(scenario_lines).strip()
    # Remove a leading "Scenario:" prefix if present
    scenario_description = re.sub(r"^[Ss]cenario:\s*", "", scenario_description).strip()

    if not scenario_description:
        print(f"  WARNING: No scenario description found in {path.name} — skipping.")
        return None

    # ── 2. Extract per-section inline comments ─────────────────────────────
    # Sections look like:
    #   # 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
    #   # Lead speed and brake trigger are sampled; ...
    #   param LEAD_SPEED = Range(10, 14)   ← code resumes here

    section_comments = {}
    current_section_num = None
    current_section_name = None
    current_comment_lines = []

    def flush_section():
        """Save accumulated comments for the current section."""
        if current_section_name and current_comment_lines:
            text = " ".join(current_comment_lines).strip()
            if text:
                section_comments[current_section_name] = text

    for line in lines[first_code_line:]:
        stripped = line.strip()
        m = SECTION_HEADER.match(stripped)
        if m:
            flush_section()
            # Parse section number from the original line (e.g., "# 2. ADV BEHAVIOR")
            num_match = re.match(r"^#\s*(\d+)\.", stripped)
            current_section_num = int(num_match.group(1)) if num_match else None
            current_section_name = m.group(1).lower().replace(" ", "_")
            current_comment_lines = []
        elif stripped.startswith("#") and current_section_name:
            # Check we are not in a skipped section
            if current_section_num not in SKIP_SECTIONS:
                comment_text = stripped.lstrip("#").strip()
                if comment_text and not re.match(r"^[-─=*]+$", comment_text):
                    current_comment_lines.append(comment_text)
        elif not stripped.startswith("#") and stripped:
            # Code line encountered — stop collecting comments for this section
            flush_section()
            current_section_name = None
            current_comment_lines = []

    flush_section()  # catch the last section

    # Filter out section 1 from section_comments just in case
    section_comments = {
        k: v for k, v in section_comments.items()
        if not k.startswith("map_and_model") and not k.startswith("map_")
    }

    return {
        "id": f"scenic_{index:03d}",
        "source_file": path.name,
        "scenario_description": scenario_description,
        "section_comments": section_comments,
        "code": text,
        "paraphrases": [],
    }


# ─────────────────────────────────────────────
# PARAPHRASER — call Gemini 2.5 Flash
# ─────────────────────────────────────────────

def parse_paraphrase_response(text: str) -> list[str]:
    """
    Extract the 3 numbered paraphrases from Gemini's response.

    Handles:
    - Minor formatting variations (extra spaces, bold ** markers)
    - Multi-line paraphrases where text wraps onto the next line
      (collects all lines between numbered markers into one string)
    """
    lines = [l.strip() for l in text.strip().splitlines() if l.strip()]

    MARKER = re.compile(r"^\*?\*?([123])\.\*?\*?\s*(.*)")

    buckets = {}
    current_num = None

    for line in lines:
        m = MARKER.match(line)
        if m:
            current_num = int(m.group(1))
            first_text = m.group(2).strip()
            buckets[current_num] = [first_text] if first_text else []
        elif current_num is not None:
            buckets[current_num].append(line)

    paraphrases = []
    for num in (1, 2, 3):
        if num in buckets and buckets[num]:
            paraphrases.append(" ".join(buckets[num]).strip())

    return paraphrases


def generate_paraphrases(
    client: genai.Client,
    description: str,
    max_retries: int = 3,
    retry_delay: float = 2.0,
) -> list[str]:
    """
    Call Gemini 2.5 Flash to generate 3 paraphrases.
    Returns a list of 3 strings, or an empty list on failure.
    """
    for attempt in range(1, max_retries + 1):
        try:
            response = client.models.generate_content(
                model=MODEL,
                config=types.GenerateContentConfig(
                    system_instruction=SYSTEM_PROMPT,
                    temperature=0.7,
                    max_output_tokens=1536,
                ),
                contents=f"Scenario description:\n{description}",
            )

            raw_text = response.text.strip()
            paraphrases = parse_paraphrase_response(raw_text)

            if len(paraphrases) == 3:
                return paraphrases

            # Fewer than 3 parsed — retry
            print(
                f"\n  Attempt {attempt}: Expected 3 paraphrases, got {len(paraphrases)}. "
                f"Raw response:\n  {raw_text[:200]}..."
            )

        except Exception as e:
            print(f"\n  Attempt {attempt} failed with error: {e}")

        if attempt < max_retries:
            time.sleep(retry_delay * attempt)  # exponential backoff

    return []  # signal failure to the caller


# ─────────────────────────────────────────────
# CHECKPOINT — resume interrupted runs
# ─────────────────────────────────────────────

def load_checkpoint(output_path: str) -> set[str]:
    """
    Read already-processed source_file names from an existing output file.
    This lets you resume a run that was interrupted midway.
    """
    done = set()
    p = pathlib.Path(output_path)
    if p.exists():
        for line in p.read_text().splitlines():
            if not line.strip():
                continue
            try:
                r = json.loads(line)
                done.add(r["source_file"])
            except json.JSONDecodeError:
                continue
    return done


# ─────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Generate augmented Scenic 3.0 JSONL dataset.")
    parser.add_argument(
        "--input_dir",
        type=str,
        required=True,
        help="Directory containing .scenic files",
    )
    parser.add_argument(
        "--output",
        type=str,
        default="augmented.jsonl",
        help="Output JSONL file path (default: augmented.jsonl)",
    )
    parser.add_argument(
        "--delay",
        type=float,
        default=0.5,
        help="Seconds to wait between API calls (default: 0.5)",
    )
    parser.add_argument(
        "--skip_paraphrase",
        action="store_true",
        help="Only extract and parse files, skip Gemini calls (useful for testing the parser)",
    )
    args = parser.parse_args()

    # ── Validate input directory ──────────────────────────────────────────
    input_dir = pathlib.Path(args.input_dir)
    if not input_dir.exists():
        raise FileNotFoundError(f"Input directory not found: {input_dir}")

    scenic_files = sorted(input_dir.glob("**/*.scenic"))
    if not scenic_files:
        raise ValueError(f"No .scenic files found in {input_dir}")
    print(f"Found {len(scenic_files)} .scenic files in {input_dir}")

    # ── Set up Gemini client ──────────────────────────────────────────────
    api_key = os.environ.get("GEMINI_API_KEY")
    if not api_key and not args.skip_paraphrase:
        raise EnvironmentError(
            "GEMINI_API_KEY environment variable is not set.\n"
            "Run: export GEMINI_API_KEY='your_key_here'"
        )

    client = genai.Client(api_key=api_key) if api_key else None

    # ── Load checkpoint (resume support) ─────────────────────────────────
    already_done = load_checkpoint(args.output)
    if already_done:
        print(f"Resuming — {len(already_done)} files already processed, skipping them.")

    # ── Process files ─────────────────────────────────────────────────────
    failed_ids = []
    parse_errors = []
    processed = 0

    with open(args.output, "a", encoding="utf-8") as out_file:
        for idx, scenic_path in enumerate(tqdm(scenic_files, desc="Processing", unit="file"), start=1):

            # Skip files already in the output (resume support)
            if scenic_path.name in already_done:
                continue

            # ── Parse the .scenic file ────────────────────────────────
            record = parse_scenic_file(str(scenic_path), index=idx)
            if record is None:
                parse_errors.append(scenic_path.name)
                continue

            # ── Generate paraphrases via Gemini ───────────────────────
            if not args.skip_paraphrase and client is not None:
                paras = generate_paraphrases(client, record["scenario_description"])

                if paras:
                    record["paraphrases"] = [
                        {"version": i + 1, "description": p}
                        for i, p in enumerate(paras)
                    ]
                else:
                    # Still write the record — paraphrases can be retried later
                    failed_ids.append(record["source_file"])
                    tqdm.write(f"  PARAPHRASE FAILED: {scenic_path.name}")

                time.sleep(args.delay)

            # ── Write record to JSONL ─────────────────────────────────
            out_file.write(json.dumps(record, ensure_ascii=False) + "\n")
            out_file.flush()  # ensure each record is saved immediately
            processed += 1

    # ── Summary report ────────────────────────────────────────────────────
    total_in_output = len(already_done) + processed
    print("\n" + "─" * 50)
    print(f"  Scenic files found       : {len(scenic_files)}")
    print(f"  Successfully processed   : {processed}")
    print(f"  Previously completed     : {len(already_done)}")
    print(f"  Total records in output  : {total_in_output}")
    print(f"  Parse errors (skipped)   : {len(parse_errors)}")
    print(f"  Paraphrase failures      : {len(failed_ids)}")
    print(f"  Output written to        : {args.output}")
    print("─" * 50)

    if parse_errors:
        print("\nFiles with parse errors:")
        for f in parse_errors:
            print(f"  - {f}")

    if failed_ids:
        print("\nFiles where Gemini paraphrasing failed (paraphrases field is empty):")
        for f in failed_ids:
            print(f"  - {f}")
        print("\nTo retry only failed files, filter your output JSONL for records with")
        print('empty paraphrases: jq \'select(.paraphrases == [])\' augmented.jsonl')


if __name__ == "__main__":
    main()