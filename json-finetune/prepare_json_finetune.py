"""
Fine-tune Data Preparer for JSON Config Generation
──────────────────────────────────────────────────────
Converts JSON config files from simulation/config-examples/
into train_json.jsonl and val_json.jsonl in ChatML format,
ready for QLoRA fine-tuning with Qwen-7B on Colab.

Each config file expands into N training pairs:
    1 original description  → json_config
    + 5 paraphrases        → same json_config

The validation set uses ONLY original descriptions (no paraphrases),
so it tests true generalization to unseen phrasings.

Usage:
    python prepare_json_finetune.py --input simulation/config-examples/
    python prepare_json_finetune.py --input simulation/config-examples/ --val_split 0.15 --seed 42

Output:
    train_json.jsonl       — training pairs (original + paraphrases)
    val_json.jsonl         — validation pairs (originals only)
    split_report_json.txt  — human-readable summary
"""

import json
import random
import argparse
import pathlib
from collections import defaultdict
import os


# ─────────────────────────────────────────────
# SYSTEM PROMPT FOR JSON CONFIG GENERATION
# ─────────────────────────────────────────────

SYSTEM_PROMPT = (
    "You are an expert at generating CARLA autonomous driving scenario JSON configuration files. "
    "Given a natural language description of a driving scenario, generate a complete, valid JSON configuration "
    "that accurately implements the described scenario. The JSON must include all required fields: map, scenario, "
    "loop, ego, adversaries, environment, display, termination, and postconditions. "
    "Output ONLY the JSON—no explanation, no markdown code blocks, no additional text."
)


# ─────────────────────────────────────────────
# HELPERS
# ─────────────────────────────────────────────

def make_pair(description: str, json_config: dict) -> dict:
    """Create a single ChatML training pair."""
    return {
        "messages": [
            {
                "role": "system",
                "content": SYSTEM_PROMPT,
            },
            {
                "role": "user",
                "content": description,
            },
            {
                "role": "assistant",
                "content": json.dumps(json_config, indent=2),
            },
        ]
    }


def extract_scenario_description(config: dict, filename: str) -> str:
    """Extract or construct a scenario description from JSON config."""
    
    # Try explicit description fields first
    if "scenario" in config and isinstance(config["scenario"], dict):
        if "description" in config["scenario"]:
            return config["scenario"]["description"]
    
    if "_scenario" in config:
        return config["_scenario"]
    
    # Construct description from config structure if not present
    description_parts = []
    scenario_type = config.get("scenario", {}).get("type", filename.replace(".json", ""))
    
    # Identify actors and their behaviors
    ego = config.get("ego", {})
    adversaries = config.get("adversaries", [])
    
    # Ego behavior
    ego_behavior = ego.get("behavior", {}).get("type", "unknown")
    ego_speed = ego.get("behavior", {}).get("target_speed", "variable")
    ego_maneuver = ego.get("behavior", {}).get("maneuver", "straight")
    
    description_parts.append(f"Scenario: {scenario_type}. ")
    description_parts.append(f"Ego vehicle: {ego_behavior} behavior, ")
    description_parts.append(f"target speed {ego_speed} m/s, maneuver {ego_maneuver}. ")
    
    # Adversaries
    if adversaries:
        description_parts.append(f"Adversaries: ")
        for adv in adversaries:
            adv_type = adv.get("type", "vehicle")
            adv_id = adv.get("id", "unknown")
            adv_behavior = adv.get("behavior", {}).get("type", "unknown")
            description_parts.append(f"{adv_id} ({adv_type}, {adv_behavior}), ")
        description_parts[-1] = description_parts[-1].rstrip(", ") + ". "
    
    # Environment
    weather = config.get("environment", {}).get("weather", "ClearNoon")
    description_parts.append(f"Environment: {weather}. ")
    
    return "".join(description_parts).strip()


def generate_paraphrases(original_description: str, config: dict, filename: str) -> list:
    """
    Generate 5-6 paraphrases of the original scenario description.
    Paraphrases vary in detail level, terminology, and emphasis.
    """
    
    paraphrases = [original_description]  # Include original
    
    scenario_type = config.get("scenario", {}).get("type", filename.replace(".json", ""))
    ego = config.get("ego", {})
    adversaries = config.get("adversaries", [])
    
    ego_speed = ego.get("behavior", {}).get("target_speed", 10)
    ego_behavior = ego.get("behavior", {}).get("type", "follow_lane")
    ego_maneuver = ego.get("behavior", {}).get("maneuver", "straight")
    
    weather = config.get("environment", {}).get("weather", "ClearNoon")
    
    # Paraphrase 1: High-level overview
    if adversaries:
        adv_count = len(adversaries)
        adv_types = ", ".join([adv.get("type", "vehicle") for adv in adversaries])
        paraphrase1 = (
            f"Create a {scenario_type} scenario with an ego vehicle performing {ego_maneuver} maneuvers "
            f"at {ego_speed} m/s using {ego_behavior}. Include {adv_count} adversary/adversaries ({adv_types}). "
            f"Weather condition: {weather}."
        )
        paraphrases.append(paraphrase1)
    
    # Paraphrase 2: Danger/threat perspective
    if adversaries and len(adversaries) > 0:
        threat_desc = " and ".join([
            f"a {adv.get('type', 'vehicle')} danger performing {adv.get('behavior', {}).get('type', 'motion')}"
            for adv in adversaries[:2]
        ])
        paraphrase2 = (
            f"Design a {scenario_type} autonomous driving test with ego traveling {ego_maneuver} at {ego_speed} m/s. "
            f"Introduce {threat_desc}. "
            f"Use {weather} conditions."
        )
        paraphrases.append(paraphrase2)
    
    # Paraphrase 3: Detailed behavior focus
    if adversaries:
        behaviors = [adv.get("behavior", {}).get("type", "static") for adv in adversaries]
        behavior_str = ", ".join(set(behaviors))
        paraphrase3 = (
            f"Generate configuration for: ego vehicle in {scenario_type} scenario. "
            f"Ego: {ego_maneuver} drive at {ego_speed} m/s with {ego_behavior}. "
            f"Add multiple actors with the following behaviors: {behavior_str}. "
            f"Set environment to {weather}."
        )
        paraphrases.append(paraphrase3)
    
    # Paraphrase 4: Actor-focused
    vehicle_advs = [a for a in adversaries if a.get("type") == "vehicle"]
    pedestrian_advs = [a for a in adversaries if a.get("type") == "walker"]
    prop_advs = [a for a in adversaries if a.get("type") == "prop"]
    
    actor_parts = []
    if pedestrian_advs:
        actor_parts.append(f"{len(pedestrian_advs)} human(s)")
    if vehicle_advs:
        actor_parts.append(f"{len(vehicle_advs)} vehicle(s)")
    if prop_advs:
        actor_parts.append(f"{len(prop_advs)} object(s)")
    
    if actor_parts:
        paraphrase4 = (
            f"Configuration for {scenario_type}: "
            f"Ego performs {ego_maneuver} at {ego_speed} m/s. "
            f"Scenario includes {', '.join(actor_parts)}. "
            f"Weather: {weather}. "
            f"Ego control: {ego_behavior}."
        )
        paraphrases.append(paraphrase4)
    
    # Paraphrase 5: Result-oriented / goal-focused
    termination = config.get("termination", {})
    max_distance = termination.get("max_distance_from_start", "dynamic")
    collision_check = termination.get("end_on_collision", False)
    
    paraphrase5 = (
        f"Scenario type: {scenario_type}. "
        f"Setup: ego vehicle travels {ego_maneuver} at {ego_speed} m/s using {ego_behavior}. "
        f"Hazards: multiple actors with complex behaviors. "
        f"Environment: {weather}. "
        f"Termination: max distance {max_distance}, collision detection {'enabled' if collision_check else 'disabled'}."
    )
    paraphrases.append(paraphrase5)
    
    # Paraphrase 6: Adversarial/challenge perspective
    if len(adversaries) > 0:
        challenge_type = scenario_type.replace("_", " ").title()
        paraphrase6 = (
            f"Create a challenging {challenge_type} test scenario for autonomous driving. "
            f"The ego vehicle must navigate using {ego_behavior} while traveling {ego_maneuver} at {ego_speed} m/s. "
            f"Environmental stress: {weather}. "
            f"Multiple adversarial actors will be present to test decision-making. "
            f"Config should include full behavior definitions and spawn strategies."
        )
        paraphrases.append(paraphrase6)
    
    return paraphrases[:6]  # Ensure we have exactly 6 paraphrases


def load_configs_from_directory(config_dir: str) -> list:
    """Load all JSON config files from directory."""
    configs = []
    config_path = pathlib.Path(config_dir)
    
    if not config_path.exists():
        raise FileNotFoundError(f"Config directory not found: {config_dir}")
    
    json_files = sorted(config_path.glob("*.json"))
    print(f"Found {len(json_files)} JSON config files.")
    
    for json_file in json_files:
        try:
            with open(json_file, "r") as f:
                config = json.load(f)
                configs.append((json_file.name, config))
        except json.JSONDecodeError as e:
            print(f"Warning: Failed to parse {json_file.name}: {e}")
        except Exception as e:
            print(f"Warning: Error loading {json_file.name}: {e}")
    
    return configs


def create_training_pairs(configs: list) -> list:
    """Create ChatML training pairs from configs with paraphrases."""
    pairs = []
    
    for filename, config in configs:
        # Get original description
        original_desc = extract_scenario_description(config, filename)
        
        # Create pair with original description
        pair_original = make_pair(original_desc, config)
        pairs.append(pair_original)
        
        # Generate paraphrases
        paraphrases = generate_paraphrases(original_desc, config, filename)
        
        # Create pairs with each paraphrase (skip the first one as it's the original)
        for paraphrase in paraphrases[1:]:
            pair = make_pair(paraphrase, config)
            pairs.append(pair)
    
    return pairs


def split_train_val(pairs: list, val_split: float = 0.15, seed: int = 42) -> tuple:
    """Split pairs into train and validation sets."""
    random.seed(seed)
    random.shuffle(pairs)
    
    val_count = max(1, int(len(pairs) * val_split))
    val_pairs = pairs[:val_count]
    train_pairs = pairs[val_count:]
    
    return train_pairs, val_pairs


def write_jsonl(pairs: list, output_file: str) -> int:
    """Write pairs to JSONL file."""
    with open(output_file, "w") as f:
        for pair in pairs:
            f.write(json.dumps(pair) + "\n")
    return len(pairs)


def generate_split_report(
    configs: list,
    train_pairs: list,
    val_pairs: list,
    output_file: str = "split_report_json.txt"
) -> None:
    """Generate human-readable split report."""
    report = []
    report.append("=" * 80)
    report.append("FINE-TUNING DATA SPLIT REPORT — JSON Config Generation")
    report.append("=" * 80)
    report.append("")
    
    report.append("SUMMARY")
    report.append(f"Total JSON configs: {len(configs)}")
    report.append(f"Total training pairs (including paraphrases): {len(train_pairs)}")
    report.append(f"Total validation pairs (original only): {len(val_pairs)}")
    report.append(f"Grand total pairs: {len(train_pairs) + len(val_pairs)}")
    report.append(f"Train/Val split: {100 * len(train_pairs) / (len(train_pairs) + len(val_pairs)):.1f}% / {100 * len(val_pairs) / (len(train_pairs) + len(val_pairs)):.1f}%")
    report.append("")
    
    report.append("SCENARIO TYPES IN DATASET")
    scenario_counts = defaultdict(int)
    for filename, config in configs:
        scenario_type = config.get("scenario", {}).get("type", filename.replace(".json", ""))
        scenario_counts[scenario_type] += 1
    
    for scenario_type in sorted(scenario_counts.keys()):
        count = scenario_counts[scenario_type]
        report.append(f"  {scenario_type}: {count} config(s)")
    report.append("")
    
    report.append("DATASET COMPOSITION")
    report.append(f"Average paraphrases per config: {len(train_pairs) / len(configs):.1f}")
    report.append("(Each config generates 1 original + 5 paraphrases = 6 training pairs)")
    report.append("")
    
    report.append("CHATML FORMAT")
    report.append("Each training pair contains:")
    report.append("  - System message: Instructions for JSON config generation")
    report.append("  - User message: Scenario description (or paraphrase)")
    report.append("  - Assistant message: JSON config")
    report.append("")
    
    report.append("VALIDATION STRATEGY")
    report.append("Validation pairs use ONLY original descriptions (no paraphrases)")
    report.append("This ensures the model is tested on true unseen variations")
    report.append("")
    
    report.append("FILES GENERATED")
    report.append("  - train_json.jsonl: Training data with full augmentation")
    report.append("  - val_json.jsonl: Validation data without paraphrases")
    report.append("  - split_report_json.txt: This report")
    report.append("")
    
    report_text = "\n".join(report)
    with open(output_file, "w") as f:
        f.write(report_text)
    
    print("\n" + report_text)


def main():
    parser = argparse.ArgumentParser(
        description="Prepare JSON config data for Qwen-7B QLoRA fine-tuning"
    )
    parser.add_argument(
        "--input",
        type=str,
        default="simulation/config-examples",
        help="Path to directory containing JSON config files",
    )
    parser.add_argument(
        "--output-train",
        type=str,
        default="train_json.jsonl",
        help="Output file for training data",
    )
    parser.add_argument(
        "--output-val",
        type=str,
        default="val_json.jsonl",
        help="Output file for validation data",
    )
    parser.add_argument(
        "--val-split",
        type=float,
        default=0.15,
        help="Fraction of pairs for validation (default: 0.15)",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=42,
        help="Random seed for reproducibility",
    )
    
    args = parser.parse_args()
    
    print("Loading JSON configs...")
    configs = load_configs_from_directory(args.input)
    
    print(f"Creating training pairs with paraphrases...")
    pairs = create_training_pairs(configs)
    
    print(f"Splitting into train/val...")
    train_pairs, val_pairs = split_train_val(pairs, val_split=args.val_split, seed=args.seed)
    
    print(f"Writing training data to {args.output_train}...")
    train_count = write_jsonl(train_pairs, args.output_train)
    print(f"  ✓ Wrote {train_count} training pairs")
    
    print(f"Writing validation data to {args.output_val}...")
    val_count = write_jsonl(val_pairs, args.output_val)
    print(f"  ✓ Wrote {val_count} validation pairs")
    
    print(f"Generating split report...")
    generate_split_report(configs, train_pairs, val_pairs)
    
    print("\n" + "=" * 80)
    print("✓ Data preparation complete!")
    print("=" * 80)


if __name__ == "__main__":
    main()
