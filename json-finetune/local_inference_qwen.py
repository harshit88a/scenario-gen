"""
Local Inference Script for Qwen2.5-Coder-7B JSON Config Generation
──────────────────────────────────────────────────────────────────

This script loads the fine-tuned Qwen2.5-Coder-7B-Instruct model with LoRA weights
and generates CARLA scenario JSON configs from natural language descriptions.

Uses the same approach as your proven Scenic fine-tuning.

Usage:
    python local_inference_qwen.py

Requirements:
    - PyTorch with CUDA support
    - transformers, peft, bitsandbytes
    - 8GB VRAM (RTX 3070 compatible with 4-bit quantization)

Files needed:
    - qwen7b_json_lora_final/ (adapter directory from Colab fine-tuning)
"""

import torch
import json
import argparse
from pathlib import Path
from transformers import AutoModelForCausalLM, AutoTokenizer, BitsAndBytesConfig
from peft import AutoPeftModelForCausalLM


class JSONConfigGenerator:
    """Generates CARLA JSON config files from scenario descriptions."""
    
    def __init__(self, lora_weights_path: str = "./qwen7b_json_lora_final", device: str = "cuda"):
        """
        Initialize the fine-tuned model for JSON config generation.
        
        Args:
            lora_weights_path: Path to the LoRA adapter directory
            device: Device to use ('cuda' or 'cpu')
        """
        print("🔧 Loading Qwen2.5-Coder-7B-Instruct with LoRA weights...")
        
        self.device = device if torch.cuda.is_available() else "cpu"
        print(f"   Device: {self.device}")
        
        if self.device == "cuda":
            print(f"   GPU: {torch.cuda.get_device_name(0)}")
            print(f"   VRAM: {torch.cuda.get_device_properties(0).total_memory / 1e9:.1f}GB")
        
        # 4-bit quantization config (fits in 8GB VRAM)
        bnb_config = BitsAndBytesConfig(
            load_in_4bit=True,
            bnb_4bit_quant_type="nf4",
            bnb_4bit_compute_dtype=torch.bfloat16,  # Use bfloat16 for better numerical stability
            bnb_4bit_use_double_quant=True,
        )
        
        # Load base model (4-bit quantized, Coder variant for JSON)
        print("   Loading base model: Qwen2.5-Coder-7B-Instruct (4-bit quantized)...")
        base_model = AutoModelForCausalLM.from_pretrained(
            "Qwen/Qwen2.5-Coder-7B-Instruct",
            quantization_config=bnb_config,
            device_map="auto",
            trust_remote_code=True,
        )
        
        # Load and merge LoRA
        print("   Loading LoRA adapter...")
        model_with_lora = AutoPeftModelForCausalLM.from_pretrained(
            lora_weights_path,
            device_map="auto",
        )
        
        print("   Merging LoRA weights...")
        self.model = model_with_lora.merge_and_unload()
        
        # Load tokenizer (matching the Coder model)
        print("   Loading tokenizer...")
        self.tokenizer = AutoTokenizer.from_pretrained("Qwen/Qwen2.5-Coder-7B-Instruct")
        
        print("✅ Model loaded and ready!\n")
    
    def generate_config(self, scenario_description: str, temperature: float = 0.7) -> dict:
        """
        Generate a JSON config from a scenario description.
        
        Args:
            scenario_description: Natural language description of the scenario
            temperature: Sampling temperature (0.7 = balanced, higher = more creative)
        
        Returns:
            Parsed JSON config dict, or None if generation failed
        """
        
        # Prepare messages
        messages = [
            {
                "role": "system",
                "content": (
                    "You are an expert at generating CARLA autonomous driving scenario JSON configuration files. "
                    "Given a natural language description of a driving scenario, generate a complete, valid JSON configuration "
                    "that accurately implements the described scenario. The JSON must include all required fields: map, scenario, "
                    "loop, ego, adversaries, environment, display, termination, and postconditions. "
                    "Output ONLY the JSON—no explanation, no markdown, no additional text."
                )
            },
            {
                "role": "user",
                "content": scenario_description
            }
        ]
        
        # Tokenize and generate
        with torch.no_grad():
            inputs = self.tokenizer.apply_chat_template(
                messages,
                tokenize=True,
                return_tensors="pt"
            ).to(self.device)
            
            outputs = self.model.generate(
                inputs,
                max_new_tokens=1024,
                temperature=temperature,
                top_p=0.9,
                do_sample=True,
                pad_token_id=self.tokenizer.eos_token_id,
            )
        
        # Decode response
        response = self.tokenizer.decode(outputs[0], skip_special_tokens=True)
        
        # Extract JSON
        json_config = self._extract_json(response)
        return json_config
    
    def _extract_json(self, response: str) -> dict:
        """Extract and parse JSON from model response."""
        try:
            json_start = response.find("{")
            json_end = response.rfind("}") + 1
            
            if json_start >= 0 and json_end > json_start:
                json_str = response[json_start:json_end]
                config = json.loads(json_str)
                return config
            else:
                print("❌ No JSON found in response")
                return None
        except json.JSONDecodeError as e:
            print(f"❌ Failed to parse JSON: {e}")
            return None
    
    def validate_config(self, config: dict) -> tuple[bool, list]:
        """
        Validate JSON config structure.
        
        Returns:
            (is_valid, list_of_issues)
        """
        issues = []
        
        # Required top-level fields
        required_fields = ["map", "scenario", "ego", "environment", "termination"]
        for field in required_fields:
            if field not in config:
                issues.append(f"Missing required field: {field}")
        
        # Validate ego structure
        if "ego" in config:
            ego = config["ego"]
            ego_required = ["blueprint", "spawn", "behavior"]
            for field in ego_required:
                if field not in ego:
                    issues.append(f"Missing ego field: {field}")
        
        # Validate adversaries if present
        if "adversaries" in config:
            adversaries = config["adversaries"]
            if not isinstance(adversaries, list):
                issues.append("adversaries must be a list")
            else:
                for i, adv in enumerate(adversaries):
                    if "id" not in adv:
                        issues.append(f"Adversary {i} missing id")
        
        return (len(issues) == 0, issues)


def main():
    parser = argparse.ArgumentParser(
        description="Generate CARLA JSON configs from scenario descriptions"
    )
    parser.add_argument(
        "--lora-path",
        type=str,
        default="./qwen7b_json_lora_final",
        help="Path to LoRA adapter directory"
    )
    parser.add_argument(
        "--scenario",
        type=str,
        help="Scenario description (if not provided, use interactive mode)"
    )
    parser.add_argument(
        "--output",
        type=str,
        help="Output file path (JSON)"
    )
    parser.add_argument(
        "--temperature",
        type=float,
        default=0.7,
        help="Sampling temperature (default: 0.7)"
    )
    parser.add_argument(
        "--interactive",
        action="store_true",
        help="Interactive mode (default if no --scenario)"
    )
    
    args = parser.parse_args()
    
    # Initialize generator
    generator = JSONConfigGenerator(args.lora_path)
    
    if args.scenario:
        # Single scenario mode
        print(f"📝 Scenario: {args.scenario}\n")
        print("🔄 Generating JSON config...")
        
        config = generator.generate_config(args.scenario, temperature=args.temperature)
        
        if config:
            # Validate
            is_valid, issues = generator.validate_config(config)
            
            print("\n✅ Generated Config:")
            print(json.dumps(config, indent=2))
            
            if is_valid:
                print("\n✅ Validation: PASSED")
            else:
                print("\n⚠️ Validation: FAILED")
                for issue in issues:
                    print(f"   - {issue}")
            
            # Save if requested
            if args.output:
                with open(args.output, "w") as f:
                    json.dump(config, f, indent=2)
                print(f"\n💾 Saved to {args.output}")
        else:
            print("❌ Failed to generate config")
    else:
        # Interactive mode
        print("\n" + "="*80)
        print("Interactive JSON Config Generator")
        print("="*80)
        print("\nType scenario descriptions and get JSON configs.")
        print("Type 'exit' to quit.\n")
        
        counter = 1
        while True:
            try:
                scenario = input(f"\n[{counter}] Scenario description (or 'exit'): ").strip()
                
                if scenario.lower() == "exit":
                    print("Goodbye! 👋")
                    break
                
                if not scenario:
                    print("Please enter a scenario description.")
                    continue
                
                print(f"\n🔄 Generating...")
                config = generator.generate_config(scenario, temperature=args.temperature)
                
                if config:
                    print("\n✅ Generated Config:")
                    print(json.dumps(config, indent=2))
                    
                    # Validate
                    is_valid, issues = generator.validate_config(config)
                    if is_valid:
                        print("\n✅ Validation: PASSED")
                    else:
                        print("\n⚠️ Validation issues found:")
                        for issue in issues:
                            print(f"   - {issue}")
                    
                    # Save option
                    save_it = input("\nSave to file? (y/n): ").strip().lower()
                    if save_it == "y":
                        filename = f"generated/scenario_{counter:03d}.json"
                        Path("generated").mkdir(exist_ok=True)
                        with open(filename, "w") as f:
                            json.dump(config, f, indent=2)
                        print(f"💾 Saved to {filename}")
                        counter += 1
                else:
                    print("❌ Failed to generate config. Try again.")
            
            except KeyboardInterrupt:
                print("\n\nInterrupted. Goodbye! 👋")
                break
            except Exception as e:
                print(f"❌ Error: {e}")


if __name__ == "__main__":
    main()
