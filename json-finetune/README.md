# JSON Fine-Tuning for Qwen-7B

This folder contains all files related to fine-tuning Qwen-7B model for generating CARLA scenario JSON configs.

## 📁 Structure

```
simulation/json-finetune/
├── train_json.jsonl              # Training data (1,245 pairs, 3.0M)
├── val_json.jsonl                # Validation data (219 pairs, 530K)
├── split_report_json.txt         # Dataset statistics
├── DATASET_README.md             # Dataset documentation
├── COMPLETE_WORKFLOW.md          # End-to-end fine-tuning guide
├── prepare_json_finetune.py      # Data preparation script
├── local_inference_qwen.py       # Local inference on RTX 3070
└── qwen7b_json_finetune.ipynb    # Colab notebook for fine-tuning
```

## 🚀 Quick Start

### 1. Fine-tune on Colab (A100)
- Upload `train_json.jsonl` and `val_json.jsonl` to Colab
- Upload/run `qwen7b_json_finetune.ipynb`
- Download `qwen7b_json_lora_final.zip`

### 2. Local Inference (RTX 3070)
```bash
# Extract weights
unzip qwen7b_json_lora_final.zip

# Install dependencies
pip install transformers peft bitsandbytes torch

# Interactive mode
python local_inference_qwen.py --interactive

# Single scenario
python local_inference_qwen.py --scenario "Intersection with ego straight..." --output config.json
```

## 📖 Documentation

- **DATASET_README.md**: Dataset statistics, usage, validation strategy
- **COMPLETE_WORKFLOW.md**: Detailed guide for entire process (4000+ words)
- **split_report_json.txt**: Breakdown of 244 JSON configs across 216 scenario types

## 📊 Dataset Summary

- **Total configs**: 244 JSON files
- **Training pairs**: 1,245 (with 5-6 paraphrases per scenario)
- **Validation pairs**: 219 (original descriptions only)
- **Train/Val split**: 85% / 15%
- **Scenario types**: 216 unique types

## 🔧 Tools

### prepare_json_finetune.py
Reproducible data preparation script. Re-run anytime to regenerate datasets:
```bash
python prepare_json_finetune.py --input ../config-examples --val-split 0.15
```

### local_inference_qwen.py
Complete inference system with validation and saving:
```bash
# Interactive (prompts for scenarios)
python local_inference_qwen.py --interactive

# Single scenario with custom temperature
python local_inference_qwen.py --scenario "..." --temperature 0.6 --output result.json
```

## 💡 Tips

- **Temperature**: 0.5-0.6 (deterministic), 0.7-0.8 (balanced), 0.9+ (creative)
- **VRAM**: 8GB sufficient on RTX 3070 with 4-bit quantization
- **Speed**: ~1-2 seconds per config on RTX 3070

---

See **COMPLETE_WORKFLOW.md** for full documentation.
