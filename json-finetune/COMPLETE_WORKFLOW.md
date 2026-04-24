# Complete Workflow: Qwen-7B Fine-Tuning for JSON Config Generation

This guide walks you through the entire process from dataset preparation to local inference.

## 📋 Overview

**Goal**: Generate CARLA scenario JSON configs from natural language descriptions using a fine-tuned Qwen-7B model.

**Timeline**: 
- Data preparation: ✅ Done (you're here)
- Colab fine-tuning: ~3 hours
- Local setup: ~30 minutes
- **Total**: ~4 hours active time + download time

---

## 🎯 Phase 1: Dataset Preparation ✅ COMPLETE

### What was generated

```
simulation/json-finetune/
├── train_json.jsonl          # 1,245 training pairs (3.0M)
├── val_json.jsonl            # 219 validation pairs (530K)
├── split_report_json.txt     # Dataset statistics report
├── prepare_json_finetune.py  # Data preparation script (for reference)
├── local_inference_qwen.py   # Local inference script for RTX 3070
├── qwen7b_json_finetune.ipynb # Colab fine-tuning notebook
└── DATASET_README.md         # Dataset documentation
```

### Dataset Statistics
- **Total configs processed**: 244 unique JSON files
- **Total training examples**: 1,464 pairs
- **Augmentation**: 5-6 paraphrases per scenario
- **Train/Val split**: 85% / 15%
- **Scenario diversity**: 216 unique scenario types

### Paraphrase Quality

Each scenario generates 6 training pairs with different phrasings:

| Style | Focus | Example |
|-------|-------|---------|
| Original | Factual | "ego vehicle travels straight at 8.0 m/s using follow_lane with 1 vehicle..." |
| P1 | High-level overview | "Design an intersection test with ego traveling straight at 8.0 m/s..." |
| P2 | Threat perspective | "...vehicle danger performing follow_lane..." |
| P3 | Detailed behavior | "ego: straight drive at 8.0 m/s with follow_lane. Add multiple actors..." |
| P4 | Actor-focused | "Create scenario with ego performing straight maneuvers at 8.0 m/s using follow_lane..." |
| P5 | Challenge-focused | "Create a challenging test scenario. The ego vehicle must navigate using follow_lane..." |

---

## 🚀 Phase 2: Fine-Tune on Colab (A100)

### Step 1: Upload Files to Colab

1. Open Google Colab: https://colab.research.google.com
2. Create new notebook or use the provided `qwen7b_json_finetune.ipynb`
3. Upgrade to **Colab Pro** (if available) to ensure A100 GPU
4. Upload `train_json.jsonl` and `val_json.jsonl` files

**Colab Notebook**: `qwen7b_json_finetune.ipynb`

### Step 2: Run Training

Execute the cells in order:

1. **Install dependencies** (~2 min)
   - Installs: transformers, peft, trl, bitsandbytes
2. **Upload training data** (~1 min)
3. **Load and prepare datasets** (~1 min)
4. **Load model and configure LoRA** (~2 min)
5. **Configure training** (~1 min)
6. **Train model** (~2-3 hours on A100)
   - Monitor loss curves (should decrease smoothly)
   - Validation loss should plateau by epoch 3-4
7. **Save and download weights** (~5 min)
   - Creates `qwen7b_json_lora_final.zip`
   - Download via browser
8. **Test inference** (~1 min)
   - Verify model works on sample

### Training Hyperparameters

```
Model: Qwen/Qwen2.5-7B-Instruct
Quantization: 4-bit (nf4)
LoRA rank: 16
LoRA alpha: 32
Epochs: 4
Batch size: 2
Gradient accumulation: 4
Learning rate: 1e-4
Max tokens: 1024
```

### Expected Results

- **Training time**: 2-3 hours on A100
- **Final training loss**: 0.4-0.6 (depends on data)
- **LoRA weights size**: 50-100MB
- **Generated ZIP**: ~60-80MB

---

## 💻 Phase 3: Local Setup (RTX 3070)

### Step 1: Download and Extract Weights

1. Download `qwen7b_json_lora_final.zip` from Colab
2. Extract to `simulation/json-finetune/` folder:
   ```bash
   cd simulation/json-finetune/
   unzip qwen7b_json_lora_final.zip
   ```
3. Verify structure:
   ```
   qwen7b_json_lora_final/
   ├── adapter_config.json
   ├── adapter_model.bin
   ├── training_args.bin
   └── README.md
   ```

### Step 2: Install Dependencies

```bash
# If not already installed
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
pip install transformers peft bitsandbytes accelerate
```

### Step 3: Generate Configs Locally

**Interactive mode** (recommended for testing):
```bash
cd simulation/json-finetune/
python local_inference_qwen.py --interactive
```

Prompts you for scenario descriptions and saves configs to `generated/` folder.

**Single scenario mode**:
```bash
cd simulation/json-finetune/
python local_inference_qwen.py \
  --scenario "Intersection with ego straight, vehicle approaching from left at high speed" \
  --output generated/scenario_001.json
```

**With custom temperature**:
```bash
cd simulation/json-finetune/
# More deterministic (0.5 = strict, follows patterns closely)
python local_inference_qwen.py --scenario "..." --temperature 0.5

# More creative (0.9 = loose, more variation)
python local_inference_qwen.py --scenario "..." --temperature 0.9
```

### Local Inference Performance

**Expected on RTX 3070 (8GB VRAM)**:
- VRAM usage: ~7-7.5GB (4-bit quantized)
- Inference time: 1-2 seconds per config
- JSON validity: >95%

**Monitor VRAM** during first run:
```bash
watch -n 1 nvidia-smi  # Monitor GPU usage in another terminal
```

---

## 🔄 Phase 4: Integration with Simulation

### Option A: Direct Script Integration

Modify your `carla_runner.py`:

```python
from local_inference_qwen import JSONConfigGenerator
import json

# Initialize once
generator = JSONConfigGenerator(lora_weights_path="./qwen7b_json_lora_final")

# In your main loop
user_scenario = input("Describe the scenario: ")
config = generator.generate_config(user_scenario)

if config:
    # Validate
    is_valid, issues = generator.validate_config(config)
    if is_valid:
        # Run simulation
        run_scenario(config)
    else:
        print(f"Invalid config: {issues}")
```

### Option B: API-Based Integration

Create a simple Flask API:

```python
from flask import Flask, request, jsonify
from local_inference_qwen import JSONConfigGenerator
import json

app = Flask(__name__)
generator = JSONConfigGenerator()

@app.route("/generate", methods=["POST"])
def generate():
    data = request.json
    scenario = data.get("scenario", "")
    
    config = generator.generate_config(scenario)
    is_valid, issues = generator.validate_config(config)
    
    return jsonify({
        "success": is_valid,
        "config": config,
        "issues": issues
    })

if __name__ == "__main__":
    app.run(host="localhost", port=5000)
```

Call from anywhere:
```bash
curl -X POST http://localhost:5000/generate \
  -H "Content-Type: application/json" \
  -d '{"scenario": "Intersection scenario with ego straight..."}'
```

---

## ✅ Quality Assurance

### Validation Checklist

- [ ] All JSON examples are valid (parser doesn't throw errors)
- [ ] Required fields present: map, scenario, ego, adversaries, environment, termination
- [ ] Spawn distances reasonable: 5-60m
- [ ] Speeds reasonable: 0-20 m/s
- [ ] Blueprint names exist in CARLA

### Testing Strategy

1. **Structural validation** (automatic)
   ```python
   is_valid, issues = generator.validate_config(config)
   ```

2. **Semantic validation** (manual spot-check)
   - Run 5-10 generated configs through actual CARLA simulation
   - Check: >5s episode duration, collision triggers work, no immediate crashes

3. **Metrics collection**
   - Success rate: % of configs that run without errors
   - Latency: avg time per config
   - Quality: manual assessment of realism

### Known Limitations

- **Multi-actor choreography**: Complex timing between 3+ actors may be imperfect
- **Edge cases**: Unusual spawn modes or behavior combinations may fail
- **Context awareness**: Model may not understand complex spatial relationships

---

## 🔧 Troubleshooting

### Issue: Out of memory on RTX 3070

**Solution**: Use smaller context or off-load to CPU:
```python
# In local_inference_qwen.py, modify device_map:
device_map = {"": "cpu"}  # CPU offloading
```

### Issue: JSON parsing errors

**Solution**: Increase max_new_tokens or adjust temperature:
```python
# Lower temperature = more deterministic = better JSON
config = generator.generate_config(scenario, temperature=0.5)
```

### Issue: Generated configs don't run in CARLA

**Solutions**:
1. Validate with `generator.validate_config()`
2. Check spawn modes against CARLA docs
3. Verify blueprint names in CARLA
4. Add failed cases to training data and re-fine-tune

### Issue: Colab session keeps disconnecting

**Solution**: 
- Upgrade to Colab Pro
- Add checkpoint saving every epoch
- Use `!tail -f /dev/null` to keep session alive

---

## 📊 Metrics & Learning Curves

After training, check Colab logs for:

1. **Training Loss**: Should decrease smoothly
   - Bad: Noisy/plateauing early
   - Good: Steady decline, plateau by epoch 3-4

2. **Validation Loss**: Should decrease then plateau
   - Bad: Diverging from training loss (overfitting)
   - Good: Following training loss closely

3. **Perplexity**: Lower is better
   - Target: <2.0 on validation set

---

## 📈 Advanced: Progressive Improvement

### Collect Failures

As you generate configs, log failures:

```python
failures = []
for scenario in test_scenarios:
    config = generator.generate_config(scenario)
    if not config or config not in simulation_runners:
        failures.append({
            "scenario": scenario,
            "config": config,
            "reason": "invalid_json"  # or run_failed, etc
        })

# Save for re-training
with open("failures.jsonl", "w") as f:
    for item in failures:
        f.write(json.dumps(item) + "\n")
```

### Re-Fine-Tune

After collecting failures:

1. Convert failures to training format:
   ```python
   def make_pair(description, config):
       return {
           "messages": [
               {"role": "system", "content": SYSTEM_PROMPT},
               {"role": "user", "content": description},
               {"role": "assistant", "content": json.dumps(config)}
           ]
       }
   ```

2. Append to `train_json.jsonl` and `val_json.jsonl`

3. Run quick re-training (1-2 epochs):
   ```python
   training_args.num_train_epochs = 2
   trainer.train()
   ```

4. Deploy new adapter weights

---

## 🎯 Success Criteria

Your setup is working when:

✅ **Data Preparation**: 1,464 training pairs generated  
✅ **Colab Training**: Model trains without OOM errors, loss decreases  
✅ **Download**: LoRA weights ~50-100MB download successful  
✅ **Local Inference**: Generate 10 configs in <20 seconds total  
✅ **Validation**: >95% of configs pass JSON schema validation  
✅ **Simulation**: >80% of generated configs run 10+ seconds in CARLA without crash  

---

## 📚 File Reference

Located in `simulation/json-finetune/`:

| File | Purpose | Size |
|------|---------|------|
| `train_json.jsonl` | Training data | 3.0M |
| `val_json.jsonl` | Validation data | 530K |
| `split_report_json.txt` | Dataset report | 8.7K |
| `DATASET_README.md` | Dataset documentation | 5KB |
| `prepare_json_finetune.py` | Data prep script | 8KB |
| `qwen7b_json_finetune.ipynb` | Colab notebook | 10KB |
| `local_inference_qwen.py` | Local inference | 10KB |
| `qwen7b_json_lora_final/` | LoRA weights | 50-100M |
| `qwen7b_json_lora_final.zip` | Weights archive | 60-80M |

---

## 🚀 Next Steps

1. [ ] Navigate to `simulation/json-finetune/` folder
2. [ ] Upload `train_json.jsonl` and `val_json.jsonl` to Colab
3. [ ] Upload/run `qwen7b_json_finetune.ipynb` on Colab Pro
4. [ ] Monitor training curves
5. [ ] Download `qwen7b_json_lora_final.zip`
6. [ ] Extract weights in `simulation/json-finetune/`
7. [ ] Run `python local_inference_qwen.py --interactive`
8. [ ] Integrate with CARLA simulation runner
9. [ ] Collect and validate 20 generated configs
10. [ ] Consider re-fine-tuning if quality <80%

---

## 💡 Tips & Best Practices

**Prompt Engineering**:
- Be specific: "intersection scenario" > "traffic scenario"
- Include speeds: "8.0 m/s" > "slow"
- Use terminology: "danger" for adversary, "humans" for pedestrians
- Describe positions: "from left", "ahead", "behind"

**Temperature Selection**:
- 0.5-0.6: Strict adherence to patterns (better for deterministic scenarios)
- 0.7-0.8: Balanced (recommended default)
- 0.9+: More variation (can lead to invalid JSON)

**Performance Tuning**:
- Batch multiple queries if  possible
- Cache model in memory if running repeatedly
- Use quantization (already done) to fit in 8GB

---

**Need help?** Check DATASET_README.md for dataset info or review troubleshooting section above.

Good luck! 🎯
