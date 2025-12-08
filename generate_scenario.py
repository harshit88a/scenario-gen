import os
import sys
import json
import torch
from datetime import datetime
from google import genai
from sentence_transformers import SentenceTransformer, util

# ==========================================
# 1. CONFIGURATION
# ==========================================
# Debug Flag
DEBUG = False  # Set to True to save prompts

# Paths
JSON_DB_PATH = 'scenario_database.json'
GENERATED_DIR = 'generated'
PROMPT_DIR = 'prompt'

# Models
EMBEDDING_MODEL_NAME = 'all-MiniLM-L6-v2'
LLM_MODEL_NAME = 'gemini-2.5-pro'

# API Key (From Environment Variable)
GEMINI_API_KEY = os.environ.get("GEMINI_API_KEY")

if not GEMINI_API_KEY:
    print("Error: GEMINI_API_KEY environment variable not found.")
    sys.exit(1)

# Initialize GenAI Client
client = genai.Client(api_key=GEMINI_API_KEY)

# ==========================================
# 2. RETRIEVAL LOGIC (Local Embeddings)
# ==========================================
def load_database(path):
    """Loads the JSON database containing scenarios and embeddings."""
    try:
        with open(path, 'r') as f:
            return json.load(f)
    except FileNotFoundError:
        print(f"Error: Database file '{path}' not found.")
        sys.exit(1)

def get_top_scenarios(query_text, db_data, model, device, top_k=3):
    """Retrieves top K similar scenarios using local embeddings."""
    # Encode the User Query
    query_embedding = model.encode(query_text, convert_to_tensor=True, device=device)

    # Extract Embeddings from JSON
    corpus_embeddings = [item['embedding'] for item in db_data]
    corpus_embeddings = torch.tensor(corpus_embeddings).to(device)

    # Calculate Cosine Similarity
    cos_scores = util.cos_sim(query_embedding, corpus_embeddings)[0]

    # Retrieve Top K Results
    top_results = torch.topk(cos_scores, k=min(top_k, len(db_data)))

    # Move to CPU for processing
    top_indices = top_results[1].cpu().numpy()
    top_scores = top_results[0].cpu().numpy()

    results = []
    print(f"\n2. Top {top_k} results with similarity score:")
    for score, idx in zip(top_scores, top_indices):
        entry = db_data[idx]
        print(f"   - Score: {score:.4f} | File: {entry['code']}")
        
        results.append({
            "score": float(score),
            "code_path": entry['code'],
            "description": entry['scenario']
        })
    
    return results

# ==========================================
# 3. PROMPT ENGINEERING
# ==========================================
def read_scenic_file(path):
    """Reads the raw content of a scenic file to use as a few-shot example."""
    try:
        # Get the directory where the script is running from
        script_dir = os.path.dirname(os.path.abspath(__file__))
        # Normalize the path: handle '../scenic-example/' or '../' prefixes
        normalized_path = path.replace('../scenic-example/', 'scenic-example/').replace('../', '')

        # Resolve the path relative to the script directory
        full_path = os.path.join(script_dir, normalized_path)
        full_path = os.path.abspath(full_path)  # Normalize the path

        if not os.path.exists(full_path):
            return f"# [SYSTEM ERROR: File not found at {full_path}]\n"

        with open(full_path, 'r') as f:
            return f.read()
    except Exception as e:
        return f"# [SYSTEM ERROR: Could not read file: {e}]\n"

def construct_prompt(user_scenario, similar_examples):
    """Builds the final prompt injecting the retrieved code as ground truth."""
    
    few_shot_context = ""
    for i, example in enumerate(similar_examples):
        code_content = read_scenic_file(example['code_path'])
        few_shot_context += f"\nExample {i+1} (Description: {example['description']}):\n```scenic\n{code_content}\n```\n"

    prompt = f"""
SYSTEM ROLE: You are an expert Autonomous Vehicle Simulation Engineer specializing in the Scenic 3.0.0 programming language and CARLA 0.9.15. Your task is to generate valid Scenic code for adversarial scenarios based on Scenario Description provided below.

CRITICAL RULES - YOU MUST FOLLOW THESE EXACTLY:
1. ONLY USE methods, classes, and blueprints that appear in the few-shot examples below.
2. NEVER invent or hallucinate any classes, methods, or blueprints that are not in the examples.
3. NEVER use deprecated Scenic syntax - only use patterns shown in the examples.
4. ONLY use CARLA blueprints that appear in the examples (e.g., vehicle.lincoln.mkz_2017, walker.pedestrian.0001).
5. ONLY use Scenic behaviors that appear in the examples (e.g., BehaviorType, Follow, FollowLaneBehavior).
6. All scenarios must use Town05 map.
7. Output MUST have exactly four labeled sections as shown below.

OUTPUT FORMAT - DO NOT DEVIATE:
# 1. MAP AND MODEL CONFIGURATION
# 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# 3. ROAD GEOMETRY
# 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT

FEW-SHOT EXAMPLES (SOURCE OF TRUTH - ONLY USE THESE):
{few_shot_context}

IMPORTANT: Generate code that closely mimics the structure and patterns from the examples above. Do not introduce new methods or classes.

TASK: Generate the Scenic code for the following scenario description. Follow the four-section format exactly.

SCENARIO DESCRIPTION: {user_scenario}

Generate the Scenic code:
"""
    return prompt

# ==========================================
# 4. GENERATION & SAVING
# ==========================================
def save_generated_code(code_text, timestamp):
    """Saves the LLM output to a .scenic file with timestamp as filename."""
    if not os.path.exists(GENERATED_DIR):
        os.makedirs(GENERATED_DIR)
    
    filename = f"{timestamp}.scenic"
    filepath = os.path.join(GENERATED_DIR, filename)

    # Clean up Markdown code blocks if LLM included them
    clean_code = code_text.replace("```scenic", "").replace("```", "").strip()

    with open(filepath, 'w') as f:
        f.write(clean_code)
    
    return filepath

def save_prompt(prompt_text, timestamp):
    """Saves the prompt to a .txt file if DEBUG is enabled."""
    if not DEBUG:
        return None
    
    if not os.path.exists(PROMPT_DIR):
        os.makedirs(PROMPT_DIR)
    
    filename = f"{timestamp}.txt"
    filepath = os.path.join(PROMPT_DIR, filename)

    with open(filepath, 'w') as f:
        f.write(prompt_text)
    
    return filepath

def main():
    # 1. Setup Retrieval Model
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    print(f"Initializing embedding model on {device}...")
    embed_model = SentenceTransformer(EMBEDDING_MODEL_NAME, device=device)
    
    # 2. Load Database
    db_data = load_database(JSON_DB_PATH)

    print("\n" + "="*50)
    print(" AV SCENARIO GENERATOR (RAG + Gemini 2.5)")
    print("="*50)

    while True:
        try:
            # 3. User Input
            user_input = input("\n>> Enter new adversarial scenario description under 200 words (or 'exit'): ").strip()
            if user_input.lower() in ['exit', 'quit']:
                break
            if not user_input:
                continue

            print(f"\n1. Processing scenario: \"{user_input}\"")

            # 4. Retrieve Context
            top_results = get_top_scenarios(user_input, db_data, embed_model, device)

            # 5. Construct Prompt
            final_prompt = construct_prompt(user_input, top_results)
            
            # Create timestamp for consistent naming
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            
            # Save prompt if DEBUG is enabled
            if DEBUG:
                prompt_path = save_prompt(final_prompt, timestamp)
                print(f"\n[DEBUG] Prompt saved to: {prompt_path}")

            # 6. Call LLM
            print("\n3. Getting response from LLM...")
            try:
                response = client.models.generate_content(
                    model=LLM_MODEL_NAME,
                    contents=final_prompt,
                    config=genai.types.GenerateContentConfig(
                        temperature=0.2  # Minimize randomness, prioritize consistency
                    )
                )
                
                generated_code = response.text
                
                # 7. Save Result
                saved_path = save_generated_code(generated_code, timestamp)
                print(f"\n4. Success! Scenic code saved to:\n   -> {saved_path}")

            except Exception as e:
                print(f"\n[Error] LLM Generation failed: {e}")

        except KeyboardInterrupt:
            print("\nExiting...")
            break

if __name__ == "__main__":
    main()