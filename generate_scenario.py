import os
import json
import torch
import sys
from datetime import datetime
from google import genai
from sentence_transformers import SentenceTransformer, util

# ==========================================
# 1. CONFIGURATION
# ==========================================
# Debug Flag
DEBUG = True  # Set to True to save prompts

DB_FILES = {
    "behavior": "./helper/extracted_components/adv_behaviour.json",
    "geometry": "./helper/extracted_components/geometry.json",
    "rel_pos": "./helper/extracted_components/rel_pos.json"
}
CONFIG_FILE = "./helper/extracted_components/model_config.txt"
GENERATED_DIR = "generated"

# Models
EMBEDDING_MODEL = 'all-MiniLM-L6-v2'
DECOMPOSITION_MODEL = 'gemini-2.5-flash'
GENERATION_MODEL = 'gemini-3-pro-preview'

GEMINI_API_KEY = os.environ.get("GEMINI_API_KEY")
if not GEMINI_API_KEY:
    print("Error: GEMINI_API_KEY not found in environment variables.")
    sys.exit(1)
    
client = genai.Client(api_key=GEMINI_API_KEY)

# ==========================================
# 2. SCENARIO DECOMPOSITION (Gemini 1.5 Flash)
# ==========================================

def decompose_scenario(user_scenario):
    #Uses Gemini 1.5 Flash to split the scenario into 3 sub-parts.
    prompt = f"""
    Your task is to decompose full descriptions of safety-critical scenarios into:
    1. Behavior: Describe the behavior of the adversarial object (you should also indicate the type of the object like pedestrians, cars, cyclists, and motorcycles)
    2. Geometry: Specify the road condition where the scenario occurs (e.g., straight road, three-way intersection).
    3. Spawn Position: Indicate the initial relative position of the adversarial object to the ego vehicle.

    Example 1:
    Scenario: Ego drives straight; Leading car brakes suddenly.
    Output:
    {{
        "behavior": "The adversarial car suddenly brakes when the ego approaches",
        "geometry": "A straight road.",
        "spawn_position": "The adversarial car is directly in front of the ego vehicle."
    }}

    Example 2:
    Scenario: TThe ego vehicle attempts a right turn at a four-way intersection, and an adversarial pedestrian steps onto the road in front of it
    Output:
    {{
        "behavior": "The adversarial pedestrian deliberately steps onto the road right in front of the ego vehicle",
        "geometry": "Lanes for turning right on a four-way intersection.",
        "spawn_position": "The adversarial pedestrian is on the right front of the ego."
    }}

    Decompose this: "{user_scenario}"
    Return strictly as JSON.
    """
    
    response = client.models.generate_content(
        model=DECOMPOSITION_MODEL,
        contents=prompt,
        config={'response_mime_type': 'application/json'}
    )
    return json.loads(response.text)

# ==========================================
# 3. COMPONENT RETRIEVAL (Vector Search)
# ==========================================
def get_top_3_snippets(query, db_path, embed_model, device, top_k=3):
    # Searches a specific category JSON for the best matching Scenic snippet.
    with open(db_path, 'r') as f:
        db = json.load(f)
    
    # Ensure tensors are on the same device to prevent RuntimeErrors
    query_emb = embed_model.encode(query, convert_to_tensor=True, device=device)
    corpus_embs = torch.tensor([item['embedding'] for item in db]).to(device)
    
    # Cosine similarity search
    scores = util.cos_sim(query_emb, corpus_embs)[0]
    top_k = min(3, len(db))
    top_results = torch.topk(scores, k=top_k)
    
    results = []
    for idx in top_results.indices:
        entry = db[idx.item()]
        # Read the raw snippet code from the stored text file path
        # with open(entry['code'], 'r') as f_snippet:
        #     results.append((f_snippet.read(), entry['text']))
        results.append((entry['code'], entry['text']))
    return results

# ==========================================
# 4. FINAL CODE GENERATION (Gemini 2.5 Pro)
# ==========================================

def construct_prompt(user_input, retrieved_context, fixed_config):
    
    # helper to format the snippets for the prompt
    def format_examples(category_dict):
        text_block = ""
        for i, (code_path, desc) in enumerate(category_dict):
            with open(code_path, 'r') as f_snippet:
                code = f_snippet.read()
            text_block += f"#Example {i+1} (Description: {desc})\n{code}\n\n"
        return text_block

    # retrieved_context is expected to be a dict: 
    # {'behavior': [(code1, desc1), ...], 'geometry': [...], 'rel_pos': [...]}
    
    behavior_examples = format_examples(retrieved_context['behavior'])
    geometry_examples = format_examples(retrieved_context['geometry'])
    rel_pos_examples = format_examples(retrieved_context['rel_pos'])

    prompt = f"""
SYSTEM ROLE: You are an expert Autonomous Vehicle Simulation Engineer specializing in the Scenic 3.0.0 programming language and CARLA 0.9.15. Your task is to generate valid Scenic code for adversarial scenarios based on Scenario Description provided below.

CRITICAL RULES - YOU MUST FOLLOW THESE EXACTLY:
1. ONLY USE methods, classes, and blueprints that appear in the few-shot examples below.
2. NEVER invent or hallucinate any classes, methods, or blueprints that are not in the examples.
3. NEVER use deprecated Scenic syntax - only use patterns shown in the examples.
4. ONLY use CARLA blueprints that appear in the examples (e.g., vehicle.lincoln.mkz_2017, walker.pedestrian.0001).
5. ONLY use Scenic behaviors that appear in the examples (e.g., BehaviorType, Follow, FollowLaneBehavior).
6. Output MUST have exactly four labeled sections as shown below.

OUTPUT FORMAT - DO NOT DEVIATE:
# 1. MAP AND MODEL CONFIGURATION (fixed)
# 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# 3. ROAD GEOMETRY
# 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT

1. MAP AND MODEL CONFIGURATION
{fixed_config}

FEW-SHOT EXAMPLES FOR 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT:
{behavior_examples}

FEW-SHOT EXAMPLES FOR 3. ROAD GEOMETRY:
{geometry_examples}

FEW-SHOT EXAMPLES FOR 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT:
{rel_pos_examples}

IMPORTANT: Generate code that closely mimics the structure and patterns from the examples above. Do not introduce new methods or classes.

TASK: Generate the Scenic code for the following scenario description. Follow the four-section format exactly.

SCENARIO DESCRIPTION: {user_input}

Generate the Scenic code:
"""
    return prompt


def main():
    # 1. Setup Gemini Client and SentenceTransformer
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    embed_model = SentenceTransformer(EMBEDDING_MODEL, device=device)
    if not os.path.exists(GENERATED_DIR): os.makedirs(GENERATED_DIR)
    
    with open(CONFIG_FILE, 'r') as f:
        fixed_config = f.read()

    while True:
        # 2. Get user input
        user_input = input("\n>> Enter scenario (or 'exit'):").strip()
        if user_input.lower() == 'exit': break

        # 3. Decompose scernario
        print("Decomposing scenario components...")
        parts = decompose_scenario(user_input)
        
        # 4 Retrieve best snippets for each part
        print("Retrieving best code snippets...")
        selected_context = {}
        for key, db_name in DB_FILES.items():
            query_text = parts.get(key, "")
            selected_context[key] = get_top_3_snippets(query_text, db_name, embed_model, device)

        # print(selected_context["behavior"][0][0])

        # 5. Construct final prompt
        print("Synthesizing final Scenic code...")
        final_prompt = construct_prompt(user_input, selected_context, fixed_config)

        # 4. If debug, save the prompt for inspection
        if DEBUG:
            debug_path = os.path.join('prompt', f"{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt")
            with open(debug_path, 'w') as f_debug:
                f_debug.write(final_prompt)
            print(f"Debug: Prompt saved to {debug_path}")

        # 5. Generate final code with Gemini 2.5 Pro
        response = client.models.generate_content(model=GENERATION_MODEL, contents=final_prompt)
        generated_code = response.text.replace("```scenic", "").replace("```", "").strip()

        # 6. Save output
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_path = os.path.join(GENERATED_DIR, f"scenario_{timestamp}.scenic")
        with open(out_path, 'w') as f:
            f.write(generated_code)
        
        print(f"Success! Code saved to: {out_path}")

if __name__ == "__main__":
    main()