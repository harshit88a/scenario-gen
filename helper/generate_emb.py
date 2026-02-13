import os
import glob
import json
import re
from sentence_transformers import SentenceTransformer

# Paths
SCENIC_SOURCE = "./../scenic-example"
OUTPUT_BASE = "extracted_components"
MODEL_NAME = "all-MiniLM-L6-v2"

# Subfolders and Flexible Regex
# This regex looks for:
# 1. The Section Header (e.g., ## 2. ADV BEHAVIOR...)
# 2. Any amount of whitespace/newlines
# 3. A comment line starting with one or more # (The description)
# 4. All code until the next "## " header or end of file
CATEGORIES = {
    "adv_behaviour": r"## 2\. ADV BEHAVIOR OF THE SURROUNDING OBJECT.*?#+\s*(.*?)\n(.*?)(?=## 3|$)",
    "geometry": r"## 3\. GEOMETRY.*?#+\s*(.*?)\n(.*?)(?=## 4|$)",
    "rel_pos": r"## 4\. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT.*?#+\s*(.*?)\n(.*?)(?=$)"
}

def setup_folders():
    for cat in CATEGORIES.keys():
        os.makedirs(os.path.join(OUTPUT_BASE, cat), exist_ok=True)

def main():
    print(f"Loading model: {MODEL_NAME}...")
    model = SentenceTransformer(MODEL_NAME)
    
    setup_folders()
    
    scenic_files = glob.glob(os.path.join(SCENIC_SOURCE, "*.scenic"))
    print(f"Found {len(scenic_files)} files to process.")

    databases = {cat: [] for cat in CATEGORIES.keys()}

    for file_path in scenic_files:
        filename = os.path.basename(file_path)
        print(f"Processing: {filename}")
        
        with open(file_path, "r", encoding="utf-8") as f:
            content = f.read()

        for cat, pattern in CATEGORIES.items():
            # re.DOTALL allows '.' to match newlines
            # re.IGNORECASE for safety
            match = re.search(pattern, content, re.DOTALL | re.IGNORECASE)
            
            if match:
                description = match.group(1).strip()
                code_snippet = match.group(2).strip()
                
                # Create filename and save code
                snippet_filename = f"{os.path.splitext(filename)[0]}_{cat}.txt"
                snippet_path = os.path.join(OUTPUT_BASE, cat, snippet_filename)
                
                with open(snippet_path, "w", encoding="utf-8") as f_out:
                    f_out.write(code_snippet)
                
                # Generate embedding
                embedding = model.encode(description).tolist()
                
                databases[cat].append({
                    "text": description,
                    "code": "./helper/" + snippet_path,
                    "embedding": embedding
                })
            else:
                print(f"  [!] Missing or malformed section {cat} in {filename}")

    # Save the 3 JSON files
    for cat, data in databases.items():
        json_out = os.path.join(OUTPUT_BASE, f"{cat}.json")
        with open(json_out, "w", encoding="utf-8") as jf:
            json.dump(data, jf, indent=4)
        print(f"Saved database: {json_out}")

if __name__ == "__main__":
    main()