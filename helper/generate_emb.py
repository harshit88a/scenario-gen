import os
import glob
import json
import re
from sentence_transformers import SentenceTransformer

SCENIC_FOLDER = "./../scenic-example"  # Path to folder with .scenic files
OUTPUT_FILE = "./../scenario_database.json"
MODEL_NAME = "all-MiniLM-L6-v2"


def extract_scenario_description(file_path):
    """
    Extracts the multi-line scenario description starting at '# Scenario:' 
    and stops before section headers such as '# 1. MAP ...'
    """
    description_lines = []
    capture_mode = False

    section_header_pattern = re.compile(r"#+\s*\d+[\.\)]")  
    # Matches: # 1. Text, ## 2) Text, ### 3. etc.

    try:
        with open(file_path, "r", encoding="utf-8") as f:
            for line in f:
                stripped = line.strip()

                # Detect start of scenario block
                if "# Scenario:" in line:
                    capture_mode = True
                    clean_line = line.split("# Scenario:", 1)[1].strip()
                    if clean_line:
                        description_lines.append(clean_line)
                    continue

                if capture_mode:
                    # Stop if next comment line is a numbered section
                    if stripped.startswith("#") and section_header_pattern.match(stripped):
                        break

                    # Continue capturing only normal comment description lines
                    if stripped.startswith("#"):
                        clean_line = stripped.lstrip("#").strip()
                        if clean_line:
                            description_lines.append(clean_line)
                        continue

                    # Stop on first non-comment non-empty line
                    if stripped != "":
                        break

        return " ".join(description_lines).strip()

    except Exception as e:
        print(f"Error reading {file_path}: {e}")
        return None


def main():
    print(f"Loading embedding model: {MODEL_NAME}...")
    model = SentenceTransformer(MODEL_NAME)

    # Find all scenic files
    search_path = os.path.join(SCENIC_FOLDER, "*.scenic")
    scenic_files = glob.glob(search_path)

    print(f"Found {len(scenic_files)} scenic files.")

    database = []

    for file_path in scenic_files:
        print(f"Processing: {os.path.basename(file_path)}")

        description = extract_scenario_description(file_path)

        if description:
            embedding_vector = model.encode(description).tolist()

            entry = {
                "scenario": description,
                "code": os.path.relpath(file_path, start="."),
                "embedding": embedding_vector
            }

            database.append(entry)
        else:
            print(f"Warning: No 'Scenario:' description found in {file_path}")

    with open(OUTPUT_FILE, "w", encoding="utf-8") as f:
        json.dump(database, f, indent=4)

    print(f"\nSuccess! Database saved to {OUTPUT_FILE} with {len(database)} entries.")


if __name__ == "__main__":
    main()
