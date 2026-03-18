import json

typo_count = 0
with open("train.jsonl") as f:
    for line in f:
        if "maneurver" in json.loads(line)["messages"][2]["content"]:
            typo_count += 1

print(f"'maneurver' typo in training data: {typo_count}")