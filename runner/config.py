"""
Scenario configuration loader.
"""

import json


def load_config(path: str) -> dict:
    with open(path) as f:
        return json.load(f)
