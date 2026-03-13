#!/usr/bin/env python3
"""
Scenic 3.x Syntax Checker
--------------------------
Checks Scenic files for syntax and compile errors WITHOUT requiring
CARLA, a map file, or any simulator to be running.

Usage:
    # Check a single file
    python check_scenic_syntax.py my_scenario.scenic

    # Check all .scenic files in a folder
    python check_scenic_syntax.py path/to/folder/

    # Check multiple files / glob patterns
    python check_scenic_syntax.py *.scenic

Requirements:
    pip install scenic

What it checks:
    - Scenic grammar / parse errors  (e.g. malformed 'new', 'param', 'behavior')
    - Scenic AST compile errors      (e.g. invalid specifiers, bad 'do'/'take' syntax)

What it does NOT check (needs CARLA + map file):
    - Whether 'network', 'ManeuverType', etc. resolve at runtime
    - Whether spawn positions satisfy 'require' constraints
    - Whether trajectory / lane lookups succeed
"""

import sys
import os
import glob

try:
    from scenic.syntax.parser import parse_string, ScenicParseError
    from scenic.syntax.compiler import compileScenicAST
except ImportError:
    print("ERROR: Scenic is not installed.")
    print("Install it with:  pip install scenic")
    sys.exit(1)


def check_file(path: str) -> tuple[bool, str]:
    """
    Parse and compile a Scenic file.
    Returns (ok: bool, message: str).
    """
    fname = os.path.basename(path)
    try:
        with open(path, "r", encoding="utf-8") as f:
            src = f.read()
    except OSError as e:
        return False, f"Cannot read file: {e}"

    try:
        tree = parse_string(src, mode="exec", filename=fname)
        compileScenicAST(tree, filename=fname)
        return True, "OK"
    except ScenicParseError as e:
        location = f"line {e.lineno}" if e.lineno else "unknown line"
        snippet = f"\n        {e.text.rstrip()}" if e.text else ""
        return False, f"{location}: {e.msg}{snippet}"
    except Exception as e:
        return False, f"{type(e).__name__}: {e}"


def collect_paths(args: list[str]) -> list[str]:
    paths = []
    for arg in args:
        if os.path.isdir(arg):
            paths.extend(sorted(glob.glob(os.path.join(arg, "*.scenic"))))
        else:
            expanded = sorted(glob.glob(arg))
            if expanded:
                paths.extend(expanded)
            else:
                paths.append(arg)   # let check_file report the missing file error
    return paths


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(0)

    paths = collect_paths(sys.argv[1:])
    if not paths:
        print("No .scenic files found for the given arguments.")
        sys.exit(1)

    passed = failed = 0
    for path in paths:
        ok, msg = check_file(path)
        tag = "PASS" if ok else "FAIL"
        fname = os.path.basename(path)
        if ok:
            print(f"  {tag}  {fname}")
            passed += 1
        else:
            print(f"  {tag}  {fname}")
            print(f"        -> {msg}")
            failed += 1

    total = passed + failed
    print(f"\n{'='*50}")
    print(f"  {passed}/{total} passed    {failed}/{total} failed")
    print(f"{'='*50}")
    sys.exit(0 if failed == 0 else 1)


if __name__ == "__main__":
    main()