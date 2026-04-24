"""
CARLA Adversarial Scenario Runner
==================================

Entry point for running adversarial driving scenarios in CARLA.

This module provides a clean command-line interface for executing scenario
configurations. The actual simulation logic is delegated to the `runner` package
which implements OOP principles for modularity and extensibility.

Supported scenarios include:
  - Junction crossing conflicts
  - Cut-in maneuvers
  - Intersection collisions
  - And more...

Usage:
    python3 carla_runner.py --config config/scenario_intersection_conflict.json
    python carla_runner.py --config config/scenario_cut_in.json --host localhost --port 2000

Example config.json:
    {
        "map": "Town05",
        "scenario": {
            "type": "intersection_conflict",
            "ego_maneuver": "straight",
            "adversary_maneuver": "left_turn"
        },
        ...
    }
"""

import argparse
import sys

from runner import run


def main():
    """
    Main entry point for the CARLA adversarial scenario runner.
    
    Parses command-line arguments and delegates to the runner module.
    """
    parser = argparse.ArgumentParser(
        description="Run adversarial driving scenarios in CARLA",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --config config/scenario_intersection_conflict.json
  %(prog)s --config config/scenario_cut_in.json --host localhost --port 2000
        """
    )
    
    parser.add_argument(
        "--config",
        required=True,
        type=str,
        help="Path to the scenario configuration JSON file (required)"
    )
    
    parser.add_argument(
        "--host",
        default="localhost",
        type=str,
        help="CARLA server hostname (default: localhost)"
    )
    
    parser.add_argument(
        "--port",
        default=2000,
        type=int,
        help="CARLA server port (default: 2000)"
    )
    
    args = parser.parse_args()
    
    try:
        run(args.config, host=args.host, port=args.port)
    except KeyboardInterrupt:
        print("\n[Interrupted] User terminated the simulation.")
        sys.exit(0)
    except FileNotFoundError as e:
        print(f"[Error] Configuration file not found: {args.config}")
        sys.exit(1)
    except ConnectionError as e:
        print(f"[Error] Failed to connect to CARLA server at {args.host}:{args.port}")
        print("       Make sure CARLA is running: ./CarlaUE4.sh")
        sys.exit(1)
    except Exception as e:
        print(f"[Error] An unexpected error occurred: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
