#!/usr/bin/env python3
"""
run_ui.py

Entry point for the RPO TestBed GUI.

Usage:
    python run_ui.py

Requires:
    - Python 3.10+
    - PIL/Pillow for logo display
    - matplotlib for plot embedding
    - Same API credentials as the CLI scenarios (via credential_helper.py)
"""

# Set matplotlib backend BEFORE any other imports to avoid Tk threading conflicts
import matplotlib
matplotlib.use('Agg')

import os
import sys

# Add examples repo root for credential_helper import
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
EXAMPLES_ROOT = os.path.dirname(SCRIPT_DIR)
if EXAMPLES_ROOT not in sys.path:
    sys.path.insert(0, EXAMPLES_ROOT)

# Add RPO_TestBed root to path for ui package import
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

# Add scenarios directory for play module imports
SCENARIOS_DIR = os.path.join(SCRIPT_DIR, "scenarios")
if SCENARIOS_DIR not in sys.path:
    sys.path.insert(0, SCENARIOS_DIR)


def main():
    """Launch the RPO TestBed UI."""
    try:
        from ui.app import main as app_main
        app_main()
    except ImportError as e:
        print(f"Error importing UI modules: {e}")
        print("\nMake sure you have the required dependencies installed:")
        print("  pip install pillow matplotlib")
        sys.exit(1)


if __name__ == "__main__":
    main()
