#!/usr/bin/env python3
"""
Entry script to trim inverse kinematics files (Windows).
Run from the repository root: python windows\\trim_ik.py [options]
"""

import sys
import os

# Add the parent directory to the path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.ik_trimmer import main

if __name__ == "__main__":
    # Verify we are in the correct directory
    if not os.path.exists("windows/config.yaml"):
        print("Warning: windows/config.yaml not found")
        print("Make sure to run this script from the root of the repository")
    
    main()
