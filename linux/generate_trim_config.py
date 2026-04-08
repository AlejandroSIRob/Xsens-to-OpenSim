#!/usr/bin/env python3
"""
Script to automatically generate a trim configuration from a folder.
Run from the repository root: python linux/generate_trim_config.py /path/to/folder
"""

import sys
import os
import argparse

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.ik_trimmer import generate_config_from_folder

def main():
    parser = argparse.ArgumentParser(
        description='Generate trim configuration from a folder of .mot files',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python linux/generate_trim_config.py /path/to/folder --output config.json
  python linux/generate_trim_config.py /path/to/folder --start 0.5 --end 2.5 --pattern "ik_*.mot"
        """
    )
    parser.add_argument('folder', type=str, help='Folder containing .mot files')
    parser.add_argument('--output', '-o', type=str, default='trim_config.json',
                       help='Output JSON file path')
    parser.add_argument('--start', '-s', type=float, default=0.0,
                       help='Default start time (seconds)')
    parser.add_argument('--end', '-e', type=float, default=1.0,
                       help='Default end time (seconds)')
    parser.add_argument('--pattern', '-p', type=str, default='*.mot',
                       help='File search pattern')
    
    args = parser.parse_args()
    
    # Verify the folder exists
    if not os.path.exists(args.folder):
        print(f"Error: Folder {args.folder} does not exist")
        sys.exit(1)
    
    result = generate_config_from_folder(
        folder_path=args.folder,
        start_time=args.start,
        end_time=args.end,
        pattern=args.pattern,
        output_json=args.output
    )
    
    if result:
        print(f"\nConfiguration saved to: {result}")
    else:
        print("\nError generating configuration")
        sys.exit(1)

if __name__ == "__main__":
    main()
