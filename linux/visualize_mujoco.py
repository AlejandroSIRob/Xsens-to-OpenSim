#!/usr/bin/env python3
"""
Script to visualize model trajectories in MuJoCo (Linux).
Run from the repository root: python linux/visualize_mujoco.py
"""

import sys
import os
import yaml
import numpy as np
import pandas as pd
import time
import glob

# Add the parent directory to the path to import repository modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def load_config(config_path):
    """Loads configuration from a YAML file."""
    with open(config_path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)

def find_mujoco_model(output_dir, config=None):
    """
    Searches for the MuJoCo model in the output directory.
    If config has 'mujoco_output_folder', that location is checked first.
    """
    search_paths = []
    
    # If we have config with a specific MuJoCo folder
    if config and 'paths' in config and 'mujoco_output_folder' in config['paths']:
        mujoco_path = config['paths']['mujoco_output_folder']
        
        # If it's an absolute path, use it directly
        if os.path.isabs(mujoco_path):
            search_paths.append(os.path.join(mujoco_path, "*.xml"))
        else:
            # If relative, combine with working directory
            mujoco_dir = os.path.join(os.getcwd(), mujoco_path)
            search_paths.append(os.path.join(mujoco_dir, "*.xml"))
    
    # Add default paths
    search_paths.extend([
        os.path.join(output_dir, "mujoco_model", "*.xml"),
        os.path.join(output_dir, "*.xml"),
        os.path.join(os.getcwd(), "mujoco_model", "*.xml")
    ])
    
    for pattern in search_paths:
        files = glob.glob(pattern)
        if files:
            return files[0]
    
    return None

def find_motion_file(output_dir):
    """
    Searches for the motion file (.mot) in the output directory.
    """
    search_patterns = [
        os.path.join(output_dir, "*.mot"),
        os.path.join(output_dir, "ik_*.mot"),
        os.path.join(output_dir, "*_ik.mot")
    ]
    
    for pattern in search_patterns:
        files = glob.glob(pattern)
        if files:
            return files[0]
    
    return None

def visualize_mujoco_trajectory(config_path, mjcf_path=None, data_path=None, 
                               speed_factor=1.0, start_time=0.0, pelvis_rotation_offset=0):
    """
    Visualizes model trajectories in MuJoCo.
    
    Args:
        config_path: Path to the configuration file
        mjcf_path: Path to MuJoCo MJCF model (if None, auto-discovered)
        data_path: Path to .mot data file (if None, auto-discovered)
        speed_factor: Speed factor (1.0 = normal speed)
        start_time: Start time in seconds
        pelvis_rotation_offset: Rotation offset for the pelvis (degrees)
    """
    try:
        import mujoco
        import mujoco.viewer
    except ImportError:
        print("ERROR: mujoco is not installed.")
        print("Install it with: pip install mujoco")
        return False
    
    # Load configuration
    config = load_config(config_path)
    paths = config['paths']
    settings = config['settings']
    sampling_rate = settings['sampling_rate']
    
    output_dir = os.path.join(os.getcwd(), paths['output_folder'])
    
    print(f"\n--- Visualizing trajectory in MuJoCo ---")
    
    # Determine file paths
    if mjcf_path is None:
        mjcf_path = find_mujoco_model(output_dir, config)
        if mjcf_path is None:
            print("ERROR: MuJoCo model not found.")
            return False
    
    if data_path is None:
        data_path = find_motion_file(output_dir)
        if data_path is None:
            print("ERROR: Motion file (.mot) not found.")
            return False
    
    print(f"MJCF model: {mjcf_path}")
    print(f"Data: {data_path}")
    print(f"Speed factor: {speed_factor}")
    
    # Verify files exist
    if not os.path.exists(mjcf_path):
        print(f"ERROR: Model not found at {mjcf_path}")
        return False
    
    if not os.path.exists(data_path):
        print(f"ERROR: Data not found at {data_path}")
        return False
    
    try:
        # Load MuJoCo model
        model = mujoco.MjModel.from_xml_path(mjcf_path)
        data = mujoco.MjData(model)
        
        # Load trajectory data
        df = pd.read_csv(data_path, sep='\t', skiprows=6)
        print(f"Data file loaded: {len(df)} frames")
        
        # Create automatic joint mapping
        mapping = {}
        joints_found = []
        for col in df.columns:
            col_clean = col.strip()
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, col_clean)
            if joint_id != -1:
                mapping[col_clean] = model.jnt_qposadr[joint_id]
                joints_found.append(col_clean)
        
        print(f"Joints found: {len(joints_found)}")
        
        # Get the time vector
        if 'time' in df.columns:
            times = df['time'].values
        else:
            times = np.arange(len(df)) / sampling_rate
        
        # Find start frame
        start_idx = 0
        for i, t in enumerate(times):
            if t >= start_time:
                start_idx = i
                break
        
        print(f"Starting at time: {times[start_idx]:.2f}s")
        print("\nPress Ctrl+C to stop")
        
        # Visualization loop - SIMPLE AND FAST VERSION
        with mujoco.viewer.launch_passive(model, data) as viewer:
            viewer.cam.distance = 4.0
            viewer.cam.azimuth = 90
            viewer.cam.elevation = -15
            
            for i in range(start_idx, len(df)):
                row = df.iloc[i]
                
                # Update joint positions
                for col_name, q_index in mapping.items():
                    if col_name in row.index:
                        val = row[col_name]
                        
                        # Special conversions
                        if col_name in ["pelvis_rotation", "pelvis_tilt", "pelvis_list"]:
                            if col_name == "pelvis_rotation":
                                val = np.deg2rad(val + pelvis_rotation_offset)
                            else:
                                val = np.deg2rad(val)
                        elif col_name in ["pelvis_tx", "pelvis_ty", "pelvis_tz"]:
                            # Translations - no conversion needed
                            pass
                        else:
                            # Other joints - convert to radians
                            val = np.deg2rad(val)
                        
                        data.qpos[q_index] = val
                
                # Advance simulation
                mujoco.mj_step(model, data)
                viewer.sync()
                
                # Simple speed control - 0.01s = ~100fps (original)
                time.sleep(0.01 / speed_factor)
                
                # Show progress every 100 frames
                if i % 100 == 0:
                    current_time = times[i] if i < len(times) else i/sampling_rate
                    print(f"Time: {current_time:.2f}s | Frame: {i}", end='\r')
        
        print("\nVisualization complete.")
        return True
        
    except KeyboardInterrupt:
        print("\nStopped by user.")
        return True
    except Exception as e:
        print(f"ERROR: {e}")
        return False

def main():
    """Main function."""
    config_path = os.path.join(os.path.dirname(__file__), "config.yaml")
    
    if not os.path.exists(config_path):
        print("ERROR: Run from the repository root: python linux/visualize_mujoco.py")
        sys.exit(1)
    
    import argparse
    parser = argparse.ArgumentParser(description='Visualize OpenSim IK trajectories in MuJoCo')
    parser.add_argument('--mjcf', type=str, help='Path to MJCF model')
    parser.add_argument('--data', type=str, help='Path to .mot file')
    parser.add_argument('--speed', type=float, default=1.0, help='Speed factor')
    parser.add_argument('--start', type=float, default=0.0, help='Start time')
    parser.add_argument('--offset', type=float, default=0, help='Pelvis offset (degrees)')
    
    args = parser.parse_args()
    
    success = visualize_mujoco_trajectory(
        config_path=config_path,
        mjcf_path=args.mjcf,
        data_path=args.data,
        speed_factor=args.speed,
        start_time=args.start,
        pelvis_rotation_offset=args.offset
    )
    
    if not success:
        sys.exit(1)

if __name__ == "__main__":
    main()
