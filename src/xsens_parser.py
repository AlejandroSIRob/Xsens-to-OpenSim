import pandas as pd
import numpy as np
from scipy.spatial.transform import Rotation as R
import os
import glob
import json
import yaml

def load_config(config_path):
    """Loads configuration from a YAML file."""
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)

def load_sensor_mapping(mapping_path):
    """Loads IMU ID to OpenSim body name mapping from a JSON file."""
    with open(mapping_path, 'r') as f:
        return json.load(f)

def parse_and_align_xsens_data(config_path):
    """Parses Xsens IMU text files and converts them to OpenSim STO format."""
    try:
        config = load_config(config_path)
        paths = config['paths']
        settings = config['settings']
        
        input_folder = paths['input_folder']
        output_folder = paths['output_folder']
        output_filename = paths['output_filename']
        target_freq = settings['sampling_rate']
        
        sensor_mapping = load_sensor_mapping(paths['mapping_file'])
    except Exception as e:
        print(f"[ERROR] Failed to load config or mapping: {e}")
        return False

    pattern = os.path.join(input_folder, "MT_*.txt")
    files = glob.glob(pattern)
    
    if not files:
        print(f"No files found in: {input_folder}")
        return False

    # --- STEP: FIND GLOBAL START ---
    print("Synchronizing sensors...")
    global_start_packet = float('inf')
    valid_files_to_process = []

    for filepath in files:
        filename = os.path.basename(filepath)
        sensor_id = next((sid for sid in sensor_mapping.keys() if sid in filename), None)
        
        if sensor_id:
            with open(filepath, 'r') as f:
                try:
                    skip = next(i for i, line in enumerate(f) if 'PacketCounter' in line)
                    temp_df = pd.read_csv(filepath, sep='\t', skiprows=skip, usecols=['PacketCounter'], nrows=5)
                    min_p = temp_df['PacketCounter'].min()
                    if min_p < global_start_packet:
                        global_start_packet = min_p
                    valid_files_to_process.append((filepath, sensor_mapping[sensor_id], skip))
                except StopIteration:
                    print(f" Warning: 'PacketCounter' not found in {filename}")

    if not valid_files_to_process:
        print("[ERROR] No files match the JSON mapping.")
        return False

    merged_data = pd.DataFrame()
    print(f"Processing {len(valid_files_to_process)} synchronized sensors from packet {global_start_packet}...")

    for filepath, col_name, skip_rows in valid_files_to_process:
        print(f"-> Reading {col_name}...")
        df = pd.read_csv(filepath, sep='\t', skiprows=skip_rows)
        
        # Clean and sort by packets, remove duplicates
        df = df.drop_duplicates(subset=['PacketCounter']).sort_values('PacketCounter')
        
        # Calculate time relative to global start
        df['time'] = (df['PacketCounter'] - global_start_packet) / target_freq
        
        # Critical fix from Windows logic: Handle index correctly
        df = df.set_index('time')
        df = df[~df.index.duplicated(keep='first')]
        df = df.sort_index()
        
        # Conversion to Quaternions
        if 'Quat_q0' in df.columns:
            quats = df[['Quat_q0', 'Quat_q1', 'Quat_q2', 'Quat_q3']].values
        elif 'Roll' in df.columns:
            euler_angles = df[['Roll', 'Pitch', 'Yaw']].values
            r = R.from_euler('xyz', euler_angles, degrees=True)
            q_scipy = r.as_quat() # x,y,z,w
            quats = np.column_stack((q_scipy[:, 3], q_scipy[:, 0], q_scipy[:, 1], q_scipy[:, 2])) # w,x,y,z
        else:
            continue

        quat_strings = [f"{q[0]:.6f},{q[1]:.6f},{q[2]:.6f},{q[3]:.6f}" for q in quats]
        df_col = pd.DataFrame(data=quat_strings, index=df.index, columns=[col_name])
        
        if merged_data.empty:
            merged_data = df_col
        else:
            merged_data = merged_data.sort_index()
            df_col = df_col.sort_index()
            merged_data = pd.merge_asof(merged_data, df_col, left_index=True, right_index=True, 
                                        tolerance=0.02, direction='nearest')

    # --- FINAL SAVE ---
    if not merged_data.empty:
        merged_data = merged_data.dropna()
        if not os.path.exists(output_folder):
            os.makedirs(output_folder)
            
        full_path = os.path.join(output_folder, output_filename)
        
        with open(full_path, 'w') as f:
            f.write(f"DataRate={target_freq}\nDataType=Quaternion\nversion=3\nOpenSimVersion=4.4\nendheader\n")
            f.write("time\t" + "\t".join(merged_data.columns) + "\n")
            for t, row in merged_data.iterrows():
                f.write(f"{t:.4f}\t" + "\t".join(row.values) + "\n")
                
        print(f"\nSUCCESS! STO file generated at: {full_path}")
        return True
    else:
        print("\n[ERROR] Could not consolidate data.")
        return False
