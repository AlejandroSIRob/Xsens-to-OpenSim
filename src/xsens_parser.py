import pandas as pd
import numpy as np
from scipy.spatial.transform import Rotation as R
import os
import glob
import json
import yaml

def load_config(config_path):
    """Loads configuration from a YAML file."""
    with open(config_path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)

def load_sensor_mapping(mapping_path):
    """Loads IMU ID to OpenSim body name mapping from a JSON file."""
    with open(mapping_path, 'r', encoding='utf-8') as f:
        return json.load(f)

def parse_and_align_xsens_data(config_path):
    """Parses Xsens IMU text files and converts them to OpenSim STO format."""
    try:
        config = load_config(config_path)
        return parse_config_dict(config)
    except Exception as e:
        print(f"[ERROR] Failed to load config or mapping: {e}")
        return False

def parse_config_dict(config):
    """
    Parses Xsens IMU text files from a loaded configuration dictionary.
    Assumes input files have already been processed by data_utils.fix_packet_counter
    to have continuous packet counters.
    
    Alignment strategy:
    - Each sensor's first packet is considered time 0 for that sensor
    - All sensors are aligned to the base IMU (pelvis_imu) timeline
    - Uses packet counters to maintain precise synchronization
    """
    try:
        paths = config['paths']
        settings = config['settings']
        
        # IMPORTANT: Use the folder with already corrected files
        input_folder = paths['input_folder']  # This should be the "_corrected" folder
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

    # --- STEP 1: IDENTIFY BASE SENSOR (pelvis_imu) ---
    base_sensor_id = None
    base_sensor_name = "pelvis_imu"
    
    for sensor_id, body_name in sensor_mapping.items():
        if body_name == base_sensor_name:
            base_sensor_id = sensor_id
            break
    
    if base_sensor_id is None:
        print(f"[ERROR] No '{base_sensor_name}' found in sensor mapping. Cannot align times.")
        return False
    
    print(f"Base sensor: {base_sensor_name} (ID: {base_sensor_id})")
    
    # --- STEP 2: LOAD AND PROCESS EACH SENSOR ---
    sensor_data = {}  # Dictionary to store data for each sensor
    base_packet_range = None
    
    print("Loading sensor data...")
    for filepath in files:
        filename = os.path.basename(filepath)
        sensor_id = next((sid for sid in sensor_mapping.keys() if sid in filename), None)
        
        if not sensor_id:
            continue
            
        body_name = sensor_mapping[sensor_id]
        print(f"  -> Loading {body_name} (ID: {sensor_id})...")
        
        # Detect start of data
        with open(filepath, 'r', encoding='utf-8') as f:
            try:
                skip_rows = next(i for i, line in enumerate(f) if 'PacketCounter' in line)
            except StopIteration:
                print(f"     Warning: 'PacketCounter' not found in {filename}, skipping.")
                continue
        
        # Load complete data
        df = pd.read_csv(filepath, sep='\t', skiprows=skip_rows)
        
        # Clean and sort by PacketCounter
        df = df.drop_duplicates(subset=['PacketCounter']).sort_values('PacketCounter')
        
        # Save sensor information
        sensor_data[sensor_id] = {
            'body_name': body_name,
            'dataframe': df,
            'first_packet': df['PacketCounter'].iloc[0],
            'last_packet': df['PacketCounter'].iloc[-1]
        }
        
        # If it's the base sensor, save its packet range
        if sensor_id == base_sensor_id:
            base_packet_range = (df['PacketCounter'].iloc[0], df['PacketCounter'].iloc[-1])
            print(f"     Base sensor packet range: {base_packet_range[0]} - {base_packet_range[1]}")
    
    if not sensor_data:
        print("[ERROR] No valid sensor data files found.")
        return False
    
    if base_packet_range is None:
        print("[ERROR] Base sensor data not found.")
        return False
    
    # --- STEP 3: ALIGN ALL SENSORS TO BASE SENSOR TIME ---
    print("\nAligning sensors to base sensor timeline...")
    
    aligned_data = pd.DataFrame()
    
    for sensor_id, data in sensor_data.items():
        body_name = data['body_name']
        df = data['dataframe'].copy()
        
        # Calculate time for this sensor: time = (packet - first_packet) / freq
        # This makes the first packet of EACH sensor be time 0 for that sensor
        df['time_local'] = (df['PacketCounter'] - data['first_packet']) / target_freq
        
        # For the base sensor, we also calculate its local time
        if sensor_id == base_sensor_id:
            df['time'] = df['time_local']
            df_base = df.set_index('time')
            df_base = df_base[~df_base.index.duplicated(keep='first')]
            df_base = df_base.sort_index()
            print(f"  Base sensor ({body_name}): {len(df_base)} frames, time range: {df_base.index[0]:.2f} - {df_base.index[-1]:.2f}s")
        else:
            # For non-base sensors, we need to align them to the base sensor time.
            # This means their time 0 (first packet) must correspond to the
            # base sensor time at which that packet occurred.
            
            # Calculate the packet offset between this sensor and the base one
            # The base time for the first packet of this sensor is:
            # t_base_at_first = (data['first_packet'] - base_packet_range[0]) / target_freq
            packet_offset = data['first_packet'] - base_packet_range[0]
            time_offset = packet_offset / target_freq
            
            # Apply offset to align with the base timeline
            df['time'] = df['time_local'] + time_offset
            
            print(f"  Sensor {body_name}: first packet at base time = {time_offset:.2f}s")
    
    # --- STEP 4: COMBINE ALL SENSORS ---
    print("\nCombining sensors...")
    
    for sensor_id, data in sensor_data.items():
        body_name = data['body_name']
        df = data['dataframe'].copy()
        
        # Recalculate time with the appropriate method for each sensor
        if sensor_id == base_sensor_id:
            df['time'] = (df['PacketCounter'] - data['first_packet']) / target_freq
        else:
            packet_offset = data['first_packet'] - base_packet_range[0]
            time_offset = packet_offset / target_freq
            df['time'] = (df['PacketCounter'] - data['first_packet']) / target_freq + time_offset
        
        # Set time as index
        df = df.set_index('time')
        df = df[~df.index.duplicated(keep='first')]
        df = df.sort_index()
        
        # Convert orientations to quaternions
        if 'Quat_q0' in df.columns:
            quats = df[['Quat_q0', 'Quat_q1', 'Quat_q2', 'Quat_q3']].values
        elif 'Roll' in df.columns:
            euler_angles = df[['Roll', 'Pitch', 'Yaw']].values
            r = R.from_euler('xyz', euler_angles, degrees=True)
            q_scipy = r.as_quat()  # x,y,z,w
            quats = np.column_stack((q_scipy[:, 3], q_scipy[:, 0], q_scipy[:, 1], q_scipy[:, 2]))  # w,x,y,z
        else:
            print(f"  Warning: No orientation columns found for {body_name}, skipping.")
            continue

        # Format as "w,x,y,z" strings
        quat_strings = [f"{q[0]:.6f},{q[1]:.6f},{q[2]:.6f},{q[3]:.6f}" for q in quats]
        df_col = pd.DataFrame(data=quat_strings, index=df.index, columns=[body_name])
        
        # Join to main dataframe
        if aligned_data.empty:
            aligned_data = df_col
            print(f"  Added {body_name}: {len(df_col)} frames")
        else:
            # Align by time index using merge_asof
            aligned_data = aligned_data.sort_index()
            df_col = df_col.sort_index()
            
            # Find the common time range
            common_start = max(aligned_data.index[0], df_col.index[0])
            common_end = min(aligned_data.index[-1], df_col.index[-1])
            
            if common_start < common_end:
                # Filter to common range before merging
                aligned_data = aligned_data[aligned_data.index >= common_start]
                df_col = df_col[df_col.index >= common_start]
                
                aligned_data = pd.merge_asof(
                    aligned_data, df_col, 
                    left_index=True, right_index=True,
                    tolerance=0.02, direction='nearest'
                )
                print(f"  Added {body_name}: common time range {common_start:.2f} - {common_end:.2f}s")
            else:
                print(f"  Warning: No time overlap for {body_name}, skipping")
    
    # --- STEP 5: FINAL CLEANUP AND SAVING ---
    if not aligned_data.empty:
        # Drop rows with NaN
        aligned_data = aligned_data.dropna()
        
        # Verify we have all expected sensors
        expected_sensors = list(sensor_mapping.values())
        missing_sensors = [s for s in expected_sensors if s not in aligned_data.columns]
        if missing_sensors:
            print(f"\nWarning: Missing sensors in final data: {missing_sensors}")
        
        # Create output directory if it doesn't exist
        if not os.path.exists(output_folder):
            os.makedirs(output_folder)
            
        full_path = os.path.join(output_folder, output_filename)
        
        # Save STO file
        with open(full_path, 'w', encoding='utf-8') as f:
            f.write(f"DataRate={target_freq}\n")
            f.write("DataType=Quaternion\n")
            f.write("version=3\n")
            f.write("OpenSimVersion=4.4\n")
            f.write("endheader\n")
            f.write("time\t" + "\t".join(aligned_data.columns) + "\n")
            
            for t, row in aligned_data.iterrows():
                f.write(f"{t:.4f}\t" + "\t".join(row.values) + "\n")
        
        # Final statistics
        duration = aligned_data.index[-1] - aligned_data.index[0]
        actual_freq = len(aligned_data) / duration if duration > 0 else 0
        
        print(f"\n{'='*60}")
        print(f"SUCCESS! STO file generated at: {full_path}")
        print(f"{'='*60}")
        print(f"  Total frames: {len(aligned_data)}")
        print(f"  Time range: {aligned_data.index[0]:.2f} - {aligned_data.index[-1]:.2f} seconds")
        print(f"  Duration: {duration:.2f} seconds")
        print(f"  Actual frequency: {actual_freq:.2f} Hz")
        print(f"  Sensors included: {', '.join(aligned_data.columns)}")
        print(f"{'='*60}")
        
        return True
    else:
        print("\n[ERROR] Could not consolidate data.")
        return False
