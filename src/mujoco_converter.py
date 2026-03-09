import os
import shutil
import yaml

def load_config(config_path):
    """Loads configuration from a YAML file."""
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)

def run_mujoco_conversion(config_path, output_mujoco_dir="mujoco_model"):
    """
    Converts an OpenSim model to MuJoCo format using myoconverter.
    """
    try:
        from myoconverter.O2MPipeline import O2MPipeline
    except ImportError:
        print("ERROR: myoconverter module is not installed.")
        print("Please install it using: pip install git+https://github.com/gabrinovas/myoconverter.git")
        return False

    config = load_config(config_path)
    paths = config['paths']
    
    work_dir = os.getcwd()
    osim_path = os.path.join(work_dir, paths['model_path'])
    geometry_folder = os.path.join(work_dir, paths['geometry_path'])
    
    output_folder = os.path.join(work_dir, output_mujoco_dir)

    print(f"\n--- 4. Converting to MuJoCo: {os.path.basename(osim_path)} ---")

    if not os.path.exists(osim_path):
        print(f"ERROR: OpenSim model not found at {osim_path}")
        return False

    if os.path.exists(output_folder):
        print(f"Cleaning existing MuJoCo output directory: {output_folder}")
        shutil.rmtree(output_folder)
    
    os.makedirs(output_folder)

    # Execute kinematics step (bones and joints)
    print("Running O2MPipeline conversion (Kinematics only)...")
    try:
        converter = O2MPipeline(osim_path, 
                                geometry_folder, 
                                output_folder, 
                                steps=[1],
                                force_all=True,
                                skip_plotting=True)
        print(f"✓ Conversion completed. MuJoCo model saved to: {output_folder}")
        return True
    except Exception as e:
        print(f"ERROR during MuJoCo conversion: {e}")
        return False
