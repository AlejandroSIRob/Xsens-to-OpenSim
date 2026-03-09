import sys
import os

# Add the parent directory to the path so we can import src
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src import xsens_parser
from src import opensim_pipeline

def main():
    config_path = os.path.join(os.path.dirname(__file__), "config.yaml")

    print("\n--- 1. Parsing and Aligning Xsens Data ---")
    success = xsens_parser.parse_and_align_xsens_data(config_path)
    
    if not success:
        print("Failed to parse Xsens data. Aborting pipeline.")
        return

    print("\n--- 2. Running OpenSim IMU Placer ---")
    calibrated_model = opensim_pipeline.run_imu_placer(config_path)
    
    if not calibrated_model:
        print("Failed to run IMU Placer. Aborting pipeline.")
        return

    print("\n--- 3. Running OpenSim Inverse Kinematics ---")
    opensim_pipeline.run_inverse_kinematics(config_path, calibrated_model)

if __name__ == "__main__":
    if not os.path.exists("windows/config.yaml"):
        print("Please run this script from the root of the repository: python windows\\run_pipeline.py")
        sys.exit(1)
        
    main()
