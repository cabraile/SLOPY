import os
import sys
from typing import Dict
import numpy as np
import open3d as o3d

from scipy.spatial.transform.rotation import Rotation
from slopy.odometry import Odometry

def split_transform(T) -> Dict[str, float]:
    roll, pitch, yaw = Rotation.from_matrix(T[:3,:3]).as_euler("xyz",degrees=False)
    x, y, z = T[:3,3]
    return {"x" : x, "y" : y, "z" : z, "roll" : roll, "pitch" : pitch, "yaw" : yaw}

def print_pose_dict(pose_dict : Dict[str,float]):
    print(f"> Position (m): ({pose_dict['x']},{pose_dict['y']},{pose_dict['z']})")
    print(f"> Euler angles (rad): ({pose_dict['roll']},{pose_dict['pitch']},{pose_dict['yaw']})")
    print(f"> Euler angles (deg): ({np.rad2deg(pose_dict['roll'])},{np.rad2deg(pose_dict['pitch'])},{np.rad2deg(pose_dict['yaw'])})")

def main() -> int:
    if len(sys.argv) == 1:
        print("Usage: python3 demo.py PATH_TO_SCANS")
        return 1

    # Load the scan names and sort them
    scans_dir = os.path.abspath(sys.argv[1])
    scans_names = os.listdir(scans_dir)
    scans_names.sort(
        key = lambda file_name : int(os.path.splitext(file_name)[0]) 
    )

    # Start registration
    T_from_prev_to_odom = np.eye(4)
    odometry = Odometry()
    for seq_idx,scan_name in enumerate(scans_names):
        print(f"\nSequence index {seq_idx}")
        print("==============================")
        # Load file
        scan_path = os.path.join(scans_dir, scan_name)
        scan_array = np.fromfile(scan_path,dtype=np.float32).reshape(-1,4)
        
        # Perform registration
        T_current_to_prev = odometry.scan_callback(scan_array)
        between_pose_dict = split_transform(T_current_to_prev)

        # Update the transformation matrix
        T_from_curr_to_odom = T_from_prev_to_odom @ T_current_to_prev
        pose_dict = split_transform(T_from_curr_to_odom)

        # Downsample the current map
        if seq_idx % 30 == 0:
            odometry.downsample_global_map()

        print("Between Pose")
        print_pose_dict(between_pose_dict)

        print("Current pose")
        print_pose_dict(pose_dict)

        T_from_prev_to_odom = T_from_curr_to_odom 
        
        if seq_idx == 10:
            break
            

    # Output the map
    global_map_pcd = odometry.get_global_map()
    o3d.io.write_point_cloud("global_map.ply", global_map_pcd, write_ascii=False, compressed=True, print_progress=True)
    return 0

if __name__ == "__main__":
    sys.exit(main())