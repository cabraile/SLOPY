import os
import sys
from typing import Dict
import numpy as np
import open3d as o3d

from scipy.spatial.transform.rotation import Rotation
from slopy.odometry import Odometry

def split_transform(T) -> Dict[str, float]:
    """Splits a transformation matrix to position and orientation."""
    R = T[:3,:3].copy()
    roll, pitch, yaw = Rotation.from_matrix(R).as_euler("xyz",degrees=False)
    x, y, z = T[:3,3]
    return {"x" : x, "y" : y, "z" : z, "roll" : roll, "pitch" : pitch, "yaw" : yaw}

def print_pose_dict(pose_dict : Dict[str,float]):
    print(f"> Position (m): ({pose_dict['x']},{pose_dict['y']},{pose_dict['z']})")
    print(f"> Euler angles (rad): ({pose_dict['roll']},{pose_dict['pitch']},{pose_dict['yaw']})")
    print(f"> Euler angles (deg): ({np.rad2deg(pose_dict['roll'])},{np.rad2deg(pose_dict['pitch'])},{np.rad2deg(pose_dict['yaw'])})")

def scan_array_to_pointcloud(scan : np.ndarray) -> o3d.geometry.PointCloud:
    """Converts the N-by-M array of the scan to a 3D point cloud.

    Drops any additional field as intensity or color.
    """
    xyz_array = scan[:,:3]
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz_array)
    return pcd

def main() -> int:
    if len(sys.argv) == 1:
        print("Usage: python3 demo.py PATH_TO_SCANS OPTIONAL_FREQUENCY")
        return 1

    voxel_size = 0.25
    downsample_every_n_steps = 30

    # Load the scan names and sort them
    scans_dir = os.path.abspath(sys.argv[1])
    scans_names = os.listdir(scans_dir)
    scans_names.sort(
        key = lambda file_name : int(os.path.splitext(file_name)[0]) 
    )
    
    # Load the frequency (if available)
    frequency = None
    if len(sys.argv) > 2:
        frequency = float(sys.argv[2])

    # Start registration
    T_from_prev_to_odom = np.eye(4)
    odometry = Odometry(voxel_size=voxel_size, frequency=frequency)
    global_map = o3d.geometry.PointCloud()
    for seq_idx,scan_name in enumerate(scans_names):
        print(f"\nSequence index {seq_idx}")
        print("==============================")
        # Load file
        scan_path = os.path.join(scans_dir, scan_name)
        scan_array = np.fromfile(scan_path,dtype=np.float32).reshape(-1,4)
        pcd = scan_array_to_pointcloud(scan_array)
        
        # Perform registration
        T_current_to_prev = odometry.register(pcd)
        between_pose_dict = split_transform(T_current_to_prev)

        # Update the transformation matrix
        T_from_curr_to_odom = T_from_prev_to_odom @ T_current_to_prev
        pose_dict = split_transform(T_from_curr_to_odom)

        # Register Point Cloud to map
        pcd_global_frame = o3d.geometry.PointCloud(pcd.points).transform(T_from_curr_to_odom)
        global_map += pcd_global_frame

        # Downsample the current map
        if seq_idx % downsample_every_n_steps == 0:
            global_map = global_map.voxel_down_sample(voxel_size)

        print("Between Pose")
        print_pose_dict(between_pose_dict)

        print("Current pose")
        print_pose_dict(pose_dict)

        T_from_prev_to_odom = T_from_curr_to_odom 
            
    # Output the map
    global_map = global_map.voxel_down_sample(voxel_size)
    o3d.io.write_point_cloud("global_map.ply", global_map, write_ascii=False, compressed=True, print_progress=True)
    return 0

if __name__ == "__main__":
    sys.exit(main())