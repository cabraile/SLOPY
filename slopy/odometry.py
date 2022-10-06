# 
import numpy as np
import open3d as o3d

def scan_array_to_pointcloud(scan : np.ndarray) -> o3d.geometry.PointCloud:
    """Converts the N-by-M array of the scan to a 3D point cloud.

    Drops any additional field as intensity or color.
    """
    xyz_array = scan[:,:3]
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz_array)
    return pcd

class Odometry:
    """Laser odometry based on the tutorial from http://www.open3d.org/docs/release/tutorial/pipelines/global_registration.html"""

    def __init__(self) -> None:
        self.last_pcd = None
        self.voxel_size = 0.1
        self.global_map = o3d.geometry.PointCloud()
        self.T_from_prev_to_odom = np.eye(4)

        # TODO: compute velocities
        self.linear_velocity = np.zeros(3)
        self.angular_velocity = np.zeros(3)

    def get_global_map(self) -> o3d.geometry.PointCloud():
        return self.global_map

    def downsample_global_map(self) -> None:
        self.global_map  = self.global_map .voxel_down_sample(self.voxel_size)

    def scan_callback(self, scan : np.ndarray) -> np.ndarray:
        # Adequate input
        pcd = scan_array_to_pointcloud(scan)
        pcd = pcd.voxel_down_sample(self.voxel_size)
        T_init = np.eye(4)
        pcd = pcd.transform(T_init)

        # Estimate normals
        radius_normal = self.voxel_size * 2
        pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

        # Store current pcd and return
        if self.last_pcd is None:
            self.last_pcd = pcd
            self.global_map += o3d.geometry.PointCloud(pcd.points)
            return self.T_from_prev_to_odom
        
        # Register
        distance_threshold = 5.0

        # - Goal: register current pcd to previous (T_from_curr_to_prev)
        result = o3d.pipelines.registration.registration_icp(
            pcd, self.last_pcd, distance_threshold, T_init,
            o3d.pipelines.registration.TransformationEstimationPointToPlane()
        )
        T_from_curr_to_prev = result.transformation

        # Store current PCD
        self.last_pcd = pcd

        # Map the registered PCD and update current estimation
        T_from_curr_to_odom = self.T_from_prev_to_odom @ T_from_curr_to_prev
        pcd_global_frame = o3d.geometry.PointCloud(pcd.points).transform(T_from_curr_to_odom)
        self.global_map += pcd_global_frame
        self.T_from_prev_to_odom = T_from_curr_to_odom

        return T_from_curr_to_prev