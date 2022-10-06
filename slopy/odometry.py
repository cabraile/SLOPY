# 
import numpy as np
import open3d as o3d


class Odometry:
    """Laser odometry based on the tutorial from http://www.open3d.org/docs/release/tutorial/pipelines/global_registration.html"""

    def __init__(self, voxel_size : float = 0.1, distance_threshold : float = 5.0) -> None:
        """
        Arguments
        -----
        voxel_size: float.
            The size (in meters) of each voxel in order to apply the voxel grid 
            downsample.
        distance_threshold: float.
            The maximum distance (in meters) two consecutive scans can be from each other.
        """
        self.last_pcd = None
        self.voxel_size = voxel_size
        self.distance_threshold = distance_threshold
        self.T_from_curr_to_odom = np.eye(4)

    def scan_callback(self, pcd : o3d.geometry.PointCloud) -> np.ndarray:
        """Performs the registration between the provided point cloud and the
        previous point cloud received.

        Arguments
        -----
        pcd: open3d.geometry.PointCloud.
            The scan point cloud received.
        
        Returns
        -----
        numpy.ndarray: The 4-by-4 transformation matrix that projects from the
            current point cloud frame to the previous frame.
        """
        # Adequate input
        pcd = pcd.voxel_down_sample(self.voxel_size)
        T_init = np.eye(4)
        pcd = pcd.transform(T_init)

        # Estimate normals
        radius_normal = self.voxel_size * 2
        pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

        # Store current pcd and return
        if self.last_pcd is None:
            self.last_pcd = pcd
            return self.T_from_curr_to_odom
        
        # Register
        # - Goal: register current pcd to previous (T_from_curr_to_prev)
        result = o3d.pipelines.registration.registration_icp(
            pcd, self.last_pcd, self.distance_threshold, T_init,
            o3d.pipelines.registration.TransformationEstimationPointToPlane()
        )
        T_from_curr_to_prev = result.transformation

        # Store current PCD
        self.last_pcd = pcd

        # Update current to odom
        T_from_prev_to_odom = self.T_from_curr_to_odom
        self.T_from_curr_to_odom = T_from_prev_to_odom @ T_from_curr_to_prev

        return T_from_curr_to_prev