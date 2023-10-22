from typing import Optional
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation

class Odometry:
    """Laser odometry based on the tutorial from http://www.open3d.org/docs/release/tutorial/pipelines/global_registration.html"""

    def __init__(
        self, 
        voxel_size : float = 0.1, 
        distance_threshold : float = 5.0,
        frequency : Optional[float] = None,
        mode: str = "point-to-point"
    ) -> None:
        """
        Arguments
        -----
        voxel_size: float.
            The size (in meters) of each voxel in order to apply the voxel grid 
            downsample.
        distance_threshold: float.
            The maximum distance (in meters) two consecutive scans can be from each other.
        frequency: float.
            Frequency in Hz the laser scanner provides messages. 
            If not provided, velocities will not be computed.
            Used for providing prior position estimates (constant velocity 
            model).
        mode : str.
            Registration mode. Options: "point-to-point" or "point-to-plane".
        """
        self.last_pcd = None
        self.voxel_size = voxel_size
        self.distance_threshold = distance_threshold
        self.T_from_curr_to_odom = np.eye(4)

        self.flag_compute_normals = (mode == "point-to-plane")

        if mode == "point-to-point":
            self.registration_mode = o3d.pipelines.registration.TransformationEstimationPointToPoint()
        elif mode == "point-to-plane":
            self.registration_mode = o3d.pipelines.registration.TransformationEstimationPointToPlane()
        else:
            raise Exception("Invalid option 'mode'. Registration modes available: 'point-to-point' or 'point-to-plane'.")
        # Computes velocity for improving the prior pose estimation 
        self.estimate_velocity = frequency is not None
        if self.estimate_velocity:
            self.delta_t = 1.0 / frequency
        else:
            self.delta_t = 0.0
        self.linear_velocity_x = 0.0
        self.linear_velocity_y = 0.0
        self.linear_velocity_z = 0.0
        self.angular_velocity_x = 0.0
        self.angular_velocity_y = 0.0
        self.angular_velocity_z = 0.0

    def displacement_transform_from_velocities(self) -> np.ndarray:
        # Compute linear displacement
        linear_displacement_x = self.linear_velocity_x * self.delta_t
        linear_displacement_y = self.linear_velocity_y * self.delta_t
        linear_displacement_z = self.linear_velocity_z * self.delta_t

        # Compute angular displacement
        angular_displacement_x = self.angular_velocity_x * self.delta_t
        angular_displacement_y = self.angular_velocity_y * self.delta_t
        angular_displacement_z = self.angular_velocity_z * self.delta_t

        # Build transformation matrix
        T_from_curr_to_prev = np.eye(4)
        T_from_curr_to_prev[:3,:3] = Rotation.from_euler(
            "xyz",
            [angular_displacement_x, angular_displacement_y, angular_displacement_z],
            degrees = False
        ).as_matrix()
        T_from_curr_to_prev[:3,3] = [linear_displacement_x, linear_displacement_y, linear_displacement_z]
        return T_from_curr_to_prev

    def get_transform_from_frame_to_init(self) -> np.ndarray:
        """Returns the transformation matrix that transforms from the current
        frame to the initial pose.
        """
        return np.copy(self.T_from_curr_to_odom)

    def register(self, pcd : o3d.geometry.PointCloud, T_prior : Optional[np.ndarray] = None, R_prior : Optional[np.ndarray] = None, t_prior : Optional[np.ndarray] = None) -> np.ndarray:
        """Performs the registration between the provided point cloud and the
        previous point cloud received.

        If it is the first scan, the returned transformation is the identity.

        Arguments
        -----
        pcd: open3d.geometry.PointCloud.
            The scan point cloud received.
        T_prior: numpy.ndarray. 
            The initial 4-by-4 transformation matrix that transforms the 
            current point cloud to the previous frame. 

            If not provided, then a prior will be computed from the velocities, 
            if a frequency was provided. Otherwise, the identity matrix will be 
            used.

            It overrides the R_prior and t_prior arguments.
        R_prior: numpy.ndarray.
            The initial 3-by-3 rotation matrix that transforms the current
            point cloud to the previous frame.
        t_prior: numpy.ndarray.
            The initial 3d translation vector that transforms the current
            point cloud to the previous frame.
        
        Returns
        -----
        numpy.ndarray: The 4-by-4 transformation matrix that projects from the
            current point cloud frame to the previous frame.
        """
        # T_init corresponds to the first "guess" of the registration
        if T_prior is None:
            if self.estimate_velocity:
                T_init = self.displacement_transform_from_velocities()
            else:
                T_init = np.eye(4)
            if R_prior is not None:
                T_init[:3,:3] = R_prior
            if t_prior is not None:
                T_init[:3,3] = t_prior
        else:
            T_init = T_prior

        # Adequate input
        pcd = pcd.voxel_down_sample(self.voxel_size)

        # Estimate normals
        if self.flag_compute_normals:
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
            self.registration_mode
        )
        T_from_curr_to_prev = result.transformation

        # Store current PCD
        self.last_pcd = pcd

        # Update current to odom
        T_from_prev_to_odom = self.T_from_curr_to_odom
        self.T_from_curr_to_odom = T_from_prev_to_odom @ T_from_curr_to_prev

        # Update velocities (if enabled)
        if self.estimate_velocity:
            R = T_from_curr_to_prev[:3,:3].copy()
            delta_x, delta_y, delta_z = T_from_curr_to_prev[:3,3]
            delta_roll, delta_pitch, delta_yaw = Rotation.from_matrix(R).as_euler("xyz", degrees=False)
            self.linear_velocity_x = delta_x / self.delta_t
            self.linear_velocity_y = delta_y / self.delta_t
            self.linear_velocity_z = delta_z / self.delta_t
            self.angular_velocity_x = delta_roll / self.delta_t
            self.angular_velocity_y = delta_pitch / self.delta_t
            self.angular_velocity_z = delta_yaw / self.delta_t

        return T_from_curr_to_prev