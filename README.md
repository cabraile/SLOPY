# About
In this repository, the "Simple Laser Odometry in Python" (SLOPY) was implemented. The goal is to have a minimalist laser odometry with as few dependencies as possible for prototyping global localization algorithms in Python. 

Therefore, do not expect very fancy graph optimization-ish features or estimations at very high frequencies, but yet a very modest pose estimation technique based on the Open3D registration tools.

# Usage
```python
from slopy.odometry import Odometry

# Instantiate the odometry module
scan_frequency = 
odometry = Odometry(
    voxel_size = voxel_size, # The voxel size (in meters) for downsampling the input
    distance_threshold = distance_threshold, # The maximum distance (in meters) two consecutive scans can be from each other
    frequency = 10.0 # (Optional) Frequency in Hertz
)

# Receive scan
pcd = o3d.geometry.PointCloud()
pcd.points = something # Fill PCD

# ...

# Register
T_current_to_prev = odometry.register(pcd)

# Get the global pose transform
T_from_curr_to_init = odometry.get_transform_from_frame_to_init()
```

# Installation (from source)
1. Install the dependencies executing `pip3 install -r requirements.txt`. 

2. Next run `python3 setup.py build && python3 setup.py install`.

# Running the demo (from source)
We provided a demo for illustrating how to use the modules. It loads the scans from a directory provided (named by their sequence ids - as in the Kitti velodyne dataset), provides frame-by-frame estimations and outputs the point cloud of the remapped environment (file named `global_map.ply`).

For running the demo, execute
```
python3 demo.py <path_to_scans_dir> <scan_frequency>
```

The first argument refers to the path to the directory in which the point clouds are stored; the second argument is optional and refers to the frequency the sensor provides the scans. 

Passing the frequency will allow the odometry module to perform velocity estimations and provide more accurate priors to the ICP registration.
