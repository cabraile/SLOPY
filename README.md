This repository is under development.

# About
In this repository, the "Simple Laser Odometry in Python" (SLOPY) was implemented. The goal is to have a minimalist laser odometry with as few dependencies as possible for prototyping global localization algorithms in Python. 

Therefore, do not expect very fancy graph optimization-ish features or estimations at high frequencies, but yet a very modest pose estimation technique based on the Open3D registration tools.

# Usage
```python
from slopy.odometry import Odometry

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
```
python3 demo.py <path_to_scans_dir>
```
