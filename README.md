This repository is under development.

# About
In this repository, the "Simple Laser Odometry in Python" (SLOPY) was implemented. The goal is to have a minimalist laser odometry with as few dependencies as possible for prototyping global localization algorithms in Python. 

Therefore, do not expect very fancy graph optimization-ish features or estimations at high frequencies, but yet a very modest pose estimation technique based on the Open3D registration tools.

# Running the demo
```
python3 demo.py <path_to_scans_dir>
```