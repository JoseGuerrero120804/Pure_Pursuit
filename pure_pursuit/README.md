# Python Package

This directory contains the Python implementation for the ROS 2 pure pursuit package.

- `pure_pursuit_controller.py` implements the path-following node. It reads waypoint coordinates from a CSV file, subscribes to odometry, and publishes velocity commands.
- `path_recorder.py` records odometry samples to CSV for creating a reference path or capturing an executed trajectory.
- `evaluate_tracking.py` compares reference and actual CSV paths, computes cross-track errors, and saves a trajectory comparison plot.
- `geometry.py` provides quaternion-to-yaw conversion.
- `path_utils.py` calculates local path curvature and identifies curve segments.
- `__init__.py` marks this directory as the importable `pure_pursuit` Python package.

The ROS executables are registered in the package setup: `pure_pursuit_controller`, `path_recorder`, and `evaluate_tracking`. Controller and recorder defaults are supplied by YAML files in `../config/`.
