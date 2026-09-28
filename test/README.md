# Tests

This directory contains the package's ROS 2 quality-check tests:

- `test_copyright.py` checks source-file copyright notices.
- `test_flake8.py` checks Python style with flake8.
- `test_pep257.py` checks Python docstring conventions.

Run the package tests from the ROS 2 workspace root after building and sourcing the workspace:

```bash
colcon test --packages-select pure_pursuit
colcon test-result --verbose
```
