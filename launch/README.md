# Launch Files

These ROS 2 launch descriptions start the pure pursuit workflow or one of its recorder nodes. Each accepts a `params_file` argument to override the corresponding parameter YAML.

- `pure_pursuit.launch.py` starts the Prius Gazebo simulation from `prius_bringup`, waits five seconds by default, then starts the `pure_pursuit_controller`. The delay is configurable with `sim_startup_delay`.
- `record_path.launch.py` starts `path_recorder` with the reference-path settings from `path_recorder_params.yaml`.
- `record_actual_trajectory.launch.py` starts `path_recorder` with the executed-trajectory settings from `actual_trajectory_recorder_params.yaml`.

Run a launch file from a sourced ROS 2 workspace, for example:

```bash
ros2 launch pure_pursuit record_path.launch.py
```

The recorder launch files expect an odometry publisher on the configured topic. Stop the recorder with Ctrl+C to write its CSV output.
