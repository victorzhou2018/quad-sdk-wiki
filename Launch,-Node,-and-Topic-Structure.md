### Launchfiles
Launchfiles are divided across system functionality:
- `mapping.launch` - launches nodes `terrain_map_publisher` and `grid_map_visualization`
- `planning.launch` - launches nodes `global_body_planner` and `local_footstep_planner`
- `control.launch` - launches node `mpc_controller` or `open_loop_controller`. Takes arg `controller` with values of `open_loop` (default) or `mpc`
- `estimation.launch` - launches nodes `ground_truth_publisher`, `ekf_estimator`, `contact_detection`, and `body_force_estimation`
- `mocap.launch` - launches node `mocap_node` with config file
- `visualization.launch` - launches nodes `robot_state_publisher` which remaps from imu data, `rviz_interface` which takes arg `use_estimate` (default false) and remaps from state_estimate to ground_truth_state unless true, and `rviz`
- `logging.launch` - launches two record nodes to record two bags - one timestamped archive, the other in spirit_log_current

There are two main high-level launch files, one each for the robot and remote computer:
- `robot_driver.launch` - should be run on the robot and calls `control.launch` and `estimation.launch`. Has args `proxy`, `mocap`, and `logging`, each of which default to false. Setting any of these to true calls the corresponding launchfiles.
- `remote_driver.launch` - should be run on the remote computer and calls `mapping.launch`, `planning.launch`, and `visualization.launch`. Has arg `proxy` (default false) which if true will call `robot_driver.launch` with `proxy:=true`.

### Nodes


### Topics