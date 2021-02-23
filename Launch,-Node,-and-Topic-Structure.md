### Launchfiles
Launchfiles are divided across system functionality:
- `mapping.launch` - launches nodes terrain_map_publisher and grid_map_visualization
- `planning.launch` - launches nodes body planner and local_footstep_planner nodes. Takes arg `body_planner` with values of `global` (default) or `twist`
- `control.launch` - launches node mpc_controller or open_loop_controller. Takes arg `controller` with values of `open_loop` (default) or `mpc`
- `estimation.launch` - launches nodes ekf_estimator, contact_detection, and body_force_estimation
- `mocap.launch` - launches node mocap_node with config file and ground_truth_publisher
- `visualization.launch` - launches nodes robot_state_publisher which remaps from imu data, rviz_interface which remaps from /state/ground_truth (can be set to /state/estimate or /state/trajectory), and rviz
- `logging.launch` - launches two record nodes to record two bags - one in `spirit_logger/bags/archive/spirit_log_<timestamp>.bag`, the other in `spirit_logger/bags/spirit_log_current.bag`

There are two main high-level launch files, one each for the robot and remote computer:
- `robot_driver.launch` - should be run on the robot and calls `control.launch` and `estimation.launch`. Has args `proxy`, `mocap`, and `logging`, each of which default to false. Setting any of these to true calls the corresponding launchfiles. It also passes arg `controller` to `control.launch` (default open_loop)
- `remote_driver.launch` - should be run on the remote computer and calls `mapping.launch`, `planning.launch`, and `visualization.launch`. Has arg `proxy` (default false) which if true will call `robot_driver.launch` with `proxy:=true` and passing the other args through (`mocap` and `logging`). Also has arg `body_planner` which is passed to `planning.launch`

### Nodes


### Topics