### Launch Files
Launch files are divided across system functionality:
- `mapping.launch` - launches nodes terrain_map_publisher and grid_map_visualization
- `planning.launch` - launches nodes global_body_planner and local_planner. Takes arg `global_planner` with values of `fgmp` (default) or `twist`, arg `local_planner` with values of `full` (default) and `body`, and arg `mpc_type` of value `nonlinear` (default) or `convex`
- `control.launch` - launches node inverse_dynamics or open_loop_controller. Takes arg `controller` with values of `inverse_dynamics` (default) or `open_loop`
- `estimation.launch` - launches nodes ekf_estimator, contact_detection, and body_force_estimation
- `mocap.launch` - launches node mocap_node with config file and ground_truth_publisher
- `visualization.launch` - launches nodes robot_state_publisher which remaps from imu data, rviz_interface which remaps from /state/ground_truth (can be set to /state/estimate or /state/trajectory), and rviz
- `logging.launch` - launches two record nodes to record two bags - one in `spirit_logger/bags/archive/spirit_log_<timestamp>.bag`, the other in `spirit_logger/bags/spirit_log_current.bag`

There are four main high-level launch files, one each for the robot and remote computers, one for gazebo, and one to execute plans:
- `robot_driver.launch` - should be run on the robot and calls `control.launch` and `estimation.launch`. Has args `proxy`, `mocap`, and `logging`, each of which default to false. Setting any of these to true calls the corresponding launch files. It also passes arg `controller` to `control.launch` (default inverse_dynamics)
- `remote_driver.launch` - should be run on the remote computer and calls `mapping.launch` and `visualization.launch`, as well as mblink_converter. Has arg `proxy` (default false) which if true will call `robot_driver.launch` with `proxy:=true` and passing the other args through (`mocap` and `logging`)
- `spirit_gazebo.launch` - see the Gazebo Simulator page for details.
- `execute_plan.launch` - Calls `planning.launch` with arg `body_planner` (default global), and `logging.launch` if `logging:=true` (default false)

### Common roslaunch calls:
Launch the simulator, visualize in RViz, stand the robot up, then execute a plan while logging:
```
roslaunch spirit_utils spirit_gazebo.launch
rostopic pub /control/mode std_msgs/UInt8 "data: 1"
roslaunch spirit_utils planning.launch global_planner:=twist mpc_type:=nonlinear logging:=true
```
Test the full stack without simulating or logging:
```
roslaunch spirit_utils remote_driver.launch proxy:=true
roslaunch spirit_utils execute_plan.launch
```
Other calls:
- `roslaunch spirit_utils robot_driver.launch mocap:=true` - Start the robot stack with the inverse_dynamics controller and mocap data
- `roslaunch spirit_utils remote_driver.launch` Start the remote stack to begin sending commands to the robot and visualizing data
- `roslaunch spirit_utils remote_driver.launch proxy:=true` - Test the remote stack with robot absent
- `roslaunch spirit_utils remote_driver.launch proxy:=true body_planner:=twist` - Test the full stack with robot absent and with simple twist-based high level body plan
- `roslaunch spirit_utils open_loop_robot_driver.launch` - Only launch open_loop_controller and mblink_converter nodes
- `roslaunch spirit_utils spirit_gazebo.launch gui:=true` - Launch the Gazebo sim and visualize the results (need to call other launch files to get it to run our software).

### Nodes and Topics

The following image demonstrates key components of the software architecture and on which hardware they run:

[Node and Topic Structure](https://cmu.app.box.com/file/861382302310)