# Frequently Asked Questions

## What is the convention for joint and leg numbering in the RobotState message?
The joint arrays in spirit_msgs::RobotState are defined such that indices 0 = abad0, 1 = hip0, 2 = knee1, 3 = abad1, 4 = hip1, 5 = knee12, 6 = abad2, 7 = hip2, 8 = knee2, 9 = abad3, 10 = hip3, 11 = knee3. Legs are numbered such that 0 = front left, 1 = back left, 2  = front right, and 3 = back right. So calling `state_msg.joints.positions[4]` would give you the joint position of the back left hip motor.

Confusingly, Ghost Robotics does NOT use this definition, and instead has their urdf and /mcu/state/jointURDF use the following ordering: 0 = hip0, 1 = knee0, 2 = hip1, 3 = knee1, 4 = hip2, 5 = knee2, 6 = hip3, 7 = knee3, 8 = abd0, 9 = abd1, 10 = abd2, 11 = abd3. To correct this, our `ground_truth_publisher.cpp` reads their message and maps from their ordering to ours, see the `joints_order_` vector in the [source code](https://github.com/robomechanics/spirit-software/blob/109168feb808f844947affadb95e71cc271dc47d/spirit_utils/src/ground_truth_publisher.cpp#L32) . Likewise our `estimator_plugin.cpp` maps from the alphabetical order in Gazebo to our order.

## Common errors

### Unable to communicate with master
```
ERROR: Unable to communicate with master!
```
No ROS master detected - if on the remote computer make sure to source launch_robot_env.sh (which sources the appropriate setup.bash). Otherwise you probably haven't issues and roslaunch commands yet.

### Can't SSH into Jetson TX2
Power cycle the USB ethernet adapter by turn on&off the USB hub port.

### Internal compiler error (during catkin_make)
This typically happens when your system runs out of RAM while compiling. Try `catkin build -j4 -l4', or go even lower than 4 (this represents the number of parallel threads used while compiling). Alternatively if you were running `catkin_make run_tests`, try compiling just the source code first, then the tests after completion.

This could also be caused by other processes running in the background which may or not be ROS related. Sometimes nodes or tests that do not get terminated properly can consume significant resources in the background. Check to make sure you don't have something eating up your CPU overhead.

### RViz crashes when using Clicked Point
This appears to be a [known issue](https://github.com/ros-visualization/rviz/issues/1082) when using interactive markers with NVidia drivers. It can be fixed by rebuilding OGRE with an updated version (and then also RViz) or reverting to Nouveau drivers, but most likely we will eliminate the use of interactive markers for this reason.

### Catkin: command not found (or any other packages)
The setup.sh script should install all the required dependencies, if not then likely sudo apt update failed. Check your packages.