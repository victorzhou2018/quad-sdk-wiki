# Frequently Asked Questions

Visit the [Issue Tracker](https://github.com/robomechanics/quad-sdk/issues) for software-related issues.

## Does Quad-SDK work for other legged morphologies (bipeds, hexapods, etc)?
As implied by the name, Quad-SDK is designed to make autonomous, agile locomotion attainable for quadrupedal platforms. However, many of the algorithms implemented here could be extended to other morphologies without major structural changes. For example, the Global Planner abstracts the legs away from the problem, and the Local Planner and Robot Driver for the most part only require information about the kinematics and dynamics which could be represented with any number of end-effectors.

However, there seems to be a growing class of quadrupeds that are readily available on the market which share similar features (quasi-direct drive, lightweight 3-DOF legs) and are well poised to do real work in research and industry. In an effort to provide useful, tested software without excessive overhead caused by many abstraction layers, we restrict our focus to this class of platform. That said, we invite the community to extend the software as desired and look forward to seeing applications to more varied morphologies.

## Will you offer ___ feature?

Our goal is for Quad-SDK to provide a common platform for researchers to deploy agile autonomy to their quadrupeds, and we've worked hard to make this as seamless as possible. However, the repository is primarily maintained by the members of the Robomechanics Lab - we are researchers with our own projects and hypotheses to evaluate! This means that we may not be able to get to each new feature request, but we again invite the community to contribute functionality that would be useful to all.

## What is the convention for joint and leg numbering in the RobotState message?
The joint arrays in spirit_msgs::RobotState are defined such that indices 0 = abad0, 1 = hip0, 2 = knee1, 3 = abad1, 4 = hip1, 5 = knee12, 6 = abad2, 7 = hip2, 8 = knee2, 9 = abad3, 10 = hip3, 11 = knee3. Legs are numbered such that 0 = front left, 1 = back left, 2  = front right, and 3 = back right. So calling `state_msg.joints.positions[4]` would give you the joint position of the back left hip motor.

# Common errors

### Unable to communicate with master
```
ERROR: Unable to communicate with master!
```
No ROS master detected - if on the remote computer make sure to source launch_robot_env.sh (which sources the appropriate setup.bash). Otherwise you probably haven't issues and roslaunch commands yet.

### Internal compiler error (during catkin build)
This typically happens when your system runs out of RAM while compiling. Try `catkin build -j4 -l4`, go even lower than 4 (this represents the number of parallel threads used while compiling), or try `catkin build --this` in the package you want to build to avoid compiling the whole workspace. Alternatively if you were running `catkin test`, try compiling just the source code first, then the tests after completion.

This could also be caused by other processes running in the background which may or not be ROS related. Sometimes nodes or tests that do not get terminated properly can consume significant resources in the background. Check to make sure you don't have something eating up your CPU overhead.

### RViz crashes when using Clicked Point
This appears to be a [known issue](https://github.com/ros-visualization/rviz/issues/1082) when using interactive markers with NVidia drivers. It can be fixed by rebuilding OGRE with an updated version (and then also RViz) or reverting to Nouveau drivers, but most likely we will eliminate the use of interactive markers for this reason.

### Catkin: command not found (or any other packages)
The setup.sh script should install all the required dependencies, if not then likely sudo apt update failed. Check your packages.

### Gazebo crashes on launch
If this is your first time running the simulator, check that gazebo can launch properly. Running `gazebo` in the command line should bring up the gazebo GUI. If you get a symbol lookup error, update your ignition math library (as described [here](https://answers.gazebosim.org//question/22071/symbol-lookup-error-both-instalation-methods/)).

Note that Gazebo can be fickle and sometimes will crash due to no clear reason. Try launching again, or checking that there are no background gazebo instances already running (and kill them with `killall -9 gzserver`).

## RML-specific questions

## I can't see the robot on the phone app
The phone app requires the Ghost ROS service to be up and running. You can check if the service is active with
```
ssh ghost@<ghost-IP>
sudo service ghost status
```
If the service has failed this should reveal the source. Often this is because the IP address is invalid - in particular the ROS_IP and ROS_MASTER_URI parameters set in `~/.environment_vars.sh` must be valid IPs. If they are set to the RML router IP (192.168.8.101) but that router is not connected, the service will fail. If operation without the RML router is desired, change the ROS_IP and ROS_MASTER_URI back to the GR default (192.168.168.105) and enter `sudo service ghost restart`.

### Can't SSH into Jetson TX2
Power cycle the USB ethernet adapter by turn on&off the USB hub port.