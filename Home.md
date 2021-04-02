# Welcome to the spirit-software wiki!

### Setup repo

1. Install Ubuntu 18.04 (dual boot or VM) ([help](https://linuxhint.com/install_ubuntu_18-04_virtualbox/))
2. Install ROS Melodic ([help](http://wiki.ros.org/melodic/Installation/Ubuntu))
3. Create catkin workspace ([help](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment))
4. Setup repo
```bash
cd ~/catkin_ws
source devel/setup.bash
cd src
git clone https://github.com/robomechanics/spirit-software.git
cd spirit-software
chmod +x setup.sh && ./setup.sh
cd ~/catkin_ws
catkin_make
```

### Run tests
Always do this before pushing to any branch to confirm that tests will pass.
```bash
roslaunch spirit_utils load_params.launch
cd ~/catkin_ws
catkin_make run_tests
```

### Setup new branch
```bash
cd ~/catkin_ws/src/spirit-software
git checkout -b descriptive-branch-name
git push --set-upstream origin descriptive-branch-name
```

### Create new package
```bash
cd ~/catkin_ws/src/spirit-software
catkin_create_pkg spirit_package_name roscpp std_msgs *other-rosdeps-here*
```
After creating a package, open package.xml and modify the package owner and contact info to be yourself

### Create documentation
```bash
cd ~/catkin_ws/src/spirit-software
doxygen Doxyfile
firefox docs/index.html
```
### Create new custom messages
Custom messages can be placed in spirit_msgs/msg. See the [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv) for details on message creation. Add your message filename to the `add_message_files()` in spirit_msgs/CMakeLists.txt, and add spirit_msgs as a dependency to your package. To implement and namespace your messages, refer to [this tutorial](http://wiki.ros.org/ROS/Tutorials/DefiningCustomMessages). 

### Ghost SDK Documentation
See the documentation [here](https://ghostusers.gitlab.io/docs/). Must have access to their repository.
