# Welcome to the spirit-software wiki!

### Setup Steps

1. Install Ubuntu 18.04 (dual boot or VM) ([help](https://linuxhint.com/install_ubuntu_18-04_virtualbox/))
2. Install ROS Melodic ([help](http://wiki.ros.org/melodic/Installation/Ubuntu))
3. Create catkin workspace ([help](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment))
4. Setup repo
```bash
cd ~/catkin_ws/src
git clone https://github.com/robomechanics/spirit-software.git
source devel/setup.bash
chmod +x setup.sh && ./setup.sh
catkin_make
catkin_make run_tests
```
