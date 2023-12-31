This document is for internal RML infrastructure, and is not relevant for external users.

Refer to [The Ghost Robotics Quickstart Guide](https://cmu.app.box.com/file/782269653421) for more detailed information on how to run the robot with the default firmware and how to handle the physical components of the robot.

### Initializing the Robot
- Connect the battery to the terminal and fasten it down with the velcro
- (Optional) If the correct joint positions matter, arrange the legs in the neutral position
- Flip the on switch
- From the remote computer, wait until the wireless network "Spirit-RML-5G" appears, then connect to it with standard lab password.
- Connect to the robot with (pwd ghost):
```
ssh ghost@192.168.8.101
```
- Set up the robot environment and optionally run the robot driver with inverse dynamics controller:
```
source launch_robot_env.sh
```
- Refer to [Launch, Node and Topic Structure](https://github.com/robomechanics/quad-software/wiki/3.-Launch,-Node,-and-Topic-Structure) for optional arguments to robot_driver if preferred over the bash script.

### Initializing the remote computer
- Before starting, make sure:
  - You are connected to the robot's wifi and have set a manual static IP of 192.168.8.102 if connected via ethernet or 192.168.8.103 if connected via wifi
  - `~/catkin_ws/src/quad-software` is a valid path (if not, just manually source whichever workspace you use)
- To run the source the environment, launch the visualization stack, and establish a heartbeat connection, open a new terminal and run the following (and optionally download/build new code, also declining to launch the remote driver will still set up your environment correctly)
```
source launch_remote_env.sh
```
- Refer to [Launch, Node and Topic Structure](https://github.com/robomechanics/quad-software/wiki/3.-Launch,-Node,-and-Topic-Structure) for optional arguments to remote_driver if preferred over the bash script.
- Press `Ctrl-C` to stop the run