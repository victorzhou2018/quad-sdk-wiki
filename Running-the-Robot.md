Refer to [The Ghost Robotics Quickstart Guide](https://cmu.app.box.com/file/782269653421) for more detailed information on how to run the robot with the default firmware.

### Initializing the Robot
- Connect the battery to the terminal and fasten it down with the velcro
- (Optional) If the correct joint positions matter, arrange the legs in the neutral position
- Flip the on switch
- From the remote computer, wait until the wireless network "Spirit40-006" appears, then connect to it
- Run 
```
ssh ghost@192.168.168.105
source launch_robot_env.sh
```
- Refer to [Launch, Node and Topic Structure](https://github.com/robomechanics/spirit-software/wiki/Launch,-Node,-and-Topic-Structure) for optional arguments
- (Optional) If you need internet connectivity, plug in an ethernet cable

### Running experiments
- Before starting, make sure:
  - You are connected to the robot's wifi and have set a manual static IP of 192.168.168.5
  - `~/catkin_ws/src/spirit-software` is a valid path (if not, just manually source whichever workspace you use)
- To run the planning stack and visualize, open a new terminal and run 
```
source ~/catkin_ws/src/spirit-software/spirit_utils/scripts/init_remote.sh
roslaunch spirit_utils remote_driver.launch
```
- Refer to [Launch, Node and Topic Structure](https://github.com/robomechanics/spirit-software/wiki/Launch,-Node,-and-Topic-Structure) for optional arguments
- Press `Ctrl-C` to stop the run