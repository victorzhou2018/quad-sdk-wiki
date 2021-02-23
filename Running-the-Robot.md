### Initializing the Robot
- Connect the battery to the terminal and fasten it down with the velcro
- (Optional) If the correct joint positions matter, arrange the legs in the neutral position
- Flip the on switch
- From the remote computer, wait until the wireless network "Spirit40-006" appears, then connect to it
- Run 
```
ssh ghost@192.168.168.105
source ~/catkin_ws/src/spirit-software/spirit_utils/scripts/init_robot.sh
roslaunch spirit_utils robot_driver.launch
```
- Optional arguments to `robot_driver.launch` are
  - `proxy:=true` launches a node to replicate the robot hardware, use if you are not near the robot (default false)
  - `mocap:=true` includes `mocap.launch`, which launches the optitrack interface node (default false)
  - `logging:=true` includes `logging.launch` which starts recording all topics into a bag (default false)
- (Optional) If you need internet connectivity, plug in an ethernet cable

### Running tests
- Before starting, make sure:
  - You are connected to the robot's wifi and have set a manual static IP of 192.168.168.5
  - `~/catkin_ws/src/spirit-software` is a valid path (if not, just manually source whichever workspace you use)
- To run the planning stack and visualize, open a new terminal and run 
```
source ~/catkin_ws/src/spirit-software/spirit_utils/scripts/init_remote.sh
roslaunch spirit_utils remote_driver.launch
```
- Optional arguments include `proxy:=true` which will launch robot_driver.launch with a proxy node replicating the robot (default false)
- Press `Ctrl-C` to stop the run
