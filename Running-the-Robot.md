### Initializing the Robot
- Connect the battery to the terminal and fasten it down with the velcro
- (Optional) If the correct joint positions matter, arrange the legs in the neutral position
- Turn on the on switch
- From the remote computer, wait until the wireless network "Spirit40-006" appears, then connect to it
- Run `ssh ghost@192.168.168.105` and enter the password
- Run `./init_charlie.sh`
- (Optional) If you need internet connectivity, plug in an ethernet cable

### Running tests and collecting data
- After initializing the robot, open a new terminal and source the spirit workspace
- Run the following, replacing the address for `ROS_IP` with your computer's IP on the Spirit network if different (or add this to the .bashrc):
```
export ROS_MASTER_URI=http://192.168.168.105:11311
export ROS_IP=192.168.168.5
```
- Run `roslaunch spirit_utils spirit.launch` to execute the planning stack, visualize the robot, and start recording (not implemented yet)
- Run (TBD) to process the data logs and store them in (TBD)
- (Optional) Upload the contents of (TBD) to Box

### Running data logger
Spirit Logger Package allows you to visualize the logged data from the robot (In .bag format) and save figures through a GUI. You need to first build the spirit software stack, which also installs the appropriate dependencies. Once finished, navigate to src/spirit-software where the spirit_logger package lives. Run the following command to launch the logger GUI:
```
rosrun spirit_logger read_bag.py "address of the target bag file".bag
```
You should see the GUI similar to the following picture. 