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
  - `with_proxy:=true` launches a node to replicate the robot hardware, use if you are not near the robot (default false)
  - `with_mocap:=true` includes `mocap.launch`, which launches the optitrack interface node (default false)
  - `record:=true` includes `record.launch` which starts recording all topics into a bag (default false)
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
- Optional arguments include `with_proxy:=true` which will launch robot_driver.launch with a proxy node replicating the robot (default false)
- Press `Ctrl-C` to stop the run

### Running the Python data logger
Spirit Logger Package allows you to visualize the logged data from the robot (In .bag format) and save figures through a GUI. You need to first build the spirit software stack, which also installs the appropriate dependencies. Once finished, navigate to src/spirit-software where the spirit_logger package lives. Run the following command to launch the logger GUI:
```
rosrun spirit_logger read_bag.py "address of the target bag file.bag"
```
You should see the GUI similar to the following picture. On the left side, you will see a list of topics (IMU and joint states are currently supported) and variables you could select to visualize. Once you selected the topic and variable fields of interest, press Load. You should see the figures of the selected topics as shown below. To visualize a separate set of topics, press Clear and repeat the previous step. To save a figure to your current directory, hit Save Figure. This will save the shown figure to a .png file.'

### Running the MATLAB data logger
Spirit Logger also contains a script to process the bags and generate MATLAB figures and animations, and automatically saves them into our specified file structure. Run
```
roscd spirit_logger/bags
cp spirit_log_current.bag <trial_name_year-month-day>.bag
roscd spirit_logger/scripts
matlab -nodesktop
processLog('<trial_name_year-month-day>')
exit
```
Note that this will by default show the animation, which must run in full in order to save properly. Alternatively you can launch the standard MATLAB GUI and edit `processLog.m` as needed to generate the proper media.

Once you have run `processLog()`, upload the newly created folder (located in `spirit_logger/logs`) to Box.
