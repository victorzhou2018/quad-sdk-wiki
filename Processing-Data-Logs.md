### Running the Python data logger
Spirit Logger Package allows you to visualize the logged data from the robot (In .bag format) and save figures through a GUI. You need to first build the spirit software stack, which also installs the appropriate dependencies. Once finished, navigate to src/spirit-software where the spirit_logger package lives. Run the following command to launch the logger GUI:
```
rosrun spirit_logger read_bag.py "address of the target bag file.bag"
```
You should see the GUI similar to the following picture. On the left side, you will see a list of topics (IMU and joint states are currently supported) and variables you could select to visualize. Once you selected the topic and variable fields of interest, press Load. You should see the figures of the selected topics as shown below. To visualize a separate set of topics, press Clear and repeat the previous step. To save a figure to your current directory, hit Save Figure. This will save the shown figure to a .png file.

### Running the MATLAB data logger
Spirit Logger also contains a script to process the bags and generate MATLAB figures and animations, and automatically saves them into our specified file structure. Run
```
roscd spirit_logger/bags
cp spirit_log_current.bag <trial_name_year-month-day>.bag
roscd spirit_logger/scripts
matlab
```
In MATLAB, open the `processLog.m` script and set the following variables:
- Set the `trialName` variable to your <trial_name_year-month-day>, or leave as a blank string (this will open a UI where you can select your bagfile directly).
- set `bAnimate` to true or false depending on whether you would like to see the trajectory animated.
- set `bSave` to true or false depending on whether you would like to save all the data and figures to a video. If yet, this will create (or overwrite) a folder named `trialName` in `spirit_logger/logs` with all your data.

You can also run matlab without the desktop GUI (`matlab -nodesktop`), and call `processLog(<trial_name_year-month-day>)` from the command window. This will automatically animate and save the data.

The folder created from `bSave = true` will be ignored by git, but can be uploaded directly to Box [here](https://cmu.app.box.com/folder/124893804526).

### Replay bag file in Rviz

Record 
```
state/ground_truth
/localplan
```

Run
```
roslaunch spirit_utils visualization.launch
rosbag play --clock -r 0.5 xyz.bag
```

Use the "Reset" button in Rviz to flash and rerun a new bag.