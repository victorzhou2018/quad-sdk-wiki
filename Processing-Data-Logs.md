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