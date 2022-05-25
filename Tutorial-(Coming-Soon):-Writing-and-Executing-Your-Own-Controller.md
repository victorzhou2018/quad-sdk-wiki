Note: This tutorial was for an RML-internal project and has since been deprecated. Users can now generate new controllers by inheriting the `LegController` abstract class (see existing examples in `robot_driver/src/controllers`). A tutorial describing these steps will be coming soon.

To write your own controller and execute it on the robot, follow these steps:

1. Write your source code for your controller class and wrapper node (these instructions will assume you have some files named `controller.cpp` and `controller_node.cpp` in the `example_package` package). You can copy `open_loop_controller.cpp` as an example.
2. Update the `CMakeLists.txt` of the package containing your code to include your source code in the project library, add an executable (need this to be able to launch the node), and link the library to the executable:
```
add_library(example_package src/controller.cpp)
add_executable(controller_node src/controller_node.cpp)
target_link_libraries(controller_node example_package ${catkin_LIBRARIES})
```
3. Write a test for your code (at a bare minimum, call the constructor) and add it to the CMakeLists.txt:
```
catkin_add_gtest(controller_test test/test_controller.cpp)
target_link_libraries(controller_test example_package ${catkin_LIBRARIES})
```
4. Build your code and run the tests:
```
(In separate terminal) roslaunch quad_utils load_params.launch
(In original terminal) cd catkin_ws
catkin_make run_tests
```
5. Modify `quad_utils/launch/control.launch` by adding the following to include an argument for your new controller:
```
<group if="$(eval arg('controller') == '<your-controller-arg>')">
    <node name="<your-controller-name>" pkg="example_package" type="controller_node" output="screen"/>
</group>
```
6. Test your controller in simulation - refer to [Gazebo Simulation](https://github.com/robomechanics/quad-software/wiki/4.-Gazebo-Simulator) for additional arguments:
```
roslaunch quad_utils quad_gazebo.launch software:=false
roslaunch quad_utils visualization.launch
roslaunch quad_utils control.launch controller:=<your-controller-arg>
```
7. Get your code downloaded onto the TX2.
   - Commit your code to your branch <your-branch-name> and push to GitHub
   - Follow the [instructions](https://github.com/robomechanics/quad-software/wiki/2.-Running-the-Robot) to power the robot on
   - Connect an ethernet cable from the wall to the robot to get internet access
   - Run the following commands from a terminal on the remote computer:
```
ssh ghost@192.168.8.101
cd ~/catkin_ws/src/quad-software
git fetch
git checkout <your-branch-name>
git pull <your-branch-name>
cd ~/catkin_ws
catkin_make
```
8. Compile your code:
```
cd ~/catkin_ws
catkin_make
```
9. Set up the robot environment (answer "n" to launching the robot driver when prompted by `launch_robot_env.sh` as this will launch the inverse dynamics controller by default):
```
cd ~
source launch_robot_env.sh
roslaunch quad_utils robot_driver.launch mocap:=true controller:=none
```
10. On the remote computer, launch your controller:
```
cd ~
source launch_remote_env.sh
roslaunch quad_utils control.launch controller:=<your-controller-arg>
```
Notes
   - You can launch your controller directly on the TX2 if preferred, but keep in mind that if you don't have any safety checks and the connection to the TX2 cuts out you won't have a way to stop your controller from running! If you run it on the remote computer and the connection drops, MBLink Converter will notice that it hasn't received new messages and will just send zeros.
   - Launching the remote driver is optional - it will display the robot on RViz but only if mocap is online and connected.