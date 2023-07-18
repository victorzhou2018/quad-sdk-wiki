Users can add a new robot to Quad-SDK by adding a number of files describing their robot. Unitree A1 and Ghost Spirit 40 are currently supported and can be used as examples when adding a new robot. 

***

## 1. Create Robot Description Files
In the `/quad_simulator` package, modify the following: 

- Add a `/quad_simulator/<robot_name>_description` folder which contains sdf and urdf to be loaded by Gazebo and RBDL. To enable the quadruped to work with the existing codes, the name of joints and links should be in the order of:

- **`body`**
- **``8``** - **`hip0`** - **``0``** - **`upper0`** - **``1``** - **`lower0`** - **``jtoe0``** - **`toe0`** 
- **``9``** - **`hip1`** - **``2``** - **`upper1`** - **``3``** - **`lower1`** - **``jtoe1``** - **`toe1`** 
- **``10``** - **`hip2`** - **``4``** - **`upper2`** - **``5``** - **`lower2`** - **``jtoe2``** - **`toe2`** 
- **``11``** - **`hip3`** - **``6``** - **`upper3`** - **``7``** - **`lower3`** - **``jtoe3``** - **`toe3`** 

where `0-11` are actuator joints and ``jtoe<0-3>`` are fixed joints. All the rest are links. The legs are in the sequence of: front left, rear left, front right and rear right. The names of collision plugin for toes have to be ``toe<0-3>_collision``.

We recommend to change xacro files and generate the sdf and urdf through
```
rosrun xacro xacro --inorder -o urdf/<robot_name>.urdf xacro/robot.xacro DEBUG:=false
gz sdf -p urdf/<robot_name>.urdf > sdf_mesh/<robot_name>.sdf
```

In the `/quad_utils` package, modify the following:

- Add a `/quad_utils/config/<robot_name>.yaml` file. 

Most launch files default to `robot_type:=spirit`. When calling any launch file with robot_type as an argument, include: `robot_type:=<robot_name>`

***

## 2. Modify Launch Files
`/quad_utils/launch`
- Add case to use robot_name_description files in `load_robot_params.launch` and `load_global_params.launch`
```
    <group if="$(eval arg('robot_type') == '<robot_name>')">
        <rosparam command="load" file="$(find quad_utils)/config/<robot_name>.yaml" />
        <param name="robot_description_sdf" textfile="$(find <robot_name>_description)/sdf_mesh/<robot_name>.sdf" />
        <param name="robot_description" command="cat $(find <robot_name>_description)/urdf/<robot_name>.urdf" />
    </group>
```

**Successful completion of steps 1 and 2 can be tested by launching the simulator**
```
roslaunch quad_utils quad_gazebo.launch robot_type:="<robot_name>"`
```
Robot should drop into environment in sitting position.

***

## 3. Generate NMPC Controller Files
* Install Matlab
    * Ensure Matlab's "Symbolic Math Toolbox" is included in installation
    * Install Casadi for Matlab (https://web.casadi.org/get/)
* Download files from quad-sdk/nmpc_controller/scripts/
    * Change `parameter.name` on line 36 in main.m to your <robot_name>
* Run matlab scripts to generate custom NMPC files 


## 4. Implement Custom NMPC Controller Files
- Upload generated .cpp files to `/nmpc_controller/gen/`
	- eval_g_<robot_name>.cpp
	- eval_hess_g_<robot_name>.cpp
	- eval_jack_g_<robot_name>.cpp
- Upload generated .h files to `/nmpc_controller/include/nnmpc_controller/gen/`
	- eval_g_<robot_name>.h
	- eval_hess_g_<robot_name>.h
	- eval_jack_g_<robot_name>.h
- Add references to newly generated nmpc files
	- Add to add_library(…) list in `/nmpc_controller/CMakeLists.txt`
	- Add include statements in `/nmpc_controller/quad_nlp.h`
- In `/nmpc_controller/include/nmpc_controller/quad_nlp.h`
	- add **ROBOT_NAME**  to `enum SystemID { }`
	- modify `static const int num_sys_id_ = 6` to match number of elements in `enum SystemID { }`
- In `/nmpc_controller/src/nmpc_controller.cpp`
    - Add case to `switch (robot_id_)` with `robot_ns_` string and default_system `SystemID`
    - Ensure case number matches order in `enum SystemID { }` list from *quad_nlp.h*
- In `/nmpc_controller/src/quad_nlp.cpp`
    - load basic leg controller functions for the **Robot_NAME** platform

***

## 5. Update Local Planner
`local_planner/src/local_planner.cpp`
- Add case for `robot_name_` == “robot_name” to set `type` = ROBOT_NAME

***

## 6. Robot Driver
- Create custom hardware interface
    - Save as
        - `robot_name_interface.cpp` in `robot_driver/src/hardware_interfaces`
        - `robot_name_interface.h` in `robot_driver/include/hardware_interfaces`
- Add call to custom hardware interface
    - `hardware_interface_ = std::make_shared<RobotNameInterface>`
- Add interface files to `robot_driver/CMakeLists.txt`
- Add include statements for interface files in `robot_driver/include/robot_driver.h`


**Successful completion of steps 1-6 can be tested by launching the simulator and standing the robot**
```
# Launch simulator
roslaunch quad_utils quad_gazebo.launch robot_type:="<robot_name>"

# Stand robot
rostopic pub /robot_1/control/mode std_msgs/UInt8 "data: 1"

# Walk with global planner
roslaunch quad_utils quad_plan.launch robot_type:="<robot_name>"
```

***

## 7. Tune Parameters
If the robot model is not working correctly in simulation, tune the robot's parameters in `/quad_utils/config/<robot_name>.yaml` and the `desired_height` parameter in `/local_planner/local_planner.yaml`