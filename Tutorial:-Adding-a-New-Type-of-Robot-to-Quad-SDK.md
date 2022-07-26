Users can add a new robot to Quad-SDK by adding a number of files describing their robot. Unitree A1 and Ghost Spirit 40 are currently supported and can be used as examples when adding a new robot. 

In the `/quad_simulator` package, modify the following: 

- Add a `/quad_simulator/<robot_name>_description` folder which contains sdf and urdf to be loaded by Gazebo and RBDL. To enable the quadruped to work with the existing codes, the name of joints and links should be in the order of:

- **`body`**
- **``8``** - **`hip0`** - **``0``** - **`upper0`** - **``1``** - **`lower0`** - **``jtoe0``** - **`toe0`** 
- **``9``** - **`hip1`** - **``2``** - **`upper1`** - **``3``** - **`lower1`** - **``jtoe1``** - **`toe1`** 
- **``10``** - **`hip2`** - **``4``** - **`upper2`** - **``5``** - **`lower2`** - **``jtoe2``** - **`toe2`** 
- **``11``** - **`hip3`** - **``6``** - **`upper3`** - **``7``** - **`lower3`** - **``jtoe3``** - **`toe3`** 

where `0-11` are actuator joints and ``jtoe<0-3>`` are fixed joints. All the rest are links. The leg is in the sequence of: front left, rear left, front right and rear right. The name of collision plugin for toes have to be ``toe<0-3>_collision``.

We recommend to change xacro files and generate the sdf and urdf through
```
rosrun xacro xacro --inorder -o urdf/<robot_name>.urdf xacro/robot.xacro DEBUG:=false
gz sdf -p urdf/<robot_name>.urdf > sdf_mesh/<robot_name>.sdf
```

In the `/quad_utils` package, modify the following:

- Add a `/quad_utils/config/<robot_name>.yaml` file. 

Most launch files default to `robot_type:=spirit`. When calling any launch file with robot_type as an argument, include: `robot_type:=<robot_name>`
