Users can add a new robot to Quad-SDK by adding a number of files describing their robot. Unitree A1 and Ghost Spirit 40 are currently supported and can be used as examples when adding a new robot. 

In the `/quad_simulator` package, modify the following: 

- Add a `/quad_simulator/<robot_name>_description` folder containing: materials, meshes, sdf, urdf. The Name of joints should be same as what stated in the Spirit and A1 urdf. We recommend to modify xacro files and generate the sdf andd urdf through
```
rosrun xacro xacro --inorder -o urdf/new_robot_type.urdf xacro/robot.xacro DEBUG:=false
gz sdf -p urdf/new_robot_type.urdf > sdf_mesh/new_robot_type.sdf
```

In the `/quad_utils` package, modify the following:

- Add a `/quad_utils/config/<robot_name>.yaml` file. 

Most launch files default to `robot_type:=spirit`. When calling any launch file with robot_type as an argument, include: `robot_type:=<robot_name>`

