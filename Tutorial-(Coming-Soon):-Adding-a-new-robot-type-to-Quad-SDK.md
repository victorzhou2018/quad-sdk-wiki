Users can add a new robot to Quad-SDK by adding a number of files describing their robot. Unitree A1 and Ghost Spirit 40 are currently supported and can be used as examples when adding a new robot. 

In the `/quad_simulator` package, modify the following: 

`/quad_simulator/gazebo_scripts/config/<robot_name>_control.yaml`
Add a `<robot_name>_control.yaml` file, this file sets joint controllers and contact publishing rates. 

`/quad_simulator/<robot_name>_description`
Add a `<robot_name>_description` folder containing: materials, meshes, sdf, urdf

In the `/quad_utils` package, modify the following:

`/quad_utils/config/<robot_name>.yaml`
Add a `<robot_name>.yaml` file in this location. 

Most launch files default to `robot_type:=spirit`. When calling any launch file with robot_type as an argument, include: `robot_type:=<robot_name>`

