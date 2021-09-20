# Gazebo Simulator

**Run Gazebo Simulator**
To launch the Gazebo simulator, run:
```roslaunch spirit_utils spirit_gazebo.launch arg:=value```

Refer to [Launch, Node and Topic Structure](https://github.com/robomechanics/spirit-software/wiki/Launch,-Node,-and-Topic-Structure) for roslaunch commands to run code on the robot. For example, stand up and track twist trajectories while logging with:
```
rostopic pub /control/mode std_msgs/UInt8 "data: 1"
roslaunch spirit_utils planning.launch global_planner:=twist mpc_type:=nonlinear logging:=true
```

Optional arguments:
| Arg Name      | Default Value | Description |
| ------------- | ------------- | ----------- |
| gui           | false         | Loads gazebo gui |
| paused        | false         | Start gazebo in paused state |
| software      | true          | Launch software stack (robot driver and visualization) |
| live_plot     | false         | Launch the live plotter |
| world         | flat          | Select the world to simulate (`step_20cm`) |
