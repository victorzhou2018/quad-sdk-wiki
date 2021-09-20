# Gazebo Simulator

**Run Gazebo Simulator**
To launch the Gazebo simulator, run:
```roslaunch spirit_utils spirit_gazebo.launch arg:=value```

For quick testing, launch with body_planner:=twist and use the twist keyboard controller to manipulate the robot. Pressing number keys corresponds to switching to that control mode (i.e. press 0 to sit, 1 to stand) and use arrow keys to send desired twists to the local planner.

Optional arguments:
| Arg Name      | Default Value | Description |
| ------------- | ------------- | ----------- |
| gui           | false         | Loads gazebo gui |
| paused        | false         | Start gazebo in paused state |
| software      | true          | Launch software stack (robot driver and visualization) |
| live_plot     | false         | Launch the live plotter |
| world         | flat          | Select the world to simulate (`step_20cm`) |
