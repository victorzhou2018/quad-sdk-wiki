The current procedure to generate terrain maps compatible with both Gazebo (which needs a .stl) and the GridMap package (which needs a .ply) is as follows:

1. Create the terrain map as a part in CAD software such as Solidworks
2. Save the part file
3. Save the part as a .stl with the following custom settings:
   - Output as: Binary
   - Units: Meters
   - Resolution: Custom, Define Maximum facet size: 0.20m
4. Save the part as a .ply with the same settings above
5. Put all three files (.sldprt, .ply, .stl) in a folder with the same name as the part
6. Put a copy of the folder in quad-sdk/quad_simulation/gazebo_scripts/worlds
7. Make sure the initial states of the robot in spirit_gazebo.launch are within the feasible region of the terrain.
8. (For RML users: upload that part to [Box](https://cmu.app.box.com/folder/140471289581))