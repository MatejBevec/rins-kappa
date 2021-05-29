
Packages:

`src/world` - world launch, world file, meshes, arm 

`src/navigation_stack` - amcl params, amcl launch, map

`src/main` - glavni (navigacijski) node(i), ostali nodi so lahko posebej

Launch:
1. `roslaunch world rins_world.launch`

2. `roslaunch navigation_stack amcl_simulation.launch`

3. `roslaunch turtlebot_rviz_launchers view_navigation.launch`

   [todo] Launch vseh ostalih node-ov.



Cylinder detection:

1. `roslaunch exercise6 find_cylinder.launch` - detection
2. `rosrun main cylinder_filter.py` - filtering and color classification

