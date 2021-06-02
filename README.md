
Packages:

`src/world` - world launch, world file, meshes, arm

`src/navigation_stack` - amcl params, amcl launch, map

`src/main` - glavni (navigacijski) node(i), ostali nodi so lahko posebej

Launch core:
1. `roslaunch world rins_world.launch`

2. `roslaunch navigation_stack amcl_simulation.launch`

3. `roslaunch turtlebot_rviz_launchers view_navigation.launch`

   [todo] Launch vseh ostalih node-ov.

Launch detectors:
  * All together: `roslaunch main detector_nodes.launch`
  * Separately:
    1. `roslaunch exercise6 find_cylinder.launch`
    2. `rosrun exercise6 face_localizer_dlib_new`
    3. `rosrun exercise6 face_detector_node`
    4. `rosrun exercise6 detect_rings`


Params:
  * Cylinder detection:
    * `cylinder_segmentation -> cloud_cb() -> NUM_POINTS_THR` - required number of inliers for cylinder detection
    * `cylinder_filter -> NUM_THR` - required repeated detections in one spot for conformation
