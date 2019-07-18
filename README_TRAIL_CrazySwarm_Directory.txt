- CFDemo_TRAIL.py //crazyswarm/ros_ws/src/crazyswarm/scripts
 ~ main python demo file that executes all swarm configurations
 
   Demo Source Folders
   coordfiles - .yaml Drone_Coordinates, different shapes for static formations
   trajfiles - .csv trajectories, matlab generated csv trajectory files for dynamic flight
   wayfiles - .csv waypoints, hardcoded coordinates assigned to specific cfs for waypoint flight

Talkers (Data.py files): //crazyswarm/ros_ws/src/crazyswarm/scripts
- TRAILGoalData.py //original talker with edits
- TRAILDemoTalker.py //calls coordinates from Demo.yaml
- PolyData.py //calls coordinates from Polygons.yaml 

Listeners (Assign.py files): //crazyswarm/ros_ws/src/crazyswarm/scripts
- TRAILGoalAssign.py //original listener with edits
- TRAILDemoListener.py //todo: include dynamic flight patterns; swarm moves together retaining formation from coordinate assignments

Launch files: //crazyswarm/ros_ws/src/crazyswarm/launch
- TRAIL_test.launch //original GoalData + GoalAssign
- TRAIL_demo.launch //DemoData + DemoAssign
- TRAIL_poly.launch //PolyData + GoalAssign

Yaml (coordinates) files: //crazyswarm/ros_ws/src/crazyswarm/launch
- Triangle.yaml
- Square.yaml
- TempleT.yaml
- Demo.yaml //triangle + square + TempleT
- Polygons.yaml //triangle + square + penta
- Pyramid.yaml

Matlab (Trajectory Generation) files: //crazyswarm/matlab/trajectorygen
- circlegen.m
- fig8gen.m
- fourcorners.m

