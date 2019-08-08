::Shortcuts::
	[Ctrl] + [Alt] + C  =  opens crazyswarm folder
	cfscripts  =  cd crazyswarm/ros_ws/src/crazyswarm/scripts
	flash51 = flashes nrf51 firmware
	flash32 = flashes stm32 firmware
	hoverswarm = roslaunch crazyswarm hover_swarm.launch
	mocaphelper = roslaunch crazyswarm mocap_helper.launch


- CFDemo_TRAIL.py //crazyswarm/ros_ws/src/crazyswarm/scripts
 ~ main python demo file that executes all swarm configurations
 
   Demo Source Folders
   coordfiles - .yaml Drone_Coordinates, different shapes for static formations
   trajfiles - .csv trajectories, matlab generated csv trajectory files for dynamic flight
   wayfiles - .csv waypoints, hardcoded coordinates assigned to specific cfs for waypoint flight

Talkers: //crazyswarm/ros_ws/src/crazyswarm/scripts
- TRAIL_talker.py //original talker with edits
- TRAIL_multitalker.py //calls coordinates from Demo.yaml
- PolyData.py //calls coordinates from Polygons.yaml 

Listeners: //crazyswarm/ros_ws/src/crazyswarm/scripts
- TRAIL_listener.py //original listener with edits
- TRAIL_multilistener.py //todo: include dynamic flight patterns; swarm moves together retaining formation from coordinate assignments

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
- pathgen.m -> matlab function used by trajectorygen.py to make trajectory paths based on user input coordinates

Other (created) python files:
- avoidtarget.py -> selects targetCF to control by teleop and be avoided by other cfs
- avoid_trajgen.py -> similar to avoidtarget but generates trajectory paths when avoiding target for smoother flight
- cf_teleop.py/c -> script defining teleop class to control CFs in avoidtarget
- downwashtest.py -> simple script that sends two CFs to fly directly above each other with a given height difference
- matlabtrial.py -> just used to practice using matlab.engine to run matlab fxns in python
- modwaypoints.py -> modified waypoints script that ideally takes the active cfs as agents instead of pre-determined
- testGroupEd.py -> just edited version of testGroup to see if groupmasks actually work
- trajectorygen.py -> takes user input and generates trajectory flight path for cfs to follow

