- tasks done with description
*- tasks started but unfinished within week
>- continuing tasks from previous week
**TODO
~? Questions

|||May 20 - 30|||

- Edited Crazyswarm code by Ben&Sammy 
	- fixed typos in comments
	- renamed 'listener' and 'talker' nodes to 'TRAILGoalData' and 'TRAILGoalAssign' 
	- Created a 'fliecount' variable that takes the number of active crazyflies to create the positionMatrix size to avoid editing the node with each use
	- Corrected use of duration matrix used in Hungarian Algorithm 
		- the shortest duration used to determine the goals for the crazyflies should be a diagonal across the matrix ('duration[c][c]') rather than down one column
	- Added to 'sendCFtoGoals' function
		- Reassigning extra active crazyflies to a hover height about their original positions if #ofcfs > #ofgoals 


- Added new .yaml files for different formations 

- Created copy crazyswarm nodes for multi-formational demos (TRAILDemoTalker & TRAILDemoListener)
 - ~Data node edits
	- Demo.yaml file includes multiple formation coordinates that each are ended with a unique final character label	
	- Created array that includes single character extensions of labeled coordinates in .yaml file
	- Created for loop that goes through the label extensions to publish the coordinates under each labeled formation onto the topic 'Assignments' 

 - ~Assign node edits
	- retained same code structure as original GoalAssign code as it runs through each message published onto the topic by the DemoData node

- Used crazyswarm matlab functions to generate trajectories for dynamic swarm flight

*- Added dynamic trajectory flight to crazyswarm nodes
  - ~Data node edits
	- created separate call functions for publishing the coordinate data for the static formations ('formationsData') and for publishing the trajectory .csv file name (dynamicsData)
  - ~Assign node edits 
	 - created a dynamicsCallback function to execute the .csv file and make the swarm of synchronized flies follow the trajectory path 
  - currently code just runs through all static formations and dynamic formations in sequence separated by the defined sleep times in the code
  ** ideally, the formation changes and dynamic trajectories would be executed after receiving specific input from the user so that different series of demonstrations can be done 

**do a test run on drones with qualisys


|||June 3 - 6|||

**test run drones individually using virtual mobile controller (maybe figure out why cf#2 is broken) 
**test run drones with python codes (og crazyswarm stuff) 
**test run drones with developed ros nodes 
**refine code structures
**figure out avoid obstacle set up (was removed from open source by USC team)
**figure out ellipsoid limits (was also removed from latest versions)

- checked start up functions of all crazyflies 
 - only 23 cfs are present (one is in Dr. Dames' office lol) 
 - #23 has broken connector for one propeller
 - #7 needs charging
 - #2 starts up fine (but Jun said it doesn't fly properly, **troubleshoot) 
 
- found possible extra source: Georgia Tech GRITS lab
 - also worked on collision avoidance using 'top hat' model (similar to ellipsoids)
 - allows for close range flight
 - paper isn't published yet? but was presented in 2017?? **double check
 
- CFDemo_TRAIL.py started new code formatted after crazyswarm files
 - singular python code that runs through the different types of swarm flights
 - takes in user input for flight type selection and source files
 - runs static formations (as long as all source yaml files are within coordfiles folder)
 - runs dynamic trajectories (all source csv files are within trajfiles folder) 
 - basic takeoff and land functions 
 *- incorporate waypoints from csv files
	- waypoints code has been incorporated but requires the appropriate cf.ids
	  to be coded into the csv file (or else it will error out)
 ** interactive demos, avoid and follow target 

[ELLIPSOIDS/DOWNWASH] 
**Ellipsoid/Downwash effect incorporation should be done in flight trajectory
generation possibly goTo function or startTrajectory 
  ** Crazyswarm said to inquire through issues on Github if function is needed? 

[RESEARCH]
**READ Georgiatech papers (they have been found, dissertation and journal article) 
 **get an idea of how avoidance between drones was incorporated 
 
[AVOID TARGET]
- Found crazyswarm files pre-removal of avoidTarget 
 - avoidTarget features aren't in firmware and don't show up in recorded edits on github 
 - build errors regarding avoid target and plan_start_avoid_target
 *-go through firmware files and other possibly related files for edits on avoid target
 **Update the current crazyswarm folder with avoidTarget functions
   (and edit based on developments made since removal) 
   ++ firmware updates aren't included in documentation (manually go through codes) 
 **create a publisher node that takes the real time position of the target
   the mocap system which feeds into both the avoid and follow functions
   ~~Issue #54 'Interaction with objects in real time' 
		talks about whether the data should be processed on board by the cfs
		or by the pc and coordinates are loaded while on the ground 
 
 -- avoidTarget package built (found avoidtarget.h and --.c
    module files for firmware from old commits (16 aug 2016)
   
 **OR find an easier way to incorporate interactive demos 
   (code functions into active code instead of using modules)


**Start isolating important codes and files and build our own crazyswarm folder with only the necessary files 
  ~ to make it easier for future editing and documentation of our own developments
  


~~~~then we can physically test the drones and see if they collide and crash and die~~~~


~? which workstation should the codes be run from and connecting to the ethernet
---just connect to the ethernet cable lol 
~? not doing real experiments or runs yet should i reserve the space (lol) 
~? do we want to coin a name for the study that is not crazyswarm cuz IP (also lol) 
 
||| June 10 - 13 ||| 
*-AVOID TARGET - transfer things from the old code
*** COME BACK TO THIS AFTER PHYSICAL TESTING FOR OTHER PROCESSES ARE FINALIZED
- OLD CODE
	- avoidTarget firmware has a lot of TODO tasks and questions in the particular version
	** maybe find the latest possible accounts of the firmware (unless it already is)
^^^too messy, focus on physical demo first for demo on Jun 24 or not who knows sheesh lol 

**PHYSICAL TESTING
  **set up crazyflies (radios and stuff) 
  -- Flashing of firmwares done for cfs 4, 5, and 6 (but all of the drones were all probably updated by jun)
  **run qualisys (calibrations)
  **flyy  
== cfclient bug -- probably just incompatible with python 3.6
	- not running with weird error (same experience with Jun)
  >>> jk worked it out by running Python 3.5 (using python and pip insteat of python3 and pip3 which were python 3.6)

* finish bootloading drones
* maybe add on the markers to the rest of the drones? 
* start flying???? 
 - yeah we gotta start flying 
 - do we have a meter stick lol aaaand masking tape (need to label starting points and stuff) 
 - also how to put away the treadmill (should i put away the treadmill for initial testing???? 
*avoid target stuff is a bit messy right now -- we'll get back to that
  
||| ***crazyflies that need charging:  |||

//19, 20, 21, 22, 23, 24, 2 are not configured with address and radio channel?
//24 not bootloaded (up in office)
//22 - blades aren't spinning all the way?
//marker decks were added to remaining crazyflies except those that are not configured

- Qualisys can be recalibrated (longer time) but for when more drones are going to be flown
- Always roslaunch crazyswarm hover_swarm.launch so that rviz launches 
 *-- cf doesnt show up on rviz
 *-- Error message: Could not find crazyswarm_server and crazyswarm_teleop
		- caused by build errors of make -j4 -l4
		- fixed with sudo pip install empy
 *-- might not be properly connecting to the qualisys system (recheck host name) 
-- when launching individual_hover.py, no error shows up. just doesn't work lol  

**Try again with qualisys
**put in reservations for the lab (ask for calendar access so we can schedule testing next week)

:for future reference: terminator and gnome don't work with python3 :/

*-> well now things work except 
	- LED ring is not showing connectivity status
	- drone is not flying
*-> print_latency: True > now drone starts up 
	- but it only lifts off a little bit then drops before land command
	- can't hold it's height lol
*-> tried it with another drone CF18
	- doesn't fly at all
	- doesn't respond to chooser.py sysOff and reboot
	- just reboots everytime flight code is run 

**keep trying and find out what's wrong
 possible sources:
 -> could be the drones settings for flight (thrust, roll, pitch, yaw etc.) 
 -> connection problems 
 -> code problems

||| June 17 - June 20 |||
>- Physical Testing Troubleshooting
 --> go through codes make sure everything is according to instructions 
 --> double check with crazyswarm codes from previous tests (jun and sammy&ben)
 --> try try again 
 
- something is wrong with current files (ros_ws/src/crazyswarm--) 
 -- qualisys wasn't fully connecting with code from latest update
 -- Jun's code works refer to folder, compare and find out if anything is different/the cause
 
~~with Jun's code
 - drones are responding and starting flight but not maintaining enough thrust to hover
 - when niceHover is used they start flying but don't make it to height or position
 - they just fly off ish, there is a slight difference in flight pattern 
	when 'flight controls' in cfclient are adjusted but minimally so
	#13 has broken LEDs? or it might be a battery thing -- it was a battery thing
	#18 just restarts when flight codes are run
   ** maybe try other drones and/or multiple drones 
   **but they really aren't hovering which is a problem
   ** find out if there are recommended settings for flight controls
   
- CF#18 and others had wrong marker configurations bcuz i am dumb
 - probably why it wasn't working in the same way as CF13 

~~Back to crazyswarm code 
 - changed LED settings in hover_swarm.launch (hopefully this should make it flash red)
 - might be the only difference cuz it was sort of just lifting off the first times
 - * changed the mass to 0.035 (includes markers) 

TODO: tomorrow morning (6/18)
- Re-calibrate qualisys (cover area around the treadmill and further out at least 2m about origin)
- Get new marker configuration values 
- try flying 13 and 18 again (now with cfclient flight controls Target pitch&roll: 0.00)
- Try flying with different flight control settings (take videos? compare the differences?)
- set up new initialPosition grid to match the treadmill set up ~ 0.3 distances between drones? 

TESTING
#13 hovers nicely and works after re-doing marker configurations 
BUT 
#18 just restarts when given flight commands still
#1 flights crazy lol (one up to ceiling, another into net)
--- could have been a firmware issue reflash everything cuz #3 worked 
FLASH using rosrun crazyflie_tools ~~ bc make cload doesn't work?? 

BUT Individual hover code works, niceHover does not for #3? but it works for 13 idk why 
CFDemo code works with two drones 
- #3 doesn't land properly 
- Could be a downwash issue? too slow to land??
- might be a flight issue try with other drones
- #13 lands fine

Try more drones, reflash firmware, start coordinates were changed to 0.4 distances marked on mats
Try up to at least 5 drones in the air the same time yay

#18 Really doesn't work even after reflashing try again some other time
#4 #11 #8 Blade mounts are broken (#8 also marker mount)
#3 one blade is not fully installed
#9 on/off button is broken
#16 isn't flying properly grr


Statics:
	Square - at least one doesn't make it to formation
	Triangle - one usually doesn't land back 
	
Dynamics:
	Start of fig 8 works then they start flying off 
	Single drone manages most of first 8 but crashes before finishing at height = 0.5 
	might be affected by treadmill?
	even with circletraj it'll start out okay but start to shake and crash
	Try smaller shaped trajectories to minimize the travel 
	idk if higher height would help?
	Could be a calibration issue? 
	Re-calibrate properly (The whole space, add more time)
	
Downwash might be affecting drones

-- Drones 19-22 have been configured, flashed, and setup 
-- try testing if they work along with #2, #18, #7 

if everything works we should have 21 working drones hopefully

Do drone tests with 6 drones at a time and try to get up to 6 in the air together in formation
and hopefully doing dynamic trajectories

** Edit coordinates of static formations to stagger x-axis even for basic shapes (downwash seems stronger than expected)
** Maybe tweak the sleep time of demo code OR make filenames easier to type lol 
** Redo calibration to extend past treadmill (hopefully can be reused for monday demo)

#2 doesn't even connect to cfclient (might need to be reconfigured)
#18 is still just restarting even after reflashing 
#20 flies bad (might be issue with propeller or motor) can hover but leans to one side if hovering for too long at low height
#4 doing the same thing as #18 just restarting and #16
#3 is a bad flyer-- takes off but can't hover


Okay ish drones (okay individually but not good with other drones at the same time?)
#5 is okay but not for long term flight
#17 nvm might have been hardware issue
#6 is also   

#22 shakey flight but flies fine more or less

Good drones: 13, 14, 15, 4, 5, 12, 10, 8, 11, 1, 19, 21, 20, 7, 22 

Statics: 
	Successfully did 5 (pentagon) (one didn't make a proper landing but that depends on the drone)
	
Dynamics:
	Successfully did 3 at the same time with reverse for half8
	smaller shapes are easier
	2 at the same time for fig8 works too (just make sure starting points are distanced enough)
	
Waypoints:
	4spin works with cfs 5,13,14,15 but cf5 crashes near the end (might be weight distribution problem or smth similar
	you can create more waypoints? 

Multiple
	3 drones (13,14,15) = dynamic (half8, reverse) -> static (3) -> hover -> land 
	
||| June 24 - 27 ||| 

Demo for WE2! 

**Charge as many extra batteries as possible
**Get set ups ready 
Demos you can use: 
Static: 2,3,4,5,6 (at least once try T13 maybe for the last one) 
Dynamic: half8 and half0 for multiple (maybe 4 or up to 6 if well spaced)
	fig8 and 4corners if only one drone 
Waypoints: 4spin, 4 drones (spaced to start according to assigned points) 
		6inout just 6 drones in hexagon go to triangles 
	edit csv file to match IDs 
* let students come up with their own shapes/coordinates using 6 drones
  - max height 2.5? max width x: (1.3,-1.3), y: (2,-2) 
	


~~~ after demo to do ::General::
** maybe try to isolate important parts of Crazyswarm package 
** honestly could we rename it probably not but at least can we have a project name or smth

*** Troubleshoot bad drones 
*** github issue requesting ellipsoid features??? 
*** or figure out how to use matlab trajectory generators? 
*** Try to do a downwash test?? fly two drones above each other with recommended distance
    - but they have to go there one at a time

\\ weird drones - can hover but not for long 
#5, #17, #6

\\\ Bad drones 
#2 not even connected
#18, #4, #16 just restart when given flight instructions
#3 starts up but doesn't fly up or hover

-> reflash all of them 
-> reconfigure #2
-> try flying them individually? and time stable flights (for all even the good ones)
 
-> Downwash test, fly two drones on top of each other at varying distances 	
	- make a code for that lol 
     - are downwash effects the same when moving? 

///IG takeover///
Try and do the Temple T13?? 
Troubleshoot the broken drones in flight
Guess we might do some downwash stuff too
And we can bother aidan and ethan too 


TODO: Figure out a way to generate non-pattern trajectories	
	like actual paths and series of instructions
	without the hardcoding maybe? 
TODO: Avoid target seriously pls how even
	- the mocap sys will read the location of the extra markers (that are not drones)
	- these coordinates should be fed in and cross referenced with the current locations of the drones
	- the drones should adjust accordingly to maintain a safe distance 
	- while also taking into account the location of other drones and maintaining that distance
	> find out how data from mocap is understood by crazyswarm package
	> and figure out how to take input for the target's location
	> might need to make a marker configuration for the target (chest T like in USC?)
	> and take that as an existing "object"
	-- use the same basis for Follow 
	- Follow can allow for a moving target/goal area for landing (e.g. turtle bots)
TODO: Labeled planning -- avoid collisions and overlapping flight paths
	^ is it possible to just make end point coordinates assigned to each Cf
	instead of having to use .csv files (but i guess it's the same thing?)
	Unlabeled planning -- smooth trajectory style flights if more complicated start-goal pairs are used
	
	^^try and consolidate and use the same input file types? .csv or .yaml
===eventually, create an algorithm that lets multirobot systems figure out the optimal
	breakdown process to achieve a goal (e.g. land that has drone on board)
	(mocap needs to monitor where the land robots are as well) 

~~~
Fly bad drones
Debug
Try and get 13 good ones and do T13
Downwash testing (using good drones, im sorry bbys)
then start on some boring code things yay (or do them first? uuuhhhhhmmm)

DEBUGGING
Restarts: 4, 16, 5, 3, 18 ---> their blades start to spin but not enough for thrust and just restarts
Fly off random direction: 3
Connects to RVIZ but didn't move: 18
Hovers okay ish: 6, 10, 17
16 sometimes connects to chooser (needs a few reboots)
16 and 18 restart late?

WEIRD THINGS
When flying 14 ish drones, #7 gets stuck midflight (really don't know why that happens)
most of the drones then start to crash 
eventually code crashes and drones just stay in their current positions with no way of stopping their flight

***Things to try
for multiple drones
- Use less radios (try stream lining or only use drones addressed to the
  same radio and fly by batch to isolate issues in connectivity possibly?) 
- try using a more simple hardcoded version that sends a larger number of
  drones to a simpler formation (T13 might have been crashing because the 
  'duration' wasn't returning realistic values from the drones that have crashed)
 
for CF #2
- try 'rosrun crazyflie_tools comCheck --uri radio://0/90/2M/E7E7...'
- look into state estimations? 
- but it might be specific to cf#2 cuz everyone else works
- check the hardware (try replacing parts one at a time, and take slow mo vids of flight?)

for the bad flies
- still don't know lol 
- maybe try updating crazyswarm package???
- then reflash... again...
- or start with a new crazyswarm package to reflash them? 
	(but if all the other drones worked with the same flashing
	then these should too) 
	
Cf#2 + other bad flies:
- get them to connect to the phone app and try to individually control them
 
  
**TODO:
	> try out the e-stop function allcfs.emergency() call python stop.py
	> make a backdoor labeling system that shuts down or at least registers when a drone crashes? 
	> ^might avoid the code crashing mid flight
	
**Learn how to use github lol pls and ros lol i still don't know what im doing :(
**learn more about drones and Th,Pitch,Yaw,Roll and aerodynamics for it
**take a closer look at the circuit board of the crazyflies
*- figure everything out i hope
*** we can look into qualisys to check for how it takes in object tracking
**- also actually figure out to record and process the data we can get from the system
*** look into other researches that have used crazyflies (and come up with something cooler to do!!)

||| July 1-4 |||

> connect bad flies to phone app to check for hardware/connectivity issues
> run a few physical tests for large swarm flying (radio issues, or change the formation (make it simpler), or try a simpler goTo code)
> find out why the LED doesn't reflect the connectivity color thing

Android crazyflie client
- #2 physically is fine? try testing it again cuz wtf
- all bad flies are fine with the app (not great at flying but that's a controls issue

*find out how to monitor the connections and data while testing physically for the bad drones
**put up github issue @crazyswarm to ask about bad drone errors

> re-try large swarm (honestly could have been a battery issue at some point) 
> but try and narrow down connection channels

what shall we do about follow and avoid target :<< 

Debugging
-- some of the bad drones were fixed with re-flashing firmware (3, 5, 16 but idk if it's a perm fix)
	^^ no idea how that worked but okay
-except: #18, #4
- #2 is a bit more in control but not really? 
  based on LED pattern, it has weak/unstable connection (green flickering to a faint red) 
  this is probably the reason for bad flight
  
  (didn't get to try large swarm yet and that's a connectivity issue which requires a lot of the debug monitoring) 

Avoid target
- checks if there is a marker within proximity of the cf
  > needs to include position of extra marker directionally relative to the cf
  > Could be based on 6 sides surrounding
  > uses that info to know which way to adjust (oppposite direction) 
  ? how bout multi drone collisions (drones on either side)
  ? moving drones and moving target
  
  
/Follow target
  - can be added as a service in the python API stuff (along with goTo, takeoff, etc.)
  - with variable distances (to the target) and type of follow
  eg. crowd around target, transpose original formation a certain distance in the direction of the target
  or follow in the footsteps of the target (make sure only one drone at a time)
  - or follow one cf (only generate one set of waypoints for that one ID) 
  let other cfs follow behind (like duckiebots??)
/Avoid 
  - to start can also be ^^
   but eventually it would be better as a running condition for drone flight
   more like a safety mech that makes sure they don't crash into anything regardless
   of the flight path or trajectories given
   more similar to ellipsoid than actually avoid target
   
   But for demo sake, you can use avoid target code (fly to random heights and pass target through)
   
~ add Follow.srv to crazyflie_driver? 
	- self, bool true/false??, position/location of target, float follow distance (in xy plane only? maintain z?)

* Find out where or how to use Debug

What if:: we made the target a 'crazyflie Type' so that qualisys/RVIZ registers it
it just can't be controlled the same way (but check crazyflie attributes since we only need it's location) 
- if we can get it's location as if it were a CF then we can use that in avoid target
- and we can introduce more than one target at a time 
- if it doesnt crash lol
- use cf.position(crazyflieById[0])
**- find a way to make this work in simulation? 
	-> represent not as just one dot (like cfs) cuz human sized T 
	-> ?what happens if bigQuad:True for cfTypes?
**** work on this to see if it's even possible? 
** if not we just need to define a new object in similar way as crazyflies
	but we only need to keep track of its position which should be easy /should/ 
	
||| July 8-11 ||| 

ACC Conference? Tues, wed, thur 11-3 
STEM Camp Demo? Monday? 8 or 15 both i guess lol 1PM

- going through crazyswarm package to find files that are related to how the mocap system
  detects and identifies the cfs 
  - create target (might need its own folder? but its not being controlled so maybe not
  - make simple code that compares locations and stuff 
  
(how are the bad bots lol)

--- find out how to incorporate the target
--- make a target with marker configuration (T with markers) 
--- figure out how to make it work in simulation?? 
	- mouse as target
	- create target with starting point that can be teleoped via keyboard

  *~~ can we do a dancey drones thing lol (make the drones move with music)
  *~~ and a Temple T Cheer formations 
  **~ come up with new demos
  


||| July 15-18 |||

*- make the target a type of CF (or a designated CF) so we can incorporate follow/avoid
*- if one CF is used as the target we can make a script that will follow the trajectory or location of the one CF
  - either takes the same source file from the initial CF 
  - or get it to actively track the location of the TargetCF and go to the same points with a delay
	^ so that only one fly needs to be given a trajectory and the rest can just follow

**-- Should Target be added as a bool feature on CFs? 
		> for now just manually assign one CF by ID but eventually? (but why have multiple targets??)
		> target CF can be input from user and called as targetCF
		
-- making code that uses one CF as a target and avoided by other cfs		

** Using waypoints (follow) and trajectories (avoid)
	-> need to find a way to put the loop that checks for current positions
	   while the target drone is moving 
** Find a teleop function to control the cfs (only to control the targetCF)
	-> with each command sent in the loop should run to check positions
	-> teleop would only work on targetCF 
	-> all other CFs would only move as a response from the status checks on target
	-> still needs to run setup for random heights
	-> live input from user for target 
	-> can use the same checking loops for a non-cf target 

- new: avoidtarget.py
 - code runs through user input to set up for avoid target algorithm 
 - one cf is isolated as the targetCF and can be controlled through
   step by step teleop or trajectory
 - distance between each cf is computed at every movement of the target
 *** - adjustment for when cfs are found to be too near each other needs to be added
 
**-- Trying to incorporate MATLAB code into python execution code
  -> So that trajectories can be generated by the code given input of certain points
  -> this can help making the teleop smoother
  -> and can make the avoid adjustments move smoother and allow drones to hover back to their starting point
  
-- Clean up old talker/listener scripts from B&S
-- Make individual python files that do each specific function from Demo file
	- for easier editing and maintenance 
	- also to keep track of the necessary libraries and imports 
	
** Film Bad Drones -- submit issue on git about it
** Adjust static formations towards -x axis
	** or just adjust the origin entirely (to avoid railing)
	** maybe add some drones to the floor and remove the top-most row of CFs
##*** get this ready for ig takeover lol 
	
- Matlab trial code worked using simple functions 
  but complex traj gen fxn from matlab was sending errors about matlab.engine
  **- debug
  *- could be a compatibility issue with crazyswarm package? 
  
- avoid target code has some issues
  - the ordering of processes in the loops are a bit weird
  *- try to make a flowchart? 
  
*- try to regularly update the git repo so we can look through the code outside of the lab

||| July 22-25 |||

>- figuring out matlab-python code (or matlab to ros???) 
^^ yay this workeeeddd
 *** but now try to skip the csv file generation step
	- also so it doesn't have to make a brand new file per cf that needs a trajectory
	*- isolate the data types for trajectory poly pieces 
	 - allow matlab result to generate directly to uploadtrajectory 

 *** or csv format of data can be created to uploadtrajectory from 
	but without creating a new file 
	
	* or only update one file as each new trajectory is generated
	but it has to be used and followed by the assigned cf before being overwritten
	
	
>- figure out the loops to check distances and stuff :<< 

>- plan out thursday takeover lol

**update initialPositions to adjust for treadmill
	* remember to account for -z heights below treadmill 
**adjust static formations in -x axis 
## recalibrate system to essentially have everything in the -x region 

TODO: AvoidTarget
1. Fix moving and checking loops for avoid 
2. Do trajectory gen without needing to create new csv files 
   (one time use data that gets uploaded and executed for cf immediately)
3. Combine traj gen to obstacle avoidance 
4. Find way to take angle between target and other cfs for avoidance
l. Smoothen teleop controls by incorporating trajgen and angles

- cf_teleop can take input in distance and angular direction to move cf
** cfs can't just move in the 'negative' of the distance because the smaller the distance, the less it will adjust
   need to maintain a certain distance (direct not components) 
   *how to determine the appropriate adjustments in the x and y to maintain?

TODO: Testing set up prep 
//1. Plot out new initialPositions for CFs 
//2. Edit static and waypoint files to adjust for -x axis adjustment
3. Make sure all CFs are charged and re-test drones to categorize performance
4. Add new tape markers inside mocap space for drones 

TODO: FollowTarget (if avoidtarget issues are resolved this would be easier) 
		(but can be done first because the loops aren't as much of an issue)
1. similar to avoidtarget stuff in terms of traj gen 
2. just plan this out

TODO: Waypoints
1. use active CFs instead of having to hardcode in csv file
2. make other fun shapes with waypoints


****i know everything is in python rn but if someone in the future is better versed in ROS they can just take the logic and change the syntax****
~~~~ just a thought :)))) ~~~~



