## autonomy_example
Examples of sending goals to move_base

Author: MC
### Files:
- send_fixed_goal      - Sends goal to `/move_base_simple/goal`
- send_fixed_goal.py   - Python version of code script above
- send_single_goal     - Sends goal to `/move_base_simple/goal` specified by input arguments
- send_single_goal.py  - Python version of code script above
- simple_master        - Uses actionlib server to repeatedly move robot to three points
- advanced_master      - Same as 'simple_master', but has feedback 

### Get point on the map from RVIZ
1. Run: `rostopic echo /clicked_point`
2. Click 'Publish Point' in RVIZ
3. Click in map

### Send goal from terminal
Run `rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}' `

### Read move_base status
Run: `rostopic echo /move_base/status`

### Documentation
[actionlib][1]
[move_base][2]

### Send single precompiled goal (cpp)
Run: `rosrun autonomy_example send_fixed_goal.cpp`

### Send single goal defined with parameters (cpp)
Run: `rosrun autonomy_example send_single_goal _x:=-5 _y:=-3 _a:=3.14`

Parameters:
- x - x coordinate
- y - y coordinate
- a - angle in radians

### Send single fixed goal (python)
Run: `rosrun autonomy_example send_fixed_goal.py`

### Send single goal defined with parameters (python)
Run: `rosrun autonomy_example send_single_goal.py _x:=-4 _y:=-3`

Parameters:
- x - x coordinate
- y - y coordinate

### Simple actionserver (cpp), 3 points
Run: `rosrun autonomy_example simple_master.cpp`

### Advanced actionserver with feedback (cpp), 3 points
Run: `rosrun autonomy_example advanced _master.cpp`

#### Create package prepared for actionlib and move_base
1. Run: `cd ~/catkin_ws/src`
2. Run: `catkin_create_pkg MYPACKAGE roscpp actionlib actionlib_msgs move_base_msgs`

[1]: http://wiki.ros.org/actionlib
[2]: http://wiki.ros.org/move_base
