# socially aware navigation in a museum setting with ontology

## overview

using a lidar scanner and camera, will make socially informed decisions in navigation using ontology

## installation

### dependencies

- **[navigation](wiki.ros.org/navigation/)** (estimating robot's position and setting goals)
- **[p2os](wiki.ros.org/p2os)** (communication with the pioneer)
- **[rplidar](wiki.ros.org/rplidar)** (laser scanning substitute for the pioneer's built in sonar)
- **[darknet_ros](wiki.ros.org/darknet_ros)** (object detection)
- **[usb_cam](wiki.ros.org/usb_cam)** (camera feed for object detection)
- **[leg_detector](wiki.ros.org/leg_detector)** (person detection (not implemented yet))
- **[stage](wiki.ros.org/stage/)** (simulation software)

### building

```
cd your_catkin_workspace/src
git clone https://github.com/UNR-RoboticsResearchLab/san_museum.git -b aaron
cd ../
catkin_make
```

## basic usage

have a robot setting random goals relative to its current position
all commands assume terminal is at `your_catkin_workspace` path

### for simulation

1. create or get a map
2. set map in sarg_2dnav_sim.launch
3. `roslaunch sarg_bot simulation.launch`

you should see something like this:

![rviz with simulated robot navigating](readme/sim.png?raw=true "Title")

### for pioneer

**on computer running roscore**

1. `roscore`
2. if you want to use joystick teleoperation, `roslaunch sarg_bot teleop_joy.launch`
3. if you want to visualize the ros' information, `rosrun rviz rviz -d src/san_museum/sarg_2dnav/config/rvizConfig.rviz`
4. `rosrun sarg_2dnav wander`

**on computer connected to pioneer**

1. create or get a map
2. set map in sarg_2dnav.launch
3. make sure all usb connections are correct (by default, pioneer is connected to ttyUSB0 and rplidar is connected to ttyUSB1)
4. set master uri to computer running roscore
5. `roslaunch sarg_bot pioneer.launch`

you should see something like this:

![rviz with simulated robot navigating](readme/pioneer.png?raw=true "Title")

*note: the computer running roscore and the computer connected to the pioneer can be the same, but this is resource intensive and not recommended*

## nodes

- `wander`<br/>
  gets current position, sets a goal, check if it meets requirements, send goal

### parameters

- none (yet)

### subscribed topics

- `/amcl_pose` [geometry_msgs/PoseWithCovarianceStamped]<br/>
  gets the current estimated position of the robot
- `/move_base/feedback` [move_base_msgs/MoveBaseActionFeedback]<br/>
  i dont really know what this does
- `/move_base/result` [move_base_msgs/MoveBaseActionResult]<br/>
  determine whether or not the goal set has been reached
- `/move_base/status` [actionlib_msgs/GoalStatusArray]<br/>
  monitor the status of the set goal

### published topics

- `/move_base/cancel` [actionlib_msgs/GoalID]<br/>
  keep track of goals (?)
- `/move_base/goal` [move_base_msgs/MoveBaseActionGoal]<br/>
  sends a goal if it meets the set requirements

### actions

- none (yet)
