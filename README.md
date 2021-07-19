# socially aware navigation in a museum setting

## overview

using a lidar scanner and camera, will make socially informed decisions in navigation

## installation

### dependencies

amcl
p2os
rplidar_ros
darknet_ros
usb_cam
leg_detector
stageros

### building

```
cd your_catkin_workspace/src
git clone https://github.com/UNR-RoboticsResearchLab/san_museum.git -b aaron
cd ../
catkin_make
```

## basic usage

have a robot setting random goals relative to its current position

### for simulation

1. create or get a map
2. set map in sarg_2dnav_sim.launch
3. `roslaunch sarg_bot simulation.launch`

### for pioneer

**on computer running roscore**

1. `roscore`
2. if you want to use joystick teleoperation, `roslaunch sarg_bot teleop_joy.launch`
3. if you want to visualize the ros' information `rosrun rviz rviz -d src/san_museum/sarg_2dnav/config/rvizConfig.rviz`
4. `rosrun sarg_2dnav wander`

**on computer connected to pioneer**

1. create or get a map
2. set map in sarg_2dnav.launch
3. make sure all usb connections are correct (by default, pioneer is connected to ttyUSB0 and rplidar is connected to ttyUSB1)
4. set master uri to computer running roscore
5. `roslaunch sarg_bot pioneer.launch`

the computer running roscore and the computer connected to the pioneer can be the same, but this is resource intensive and not recommended

## nodes

- `wander`
  gets current position, sets a goal, check if it meets requirements, send goal

### parameters

- none (yet)

### subscribed topics

- `/amcl_pose` [geometry_msgs/PoseWithCovarianceStamped]
  gets the current estimated position of the robot
- `/move_base/feedback` [move_base_msgs/MoveBaseActionFeedback]
  i dont really know what this does
- `/move_base/result` [move_base_msgs/MoveBaseActionResult]
  determine whether or not the goal set has been reached
- `/move_base/status` [actionlib_msgs/GoalStatusArray]
  monitor the status of the set goal

### published topics

- `/move_base/cancel` [actionlib_msgs/GoalID]
  keep track of goals (?)
- `/move_base/goal` [move_base_msgs/MoveBaseActionGoal]
  sends a goal if it meets the set requirements

### actions

- none (yet)
