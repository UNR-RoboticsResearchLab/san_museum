#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/GetPlan.h>

#include "PoseListener.h"
#include "PeopleListener.h"

bool goalIsOk (double goalX, double goalY, double currentX, double currentY, std::vector <std::vector <double>> successfulLocations);
bool goToGoal (double x, double y);
std::vector <std::vector <double>> saveGoal (double x, double y, std::vector <std::vector <double>> destination, bool hasCapacity);
void fillPathRequest (nav_msgs::GetPlan::Request & request, double startX, double startY, double goalX, double goalY);
bool callPlanningService (ros::ServiceClient & serviceClient, nav_msgs::GetPlan & serviceMessage);

int main (int argc, char ** argv)
{
  // initialize node
  ros::init (argc, argv, "wander");
  //ROS_DEBUG ("initialized node wander");

  // create listener object for amcl pose
  PoseListener currentPose;
  PeopleListener peoplePresent;

  ros::Rate loop_rate (1);
  ros::spinOnce ();
  loop_rate.sleep();
  // spinOnce has to be called twice before getting correct pose, there's probably a better way to do this
  ros::spinOnce ();

  // seed for random number generator
  srand (time (NULL));

  // maximum attempts to find a goal (stop infinite loop)
  int maxTries = 1000;

  // previous locations that did have a path
  std::vector <std::vector <double>> previousLocations;

  // loop until ctrl-c is pressed or ros::shutdown is called
  while (ros::ok ())
  {
    // check current location
    double poseAMCLx = currentPose.getPoseX ();
    double poseAMCLy = currentPose.getPoseY ();

    std::vector <double> reliableLocation = peoplePresent.getMostReliableLocation ();

    //ROS_INFO ("pose: (%f, %f)", poseAMCLx, poseAMCLy);

    // goal coordinates
    double xGoal;
    double yGoal;

    // whether or not the goal has been reached
    bool goalReached = false;

    // set a goal to a location relative to the current pose
    ROS_INFO ("finding suitable goal...");

    xGoal = poseAMCLx + reliableLocation.at (0);
    yGoal = poseAMCLy + reliableLocation.at (1);

    // try to see if a path can be make to the goal
    if (goalIsOk (xGoal, yGoal, poseAMCLx, poseAMCLy, previousLocations))
    {
      goalReached = goToGoal (xGoal, yGoal);
    }

    if (goalReached)
    {
      ROS_INFO ("goal reached\n");
      previousLocations = saveGoal (xGoal, yGoal, previousLocations, true);
    }

    else
    {
      ROS_WARN ("goal not reached\n");
    }

    // get new position data
    ros::spinOnce ();
    loop_rate.sleep();
  }

  return 0;
}

// check if the goal is too close to current location, a failed goal, or a previous location
bool goalIsOk (double goalX, double goalY, double currentX, double currentY, std::vector <std::vector <double>> successfulLocations)
{
  ros::NodeHandle goalCheckNode;
  ros::ServiceClient planClient = goalCheckNode.serviceClient <nav_msgs::GetPlan> ("move_base/make_plan", true);
  nav_msgs::GetPlan planSrv;

  // fill in the request for make_plan service
  fillPathRequest (planSrv.request, currentX, currentY, goalX, goalY);

  // if make_plan cannot find a plan
  if (!callPlanningService (planClient, planSrv))
  {
    //ROS_INFO ("goal not ok, no path from planner");
    return false;
  }

  //ROS_INFO ("potential goal found");

  return true;
}

// from http://edu.gaitech.hk/turtlebot/map-navigation.html
bool goToGoal (double x, double y)
{
  // define a client for to send goal requests to the move_base server through a SimpleActionClient
  actionlib::SimpleActionClient <move_base_msgs::MoveBaseAction> ac ("move_base", true);

  // wait for the action server to come up
  while (!ac.waitForServer (ros::Duration (5.0)))
  {
    ROS_DEBUG ("waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now ();

  // set goal coordinates
  goal.target_pose.pose.position.x =  x;
  goal.target_pose.pose.position.y =  y;
  goal.target_pose.pose.position.z =  0.0;
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = 0.0;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO ("goal: (%f, %f)", x, y);

  // send the goal
  ac.sendGoal (goal);
  ac.waitForResult ();

  if (ac.getState () == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
   //ROS_DEBUG ("robot reached the destination");
   return true;
  }

  else
  {
   //ROS_WARN ("robot did not reach the destination");
   return false;
  }
}

std::vector <std::vector <double>> saveGoal (double x, double y, std::vector <std::vector <double>> destination, bool hasCapacity)
{
  // max amount of goals to store if a vector has a "max capacity"
  // set this too high and the robot can corner itself
  // set it too low and the robot will probably stay in the same area
  int maxGoals = 5;

  std::vector <double> tempGoal;

  for (int index = 0; index < 2; index++)
  {
    // add coordinates to temporary goal vector
    tempGoal.push_back (x);
    tempGoal.push_back (y);
  }

  // add temporary goal vector to input vector
  destination.push_back (tempGoal);

  // if input vector is larger than its set capacity
  if (destination.size () > maxGoals && hasCapacity)
  {
    // delete the oldest goal to maintain max capacity
    destination.erase (destination.begin ());
  }

  // return updated vector
  return destination;
}

// from https://www.programmersought.com/article/85495009501/
void fillPathRequest (nav_msgs::GetPlan::Request & request, double startX, double startY, double goalX, double goalY)
{
  // set frame for starting position
  request.start.header.frame_id = "map";

  // set coordinates for starting position
  request.start.pose.position.x = startX;
  request.start.pose.position.y = startY;

  request.start.pose.orientation.w = 1.0;

  // set frame for ending position
  request.goal.header.frame_id = "map";

  // set coordinates for ending position
  request.goal.pose.position.x = goalX;
  request.goal.pose.position.y = goalY;

  request.goal.pose.orientation.w = 1.0;

  // from getplan service documentaion:
  // If the goal is obstructed, how many meters the planner can relax the constraint in x and y before failing.
  request.tolerance = 0.0;
}

// from https://www.programmersought.com/article/85495009501/
bool callPlanningService (ros::ServiceClient & serviceClient, nav_msgs::GetPlan & serviceMessage)
{
  // perform the actual path planner call
  // execute the actual path planner
  if (serviceClient.call (serviceMessage))
  {
    // srv.response.plan.poses is the container for storing the results, traversed and taken out
    if (!serviceMessage.response.plan.poses.empty ())
    {
      // std::for_each(srv.response.plan.poses.begin(),srv.response.plan.poses.end(),myfunction);
      //ROS_DEBUG ("make_plan success");
      return true;
    }
  }

  return false;
}
