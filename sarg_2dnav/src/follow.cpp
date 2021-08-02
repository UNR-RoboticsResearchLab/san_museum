#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/GetPlan.h>

#include "PoseListener.h"
#include "PeopleListener.h"
#include "ObjectListener.h"

bool goalIsOk (double goalX, double goalY, double currentX, double currentY, ObjectListener objectsDetected);
bool goToGoal (double x, double y);
void fillPathRequest (nav_msgs::GetPlan::Request & request, double startX, double startY, double goalX, double goalY);
bool callPlanningService (ros::ServiceClient & serviceClient, nav_msgs::GetPlan & serviceMessage);

int main (int argc, char ** argv)
{
  // initialize node
  ros::init (argc, argv, "follow");
  //ROS_DEBUG ("initialized node follow");

  // create listener object for amcl pose
  PoseListener currentPose;
  PeopleListener peoplePresent;
  ObjectListener objectsPresent;

  ros::Rate loop_rate (1);
  ros::spinOnce ();
  loop_rate.sleep();
  // spinOnce has to be called twice before getting correct pose, there's probably a better way to do this
  ros::spinOnce ();

  // loop until ctrl-c is pressed or ros::shutdown is called
  while (ros::ok ())
  {
    // check current location
    double poseAMCLx = currentPose.getPoseX ();
    double poseAMCLy = currentPose.getPoseY ();

    std::vector <std::vector <double>> reliableLocations = peoplePresent.sortByReliability ();

    //ROS_INFO ("pose: (%f, %f)", poseAMCLx, poseAMCLy);

    // goal coordinates
    double xGoal = poseAMCLx;
    double yGoal = poseAMCLy;

    // whether or not the goal has been reached
    bool goalReached = false;
    bool pathCheck = false;

    ROS_INFO ("finding suitable goal...");

    int index = reliableLocations.size ();

    double xTransform = 0;
    double yTransform = 0;

    // test goal, also check if index is below 1
    while (!pathCheck && index > 0)
    {
      // iterate people locations backwards (from most reliable to least)
      index -= 1;

      // how far away from the current position the robot should move
      // problem: the people coordinates do not account for a robot that is rotated
      xTransform = reliableLocations.at (index).at (0);
      yTransform = reliableLocations.at (index).at (1);

      // set a goal to test, divide transform by 2 to stop goal from being set directly on a person
      xGoal = (poseAMCLx + xTransform) / 2;
      yGoal = (poseAMCLy + yTransform) / 2;

      std::cout << "testing person " << index + 1 << std::endl;
      pathCheck = goalIsOk (xGoal, yGoal, poseAMCLx, poseAMCLy, objectsPresent);
    }

    std::cout << "sending goal\n";
    goalReached = goToGoal (xGoal, yGoal);

    if (goalReached)
    {
      ROS_INFO ("goal reached\n");
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
bool goalIsOk (double goalX, double goalY, double currentX, double currentY, ObjectListener objectsDetected)
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
      ROS_DEBUG ("make_plan success");
      return true;
    }
  }

  return false;
}
