#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/GetPlan.h>

double poseAMCLx, poseAMCLy;
bool goalReached = false;
// how different a new location has to be from a failed goal, previous location, or current location
// setting this too high will cause an infinite loop
double locationThreshold = 1;

bool goalIsOk (double goalX, double goalY, double nowX, double nowY, std::vector <std::vector <double>> successfulLocations);
bool goToGoal (double x, double y);
void amclCallback (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & msgAMCL);
void saveGoal (double x, double y, std::vector <std::vector <double>> destination, bool hasCapacity);
void fillPathRequest (nav_msgs::GetPlan::Request & request, float start_x, float start_y, float goal_x, float goal_y);
bool callPlanningService (ros::ServiceClient & serviceClient, nav_msgs::GetPlan & srv);

int main (int argc, char ** argv)
{
  ros::init (argc, argv, "wander");
  //ROS_INFO ("initialized node");
  ros::NodeHandle n;
  //ROS_INFO ("created nodehandle n");

  // subscribe to amcl pose to get estimated robot position
  ros::Subscriber amclSub = n.subscribe ("amcl_pose", 100, amclCallback);
  //ROS_INFO ("subscribed to amcl_pose");

  ros::Rate loop_rate (10);
  ros::spinOnce ();
  loop_rate.sleep();
  // spinOnce has to be called twice before getting correct pose, there's probably a better way to do this
  ros::spinOnce ();

  // seed for random number generator
  srand (time (NULL));

  // maximum attempts to find a goal (stop infinite loop)
  int maxTries = 1000;

  // previous locations that did not have a path
  // this is no longer needed since the planner is now checked
  //std::vector <std::vector <double>> badLocations;
  // previous locations that did have a path
  std::vector <std::vector <double>> previousLocations;

  // loop until ctrl-c is pressed or ros::shutdown is called
  while (ros::ok ())
  {
    // check current location
    ROS_INFO ("pose: (%f, %f)", poseAMCLx, poseAMCLy);

    double xGoal;
    double yGoal;

    // how random the new goal will be
    int randomness = 10;
    // multiplier to randomness (this is done because you cant modulo a double)
    double initialScale = 0.25;
    double scale = initialScale;
    // attempt counter
    int tries = 0;

    // set a goal to a location relative to the current pose
    ROS_INFO ("finding suitable goal...");

    do
    {
      // increase attempt counter by 1
      tries += 1;

      // randomness is multiplied and subtracted to include negative values
      xGoal = poseAMCLx + scale * ((rand () % (randomness * 2)) - (randomness));
      yGoal = poseAMCLy + scale * ((rand () % (randomness * 2)) - (randomness));

      //ROS_INFO ("goal: (%f, %f)", xGoal, yGoal);

      // increase multiplier to avoid an infinite loop
      scale += 0.01;

      // maximum value for scale so the goal isnt too far away
      if (scale > 0.5)
      {
        scale = initialScale;
      }

      if (tries > maxTries)
      {
        ROS_INFO ("could not find goal after %d tries", maxTries);
        return 1;
      }
    }
    // if goal is too close to current or failed location, find a new goal
    while (!goalIsOk (xGoal, yGoal, poseAMCLx, poseAMCLy, previousLocations));

    // try to see if a path can be make to the goal
    goalReached = goToGoal (xGoal, yGoal);

    if (goalReached)
    {
      ROS_INFO ("goal reached\n");
      saveGoal (xGoal, yGoal, previousLocations, true);
    }

    else
    {
      ROS_INFO ("goal not reached\n");

      // this is no longer needed since the planner is now checked
      //saveGoal (xGoal, yGoal, badLocations, false);
    }

    // get new position data
    ros::spinOnce ();
    loop_rate.sleep();
  }

  return 0;
}

// check if the goal is too close to current location, a failed goal, or a previous location
bool goalIsOk (double goalX, double goalY, double nowX, double nowY, std::vector <std::vector <double>> successfulLocations)
{
  ros::NodeHandle goalCheckNode;
  ros::ServiceClient planClient = goalCheckNode.serviceClient <nav_msgs::GetPlan> ("move_base/make_plan", true);
  nav_msgs::GetPlan planSrv;

  // fill in the request for make_plan service
  fillPathRequest (planSrv.request, poseAMCLx, poseAMCLy, goalX, goalY);

  // if make_plan cannot find a plan
  if (!callPlanningService (planClient, planSrv))
  {
    //ROS_INFO ("goal not ok, no path from planner");
    return false;
  }

  // if goal is too close too current location
  if (abs (goalX - nowX) < locationThreshold && abs (goalY - nowY) < locationThreshold)
  {
    //ROS_INFO ("goal not ok, too close to current location");
    return false;
  }

  /*
    // this is no longer needed since the planner is now checked

    // for every failed goal so far
    for (int index = 0; index < failedLocations.size (); index++)
    {
      // if goal is too close to a failed goal
      if (abs (goalX - failedLocations.at (index).at (0)) < locationThreshold && abs (goalY - failedLocations.at (index).at (1)) < locationThreshold)
      {
        //ROS_INFO ("goal not ok, too close to failed location");
        return false;
      }
    }
  */

  // for every previous location
  // this doesnt look like its working anymore and i dont know why
  for (int index = 0; index < successfulLocations.size (); index++)
  {
    // if goal is too close to a previous location
    if (abs (goalX - successfulLocations.at (index).at (0)) < locationThreshold && abs (goalY - successfulLocations.at (index).at (1)) < locationThreshold)
    {
      //ROS_INFO ("goal not ok, too close to previous location");
      return false;
    }
  }

  ROS_INFO ("potential goal found");

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
    ROS_INFO ("waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now ();

  // moving towards the goal
  goal.target_pose.pose.position.x =  x;
  goal.target_pose.pose.position.y =  y;
  goal.target_pose.pose.position.z =  0.0;
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = 0.0;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO ("goal: (%f, %f)", x, y);

  /*
    CHANGE FROM JUST SENDGOAL TO MAKEPLAN !!
  */

  ac.sendGoal (goal);
  ac.waitForResult ();

  if (ac.getState () == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
   //ROS_INFO ("robot reached the destination");
   return true;
  }

  else
  {
   //ROS_INFO("robot did not reach the destination");
   return false;
  }
}

// from https://answers.ros.org/question/248046/subscribing-to-amcl-pose/
void amclCallback (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & msgAMCL)
{
  poseAMCLx = msgAMCL -> pose.pose.position.x;
  poseAMCLy = msgAMCL -> pose.pose.position.y;
}

void saveGoal (double x, double y, std::vector <std::vector <double>> destination, bool hasCapacity)
{
  // max amount of goals to store if a vector has a "max capacity"
  int maxGoals = 5;

  if (destination.size () > maxGoals && hasCapacity)
  {
    destination.erase (destination.begin (), destination. begin());
  }

  std::vector <double> tempGoal;
  for (int index = 0; index < 2; index++)
  {
    // remember this successful goal
    tempGoal.push_back (x);
    tempGoal.push_back (y);
  }

  destination.push_back (tempGoal);
}

// from https://www.programmersought.com/article/85495009501/
void fillPathRequest (nav_msgs::GetPlan::Request & request, float start_x, float start_y, float goal_x, float goal_y)
{
  request.start.header.frame_id = "map";
  request.start.pose.position.x = start_x;
  request.start.pose.position.y = start_y;
  request.start.pose.orientation.w = 1.0;
  request.goal.header.frame_id = "map";
  request.goal.pose.position.x = goal_x;
  request.goal.pose.position.y = goal_y;
  request.goal.pose.orientation.w = 1.0;
  request.tolerance = 0.0;
}

// from https://www.programmersought.com/article/85495009501/
bool callPlanningService (ros::ServiceClient & serviceClient, nav_msgs::GetPlan & srv)
{
  // perform the actual path planner call
  // execute the actual path planner
  if (serviceClient.call (srv))
  {
    // srv.response.plan.poses is the container for storing the results, traversed and taken out
    if (!srv.response.plan.poses.empty ())
    {
      // std::for_each(srv.response.plan.poses.begin(),srv.response.plan.poses.end(),myfunction);
      //ROS_INFO ("make_plan success");
      return true;
    }
  }

  return false;
}
