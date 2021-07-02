#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
//#include <nav_msgs/GetPlan.h>

double poseAMCLx, poseAMCLy;
bool goalReached = false;
// how different a new location has to be from a failed goal, previous location, or current location
// setting this too high will cause an infinite loop
double locationThreshold = 0.5;
/*
  // client for pathmaking service
  ros::ServiceClient pathClient;
  nav_msgs::GetPlan planService;
*/

bool goalIsOk (double goalX, double goalY, double nowX, double nowY, std::vector <std::vector <double>> failedLocations, std::vector <std::vector <double>> successfulLocations);
bool goToGoal (double x, double y);
void amclCallback (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & msgAMCL);

int main (int argc, char ** argv)
{
  ros::init (argc, argv, "wander");
  ros::NodeHandle n;

  // subscribe to amcl pose to get estimated robot position
  ros::Subscriber amcl_sub = n.subscribe ("amcl_pose", 100, amclCallback);

  /*
    pathClient = n.serviceClient <nav_msgs::GetPlan> ("move_base/make_plan");

    planService.request.tolerance = 1.5;
  */

  ros::Rate loop_rate (10);
  ros::spinOnce ();
  loop_rate.sleep();
  // spinOnce has to be called twice before getting correct pose, there's probably a better way to do this
  ros::spinOnce ();

  srand (time (NULL));

  // previous locations that did not have a path
  std::vector <std::vector <double>> badLocations;
  // previous locations that did have a path
  std::vector <std::vector <double>> previousLocations;

  // loop until ctrl-c is pressed or ros::shutdown is called
  while (ros::ok ())
  {
    // get current location
    double currentX = poseAMCLx;
    double currentY = poseAMCLy;

    ROS_INFO ("currently at: (%f, %f)", currentX, currentY);

    double xGoal;
    double yGoal;

    // how random the new goal will be
    int randomness = 10;
    // multiplier to randomness (this is done because you cant modulo a double)
    double initialScale = 0.25;
    double scale = initialScale;

    // set a goal to a location relative to the current pose
    ROS_INFO ("finding suitable goal...");

    // maybe also make the program exit after too many path fails? (prevent hang)
    do
    {
      // randomness is multiplied and subtracted to include negative values
      xGoal = currentX + scale * ((rand () % (randomness * 2)) - (randomness));
      yGoal = currentY + scale * ((rand () % (randomness * 2)) - (randomness));

      //ROS_INFO ("goal: (%f, %f)", xGoal, yGoal);

      // increase multiplier to avoid an infinite loop
      scale += 0.01;

      // maximum value for scale so the goal isnt too far away
      if (scale > 0.5)
      {
        scale = initialScale;
      }
    }
    // if goal is too close to current or failed location, find a new goal
    while (!goalIsOk (xGoal, yGoal, currentX, currentY, badLocations, previousLocations));

    // try to see if a path can be make to the goal
    goalReached = goToGoal (xGoal, yGoal);

    if (goalReached)
    {
      ROS_INFO ("goal reached\n");

      std::vector <double> tempPreviousGoal;
      for (int index = 0; index < 2; index++)
      {
        // remember this successful goal
        tempPreviousGoal.push_back (xGoal);
        tempPreviousGoal.push_back (yGoal);
      }

      previousLocations.push_back (tempPreviousGoal);
    }

    else
    {
      ROS_INFO ("goal not reached\n");

      // i think this approach doesnt make a distinction between a goal failure from the global or local planner
      // maybe add something to only store goals failed by global planner?

      std::vector <double> tempBadGoal;
      for (int index = 0; index < 2; index++)
      {
        // remember this failed goal
        tempBadGoal.push_back (xGoal);
        tempBadGoal.push_back (yGoal);
      }

      badLocations.push_back (tempBadGoal);
    }

    // get new position data
    ros::spinOnce ();
    loop_rate.sleep();
  }

  return 0;
}

// check if the goal is too close to current location, a failed goal, or a previous location
bool goalIsOk (double goalX, double goalY, double nowX, double nowY, std::vector <std::vector <double>> failedLocations, std::vector <std::vector <double>> successfulLocations)
{
  // if goal is too close too current location
  if (abs (goalX - nowX) < locationThreshold && abs (goalY - nowY) < locationThreshold)
  {
    //ROS_INFO ("goal not ok, too close to current location");
    return false;
  }

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

  // for every previous location
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

  //planService.request.goal = goal.target_pose;

  ROS_INFO ("sending goal location: (%f, %f)", x, y);

  /*
    // if plan can be made
    bool callResult = pathClient.call (planService) ? 1 : 0;
    ROS_INFO("Make plan: %d", (callResult));
    if (pathClient.call (callResult))
  */
  {
    ac.sendGoal (goal);
  }


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

  /*
    move_base_msgs::MoveBaseGoal current;

    // create a message to send to planService, there has to be a better way to do this
    current.target_pose.header.frame_id = "map";
    current.target_pose.header.stamp = ros::Time::now ();

    current.target_pose.pose.position.x =  poseAMCLx;
    current.target_pose.pose.position.y =  poseAMCLy;
    current.target_pose.pose.position.z =  0.0;
    current.target_pose.pose.orientation.x = 0.0;
    current.target_pose.pose.orientation.y = 0.0;
    current.target_pose.pose.orientation.z = 0.0;
    current.target_pose.pose.orientation.w = 1.0;

    planService.request.goal = current.target_pose;
  */
}
