// code based off http://edu.gaitech.hk/turtlebot/map-navigation.html

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

double poseAMCLx, poseAMCLy;
bool goalReached = false;
// how different a new location has to be from a failed one
double locationThreshold = 0.5;

bool goalIsOk (double goalX, double goalY, double nowX, double nowY, std::vector <std::vector <double>> failedLocations, std::vector <std::vector <double>> successfulLocations);
bool goToGoal (double x, double y);
void amclCallback (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & msgAMCL);

int main (int argc, char ** argv)
{
  ros::init (argc, argv, "wander");
  ros::NodeHandle n;

  // subscribe to amcl pose to get estimated robot position
  ros::Subscriber amcl_sub = n.subscribe ("amcl_pose", 100, amclCallback);

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
    double currentX = poseAMCLx;
    double currentY = poseAMCLy;

    ROS_INFO ("amcl pose: (%f, %f)", currentX, currentY);

    double xGoal;
    double yGoal;

    // how random the new goal will be
    int randomness = 10;
    // multiplier to randomness (this is done because you cant modulo a double)
    double scale = 0.25;

    // set a goal to a location relative to the current pose
    ROS_INFO ("finding suitable goal...");
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
        scale = 0.25;
      }
    }
    // if goal is too close to current or failed location, find a new goal
    while (!goalIsOk (xGoal, yGoal, currentX, currentY, badLocations, previousLocations));
    goalReached = goToGoal (xGoal, yGoal);

    if (goalReached)
    {
      ROS_INFO ("goal reached");

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
      ROS_INFO ("goal not reached");

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

bool goalIsOk (double goalX, double goalY, double nowX, double nowY, std::vector <std::vector <double>> failedLocations, std::vector <std::vector <double>> successfulLocations)
{
  // how many past locations should be "remembered"
  int locationMemory = 100;

  if (abs (goalX - nowX) < locationThreshold && abs (goalY - nowY) < locationThreshold)
  {
    //ROS_INFO ("goal not ok, too close to current location");
    return false;
  }

  for (int index = 0; index < failedLocations.size (); index++)
  {
    if (abs (goalX - failedLocations.at (index).at (0)) < locationThreshold && abs (goalY - failedLocations.at (index).at (1)) < locationThreshold)
    {
      //ROS_INFO ("goal not ok, too close to failed location");
      return false;
    }
  }

  for (int index = successfulLocations.size (); index > successfulLocations.size () - locationMemory; index--)
  {
    if (abs (goalX - successfulLocations.at (index - 1).at (0)) < locationThreshold * 5 && abs (goalY - successfulLocations.at (index - 1).at (1)) < locationThreshold * 5)
    {
      //ROS_INFO ("goal not ok, too close to previous location");
      return false;
    }
  }

  ROS_INFO ("potential goal found");

  return true;
}

bool goToGoal (double x, double y)
{
  //define a client for to send goal requests to the move_base server through a SimpleActionClient
   actionlib::SimpleActionClient <move_base_msgs::MoveBaseAction> ac ("move_base", true);

   //wait for the action server to come up
   while (!ac.waitForServer (ros::Duration (5.0)))
   {
     ROS_INFO ("waiting for the move_base action server to come up");
   }

   move_base_msgs::MoveBaseGoal goal;

   //set up the frame parameters
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

   ROS_INFO ("sending goal location: (%f, %f)", x, y);
   ac.sendGoal (goal);

   ac.waitForResult ();

   if (ac.getState () == actionlib::SimpleClientGoalState::SUCCEEDED)
   {
     ROS_INFO ("robot reached the destination");
     return true;
   }
   else
   {
     ROS_INFO("robot did not reach the destination");
     return false;
   }
}

// from https://answers.ros.org/question/248046/subscribing-to-amcl-pose/
void amclCallback (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & msgAMCL)
{
  poseAMCLx = msgAMCL -> pose.pose.position.x;
  poseAMCLy = msgAMCL -> pose.pose.position.y;
}
