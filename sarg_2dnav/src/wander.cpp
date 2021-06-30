// code based off http://edu.gaitech.hk/turtlebot/map-navigation.html

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <ros/time.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <vector>

double poseAMCLx, poseAMCLy;
bool goalReached = false;
// how different a new location has to be from a failed one
double locationThreshold = 1;

bool goalIsOk (double goalX, double goalY, double nowX, double nowY, std::vector <std::vector <double>> pastLocations);
bool goToGoal (double x, double y);
void amclCallback (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & msgAMCL);

int main (int argc, char ** argv)
{
  /*
    // predefined locations for the robot to travel to
    int locations = 3;
    int coordinates [locations][2] = {{-99, -82}, {-104, -60}, {-86, -154}};
    int choice;
  */

  ros::init (argc, argv, "wander");
  ros::NodeHandle n;

  // subscribe to amcl pose to get estimated robot position
  ros::Subscriber amcl_sub = n.subscribe ("amcl_pose", 100, amclCallback);

  // spinOnce has to be called twice before getting correct pose, there's probably a better way to do this
  ros::Rate loop_rate (10);
  ros::spinOnce ();
  loop_rate.sleep();
  ros::spinOnce ();

  srand (time (NULL));

  // a previous location that did not have a path
  //double badLocationX, badLocationY;
  std::vector <std::vector <double>> badLocations;

  // choose a random predefined location to set a path to
  //int choice = rand () % locations;

  // set a path to goal and take it
  //goalReached = goToGoal (coordinates [choice][0], coordinates [choice][1]);

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
    double scale = 0;

    // set a goal to a location relative to the current pose
    do
    {
      ROS_INFO ("finding suitable goal...");

      // randomness is multiplied and subtracted to include negative values
      xGoal = currentX + scale * ((rand () % (randomness * 2)) - (randomness));
      yGoal = currentY + scale * ((rand () % (randomness * 2)) - (randomness));

      // increase multiplier to avoid an infinite loop
      scale += 0.01;

      // maximum value for scale so the goal isnt too far away
      if (scale > 0.5)
      {
        scale = 0;
      }
    }
    // if goal is too close to current or failed location, find a new goal
    //while ((xGoal - badLocationX < locationThreshold && yGoal - badLocationY < locationThreshold) || (xGoal - currentX < locationThreshold && yGoal - currentY < locationThreshold));
    while (!goalIsOk (xGoal, yGoal, currentX, currentY, badLocations));
    goalReached = goToGoal (xGoal, yGoal);

    if (goalReached)
    {
      ROS_INFO ("goal reached");
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

bool goalIsOk (double goalX, double goalY, double nowX, double nowY, std::vector <std::vector <double>> pastLocations)
{
  if (goalX - nowX < locationThreshold && goalY - nowY < locationThreshold)
  {
    ROS_INFO ("goal not ok, too close to current location");
    return false;
  }
  for (int index = 0; index < pastLocations.size (); index++)
  {
    if (goalX - pastLocations.at (index).at (0) < locationThreshold && goalY - pastLocations.at (index).at (1) < locationThreshold)
    {
      ROS_INFO ("goal not ok, too close to failed location");
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
