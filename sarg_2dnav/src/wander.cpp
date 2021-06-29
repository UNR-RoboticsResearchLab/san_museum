// code based off http://edu.gaitech.hk/turtlebot/map-navigation.html

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <ros/time.h>
#include <istream>
//#include <tf/transform_listener.h>

bool goToGoal (int x, int y);

bool goalReached = false;

int main (int argc, char ** argv)
{
  int locations = 3;
  int coordinates [locations][2] = {{-99, -82}, {-104, -60}, {-86, -154}};
  int choice;

  bool quit = 0;

  // loop the process of choosing a location and moving to it
  while (!quit)
  {
    ros::init (argc, argv, "wander");
    ros::NodeHandle n;
    ros::spinOnce ();

    srand (time (NULL));

    // choose a random location to set a path to
    int choice = rand () % locations;

    // set a path to goal and take it
    goalReached = goToGoal (coordinates [choice][0], coordinates [choice][1]);

    /*
      // from https://answers.ros.org/question/31815/getting-coordinates-of-turtlebot/
      tf::TransformListener listener;
      geometry_msgs::PoseStamped pBase, pMap;
      pBase.header.frame_id = "base_laser_link";
      pBase.pose.position.x = 0.0;
      pBase.pose.position.y = 0.0;
      pBase.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
      ros::Time current_transform = ros::Time::now();
      listener.getLatestCommonTime(pBase.header.frame_id, "map", current_transform, NULL);
      pBase.header.stamp = current_transform;
      listener.transformPose("map", pBase, pMap);
      // pMap now contains the pose of the robot transformed into map
      // coordinates according to the TF data available at time "current_transform"

      // set a path to a random goal close by and take it
      goalReached = goToGoal (pMap.pose.position.x + rand () % 5 + 1, pMap.pose.position.y + rand () % 5 + 1);
    */

    if (goalReached)
    {
      ROS_INFO ("goal reached");
      ros::spinOnce ();
    }

    else
    {
      ROS_INFO ("goal not reached");
    }

    // if q is pressed, end loop
    if (std::cin.get() == 'q')
    {
      quit = 1;
    }
  }

  return 0;
}

bool goToGoal (int x, int y)
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

   /* moving towards the goal*/

   goal.target_pose.pose.position.x =  x;
   goal.target_pose.pose.position.y =  y;
   goal.target_pose.pose.position.z =  0.0;
   goal.target_pose.pose.orientation.x = 0.0;
   goal.target_pose.pose.orientation.y = 0.0;
   goal.target_pose.pose.orientation.z = 0.0;
   goal.target_pose.pose.orientation.w = 1.0;

   ROS_INFO ("sending goal location");
   ac.sendGoal (goal);

   ac.waitForResult ();

   if (ac.getState () == actionlib::SimpleClientGoalState::SUCCEEDED)
   {
     ROS_INFO ("You have reached the destination");
     return true;
   }
   else
   {
     ROS_INFO("The robot failed to reach the destination");
     return false;
   }
}
