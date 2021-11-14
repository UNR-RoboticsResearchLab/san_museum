#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
//#include "tf/transform_listener.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
//#include "san_feature_extractor/PaccetFeatures.h"
//#include <tf/transform_broadcaster.h>

#include <cmath>
#include <ctime>// include this header

#include <iostream>
#include <fstream>

using namespace std;

ofstream robot;
ofstream human;


// bool computeFeatures(san_feature_extractor::PaccetFeatures::Request &req, san_feature_extractor::PaccetFeatures::Response &res)
// {
//   ROS_INFO("Received request ");
//   res.intDist = sqrt(pow((globalPose1.pose.position.x - req.futureTrajectory.x), 2) + pow((globalPose1.pose.position.y - req.futureTrajectory.y), 2));

//   ROS_INFO("Inter personal distance is : [%f]", res.intDist);
//   return true;
// }


/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void robot0Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->header.frame_id.c_str());
  robot << msg->pose.pose.position.x << "\t" << msg->pose.pose.position.y << endl;
  
  
}

void robot1Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->header.frame_id.c_str());
  human << msg->pose.pose.position.x << "\t" << msg->pose.pose.position.y << endl;
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "write_to_file");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;
  //ros::Rate loop_rate(100);

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub1 = n.subscribe("/robot_0/base_pose_ground_truth", 1000, robot0Callback);
  ros::Subscriber sub2 = n.subscribe("/robot_1/base_pose_ground_truth", 1000, robot1Callback);
  //ros::ServiceServer service = n.advertiseService("san_paccet_features", computeFeatures);
  
  ROS_INFO("Ready to writeeeeeeeeeee!!!!1");
  robot.open("/home/scott-san/catkin_ws/robot.txt");
  human.open("/home/scott-san/catkin_ws/human.txt");

  // while(ros::ok())
  // {
  //     ROS_INFO("In loop");
  //     tf_human.publish(globalPose1);
  //     ros::spinOnce();
  //     loop_rate.sleep();
  // }

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();
  cout << "Hello kill me" << endl;
  robot.close();
  human.close();

  return 0;
}

