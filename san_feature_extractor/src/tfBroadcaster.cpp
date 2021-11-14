#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>


void poseCallback(const nav_msgs::Odometry::ConstPtr& msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, msg->twist.twist.angular.z);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "robot_1/odom"));
}



/*void testposeCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  //transform.setOrigin( tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0) );
  transform.setOrigin( tf::Vector3(msg->info.origin.position.x, msg->info.origin.position.y, 0.0) );
  tf::Quaternion q;
  //q.setRPY(0, 0, msg->twist.twist.angular.z);
  q.setRPY(0, 0, msg->info.origin.orientation..z);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "map"));
  //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "robot_0/odom"));
}*/


int main(int argc, char** argv){
  ros::init(argc, argv, "san_missing_tf_broadcaster");

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("robot_1/odom", 10, &poseCallback);

  //ros::Subscriber testsub = node.subscribe("map", 10, &testposeCallback);

  ros::spin();
  return 0;
};
