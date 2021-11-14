#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>




void tfposeCallback(const nav_msgs::Odometry::ConstPtr& msg){
  static tf::TransformBroadcaster tfbr;
  tf::Transform tftransform;
  tftransform.setOrigin( tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, msg->twist.twist.angular.z);
  tftransform.setRotation(q);
  tfbr.sendTransform(tf::StampedTransform(tftransform, ros::Time::now(), "/world", "robot_1/odom"));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "san_missing_tf_broadcaster");

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("robot_0/odom", 10, &tfposeCallback);

  ros::spin();
  return 0;
};
