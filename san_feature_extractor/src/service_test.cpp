#include "ros/ros.h"
#include "san_feature_extractor/PaccetFeatures.h"
#include "geometry_msgs/Point.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "testing_client");
  

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<san_feature_extractor::PaccetFeatures>("san_paccet_features", true);
  san_feature_extractor::PaccetFeatures srv;
  geometry_msgs::Point pnt;
  pnt.x = 1;
  pnt.y = 1;
  pnt.z = 0;
  srv.request.futureTrajectory = pnt;
  ros::Rate r(10);
  while(ros::ok())
  {
    ROS_INFO("Am in the loop");
  if (client.call(srv))
  {
    ROS_INFO("intDist is : %f", srv.response.intDist);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }
  }
  ros::spinOnce();
  r.sleep();
  return 0;
}
