#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "geometry_msgs/Point.h"
#include "san_feature_extractor/newMarkerMsg.h"

#include <cmath>



ros::Publisher marker_pub;

    visualization_msgs::Marker points, line_stripL;
    

/**
 * Hallway visualization callback function.
 */
void newHallwayVisualizationCallback(const san_feature_extractor::newMarkerMsg::ConstPtr& msg)
{
  ROS_INFO("I heard: ");
    points.points.push_back(msg->pointL1);
    line_stripL.points.push_back(msg->pointL1);
    ROS_INFO("pointL1: %f, %f", msg->pointL1.x, msg->pointL1.y);
    points.points.push_back(msg->pointL2);
    line_stripL.points.push_back(msg->pointL2);
    ROS_INFO("pointL2: %f, %f", msg->pointL2.x, msg->pointL2.y);
    
    marker_pub.publish(points);
    marker_pub.publish(line_stripL);
    points.points.clear();
    line_stripL.points.clear();
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "new_points_and_lines");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("new_hallway_marker_points", 1000, newHallwayVisualizationCallback);
  marker_pub = n.advertise<visualization_msgs::Marker>("new_visual_marker", 100);

  //ros::Rate r(30);

  //float f = 0.0;
  // while (ros::ok())s
  // {
    points.header.frame_id = line_stripL.header.frame_id = "/map";
    points.header.stamp = line_stripL.header.stamp = ros::Time::now();
    points.ns = line_stripL.ns = "new_points_and_lines";
    points.action = line_stripL.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_stripL.pose.orientation.w = 1.0;



    points.id = 0;
    line_stripL.id = 1;


    points.type = visualization_msgs::Marker::POINTS;
    line_stripL.type = visualization_msgs::Marker::LINE_STRIP;



    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_stripL.scale.x = 0.1;
    

    // Points are green
    points.color.b = 1.0f;
    points.color.a = 1.0;

    // Line strip is blue
    line_stripL.color.g = 1.0;
    line_stripL.color.a = 1.0;

    
    ROS_INFO("I am running ");


    // Create the vertices for the points and lines
    // for (uint32_t i = 0; i < 100; ++i)
    // {
    //   float y = 5 * sin(f + i / 100.0f * 2 * M_PI);
    //   float z = 5 * cos(f + i / 100.0f * 2 * M_PI);

    //   geometry_msgs::Point p;
    //   p.x = (int32_t)i - 50;
    //   p.y = y;
    //   p.z = z;

    //   points.points.push_back(p);
    //   line_strip.points.push_back(p);

      
    // }


    // marker_pub.publish(points);
    // marker_pub.publish(line_strip);
  

    //r.sleep();

  //}
    ros::spin();

    return 0;
}