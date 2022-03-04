// http://wiki.ros.org/navigation/Tutorials/Writing%20a%20Local%20Path%20Planner%20As%20Plugin%20in%20ROS

#ifndef SANLOCALPLANNER_H_
#define SANLOCALPLANNER_H_

#include <nav_core/base_local_planner.h>
#include <tf/transform_listener.h>

#include <ros/ros.h>

namespace san_local_planner
{
  class SanLocalPlanner : public nav_core::BaseLocalPlanner
  {
    public:
      SanLocalPlanner ()
      {

      }

      SanLocalPlanner (std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS * costmap_ros)
      {
          initialize (name, tf, costmap_ros);
      }

      ~ SanLocalPlanner ()
      {

      }
      // void initialize (std::string name, tf::TransformListener * tf, costmap_2d::Costmap2DROS * costmap_ros)
      void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
      {
        if (false)
        {

        }
      }

      bool setPlan (const std::vector <geometry_msgs::PoseStamped> & orig_global_plan)
      {
        if (false)
        {
          ROS_ERROR ("planner not initialized, call initialize () first");

          return false;
        }

        return true;
      }

      bool computeVelocityCommands (geometry_msgs::Twist & cmd_vel)
      {
        if (false)
        {
          ROS_ERROR ("planner not initialized, call initialize () first");

          return false;
        }

        return true;
      }

      bool isGoalReached ()
      {
        if (false)
        {
          ROS_ERROR ("planner not initialized, call initialize () first");

          return false;
        }

        return false;
      }
  };
};

#endif
