#ifndef POSE_LISTENER_H_
#define POSE_LISTENER_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// class for getting amcl pose
class PoseListener
{
  private:
    ros::NodeHandle poseNode;
    //ROS_DEBUG ("created nodehandle poseNode");

    // subscribe to amcl pose to get estimated robot position
    ros::Subscriber amclSub = poseNode.subscribe ("amcl_pose", 100, & PoseListener::amclCallback, this);
    //ROS_DEBUG ("subscribed to amcl_pose");

    // vector to store coordinates
    std::vector <double> poseAMCL;

  public:
    std::vector <double> getPose ()
    {
      return poseAMCL;
    }

    double getPoseX ()
    {
      // at (0) is the x coordinate
      return poseAMCL.at (0);
    }

    double getPoseY ()
    {
      // at (1) is the y coordinate
      return poseAMCL.at (1);
    }

    void setPose (double x, double y)
    {
      // clear the vector before pushing back
      // this will stop the vector from becoming larger than 2 doubles (x and y coordinates)
      poseAMCL.clear ();

      // insert x coordinate
      poseAMCL.push_back (x);

      // insert y coordinate
      poseAMCL.push_back (y);
    }

    void amclCallback (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & AMCLmessage)
    {
      // set vector to current pose from amcl message
      setPose (AMCLmessage -> pose.pose.position.x, AMCLmessage -> pose.pose.position.y);
    }
};

#endif
