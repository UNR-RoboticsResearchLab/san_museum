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
    std::vector <double> getPose ();
    // return x coordinate
    double getPoseX ();
    // return y coordinate
    double getPoseY ();

    // set coordinates
    void setPose (double x, double y);

    // receieve and process amcl message
    void amclCallback (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & AMCLmessage);
};
