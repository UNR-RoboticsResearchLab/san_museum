#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "PoseListener.h"

double PoseListener::getPoseX ()
{
  return poseAMCL.at (0);
}

double PoseListener::getPoseY ()
{
  return poseAMCL.at (1);
}

void PoseListener::setPose (double x, double y)
{
  poseAMCL.clear ();
  poseAMCL.push_back (x);
  poseAMCL.push_back (y);
}

void PoseListener::amclCallback (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & AMCLmessage)
{
  setPose (AMCLmessage -> pose.pose.position.x, AMCLmessage -> pose.pose.position.y);
}
