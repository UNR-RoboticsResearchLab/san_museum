#include "PoseListener.h"

double PoseListener::getPoseX ()
{
  // at (0) is the x coordinate
  return poseAMCL.at (0);
}

double PoseListener::getPoseY ()
{
  // at (1) is the y coordinate
  return poseAMCL.at (1);
}

double PoseListener::getRotation ()
{
  return poseAMCL.at (2);
}

void PoseListener::setPose (double x, double y, double r)
{
  // clear the vector before pushing back
  // this will stop the vector from becoming larger than 2 doubles (x and y coordinates)
  poseAMCL.clear ();

  // insert x coordinate
  poseAMCL.push_back (x);

  // insert y coordinate
  poseAMCL.push_back (y);

  // insert rotation
  poseAMCL.push_back (r);
}

void PoseListener::amclCallback (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & AMCLmessage)
{
  // set vector to current pose from amcl message
  setPose (AMCLmessage -> pose.pose.position.x, AMCLmessage -> pose.pose.position.y, AMCLmessage -> pose.pose.orientation.z);
}
