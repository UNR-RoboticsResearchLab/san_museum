#include <geometry_msgs/PoseWithCovarianceStamped.h>
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

void PoseListener::setPose (double x, double y)
{
  // clear the vector before pushing back
  // this will stop the vector from becoming larger than 2 doubles (x and y coordinates)
  poseAMCL.clear ();

  // insert x coordinate
  poseAMCL.push_back (x);

  // insert y coordinate
  poseAMCL.push_back (y);
}

void PoseListener::amclCallback (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & AMCLmessage)
{
  // set vector to current pose from amcl message
  setPose (AMCLmessage -> pose.pose.position.x, AMCLmessage -> pose.pose.position.y);
}
