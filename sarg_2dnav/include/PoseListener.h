#include <vector>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class PoseListener
{
  private:
    std::vector <double> poseAMCL;

  public:
    double getPoseX ();
    double getPoseY ();

    void setPose (double x, double y);

    void amclCallback (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & AMCLmessage);
};
