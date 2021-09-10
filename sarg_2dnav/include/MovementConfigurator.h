#include <ros/ros.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

class MovementConfigurator
{
  private:
    // thanks https://answers.ros.org/question/12276/is-there-a-c-api-for-a-dynamic-reconfigure-client/?answer=64043#post-id-64043
    dynamic_reconfigure::ReconfigureRequest request;
    dynamic_reconfigure::ReconfigureResponse response;
    dynamic_reconfigure::DoubleParameter parameter;
    dynamic_reconfigure::Config config;

    std::string parameterName;
    double parameterValue;

  public:
    void setAccelerationLimit (char type, double limit);
    void setVelocityLimit (char type, double limit);
};
