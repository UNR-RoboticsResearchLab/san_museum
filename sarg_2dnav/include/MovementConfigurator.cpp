#include "MovementConfigurator.h"

void MovementConfigurator::setAccelerationLimit (char type, double limit)
{
  switch (type)
  {
    case 'x':
    {
      parameterName = "acc_lim_x";

      break;
    }

    case 'y':
    {
      parameterName = "acc_lim_y";

      break;
    }

    case 't':
    {
      parameterName = "acc_lim_th";

      break;
    }
  }

  // thanks https://answers.ros.org/question/12276/is-there-a-c-api-for-a-dynamic-reconfigure-client/?answer=64043#post-id-64043
  parameter.name = parameterName;
  parameter.value = limit;
  config.doubles.push_back (parameter);

  request.config = config;

  ros::service::call ("move_base/DWAPlannerROS/set_parameters", request, response);
}

void MovementConfigurator::setVelocityLimit (char type, double limit)
{
  switch (type)
  {
    case 'x':
    {
      parameterName = "max_vel_x";

      break;
    }

    case 'y':
    {
      parameterName = "max_vel_y";

      break;
    }

    case 'r':
    {
      parameterName = "max_rot_vel";

      break;
    }
  }

  parameter.name = parameterName;
  parameter.value = limit;
  config.doubles.push_back (parameter);

  request.config = config;

  // thanks https://answers.ros.org/question/12276/is-there-a-c-api-for-a-dynamic-reconfigure-client/?answer=64043#post-id-64043
  ros::service::call ("move_base/DWAPlannerROS/set_parameters", request, response);
}
