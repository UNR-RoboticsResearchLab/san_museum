#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/GetPlan.h>
#include <sound_play/sound_play.h>

#include "Behaviors.h"

char readRoom (int roomDensity, double roomVulnerability);

std::vector <double> findBehaviorGoal (std::vector <double> currentCoordinates, std::vector <std::vector <double>> peopleLocations, char goalType);

int main (int argc, char ** argv)
{
  // initialize node
  ros::init (argc, argv, "san");
  //ROS_DEBUG ("initialized node san");

  //sound_play::SoundClient sound;

  // declare listeners
  // PoseListener subscribes to amcl_pose topic
  PoseListener currentPose;
  // PeopleListener subscribes to people topic
  PeopleListener peoplePresent;

  // behavior handling
  Behaviors behavior;

  ros::Rate loop_rate (1);
  ros::spinOnce ();
  loop_rate.sleep();
  // spinOnce has to be called twice before getting correct pose, there's probably a better way to do this
  ros::spinOnce ();

  // loop until ctrl-c is pressed or ros::shutdown is called
  while (ros::ok ())
  {
    // room vulnerability: 1 = sociable, 10 = serious
    double vulnerability = 6;

    // coordinates of the set goal
    std::vector <double> goal;

    // whether or not the goal has been reached
    bool goalReached = false;

    // set a goal based on vulnerability and density
    goal = findBehaviorGoal (currentPose.getPose (), peoplePresent.getPeopleLocations (), readRoom (peoplePresent.getPeopleLocations ().size (), vulnerability));
    // go to goal and check if goal was reached
    goalReached = behavior.goToGoal (goal);

    if (goalReached)
    {
      ROS_INFO ("behavior finished successfully\n");
    }

    else
    {
      ROS_WARN ("behavior not successful\n");
    }

    // get new information from subscriptions
    ros::spinOnce ();
    loop_rate.sleep();
  }

  return 0;
}

char readRoom (int roomDensity, double roomVulnerability)
{
  // midpoint between high and low density
  int mediumDensity = 5;
  // midpoint between high and low vulnerability
  int mediumVulnerability = 5;

  bool highDensity = true;
  bool highVulnerability = true;

  // check density
  if (roomDensity < mediumDensity)
  {
    highDensity = false;
  }

  // check vulnerability
  if (roomVulnerability < mediumVulnerability)
  {
    highVulnerability = false;
  }

  // return room reading
  if (highVulnerability == false)
  {
    // ROS_INFO ("low vulnerability");
    if (highDensity == false)
    {
      // engaging
      // ROS_INFO ("low density");
      return 'e';
    }

    // conservative
    // ROS_INFO ("high density")
    return 'c';
  }

  // ROS_INFO ("high vulnerability");
  if (highDensity == false)
  {
    // reserved
    // ROS_INFO ("low density");
    return 'r';
  }

  // stationary
  // ROS_INFO ("high density");
  return 's';
}

std::vector <double> findBehaviorGoal (std::vector <double> currentCoordinates, std::vector <std::vector <double>> peopleLocations, char goalType)
{
  // for use of the at () vector function
  int x = 0;
  int y = 1;

  MovementConfigurator movementLimiter;

  // stores goals to be tested
  std::vector <double> potentialGoal;
  // whether or not the goal meets the requirements
  bool goalIsOk = false;

  // search for a goal
  switch (goalType)
  {
    // engaging
    case 'e':
    {
      Engaging engagingBehavior;
      potentialGoal = engagingBehavior.findGoal (currentCoordinates, peopleLocations);
      ROS_INFO ("engaging class behavior finished");

      break;
    }
    // conservative
    case 'c':
    {
      Conservative conservativeBehavior;
      potentialGoal = conservativeBehavior.findGoal (currentCoordinates, peopleLocations);
      ROS_INFO ("conservative class behavior finished");

      break;
    }

    // reserved
    case 'r':
    {
      Reserved reservedBehavior;
      potentialGoal = reservedBehavior.findGoal (currentCoordinates, peopleLocations);
      ROS_INFO ("reserved class behavior finished");

      break;
    }

    // stationary
    case 's':
    {
      Stationary stationaryBehavior;
      potentialGoal = stationaryBehavior.findGoal (currentCoordinates, peopleLocations);
      ROS_INFO ("stationary class behavior finished");

      break;
    }
  }

  return potentialGoal;
}
