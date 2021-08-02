#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/GetPlan.h>

#include "PoseListener.h"
#include "PeopleListener.h"

int readRoom (int roomDensity, double roomVulnerability);

std::vector <double> findGoal (std::vector <std::vector <double>> peopleLocations, std::vector <double> currentCoordinates, char goalType);

bool checkGoal (std::vector <double> currentCoordinates, std::vector <double> goalCoordinates);
bool goToGoal (std::vector <double> goalCoordinates);

void fillPathRequest (nav_msgs::GetPlan::Request & request, std::vector <double> startCoordinates, std::vector <double> endCoordinates);
bool callPlanningService (ros::ServiceClient & serviceClient, nav_msgs::GetPlan & serviceMessage);

int main (int argc, char ** argv)
{
  // initialize node
  ros::init (argc, argv, "san");
  //ROS_DEBUG ("initialized node san");

  PoseListener currentPose;
  PeopleListener peoplePresent;

  srand (time (NULL));

  ros::Rate loop_rate (1);
  ros::spinOnce ();
  loop_rate.sleep();
  // spinOnce has to be called twice before getting correct pose, there's probably a better way to do this
  ros::spinOnce ();

  // loop until ctrl-c is pressed or ros::shutdown is called
  while (ros::ok ())
  {
    std::vector <double> currentLocation = currentPose.getPose ();
    std::vector <std::vector <double>> people = peoplePresent.sortByReliability ();

    int density = people.size ();
    double vulnerability = rand () % 10 + 1;

    std::vector <double> goal;

    // whether or not the goal has been reached
    bool goalReached = false;

    switch (readRoom (density, vulnerability))
    {
      // engaging: low vulnerability, low density
      case 0:
        ROS_INFO ("engaging behavior");
        goal = findGoal (people, currentLocation, 'e');
        goalReached = goToGoal (goal);
        break;

      // conservative: low vulnerability, high density
      case 1:
        ROS_INFO ("conservative behavior");
        goal = findGoal (people, currentLocation, 'c');
        goalReached = goToGoal (goal);
        break;

      // reserved: high vulnerability, low density
      case 2:
        ROS_INFO ("reserved behavior");
        goal = findGoal (people, currentLocation, 'r');
        goalReached = goToGoal (goal);
        break;

      // stationary: high vulnerability, high density
      case 3:
        ROS_INFO ("stationary behavior");
        goal = findGoal (people, currentLocation, 's');
        goalReached = goToGoal (goal);
        break;
    }

    if (goalReached)
    {
      ROS_INFO ("behavior finished successfully\n");
    }

    else
    {
      ROS_WARN ("behavior not successful\n");
    }

    // get new position data
    ros::spinOnce ();
    loop_rate.sleep();
  }

  return 0;
}

int readRoom (int roomDensity, double roomVulnerability)
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
    if (highDensity == false)
    {
      // engaging
      return 0;
    }

    // conservative
    return 1;
  }

  if (highDensity == false)
  {
    // reserved
    return 2;
  }

  // stationary
  return 3;
}

std::vector <double> findGoal (std::vector <std::vector <double>> peopleLocations, std::vector <double> currentCoordinates, char goalType)
{
  int x = 0;
  int y = 1;

  std::vector <double> origin (2, 0);
  std::vector <double> stationaryLocation = {27.5, -41.5};
  std::vector <std::vector <double>> reservedLocations = {{31.6, -43.9}, {28.5, -44.3}, {27.6, -37.6}};

  std::vector <double> potentialGoal;
  bool goalIsOk = false;

  switch (goalType)
  {
    // engaging
    case 'e':
    {
      // todo: set speed to normal

      int index = peopleLocations.size ();

      do
      {
        index -= 1;

        potentialGoal.clear ();

        potentialGoal.push_back ((currentCoordinates.at (x) + peopleLocations.at (index).at (x)) / 2);
        potentialGoal.push_back ((currentCoordinates.at (y) + peopleLocations.at (index).at (y)) / 2);

        goalIsOk = checkGoal (currentCoordinates, potentialGoal);
      }
      while (index > 0 && !goalIsOk);

      // if goal has not been found
      if (!goalIsOk)
      {
        ROS_INFO ("could not find person to interact with, falling back to wandering");
        srand (time (NULL));
        // wander to convey less serious tone within museum
        do
        {
          potentialGoal.clear ();

          potentialGoal.push_back (currentCoordinates.at (x) + (rand () % 100 - 50) / 10);
          potentialGoal.push_back (currentCoordinates.at (y) + (rand () % 100 - 50) / 10);

          goalIsOk = checkGoal (currentCoordinates, potentialGoal);
        }
        while (!goalIsOk);
      }

      break;
    }
    // conservative
    case 'c':
    {
      // todo: set speed to slow

      int index = peopleLocations.size ();

      do
      {
        index -= 1;

        potentialGoal.clear ();

        potentialGoal.push_back ((currentCoordinates.at (x) + peopleLocations.at (index).at (x)) / 2);
        potentialGoal.push_back ((currentCoordinates.at (y) + peopleLocations.at (index).at (y)) / 2);

        goalIsOk = checkGoal (currentCoordinates, potentialGoal);
      }
      while (index > 0 && !goalIsOk);

      // if goal has not been found
      if (!goalIsOk)
      {
        ROS_INFO ("could not find person to interact with, staying in current position");

        // stay where you are
        potentialGoal = currentCoordinates;
      }

      break;
    }

    // reserved
    case 'r':
    {
      // todo: set speed to slower

      bool alreadyAtReservedLocation;

      // only go to predetermined locations

      for (int index = 0; index < reservedLocations.size (); index += 1)
      {
        if (abs (currentCoordinates.at (x) - reservedLocations.at (index).at (x)) < 1 && abs (currentCoordinates.at (y) - reservedLocations.at (index).at (y)) < 1)
        {
          alreadyAtReservedLocation = true;
        }
      }

      if (alreadyAtReservedLocation)
      {
        ROS_INFO ("already at reserved location, staying in current position");
        goalIsOk = true;
        potentialGoal = currentCoordinates;
      }

      else
      {
        int randomReservedLocation = rand () % 3;

        potentialGoal.clear ();

        potentialGoal.push_back (reservedLocations.at (randomReservedLocation).at (x));
        potentialGoal.push_back (reservedLocations.at (randomReservedLocation).at (y));

        goalIsOk = checkGoal (currentCoordinates, potentialGoal);
      }

      // if goal has not been found
      if (!goalIsOk)
      {
        ROS_INFO ("could not go to predetermined location, staying in current position");
        // stay where you are
        potentialGoal = currentCoordinates;
      }

      break;
    }

    // stationary
    case 's':
    {
      // todo: set speed to slowest

      // go to designated stationary location

      potentialGoal = stationaryLocation;

      goalIsOk = checkGoal (currentCoordinates, potentialGoal);

      // if no path to stationary location is available
      if (!goalIsOk)
      {
        ROS_INFO ("could not go to stationary location, staying in current position");
        // stay where you are
        potentialGoal = currentCoordinates;
      }

      break;
    }
  }

  return potentialGoal;
}

// check if the goal is too close to current location, a failed goal, or a previous location
bool checkGoal (std::vector <double> currentCoordinates, std::vector <double> goalCoordinates)
{
  ros::NodeHandle goalCheckNode;
  ros::ServiceClient planClient = goalCheckNode.serviceClient <nav_msgs::GetPlan> ("move_base/make_plan", true);
  nav_msgs::GetPlan planSrv;

  int x = 0;
  int y = 1;

  double locationThreshold = 1;

  // fill in the request for make_plan service
  fillPathRequest (planSrv.request, currentCoordinates, goalCoordinates);

  // if make_plan cannot find a plan
  if (!callPlanningService (planClient, planSrv))
  {
    //ROS_INFO ("goal not ok, no path from planner");
    return false;
  }

  // if goal is too close too current location
  if (abs (goalCoordinates.at (x) - currentCoordinates.at (x)) < locationThreshold && abs (goalCoordinates.at (y) - currentCoordinates.at (y)) < locationThreshold)
  {
    //ROS_INFO ("goal not ok, too close to current location");
    return false;
  }

  //ROS_INFO ("potential goal found");

  return true;
}

// from http://edu.gaitech.hk/turtlebot/map-navigation.html
bool goToGoal (std::vector <double> goalCoordinates)
{
  int x = 0;
  int y = 1;

  // define a client for to send goal requests to the move_base server through a SimpleActionClient
  actionlib::SimpleActionClient <move_base_msgs::MoveBaseAction> ac ("move_base", true);

  // wait for the action server to come up
  while (!ac.waitForServer (ros::Duration (5.0)))
  {
    ROS_DEBUG ("waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now ();

  // set goal coordinates
  goal.target_pose.pose.position.x =  goalCoordinates.at (x);
  goal.target_pose.pose.position.y =  goalCoordinates.at (y);
  goal.target_pose.pose.position.z =  0.0;
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = 0.0;
  goal.target_pose.pose.orientation.w = 1.0;

  //ROS_INFO ("goal: (%f, %f)", goalCoordinates.at (x), goalCoordinates.at (y));

  // send the goal
  ac.sendGoal (goal);
  ac.waitForResult ();

  if (ac.getState () == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
   //ROS_DEBUG ("robot reached the destination");
   return true;
  }

  else
  {
   //ROS_WARN ("robot did not reach the destination");
   return false;
  }
}

// from https://www.programmersought.com/article/85495009501/
void fillPathRequest (nav_msgs::GetPlan::Request & request, std::vector <double> startCoordinates, std::vector <double> endCoordinates)
{
  int x = 0;
  int y = 1;

  // set frame for starting position
  request.start.header.frame_id = "map";

  // set coordinates for starting position
  request.start.pose.position.x = startCoordinates.at (x);
  request.start.pose.position.y = startCoordinates.at (y);

  request.start.pose.orientation.w = 1.0;

  // set frame for ending position
  request.goal.header.frame_id = "map";

  // set coordinates for ending position
  request.goal.pose.position.x = endCoordinates.at (x);
  request.goal.pose.position.y = endCoordinates.at (y);

  request.goal.pose.orientation.w = 1.0;

  // from getplan service documentaion:
  // If the goal is obstructed, how many meters the planner can relax the constraint in x and y before failing.
  request.tolerance = 0.0;
}

// from https://www.programmersought.com/article/85495009501/
bool callPlanningService (ros::ServiceClient & serviceClient, nav_msgs::GetPlan & serviceMessage)
{
  // perform the actual path planner call
  // execute the actual path planner
  if (serviceClient.call (serviceMessage))
  {
    // srv.response.plan.poses is the container for storing the results, traversed and taken out
    if (!serviceMessage.response.plan.poses.empty ())
    {
      // std::for_each(srv.response.plan.poses.begin(),srv.response.plan.poses.end(),myfunction);
      //ROS_DEBUG ("make_plan success");
      return true;
    }
  }

  return false;
}
