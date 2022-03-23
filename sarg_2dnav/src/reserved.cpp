#include <fstream>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/GetPlan.h>
#include <sound_play/sound_play.h>

#include "PoseListener.h"
#include "PeopleListener.h"
#include "MovementConfigurator.h"

//std::vector <std::vector <double>> loadLocations ();

std::vector <double> findGoal (std::vector <double> currentCoordinates, std::vector <std::vector <double>> peopleLocations);

bool checkGoal (std::vector <double> currentCoordinates, std::vector <double> goalCoordinates);
bool goToGoal (std::vector <double> goalCoordinates);

void fillPathRequest (nav_msgs::GetPlan::Request & request, std::vector <double> startCoordinates, std::vector <double> endCoordinates);
bool callPlanningService (ros::ServiceClient & serviceClient, nav_msgs::GetPlan & serviceMessage);

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

  ros::Rate loop_rate (1);
  ros::spinOnce ();
  loop_rate.sleep();
  // spinOnce has to be called twice before getting correct pose, there's probably a better way to do this
  ros::spinOnce ();

  // loop until ctrl-c is pressed or ros::shutdown is called
  while (ros::ok ())
  {
    // room vulnerability: 1 = sociable, 10 = serious
    double vulnerability = 3;

    // coordinates of the set goal
    std::vector <double> goal;

    // whether or not the goal has been reached
    bool goalReached = false;

    // set a goal based on vulnerability and density
    goal = findGoal (currentPose.getPose (), peoplePresent.getPeopleLocations ());
    // go to goal and check if goal was reached
    goalReached = goToGoal (goal);

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

/*
std::vector <std::vector <double>> loadLocations ()
{
  std::vector <std::vector <double>> locations;

  // for use of the at () vector function
  int x = 0;
  int y = 1;

  std::string reservedLine = "reserved:";

  std::string currentLine;

  std::ifstream locationsFile ("locations.txt");

  if (!locationsFile.is_open ())
  {
    ROS_WARN ("could not open locations.txt");

    return locations;
  }

  while (getline (locationsFile, currentLine))
  {
    if (currentLine == reservedLine)
    {
      std::cout << "found line \"reserved:\"" << std::endl;
      // go to the next line, that one has the coordinates
      getline (locationsFile, currentLine);

      // create a stream so the string can be properly evaluated
      std::stringstream currentLineStream;
      currentLineStream >> currentLine;

      // every coordinate found in the file will be stored here
      std::vector <double> coordinates;

      // loop until end of string
      while (!currentLineStream.eof ())
      {

      }

      // there should always be an even number of coordinates so this shouldnt break
      for (int index = 0; index < coordinates.size (); index += 2)
      {
        //std::vector <double> location = {coordinates.at (index), coordinates.at (index + 1)};
        //locations.push_back (location);
      }

      locationsFile.close ();

      ROS_INFO ("reserved locations loaded");
    }
  }

  return locations;
}
*/

std::vector <double> findGoal (std::vector <double> currentCoordinates, std::vector <std::vector <double>> peopleLocations)
{
  // for use of the at () vector function
  int x = 0;
  int y = 1;

  MovementConfigurator movementLimiter;

  // a set of predetermined locations for the robot to go to during reserved behavior
  std::vector <std::vector <double>> reservedLocations = {{0, 0}, {0, 0}};

  // stores goals to be tested
  std::vector <double> potentialGoal;
  // whether or not the goal meets the requirements
  bool goalIsOk = false;

  // search for a goal
  ROS_INFO ("reserved behavior");

  movementLimiter.setVelocityLimit ('x', 0.25);

  bool alreadyAtReservedLocation = false;
  double locationThreshold = 0.33;

  // only go to predetermined locations

  for (int index = 0; index < reservedLocations.size (); index += 1)
  {
    if (abs (currentCoordinates.at (x) - reservedLocations.at (index).at (x)) < locationThreshold && abs (currentCoordinates.at (y) - reservedLocations.at (index).at (y)) < locationThreshold)
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

  double locationThreshold = 0.33;

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
