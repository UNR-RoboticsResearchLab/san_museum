#ifndef BEHAVIORS_H_
#define BEHAVIORS_H_

#include <fstream>

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/GetPlan.h>
#include <sound_play/sound_play.h>

#include "PoseListener.h"
#include "PeopleListener.h"
#include "MovementConfigurator.h"

class Behaviors
{
  public:
    virtual std::vector <double> findGoal (std::vector <double> currentCoordinates, std::vector <std::vector <double>> peopleLocations)
    {
      // set a navigation goal

      return {0, 0};
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

  protected:
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

    bool checkGoal (std::vector <double> currentCoordinates, std::vector <double> goalCoordinates, double locationThreshold)
    {
      ros::NodeHandle goalCheckNode;
      ros::ServiceClient planClient = goalCheckNode.serviceClient <nav_msgs::GetPlan> ("move_base/make_plan", true);
      nav_msgs::GetPlan planSrv;

      int x = 0;
      int y = 1;

      // fill in the request for make_plan service
      fillPathRequest (planSrv.request, currentCoordinates, goalCoordinates);

      // if make_plan cannot find a plan
      if (!callPlanningService (planClient, planSrv))
      {
        ROS_INFO ("goal not ok, no path from planner");
        return false;
      }

      // if goal is too close too current location
      if (abs (goalCoordinates.at (x) - currentCoordinates.at (x)) < locationThreshold && abs (goalCoordinates.at (y) - currentCoordinates.at (y)) < locationThreshold)
      {
        ROS_INFO ("goal not ok, too close to current location");
        return false;
      }

      ROS_INFO ("potential goal found");

      return true;
    }
};

class Engaging : public Behaviors
{
  public:
    std::vector <double> findGoal (std::vector <double> currentCoordinates, std::vector <std::vector <double>> peopleLocations)
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
      ROS_INFO ("engaging behavior");

      movementLimiter.setVelocityLimit ('x', 0.45);

      int index = peopleLocations.size ();

      do
      {
        // iterate through the people vector backwards (its sorted from least reliable to most)
        index -= 1;

        // forget the previous invalid goal
        potentialGoal.clear ();

        // set a new goal halfway between the robot and a person
        potentialGoal.push_back ((currentCoordinates.at (x) + peopleLocations.at (index).at (x)) / 2);
        potentialGoal.push_back ((currentCoordinates.at (y) + peopleLocations.at (index).at (y)) / 2);

        // test the goal
        goalIsOk = checkGoal (currentCoordinates, potentialGoal, 0.5);
      }
      // while there is a person location to test and a valid goal has not been found
      while (index > 0 && !goalIsOk);

      // if goal between a person has not been found
      if (!goalIsOk)
      {
        ROS_INFO ("could not find person to interact with, falling back to wandering");
        srand (time (NULL));
        // wander randomly to convey less serious tone within museum
        do
        {
          // forget the previous invalid goal
          potentialGoal.clear ();

          // set a goal at a random location relative to the robot's current location
          potentialGoal.push_back (currentCoordinates.at (x) + (rand () % 100 - 50) / 10);
          potentialGoal.push_back (currentCoordinates.at (y) + (rand () % 100 - 50) / 10);

          // test the goal
          goalIsOk = checkGoal (currentCoordinates, potentialGoal, 0.5);
        }
        // while a valid goal has not been found
        while (!goalIsOk);
      }

      return potentialGoal;
    }

  protected:
    // code adapted from http://www.cplusplus.com/forum/general/47399/#msg274269
    void saveLocation (std::vector <double> currentCoordinates, char locationType)
    {
      // for use of the at () vector function
      int x = 0;
      int y = 1;

      std::string stationaryLine = "stationary:";
      std::string reservedLine = "reserved:";

      std::string currentLine;

      // you have to write the full path or else the file path will depend on where the node is run (https://answers.ros.org/question/11642/write-to-a-file-from-a-c-ros-node/)
      std::ofstream temporaryFile ("locationsTemporary.txt");
      std::ifstream locationsFile ("locations.txt");

      if (!locationsFile.is_open () || !temporaryFile.is_open ())
      {
        if (!locationsFile.is_open () && !temporaryFile.is_open ())
        {
          ROS_WARN ("could not open both locations.txt and locationsTemporary.txt");
        }

        else if (!locationsFile.is_open ())
        {
          ROS_WARN ("could not open locations.txt, creating");

          // open the file for writing
          std::ofstream locationsFileCreated ("locations.txt");

          // create and set locations.txt to expected format
          locationsFileCreated << "stationary:" << std::endl;
          locationsFileCreated << std::endl;
          locationsFileCreated << "reserved:" << std::endl;
          locationsFileCreated << std::endl;

          locationsFileCreated.close ();

          ROS_INFO ("created locations.txt");

          // reopen the file for reading
          std::ifstream locationsFile ("locations.txt");
        }

        else
        {
          ROS_WARN ("could not open locationsTemporary.txt");
        }

        return;
      }

      while (getline (locationsFile, currentLine))
      {
        if (locationType == 's' && currentLine == stationaryLine)
        {
          temporaryFile << "stationary:" << std::endl;

          // just to stop unwanted lines of text from appearing
          getline (locationsFile, currentLine);

          temporaryFile << "( " << currentCoordinates.at (x) << " , " << currentCoordinates.at (y) << " )" << std::endl;

          ROS_INFO ("location saved for stationary behavior");
        }

        else if (locationType == 'r' && currentLine == reservedLine)
        {
          temporaryFile << "reserved:" << std::endl;

          // we want to append to the list of reserved locations
          getline (locationsFile, currentLine);

          temporaryFile << currentLine << "( " << currentCoordinates.at (x) << ", " << currentCoordinates.at (y) << " ) " << std::endl;

          ROS_INFO ("location saved for reserved behavior");
        }

        // this means that the current line is not be what we want to modify
        else
        {
          temporaryFile << currentLine << std::endl;
        }
      }

      temporaryFile.close ();
      locationsFile.close ();

      remove ("locations.txt");

      rename ("locationsTemporary.txt", "locations.txt");

      return;
    }
};

class Conservative : public Behaviors
{
  public:
    std::vector <double> findGoal (std::vector <double> currentCoordinates, std::vector <std::vector <double>> peopleLocations)
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
      ROS_INFO ("conservative behavior");

      movementLimiter.setVelocityLimit ('x', 0.35);

      int index = peopleLocations.size ();

      do
      {
        // iterate through the people vector backwards (its sorted from least reliable to most)
        index -= 1;

        // forget the previous invalid goal
        potentialGoal.clear ();

        // set a new goal, todo: keep a larger distance from people
        potentialGoal.push_back ((currentCoordinates.at (x) + peopleLocations.at (index).at (x)) / 2);
        potentialGoal.push_back ((currentCoordinates.at (y) + peopleLocations.at (index).at (y)) / 2);

        // test the goal
        goalIsOk = checkGoal (currentCoordinates, potentialGoal, 1.0);
      }
      // while a valid goal has not been found
      while (index > 0 && !goalIsOk);

      // if goal has not been found
      if (!goalIsOk)
      {
        ROS_INFO ("could not find person to interact with, staying in current position");

        // stay where you are
        potentialGoal = currentCoordinates;
        ROS_INFO ("potentialGoal set");
      }

      return potentialGoal;
    }

  /*
  protected:
    void saveLocation (std::vector <double> currentCoordinates, char locationType);
  */
};

class Reserved : public Behaviors
{
  public:
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
        ROS_INFO ("potentialGoal set");
      }

      else
      {
        int randomReservedLocation = rand () % 3;

        potentialGoal.clear ();

        potentialGoal.push_back (reservedLocations.at (randomReservedLocation).at (x));
        potentialGoal.push_back (reservedLocations.at (randomReservedLocation).at (y));

        goalIsOk = checkGoal (currentCoordinates, potentialGoal, locationThreshold);
      }

      // if goal has not been found
      if (!goalIsOk)
      {
        ROS_INFO ("could not go to predetermined location, staying in current position");
        // stay where you are
        potentialGoal = currentCoordinates;
        ROS_INFO ("potentialGoal set");
      }

      return potentialGoal;
    }

  /*
  protected:
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
};

class Stationary : public Behaviors
{
  public:
    std::vector <double> findGoal (std::vector <double> currentCoordinates, std::vector <std::vector <double>> peopleLocations)
    {
      // for use of the at () vector function
      int x = 0;
      int y = 1;

      MovementConfigurator movementLimiter;

      // the "kiosk" location for the robot to stay in during stationary behavior
      std::vector <double> stationaryLocation = {-3.35937, -3.2264};

      // stores goals to be tested
      std::vector <double> potentialGoal;
      // whether or not the goal meets the requirements
      bool goalIsOk = false;

      // search for a goal
      ROS_INFO ("stationary behavior");

      ROS_INFO ("setting velocity limit to 0.15 ...");
      movementLimiter.setVelocityLimit ('x', 0.15);

      // go to designated stationary location

      potentialGoal = stationaryLocation;

      ROS_INFO ("checking goal ...");
      goalIsOk = checkGoal (currentCoordinates, potentialGoal, 0.33);

      // play sound here
      //sound.playWave ("file location");

      // if no path to stationary location is available
      if (!goalIsOk)
      {
        ROS_INFO ("could not go to stationary location, staying in current position");
        // stay where you are
        potentialGoal = currentCoordinates;
        ROS_INFO ("potentialGoal set");
      }

      return potentialGoal;
    }

  /*
  protected:
    std::vector <std::vector <double>> loadLocations ();
  */
};

#endif
