#include <ros/ros.h>
#include <people_msgs/People.h>

// class for getting locations of people
class PeopleListener
{
  private:
    ros::NodeHandle peopleNode;
    //ROS_DEBUG ("created nodehandle peopleNode");

    // subscribe to people to get locations of people from leg detection
    ros::Subscriber peopleSub = peopleNode.subscribe ("people", 100, & PeopleListener::peopleCallback, this);
    //ROS_DEBUG ("subscribed to people");

    // vector to store coordinates of people
    std::vector <std::vector <double>> peopleLocations;

  public:
    // return vector of people locations
    std::vector <std::vector <double>> getPeopleLocations ();

    // sort the vector of people from least reliable to most reliable
    std::vector <std::vector <double>> sortByReliability ();

    // set coordinates and reliability of person
    void setPersonLocation (double x, double y, double r);
    
    // receieve and people message
    void peopleCallback (const people_msgs::People::ConstPtr & peopleMessage);

    // delete people location array
    void clearLocations ();
};
