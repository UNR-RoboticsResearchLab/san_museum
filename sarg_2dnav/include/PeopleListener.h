#include <ros/ros.h>
#include <people_msgs/People.h>

// class for getting amcl pose
class PeopleListener
{
  private:
    ros::NodeHandle peopleNode;
    //ROS_DEBUG ("created nodehandle n");

    // subscribe to amcl pose to get estimated robot position
    ros::Subscriber peopleSub = peopleNode.subscribe ("people", 100, & PeopleListener::peopleCallback, this);
    //ROS_DEBUG ("subscribed to amcl_pose");

    // vector to store coordinates
    std::vector <std::vector <double>> peopleLocations;

  public:
    // return x coordinate
    std::vector <std::vector <double>> getPeopleLocations ();
    std::vector <std::vector <double>> sortByReliability ();

    // set coordinates
    void setPersonLocation (double x, double y, double r);

    // receieve and process amcl message
    void peopleCallback (const people_msgs::People::ConstPtr & peopleMessage);

    void clearLocations ();
};
