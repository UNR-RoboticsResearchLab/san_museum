#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

// class for getting locations of people
class ObjectListener
{
  private:
    ros::NodeHandle objectNode;
    //ROS_DEBUG ("created nodehandle peopleNode");

    // subscribe to people to get locations of people from leg detection
    ros::Subscriber objectSub = objectNode.subscribe ("darknet_ros/bounding_boxes", 100, & ObjectListener::objectCallback, this);
    //ROS_DEBUG ("subscribed to people");

    // vector to store coordinates of people
    std::vector <std::string> objects;

  public:
    // return vector of people locations
    std::vector <std::string> getObjects();

    // return whether or not a certain object is detected
    bool hasObject (std::string searchObject);

    // set coordinates and reliability of person
    void addObject (std::string object);

    // receieve and people message
    void objectCallback (const darknet_ros_msgs::BoundingBoxes::ConstPtr & objectMessage);

    // delete people location array
    void clearObjects ();
};
