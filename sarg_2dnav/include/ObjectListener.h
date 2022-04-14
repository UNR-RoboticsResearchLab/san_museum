#ifndef OBJECT_LISTENER_H_
#define OBJECT_LISTENER_H_

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
    std::vector <std::string> getObjects()
    {
      return objects;
    }

    void addObject (std::string object)
    {
      // add to peopleLocations in push_back in format (x coordinate, y coordinate, reliability)
      objects.push_back (object);
    }

    bool hasObject (std::string searchObject)
    {
      for (int index = 0; index < objects.size (); index += 1)
      {
        if (objects.at (index) == searchObject)
        {
          //ROS_INFO ("%s found", searchObject);

          return true;
        }
      }

      //ROS_INFO ("%s not found", searchObject);
      return false;
    }

    void objectCallback (const darknet_ros_msgs::BoundingBoxes::ConstPtr & objectMessage)
    {
      // clear previously stored people locations (since people move)
      clearObjects ();

      // add in new locations of people
      for (int index = 0; index < objectMessage -> bounding_boxes.size (); index += 1)
      {
        std::string objectType = objectMessage -> bounding_boxes [index].Class;

        addObject (objectType);
      }
    }

    void clearObjects ()
    {
      objects.clear ();
    }
};

#endif
