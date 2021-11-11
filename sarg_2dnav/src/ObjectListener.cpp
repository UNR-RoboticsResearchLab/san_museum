#include "ObjectListener.h"

std::vector <std::string> ObjectListener::getObjects()
{
  return objects;
}

void ObjectListener::addObject (std::string object)
{
  // add to peopleLocations in push_back in format (x coordinate, y coordinate, reliability)
  objects.push_back (object);
}

bool ObjectListener::hasObject (std::string searchObject)
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

void ObjectListener::objectCallback (const darknet_ros_msgs::BoundingBoxes::ConstPtr & objectMessage)
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

void ObjectListener::clearObjects ()
{
  objects.clear ();
}
