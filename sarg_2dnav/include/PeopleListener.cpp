#include "PeopleListener.h"

std::vector <std::vector <double>> PeopleListener::getPeopleLocations ()
{
  return peopleLocations;
}

std::vector <double> PeopleListener::getMostReliableLocation ()
{
  /*
  * ROS_INFO ("peopleLocations size: %d", peopleLocations.size ());
  * for (int index = 0; index < peopleLocations.size (); index += 1)
  * {
  *   ROS_INFO ("Person %d at (%.2f, %.2f) with %.2f reliability", index + 1, peopleLocations.at (index).at (0), peopleLocations.at (index).at (1), peopleLocations.at (index).at (2));
  * }
  */

  if (peopleLocations.size () > 0)
  {
    double mostReliable = peopleLocations.front ().at (2);
    int mostReliableIndex = 0;

    for (int index = 0; index < peopleLocations.size (); index += 1)
    {
      if (peopleLocations.at (index).at (2) > mostReliable)
      {
        mostReliable = peopleLocations.at (index).at (2);
        mostReliableIndex = index;
      }
    }

    ROS_INFO ("most reliable person at (%.2f, %.2f)", peopleLocations.at (mostReliableIndex).at (0), peopleLocations.at (mostReliableIndex).at (1));

    return peopleLocations.at (mostReliableIndex);
  }

  return {0, 0, 0};
}

void PeopleListener::setPersonLocation (double x, double y, double r)
{
  peopleLocations.push_back (std::vector <double> ({x, y, r}));
}

void PeopleListener::peopleCallback (const people_msgs::People::ConstPtr & peopleMessage)
{
  clearLocations ();

  for (int index = 0; index < peopleMessage -> people.size (); index += 1)
  {
    setPersonLocation (peopleMessage -> people [index].position.x, peopleMessage -> people [index].position.y, peopleMessage -> people [index].reliability);
  }
}

void PeopleListener::clearLocations ()
{
  peopleLocations.clear ();
}
