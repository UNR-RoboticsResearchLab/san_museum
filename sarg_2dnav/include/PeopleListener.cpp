#include "PeopleListener.h"

std::vector <std::vector <double>> PeopleListener::getPeopleLocations ()
{
  return peopleLocations;
}

std::vector <std::vector <double>> PeopleListener::sortByReliability ()
{
  if (peopleLocations.size () > 0)
  {
    ROS_INFO ("people present, sorting...");

    int x = 0;
    int y = 1;
    int r = 2;

    std::vector <std::vector <double>> reliabilitySorted = peopleLocations;

    //https://en.cppreference.com/w/cpp/algorithm/sort
    std::sort
    (
      reliabilitySorted.begin (),
      reliabilitySorted.end (),
      [] (const std::vector <double> & a, const std::vector <double> & b)
      {
        return a [2] < b [2];
      }
    );

    /*
    * for (int index = 0; index < reliabilitySorted.size (); index += 1)
    * {
    *   ROS_INFO ("person %d reliability: %.3f", index + 1, reliabilitySorted.at (index).at (r));
    * }
    */

    return reliabilitySorted;
  }

  return std::vector <std::vector <double>> {{0, 0, 0}};
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
    double x = peopleMessage -> people [index].position.x;
    double y = peopleMessage -> people [index].position.y;
    double r = peopleMessage -> people [index].reliability;

    setPersonLocation (x, y, r);
  }
}

void PeopleListener::clearLocations ()
{
  peopleLocations.clear ();
}
