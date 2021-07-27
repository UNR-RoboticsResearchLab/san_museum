#include "PeopleListener.h"

std::vector <std::vector <double>> PeopleListener::getPeopleLocations ()
{
  return peopleLocations;
}

std::vector <std::vector <double>> PeopleListener::sortByReliability ()
{
  // if peopleLocations has items
  if (peopleLocations.size () > 0)
  {
    //ROS_INFO ("people present, sorting...");

    // copy the vector of people locations
    std::vector <std::vector <double>> reliabilitySorted = peopleLocations;

    // sort the copied vector
    // https://en.cppreference.com/w/cpp/algorithm/sort
    std::sort
    (
      reliabilitySorted.begin (),
      reliabilitySorted.end (),
      [] (const std::vector <double> & a, const std::vector <double> & b)
      {
        return a [2] < b [2];
      }
    );

    return reliabilitySorted;
  }

  // if peopleLocations is empty, return a vector of zeroes
  return std::vector <std::vector <double>> {{0, 0, 0}};
}

void PeopleListener::setPersonLocation (double x, double y, double r)
{
  // add to peopleLocations in push_back in format (x coordinate, y coordinate, reliability)
  peopleLocations.push_back (std::vector <double> ({x, y, r}));
}

void PeopleListener::peopleCallback (const people_msgs::People::ConstPtr & peopleMessage)
{
  // clear previously stored people locations (since people move)
  clearLocations ();

  // add in new locations of people
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
