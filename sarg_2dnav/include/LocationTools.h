#ifndef LOCATIONTOOLS_H_
#define LOCATIONTOOLS_H_

#include <fstream>

class LocationTools
{
  public:
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
      std::ofstream temporaryFile (locationsTempFilename);
      std::ifstream locationsFile (locationsFilename);

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
          std::ofstream locationsFileCreated (locationsFilename);

          // create and set locations.txt to expected format
          locationsFileCreated << "stationary:" << std::endl;
          locationsFileCreated << std::endl;
          locationsFileCreated << "reserved:" << std::endl;
          locationsFileCreated << std::endl;

          locationsFileCreated.close ();

          ROS_INFO ("created locations.txt");

          // reopen the file for reading
          std::ifstream locationsFile (locationsFilename);
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

      remove (locationsFilename);

      rename (locationsTempFilename, locationsFilename);

      return;
    }

    std::vector <std::vector <double>> loadLocations (char locationType)
    {
      std::vector <std::vector <double>> locations;

      // for use of the at () vector function
      int x = 0;
      int y = 1;

      std::string currentLine;

      std::ifstream locationsFile (locationsFilename);

      if (!locationsFile.is_open ())
      {
        ROS_WARN ("could not open locations.txt");

        std::vector <double> location;

        // in case of failure, return (0, 0)
        location.push_back (0);
        location.push_back (0);

        locations.push_back (location);

        return locations;
      }

      switch (locationType)
      {
        case 'r':
        {
          std::string reservedLine = "reserved:";

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
                // assign values to coordinates vector
              }

              locations = listToCoordinates (coordinates, 2);

              locationsFile.close ();

              ROS_INFO ("reserved locations loaded");
            }
          }

          break;
        }

        case 's':
        {
          std::string stationaryLine = "stationary:";

          while (getline (locationsFile, currentLine))
          {
            if (currentLine == stationaryLine)
            {
              std::cout << "found line \"stationary:\"" << std::endl;
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
                // assign values to coordinates vector
              }

              locations = listToCoordinates (coordinates, 2);

              locationsFile.close ();

              ROS_INFO ("stationary locations loaded");
            }
          }

          break;
        }
      }

      return locations;
    }

  protected:
    std::vector <std::vector <double>> listToCoordinates (std::vector <double> original, int dimensions)
    {
      // the complete set
      std::vector <std::vector <double>> coordinates;

      for (int index = 0; index < original.size (); index += 1)
      {
        // a single coordinate pair (or group depending on dimensions)
        std::vector <double> point;

        point.push_back (original.at (index));

        // condition is met when all dimensions of a point are filled
        if (index % dimensions == dimensions - 1)
        {
          // add point to set of coordinates
          coordinates.push_back (point);
        }
      }

      return coordinates;
    }

  private:
    std::string locationsFilename = ("locations.txt");
    std::string locationsTempFilename = ("locationsTemporary.txt");
};

#endif
