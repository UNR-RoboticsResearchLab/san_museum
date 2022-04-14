/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *\
* PaCcET.h                                                                                                                        *
* Multi-Objective_Project                                                                                                         *
*                                                                                                                                 *
* Created by Scott S Forer on 4/18/17.                                                                                            *
* Copyright © 2017 Scott S Forer. All rights reserved.                                                                            *
*                                                                                                                                 *
* https://github.com/UNR-RoboticsResearchLab/unr_san/blob/art_san/san_trajectory_planner/include/san_trajectory_planner/PaCcET.h  *
\* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#ifndef PaCcET_h
#define PaCcET_h

#define PaCcET_VERBOSE 0

#define PFRONT_THRESHOLD 50
#define PFRONT_BUFFER 10

#define OBJECTIVES 6
#define PI 3.1415

#ifndef VECTOR_INCLUDE
#define VECTOR_INCLUDE

#include <vector>
#include <list>
#include <numeric>

#include <iostream>
#include <cstdlib>
#include <cmath>
#include <time.h>
#include <stdio.h>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <cassert>
#include <ctime>
#include <random>

#endif

class PaCcET
{
  public:
    std::vector <std::vector <double>> get_PFront ()
    {
      return PFront;
    }

    void exhaustive_to_file ()
    {
      // all points that have ever been in P_I^*
      FILE * PFILE;

      std::cout << "exhaustive in" << std::endl;

      PFILE = fopen ("exhaustive_pareto.txt", "w");

      for (int index = 0; index < exhaustive_PFront.size (); index += 1)
      {
        for (int subIndex = 0; subIndex < exhaustive_PFront.at (index).size (); subIndex += 1)
        {
          report (PFILE, exhaustive_PFront.at (index).at (subIndex));
        }

        newline (PFILE);
      }

      fclose (PFILE);

      std::cout << "exhaustive out" << std::endl;
    }

    void PFront_to_file ()
    {
      // only current P_I^*
      FILE * PFILE;

      std::cout << "Pfront to file in" << std::endl;

      PFILE = fopen ("T_final_front.txt", "w");

      for (int index = 0; index < PFront.size (); index += 1)
      {
        for (int subIndex = 0; subIndex < PFront.at (index).size (); subIndex += 1)
        {
          report (PFILE, PFront.at (index).at (subIndex));
        }

        newline (PFILE);
      }

      std::cout << "Pfront to file out" << std::endl;
    }

    // PARETO UTILITIES
    bool Pareto_Check (vector <double> unscaled_coords)
    {
      // is the given point dominated by any point in the Pareto front?
      // for each Pareto point
      for (int index = 0; index < PFront.size (); index += 1)
      {
        // if the point is dominated
        if (does_v1_dominate_v2 (PFront.at (index), unscaled_coords))
        {
          return false;
        }
      }

      // does the point dominate any points in the Pareto front?
      vector <int> eliminate;

      for (int index = 0; index < PFront.size (); index += 1)
      {
        // if the new point scored higher or equal on all critera
        if (does_v1_dominate_v2 (unscaled_coords, PFront.at (index)))
        {
          // the Pareto point is dominated and should be eliminated
          eliminate.push_back (index);
        }
      }

      // eliminate dominated points in the Pareto front
      for (int index = eliminate.size () - 1; index >= 0; index -= 1)
      {
        // eliminate from end to beginning so calculated indicies stay valid
        int spot = eliminate.at (index);

        PFront.erase (PFront.begin () + spot);
      }

      // add new point in correct spot in Pareto front
      PFront.push_back (unscaled_coords);

      // also add it to the exhaustive list
      exhaustive_PFront.push_back (unscaled_coords);

      thresh_PFront ();

      // recalculate the dominated hyperspace (idk)
      nad_ut ();
      calculate_scaled_pareto ();

      return true;
    }

    // PaCcET FUNCTIONALITY
    void Pareto_Reset ()
    {
      PFront.clear ();
      scPFront.clear ();
      utopia.clear ();
      nadir.clear ();
      input.clear ();
      output.clear ();
    }

    void execute_N_transform(vector <double> * pinputs);

    // I/O
    void cout_pareto ()
    {
      std::cout << "Current Non-Dominated Set:" << std::endl;

      for (int index = 0; index < PFront.size (); index += 1)
      {
        for (int subIndex = 0; subIndex < PFront.at (index).size (); subIndex += 1)
        {
          std::cout << PFront.at (index).at (subIndex) << "\t";
        }

        std::cout << std::endl;
      }

      std::cout << std::endl;
    }

    void cout_scaled_pareto ()
    {
      std::cout << std::endl;
      std::cout << "Current Non-Dominated Set (NORM):" << std::endl;

      for (int index = 0; index < scPFront.size (); index += 1)
      {
        for (int subIndex = 0; subIndex < PFront.at (index).size (); subIndex += 1)
        {
          std:cout << scPFront.at (index).at (subIndex) << "\t";
        }

        std::cout << std::endl;
      }

      std::cout << std::endl;
    }

    // QUADRET FUNCTIONS
    int get_PFront_size ()
    {
      return PFront.size ();
    }

    std::vector <double> get_ut ()
    {
      return utopia;
    }

    bool does_v1_dominate_v2 (std::vector <double> v1, std::vector <double> v2)
    {
      int counter = 0;

      for (int index = 0; index < OBJECTIVES; index += 1)
      {
        // if v1 scores better on a criteria, increment counter
        if (v1.at (index) <= v2.at (index))
        {
          counter += 1;
        }
      }

      // if v1 scored higher or equal on all criteria then v2 is dominated
      if (counter == OBJECTIVES)
      {
        return true;
      }

      return false;
    }

  protected:
    void report (FILE * pFILE, double value)
    {
      // report to text file
      fprintf (pFILE, "%.5f\t", value);
    }

    void newline (FILE * pFILE)
    {
      // report to text file
      fprintf (pFILE, "\b \b\n");
    }

  private:
    void N_Pro_transform ();
    void N_Dummy_transform ();

    // Scaling, Nadir, Utopia calculations
    void scale ();
    void calculate_scaled_pareto ();
    void nad_ut ();
    std::vector <double> utopia;
    std::vector <double> nadir;

    std::vector <std::vector <double>> PFront; // P_I^*
    std::vector <std::vector <double>> scPFront; // P_I^{*,norm}

    // Components
    double calc_v_1 (std::vector <double>);            // eq. 6
    double calc_v_B (std::vector <double>);            // eq. 7
    double calc_v_hp (std::vector <double>);           // eq. 8
    double calc_dtau (double, double, double);         // eq. 9

    // Domination utilities
    void eliminate_not_dominating (std::list <std::vector <double>> & scPFront_temp, std::vector <double> td);

    std::vector <std::vector <double>> exhaustive_PFront; // all points that have ever been part of PFront

    std::vector <double> input;
    std::vector <double> output;
    void take_input (std::vector <double>* coords);
    void give_output (std::vector <double>* coords);

    void thresh_PFront ();
    void rand_thresh ();
};

#endif
