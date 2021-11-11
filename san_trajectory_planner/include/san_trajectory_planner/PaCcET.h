//
//  PaCcET.h
//  Multi-Objective_Project
//
//  Created by Scott S Forer on 4/18/17.
//  Copyright © 2017 Scott S Forer. All rights reserved.
//

#ifndef PaCcET_h
#define PaCcET_h

#define PaCcET_VERBOSE 0

#define PFRONT_THRESHOLD 50
#define PFRONT_BUFFER 10

#define OBJECTIVES 2
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

using namespace std;

class PaCcET{
private:
    void N_Pro_transform();
    void N_Dummy_transform();
    
    /// Scaling, Nadir, Utopia calculations
    void scale();
    void calculate_scaled_pareto();
    void nad_ut();
    vector<double> utopia;
    vector<double> nadir;
    
    vector< vector<double> > PFront; /// P_I^*
    vector< vector<double> > scPFront; /// P_I^{*,norm}
    
    /// Components
    double calc_v_1(vector<double>);            // eq. 6
    double calc_v_B(vector<double>);            // eq. 7
    double calc_v_hp(vector<double>);           // eq. 8
    double calc_dtau(double, double, double);   // eq. 9
    
    /// Domination utilities
    void eliminate_not_dominating(list<vector<double> >& scPFront_temp, vector<double> td);
    
    
    vector< vector<double> > exhaustive_PFront; /// all points that have ever been part of PFront
    
    vector<double> input;
    vector<double> output;
    void take_input(vector<double>* coords);
    void give_output(vector<double>* coords);
    
    void thresh_PFront();
    void rand_thresh();
    
public:
    vector<vector<double> > get_PFront();
    
    void exhaustive_to_file();
    void PFront_to_file();
    
    /// PARETO UTILITIES
    bool Pareto_Check(vector<double>);
    
    
    /// PaCcET FUNCTIONALITY
    void Pareto_Reset();
    void execute_N_transform(vector<double>* pinputs);
    
    /// I/O
    void cout_pareto();
    void cout_scaled_pareto();
    
    /// QUADRET FUNCTIONS
    int get_PFront_size();
    vector<double> get_ut();
    bool does_v1_dominate_v2(vector<double> v1, vector<double> v2);
};


#endif /* PaCcET_h */
