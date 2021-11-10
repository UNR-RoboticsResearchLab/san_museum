#include <san_trajectory_planner/PaCcET.h>

vector<vector<double> > PaCcET::get_PFront()
{
    return PFront;
}




vector<double> PaCcET::get_ut(){
    return utopia;
}

int PaCcET::get_PFront_size(){
    return PFront.size();
}

void report(FILE* pFILE, double value) { /// report to text file
    fprintf(pFILE, "%.5f\t", value);
}

void newline(FILE* pFILE) { /// report to text file
    fprintf(pFILE, "\b \b\n");
}

void PaCcET::exhaustive_to_file(){
    /// All points that have ever been in P_I^*
    FILE* PFILE;
    cout << "exhaustive in" << endl;
    PFILE=fopen("exhaustive_pareto.txt","w");
    for(int i=0; i<exhaustive_PFront.size(); i++){
        for(int j=0; j<exhaustive_PFront.at(i).size(); j++){
            report(PFILE,exhaustive_PFront.at(i).at(j));
        }
        newline(PFILE);
    }
    fclose(PFILE);
    cout << "exhaustive out" << endl;
}

void PaCcET::PFront_to_file(){
    /// Only current P_I^*
    FILE* PFILE;
    cout << "Pfront to file in" << endl;
    PFILE=fopen("T_final_front.txt","w");
    for(int i=0; i<PFront.size(); i++){
        for(int j=0; j<PFront.at(i).size(); j++){
            report(PFILE,PFront.at(i).at(j));
        }
        newline(PFILE);
    }
    cout << "Pfront to file out" << endl;
}

void PaCcET::Pareto_Reset(){
    PFront.clear();
    scPFront.clear();
    utopia.clear();
    nadir.clear();
    input.clear();
    output.clear();
}

void PaCcET::cout_pareto(){
    cout << "Current Non-Dominated Set:" << endl;
    int P = PFront.size();
    for(int p=0; p<P; p++){
        for(int q=0; q<PFront.at(p).size(); q++){
            cout << PFront.at(p).at(q) << "\t";
        }
        cout << endl;
    }
    cout << endl;
}

void PaCcET::cout_scaled_pareto(){
    cout << endl;
    cout << "Current Non-Dominated Set (NORM):" << endl;
    int P = scPFront.size();
    for(int p=0; p<P; p++){
        for(int q=0; q<PFront.at(p).size(); q++){
            cout << scPFront.at(p).at(q) << "\t";
        }
        cout << endl;
    }
    cout << endl;
}

bool PaCcET::does_v1_dominate_v2(vector<double> v1, vector<double> v2){
    int counter=0;
    for(int obj=0; obj<OBJECTIVES; obj++){
        /// If v1 scores better on a criteria, increment counter
        if(v1.at(obj) <= v2.at(obj)){
            counter++;
        }
    }
    if(counter==(OBJECTIVES)){
        /// If v1 scored higher or equal on all criteria...
        /// v2 is dominated.
        /// Return True.
        return true;
    }
    /// It is not dominated. Return False.
    return false;
}


bool PaCcET::Pareto_Check(vector<double> unscaled_coords){
    /// <Display the point in question>
    //cout << "Pareto Checking Point: ";
    //for(int i=0; i<coords.size(); i++){
    //    cout << coords.at(i) << "\t";
    //}
    //cout << endl;
    
    /// <Is it dominated by any point in the Pareto front??>
    /// For each Pareto point
    for(int pt=0; pt<PFront.size(); pt++){
        // determine if coords is dominated:
        if(does_v1_dominate_v2(PFront.at(pt),unscaled_coords)){
            return false;
        }
    }
    
    /// <Does it dominate any points on the Pareto front?>
    vector<int> eliminate;
    for(int pt=0; pt<PFront.size(); pt++){
        if(does_v1_dominate_v2(unscaled_coords,PFront.at(pt))){
            /// If the new point scored higher or equal on all criteria
            /// The "Pareto" point is dominated, and should be eliminated.
            eliminate.push_back(pt);
        }
    }
    
    /// <Eliminate dominated points on the Pareto Front>
    for(int e=eliminate.size()-1; e>=0; e--){
        /// We eliminate from end -> beginning so that the indices we calculated remain valid.
        int spot = eliminate.at(e);
        PFront.erase(PFront.begin()+spot);
    }
    
    /// <Add new point in correct spot of Pareto Front>
    PFront.push_back(unscaled_coords);
    // also add it to master list.
    exhaustive_PFront.push_back(unscaled_coords);
    
    thresh_PFront();
    
    /// Since Pareto Front has changed, we recalculate the dominated hyperspace.
    nad_ut();
    calculate_scaled_pareto();
    return true;
}

void PaCcET::thresh_PFront(){
    /// Function: Makes sure that the PFront is maintained at below a threshold size.
    /// If PFront is over the threshold
    if(PFront.size()>=PFRONT_THRESHOLD+PFRONT_BUFFER){
        rand_thresh();
    }
}

void PaCcET::rand_thresh(){
    while(PFront.size()>=PFRONT_THRESHOLD){
        PFront.erase(PFront.begin()+rand()%PFront.size());
    }
}

void PaCcET::calculate_scaled_pareto(){
    /// update vector< vector<double> > scPFront;
    //cout << "Calculating Scaled Pareto\t";
    scPFront.clear();
    for(int i=0; i < PFront.size(); i++){
        vector<double> dual;
        for(int j=0; j < PFront.at(i).size(); j++){
            //cout << "Pfront at i size: " << PFront.at(i).size() << endl;
            //cout << "Nadir size: " << nadir.size() << endl;
            double val = PFront.at(i).at(j);
            double min = utopia.at(j);
            double range = nadir.at(j)-utopia.at(j);
            double scval = (val - min) / range;
            dual.push_back(scval);
        }
        scPFront.push_back(dual);
    }
    //cout_scaled_pareto();
}

void PaCcET::nad_ut(){
    /// 1) calculate utopia from PFront
    /// 2) calculate nadir from PFront
    
    // 1)
    utopia.clear();
    double min;
    double mindex;
    
    for(int o=0; o<OBJECTIVES; o++){
        min=99999999999;
        mindex=-1;
        for(int p=0; p<PFront.size(); p++){
            if(PFront.at(p).at(o) < min){
                min=PFront.at(p).at(o);
                mindex=p;
            }
        }
        utopia.push_back(PFront.at(mindex).at(o)-0.001);
    }
    /// Added constant avoids atan2 problems at the edges of the PFront.
    
    // 2)
    nadir.clear();
    double max;
    double maxdex;
    for(int o=0; o<OBJECTIVES; o++){
        max=-999999999999;
        maxdex=-1;
        for(int p=0; p<PFront.size(); p++){
            if(PFront.at(p).at(o)>max){
                max=PFront.at(p).at(o);
                maxdex=p;
            }
        }
        nadir.push_back(PFront.at(maxdex).at(o));
    }
    //cout << "Nadir: " << nadir.at(0) << "\t" << nadir.at(1) << endl;
    //cout << "Utopia: " << utopia.at(0) << "\t" << utopia.at(1) << endl;
}

void PaCcET::N_Dummy_transform() {
    output.clear();
    for(int i=0; i<input.size(); i++){
        output.push_back(input.at(i));
    }
}

void PaCcET::N_Pro_transform() {
    /// For notation, See Yliniemi and Tumer, SEAL 2014.
    
    vector<double> deltas;
    int I=input.size();
    
    for(int i=0; i<I; i++){
        deltas.push_back(input.at(i));
        //cout << "DELTAS: " << i << " " << deltas.at(i) << endl;
    }
    
    vector<double> directional_ratios;
    
    double sumdeltassq=0.0;
    for(int i=0; i<I; i++){
        sumdeltassq+=deltas.at(i)*deltas.at(i);
    }
    sumdeltassq=sqrt(sumdeltassq);
    for(int i=0; i<I; i++){
        /// More truly: directional cosines.
        directional_ratios.push_back(deltas.at(i)/sumdeltassq);
    }
    
    /// <FIND v_1>
    double v_1;
    v_1 = calc_v_1(deltas);
    if(PaCcET_VERBOSE > 0){
        cout << "v_1: " << v_1 << endl;
    }
    
    /// <FIND v_B>
    double v_B;
    v_B = calc_v_B(directional_ratios);
    if(PaCcET_VERBOSE > 0){
        cout << "v_B: " << v_B << endl;
    }
    
    /// <FIND v_hp>
    double v_hp;
    v_hp = calc_v_hp(directional_ratios);
    if(PaCcET_VERBOSE > 0){
        cout << "v_hp: " << v_hp << endl;
    }
    
    /// <CALCULATE dtau>
    double dtau = v_1*v_hp/v_B;
    if(PaCcET_VERBOSE > 0){
        cout << "dtau: " << dtau << endl;
    }
    
    output.clear();
    for(int i=0; i<I; i++){
        output.push_back(dtau*directional_ratios.at(i));
    }
    
    
}

double PaCcET::calc_v_1(vector<double> deltas){
    int I=input.size();
    
    double v_1=0;
    for(int i=0; i<I; i++){
        v_1+=deltas.at(i)*deltas.at(i);
    }
    v_1=sqrt(v_1);
    return v_1;
}

void PaCcET::eliminate_not_dominating(list<vector<double> >& scPFront_temp, vector<double> td){
    
    if(scPFront_temp.size() ==1)
    {
        //cout << "SHORTCUT!" << endl;
        /// only one is dominating, no need to reduce any more.
        return;
    }
    
    bool dominated;
    
    //cout << "ELIMINATE IN: " << scPFront_temp.size() << endl;
    //for(int i=0; i< td.size(); i++){
    //cout << td.at(i) << "\t";
    //}
    //cout << endl;
    //for (std::list< vector<double> >::iterator it=scPFront_temp.begin(); it != scPFront_temp.end(); ++it){
    //vector<double> aaa = *it;
    //for(int j=0; j<aaa.size(); j++){
    //cout << "XXX " << j << " " << aaa.at(j) << endl;
    //}
    //}
    
    for (std::list< vector<double> >::iterator it=scPFront_temp.begin(); it != scPFront_temp.end(); ++it){
        dominated = does_v1_dominate_v2(*it,td);
        if(dominated==true){
            // nothing happens
            //cout << "STILL ALIVE" << endl;
        }
        if(dominated==false){
            // we don't need to use this pfront point for further calc_v_B calculations on this iteration:
            scPFront_temp.erase(it);
            //cout << "ELIMINATED!" << endl;
        }
    }
    
    //cout << "ELIMINATE OUT: " <<scPFront_temp.size() << endl;
    //for(int p=0; p<scPFront_temp.size(); p++){
    //    dominated = does_v1_dominate_v2(scPFront_temp.at(p),td);
    //    if(dominated==true){
    // nothing happens
    //   }
    //   if(dominated==false){
    // we don't need to use this pfront point for further calc_v_B calculations on this iteration:
    //        scPFront_temp.erase(scPFront_temp.begin()+p);
    //   }
    //}
}

double PaCcET::calc_v_B(vector<double> directional_ratios){
    static int iii;
    iii++;
    
    /// normalize deltas:
    /*
     double sumdirr=0.0;
     for( int i=0; i< directional_ratios.size(); i++){
     sumdirr += directional_ratios.at(i);
     }
     for( int i=0; i< directional_ratios.size(); i++){
     directional_ratios.at(i)/=sumdirr;
     }
     */
    
    //return 0.1; 6-> 18
    int I=input.size();
    /// <FIND v_B>
    double v_B;
    /// find distance along utopia->input vector which is first dominated.
    double lowerbound = 0;
    double upperbound = 0;
    for(int i=0; i<I; i++){
        upperbound+=1;
    }
    upperbound = 2*sqrt(upperbound)+0.1;
    double candidate;
    double margin=upperbound-lowerbound;
    bool dominated;
    vector<double> td;
    td.resize(I);
    //vector< vector<double> > scPFront_temp = scPFront;
    list< vector<double> > scPFront_temp;
    std::copy( scPFront.begin(), scPFront.end(), std::back_inserter( scPFront_temp ) );
    
    static int jjj;
    
    //return 0.1; 0->6
    while(margin>0.01){
        jjj++;
        dominated=false;
        candidate=(upperbound+lowerbound)/2;
        for(int i=0; i<I; i++){
            td.at(i) = candidate*directional_ratios.at(i);
        }
        
        for (std::list< vector<double> >::iterator it=scPFront_temp.begin(); it != scPFront_temp.end(); ++it){
            dominated = does_v1_dominate_v2(*it,td);
            if(dominated==true){break;}
        }
        
        //for(int p=0; p<scPFront_temp.size(); p++){
        //dominated = does_v1_dominate_v2(scPFront_temp.at(p),td);
        //if(dominated==true){break;} // once we know it is dominated, we don't need to continue calculating.
        //}
        
        if(dominated==true){upperbound = candidate;}
        //if(dominated==true){upperbound = candidate; eliminate_not_dominating(scPFront_temp,td);}
        if(dominated==false){lowerbound = candidate;}
        margin=upperbound-lowerbound;
        //cout << "SCPFRONT TEMP SIZE: " << scPFront_temp.size() << endl;
    }
    v_B=(upperbound+lowerbound)/2;
    
    //cout << "INSIDE CALC_D: " << iii << " , " << jjj << endl;
    return v_B;
    
}

double PaCcET::calc_v_hp(vector<double> directional_ratios){
    int I=input.size();
    /// <FIND v_hp>
    double v_hp;
    
    vector<double> td;
    td.resize(I);
    
    double lowerbound = 0;
    double upperbound = 0;
    for(int i=0; i<I; i++){
        upperbound+=1;
    }
    upperbound = 2*sqrt(upperbound)+0.1;
    double margin=upperbound-lowerbound;
    while(margin>0.0001){
        bool dominated=false;
        double candidate=(upperbound+lowerbound)/2;
        for(int i=0; i<I; i++){
            td.at(i) = candidate*directional_ratios.at(i);
        }
        if(accumulate(td.begin(),td.end(),0.0) <= 1){dominated = true;}
        if(dominated==true){lowerbound = candidate;}
        if(dominated==false){upperbound = candidate;}
        margin=upperbound-lowerbound;
    }
    v_hp = (upperbound+lowerbound)/2;
    return v_hp;
}

double PaCcET::calc_dtau(double v_1, double v_B, double v_hp){
    double dtau = v_1*v_hp/v_B;
    return dtau;
}

void PaCcET::take_input(vector<double>* pcoords){
    /// Assign input to class variable.
    
    if(pcoords->size() != OBJECTIVES){cout << "Are we doing a " << OBJECTIVES << " objective problem or not?" << endl;}
    assert(pcoords->size() == OBJECTIVES);
    input.clear();
    for(int obj=0; obj<OBJECTIVES; obj++){
        input.push_back(pcoords->at(obj));
    }
}

void PaCcET::give_output(vector<double>* pcoords){
    /// Assign output to external variable.
    for(int obj=0; obj<OBJECTIVES; obj++){
        pcoords->at(obj) = output.at(obj);
    }
}

void PaCcET::scale(){
    /// Scale values of objectives to be less than one, with the nadir point taking on (1,1).
    if(PaCcET_VERBOSE > 0){
        cout << "SCALING BEFORE!\t";
        cout << input.at(0) << "," << input.at(1) << endl;
    }
    for(int obj=0; obj<OBJECTIVES; obj++){
        double val = input.at(obj);
        double range = nadir.at(obj)-utopia.at(obj);
        double scval = (val - utopia.at(obj)) / range;
        input.at(obj) = scval;
    }
    if(PaCcET_VERBOSE > 0){
        cout << "SCALING AFTER!\t";
        cout << input.at(0) << "," << input.at(1) << endl;
    }
}

void PaCcET::execute_N_transform(vector<double>* pinputs){
    take_input(pinputs);
    scale();
    N_Pro_transform();
    give_output(pinputs);
}

void Pro_Pareto_Filter_Testing(){
    PaCcET T;
    vector<double> coords;
    for(int i=0; i<1000; i++){
        cout << "Trial " << i << endl;
        coords.push_back(rand()%100000);
        coords.push_back(rand()%100000);
        cout << "coords 0\t" << coords.at(0) << endl;
        cout << "coords 1\t" << coords.at(1) << endl;
        T.Pareto_Check(coords);
        coords.clear();
        T.cout_pareto();
    }
    cout << "Placement" << endl;
    coords.push_back(-100000);
    coords.push_back(-100000);
    T.Pareto_Check(coords);
    T.cout_pareto();
    coords.clear();
}

void Procedural_Testing(){
    PaCcET T;
    vector<double> coords;
    vector<double>* pcoords = &coords;
    for(int i=0; i<100; i++){
        cout << "Trial " << i << endl;
        coords.push_back(rand()%1000);
        coords.push_back(rand()%1000);
        cout << "coords 0:\t" << coords.at(0) << endl;
        cout << "coords 1:\t" << coords.at(1) << endl;
        T.Pareto_Check(coords);
        T.execute_N_transform(pcoords);
        cout << "coords after 0\t" << coords.at(0) << endl;
        cout << "coords after 1\t" << coords.at(1) << endl;
        coords.clear();
        T.cout_pareto();
        T.cout_scaled_pareto();
    }
    cout << "Placement" << endl;
    coords.push_back(-1);
    coords.push_back(-1);
    T.Pareto_Check(coords);
    T.cout_pareto();
    T.cout_scaled_pareto();
    coords.clear();
}
