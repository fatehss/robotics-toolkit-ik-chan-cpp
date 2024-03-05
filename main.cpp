#include "structs.h"
#include <iostream>
#include <vector>
#include  "ik.h"
#include "linalg.h"
#include <cmath>

int main() {

/*
    // T_1
    {1, 0, 0, 0,
     0, 0, 1, 0, 
     0, -1, 0, 0,
      0, 0, 0, 1},
    //{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    // T_2
    {0, -1, 0, 0, -1, 0, 0, -0.71, 0, 0, -1, 0, 0, 0, 0, 1},
    // T_3
    {1, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1},
    // T_4
    {1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, -0.54, 0, 0, 0, 1},
    // T_5
    {1, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0, 0.15, 0, 0, 0, 1},
    // T_6
    {1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, -0.16, 0, 0, 0, 1}

*/
    int n = 6;
    int m = 6;
    double alpha[6] = {-PI_2, PI, -PI_2, PI_2, -PI_2, PI};
    double T_matrices[6][16];
    double d[6] = {0,0,0,-0.54,0.15,0.16};
    double a[6] = {0,.710,0,0,0,0};

    //Create dh matrix (without eta params) for each link
    for (int i=0; i<n; ++i)
    {
        T_matrices[i][0] = 1; T_matrices[i][1] = -cos(alpha[i]); T_matrices[i][2] = sin(alpha[i]); T_matrices[i][3] = a[i];

        T_matrices[i][4] = 1; T_matrices[i][5] = cos(alpha[i]); T_matrices[i][6] = -sin(alpha[i]); T_matrices[i][7] = a[i];

        T_matrices[i][8] = 0; T_matrices[i][9] = sin(alpha[i]); T_matrices[i][10] = cos(alpha[i]); T_matrices[i][11] = d[i];

        T_matrices[i][12] = 0; T_matrices[i][13] = 0; T_matrices[i][14] = 0; T_matrices[i][15] = 1;

    }


    double qlims[6][2] = {
        {-1.0*PI, 1.0*PI},  // First joint limits
        {-1.0*PI, 1.0*PI},  // Second joint limits
        {-1.5*PI, 1.5*PI},  // Third joint limits
        {-1.0556*PI, 1.0556*PI},  // Fourth joint limits
        {-1.0*PI, 1.0*PI},  // Fifth joint limits
        {-1.0556*PI, 1.0556*PI}   // Sixth joint limits
    };


    double q_range2[n];

    for (int i = 0; i < n; ++i) {
        q_range2[i] = qlims[i][1] - qlims[i][0];
    }


    double qlim_l[6]={qlims[0][0],qlims[1][0],qlims[2][0],qlims[3][0],qlims[4][0],qlims[5][0] } ;
    double qlim_h[6]={qlims[0][1],qlims[1][1],qlims[2][1],qlims[3][1],qlims[4][1],qlims[5][1] } ;
    // Example data for initialization
    // isstaticsym should be 0
    // isjoint should be 1
    // jindex should be i
    // axis =?

    /*
    ET et_arr[6] = {
        {0,1,0,0, 0, T_matrices[0], qlims[0], op, MapMatrix4dc(T_matrices[0])},
        {0,1,0,1, 0, T_matrices[1], qlims[1], op, MapMatrix4dc(T_matrices[1])},
        {0,1,0,2, 0, T_matrices[2], qlims[2], op, MapMatrix4dc(T_matrices[2])},
        {0,1,0,3, 0, T_matrices[3], qlims[3], op, MapMatrix4dc(T_matrices[3])},
        {0,1,0,4, 0, T_matrices[4], qlims[4], op, MapMatrix4dc(T_matrices[4])},
        {0,1,0,5, 0, T_matrices[5], qlims[5], op, MapMatrix4dc(T_matrices[5])},
    };
    */

    ET** ets = new ET*[n];

    for (int i=0; i<n; ++i){
        ets[i] = new ET {0,1,0,i, 0, T_matrices[i], qlims[i], noop, MapMatrix4dc(T_matrices[i])};
    }

    ETS *ets_instance = new ETS{ets, n, m, qlim_l, qlim_h, q_range2};


    
//______________________________________________________________________________________________________________________
//                  IK_LM_Chan stuff

/*

End positions are:

X = 0.65170  
Y = 1.55176    
Z = 5.46487   
'
The euler angles are:

W = 0.789028448
P = -0.077981311
R = 0.213506127

This is the rotation matrix:

[  0.9743240, -0.2112438, -0.0779023;
   0.0952528,  0.7002522, -0.7075124;
   0.2040089,  0.6819260,  0.7023941 ]

So, the Tep should be

[   R   [xyz]T
    0      1   ]

*/
    //Target end effector pose
    Eigen::Matrix4d Tep;
    Tep << 0.9743240, -0.2112438, -0.0779023, 0.65170,
        0.0952528,  0.7002522, -0.7075124, 1.55176,
        0.2040089,  0.6819260,  0.7023941, 5.46487,
        0,          0,          0,         1;

    // initial q0 guess
    double q_initial[6] = {
    66.805 * M_PI / 180.0, 
    5.220 * M_PI / 180.0, 
    -24.885 * M_PI / 180.0, 
    104.846 * M_PI / 180.0, 
    0.792 * M_PI / 180.0, 
    -3.758 * M_PI / 180.0
    };

    MapVectorX q0 = MapVectorX(q_initial, 6);
    int ilimit = 30;
    int slimit = 100;
    double tol = 0.000001;
    int reject_jl = 1;

    struct IKResults {
    Eigen::VectorXd q; // Assuming MapVectorX can be replaced by Eigen::VectorXd for C++
    int it;
    int search;
    int solution;
    double E;
};
    IKResults results = {Eigen::VectorXd::Zero(6), 0, 0, 0, 0.0};
    double q_temp[6] = {0,0,0,0,0,0};
    MapVectorX q = MapVectorX(q_temp, 6);
    int *it = new int(0);
    int *search = new int(0);
    int *solution = new int(0);
    double *E = new double(0);
    double weights[6] = {1,1,1,1,1,1};
    MapVectorX we = MapVectorX(weights,6);
    double lambda = 1.0;
    _IK_LM_Chan(ets_instance, Tep, q0, ilimit, slimit, tol, reject_jl,
    q, it, search, solution, E, lambda, we);

    std::cout << "Results:\n"
              << "Calculated Joint Configuration:\n " << q << "\n"
              << "Total Iterations: " << *it << "\n"
              << "Global Searches: " << *search << "\n"
              << "Solution Found: " << (*solution ? "Yes" : "No") << "\n"
              << "Final Error Value: " << *E << "\n"
              << "Damping Factor (Lambda): " << lambda << "\n"
              << "Weighting Vector: " << we << std::endl;
    delete ets_instance;
    for (int i = 0; i < n; ++i) {
    delete[] ets[i]; // Delete each row
    }
    delete[] ets; // Delete the array of pointers
    return 0;
    
}