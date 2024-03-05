#include "structs.h"
#include <iostream>
#include <vector>
#include  "ik.h"

int main() {

double T_matrices[6][16] = {
    // T_1
    {0.984807753012208, -0.000000000000000, -0.173648177666930, 0,
    0.173648177666930,  0.000000000000000,  0.984807753012208, 0,
    0,                 -1.000000000000000,  0.000000000000000, 0,
    0,                  0,                  0,                 1.000000000000000},
    // T_2
    {0.171010071662834, -0.969846310392954,  0.173648177666930, 0.121417150880612,
    0.030153689607046, -0.171010071662834, -0.984807753012208, 0.021409119621002,
    0.984807753012208,  0.173648177666930, -0.000000000000000, 0.699213504638668,
    0,                  0,                  0,                 1.000000000000000},
    // T_3
    {-0.171010071662834, -0.173648177666930, -0.969846310392954, 0.121417150880612,
    -0.030153689607046,  0.984807753012208, -0.171010071662834, 0.021409119621002,
    0.984807753012208,  0.000000000000000, -0.173648177666930, 0.699213504638668,
    0,                  0,                  0,                 1.000000000000000},
    // T_4
    {-0.198565734023778, -0.969846310392954,  0.141314484355892, 0.645134158492808,
    0.141314484355892, -0.171010071662834, -0.975082443643152, 0.113754558318933,
    0.969846310392954, -0.173648177666930,  0.171010071662834, 0.792983520578810,
    0,                  0,                  0,                 1.000000000000000},
    // T_5
    {-0.363961118765909, -0.141314484355892, -0.920631587844940, 0.666331331146192,
    0.109472012499662,  0.975082443643152, -0.192951047103075, -0.032507808227540,
    0.924958476098220, -0.171010071662834, -0.339422116079567, 0.818635031328235,
    0,                  0,                  0,                 1.000000000000000},
    // T_6
    {-0.382970734242007,  0.075966414791287,  0.920631587844940, 0.813632385201382,
    0.277130176061168, -0.941259134849768,  0.192951047103075, -0.001635640691048,
    0.881210691168942,  0.329029398208769,  0.339422116079567, 0.872942569900966,
    0,                  0,                  0,                 1.000000000000000}
};

    double d[6] = {0,0,0,-0.54,0.15,0.16};
    double a[6] = {0,.710,0,0,0,0};
    double alpha[6] = {-PI_2, PI, -PI_2, PI_2, -PI_2, PI};
    double qlims[6][2] = {
        {-1.0*PI, 1.0*PI},  // First joint limits
        {-1.0*PI, 1.0*PI},  // Second joint limits
        {-1.5*PI, 1.5*PI},  // Third joint limits
        {-1.0556*PI, 1.0556*PI},  // Fourth joint limits
        {-1.0*PI, 1.0*PI},  // Fifth joint limits
        {-1.0556*PI, 1.0556*PI}   // Sixth joint limits
    };
    int n = 6;
    int m = 6;

    double q_range2[n];

    for (int i = 0; i < n; ++i) {
        q_range2[i] = qlims[i][1] - qlims[i][0];
    }


    double qlim_l[6]={qlims[0][0],qlims[1][0],qlims[2][0],qlims[3][0],qlims[4][0],qlims[5][0] } ;
    double qlim_h[6]={qlims[0][1],qlims[1][1],qlims[2][1],qlims[3][1],qlims[4][1],qlims[5][1] } ;
    // Example data for initialization
    double exampleTransform[16] = {0}; // Assuming this is meant to represent a 4x4 matrix
    double exampleQlim[2] = {0, 1}; // Example joint limits

    // isstaticsym should be 0
    // isjoint should be 1
    // jindex should be i
    // axis 
    // ET joint0 = 
    ET et_arr[6] = {
        {0,1,0,0, 0, T_matrices[0], qlims[0], noop, MapMatrix4dc(T_matrices[0])},
        {0,1,0,1, 0, T_matrices[1], qlims[1], noop, MapMatrix4dc(T_matrices[1])},
        {0,1,0,2, 0, T_matrices[2], qlims[2], noop, MapMatrix4dc(T_matrices[2])},
        {0,1,0,3, 0, T_matrices[3], qlims[3], noop, MapMatrix4dc(T_matrices[3])},
        {0,1,0,4, 0, T_matrices[4], qlims[4], noop, MapMatrix4dc(T_matrices[4])},
        {0,1,0,5, 0, T_matrices[5], qlims[5], noop, MapMatrix4dc(T_matrices[5])},
    };
    ET** ets = new ET*[n];

    for (int i=0; i<n; ++i){
        ets[i] = new ET {0,1,0,i, 0, T_matrices[i], qlims[i], noop, MapMatrix4dc(T_matrices[i])};
    }

    ETS ets_instance = {ets, n, m, qlim_l, qlim_h, q_range2};



    // Alternatively, if you need dynamic memory or complex initialization, you might use `new` or smart pointers
    std::cout<<"gucci gang \n\n\n\n";
    
    
    for (int i = 0; i < n; ++i) {
    delete[] ets[i]; // Delete each row
    }
    delete[] ets; // Delete the array of pointers
    return 0;
    
}