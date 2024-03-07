/* structs.cpp */

#include "structs.h"
#include <iostream>


void ET::op(double* ret, double eta)
{
        // based on the context, this is likely copying the transformation matrix computed with eta into data

        //first need to add (informally) sin(eta) cos(eta) to the matrix
        //T[0] *= sin(eta);

        // Have already filled out sin|cos(alpha) in the matrix, now adapting for sin|cos(eta)
        eta += offset;
        double* newArr = new double[16]; //copy array values into new array
        for (int i = 0; i < 16; ++i) {
                newArr[i] = T[i];
        }

        MapMatrix4dc Tm2 = MapMatrix4dc(newArr); 
        Tm2(0,0) *= cos(eta);
        Tm2(0,1) *= sin(eta);
        Tm2(0,2) *= sin(eta);
        Tm2(0,3) *= cos(eta);
        Tm2(1,0) *= sin(eta);
        Tm2(1,1) *= cos(eta);
        Tm2(1,2) *= cos(eta);
        Tm2(1,3) *= sin(eta);


        for (int i =0; i<16; ++i){
                ret[i] = newArr[i];
        }
  
        delete[] newArr;
 
}




/*
// Function to compute the transformation matrix for a joint, given its DH parameters and theta
Eigen::Matrix4d computeDHMatrix(double theta, double d, double a, double alpha) {
    Eigen::Matrix4d T;
    T << cos(theta), -sin(theta) * cos(alpha),  sin(theta) * sin(alpha), a * cos(theta),
         sin(theta),  cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta),
         0,            sin(alpha),              cos(alpha),             d,
         0,            0,                       0,                      1;
    return T;
}


*/
