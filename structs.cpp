/* structs.cpp */
/*

// #include <Eigen/Dense>
double arr[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
ET::ET(int _isstaticsym, int _isjoint, int _isflip, int _jindex,
         int _axis, double *_T, double *_glim, 
        void (*_op)(double *data, double eta),
        MapMatrix4dc _Tm ): 
        isstaticsym(_isstaticsym), isjoint(_isjoint), isflip(_isflip), jindex(_jindex), op(op), Tm(_Tm)
        { }


*/
#include "structs.h"
#include <iostream>


void ET::op(double* data, double eta)
{

        //first need to add (informally) sin(eta) cos(eta) to the matrix
        //T[0] *= sin(eta);

        // TODO: need to create new matrix here as there is repeat modification
        eta -= offset;
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

        MapMatrix4dc other_matrix = MapMatrix4dc(data);


        for (int i =0; i<16; ++i){
                data[i] = newArr[i];
        }
  
        delete[] newArr;
 
}




