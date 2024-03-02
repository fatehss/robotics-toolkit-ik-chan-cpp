/**
 * \file structs.h
 * \author Jesse Haviland
 *
 */
/* structs.h */

#ifndef STRUCTS_H
#define STRUCTS_H

// #ifdef __cplusplus
#include <Eigen/Dense>
// #endif /* __cplusplus */

#include "linalg.h"

struct ETS
{
        /**********************************************************
         *************** kinematic parameters *********************
                **********************************************************/
        ET **ets;
        int n;
        int m;

        // While this information is stored in the ET's
        // Its much faster for IK to cache it here
        double *qlim_l;
        double *qlim_h;
        double *q_range2;
};

// dummy function
static void noop(double* /*data*/, double /*eta*/) {}
double arr[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
class ET
{
public:
        ET(int isstaticsym, int isjoint, int isflip, int jindex,
         int axis, double *T, double *glim, 
        void (*op)(double *data, double eta) = noop, 
        MapMatrix4dc Tm = Eigen::Map<Matrix4dc>(arr));

        int isstaticsym; /* this ET is static and has a symbolic value */
        int isjoint;
        int isflip;
        int jindex;
        int axis;
        double *T;    /* link static transform */
        double *qlim; /* joint limits */
        void (*op)(double *data, double eta);

        // #ifdef __cplusplus
        // Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> Tm;
        MapMatrix4dc Tm;
        // #endif /* __cplusplus */

};



#endif