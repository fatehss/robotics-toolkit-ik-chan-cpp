/**
 * \file ik.h
 * \author Jesse Haviland
 *
 */
/* ik.h */

#ifndef _IK_H_
#define _IK_H_

// #include <Python.h>
#include "structs.h"
#include "linalg.h"



/**
 * @brief Solves the inverse kinematics problem using a Levenberg-Marquardt approach with additional features.
 *
 * @param ets Pointer to a structure containing robot kinematic model information.
 * @param Tep Target end-effector pose (4x4 transformation matrix).
 * @param q0 Initial joint configuration (if size matches, otherwise randomize).
 * @param ilimit Maximum iterations per local search.
 * @param slimit Maximum number of global search attempts.
 * @param tol Error tolerance for solution convergence.
 * @param reject_jl Flag: if true, reject solutions violating joint limits.
 * @param q Output: calculated joint configuration.
 * @param it Output: total iterations performed.
 * @param search Output: number of global searches performed.
 * @param solution Output: flag indicating if a solution was found (1 = yes, 0 = no).
 * @param E Output: final error value.
 * @param lambda Levenberg-Marquardt damping factor.
 * @param we Weighting vector for the 6 degrees of freedom (default is identity).
 */
void _IK_LM_Chan(
    ETS *ets,            
    Matrix4dc Tep,      
    MapVectorX q0,      
    int ilimit,          
    int slimit,         
    double tol,         
    int reject_jl,     
    MapVectorX q,       
    int *it,            
    int *search,         
    int *solution,      
    double *E,          
    double lambda,      
    MapVectorX we       
); 



void _pseudo_inverse(Eigen::Map<Eigen::MatrixXd> J, Eigen::Map<Eigen::MatrixXd> J_pinv, double damping);
void _rand_q(ETS *ets, MapVectorX q);
int _check_lim(ETS *ets, MapVectorX q);
void _angle_axis(MapMatrix4dc Te, Matrix4dc Tep, MapVectorX e);


#endif