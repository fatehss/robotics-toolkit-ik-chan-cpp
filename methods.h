/**
 * \file methods.h
 * \author Jesse Haviland
 *
 */
/* methods.h */

#ifndef _METHODS_H_
#define _METHODS_H_

// #include <Python.h>
#include "structs.h"
#include "linalg.h"

/**
 * Checks if the given joint configuration `q` is within the specified limits for all joints.
 * 
 * @param ets Pointer to the ETS structure containing information about the robotic manipulator, including joint limits.
 * @param q The current joint configuration as a map to Eigen::VectorXd.
 * @return int Returns 1 if all joint values are within their respective limits, 0 otherwise.
 */
int _check_lim(ETS *ets, MapVectorX q);

/**
 * Computes the angle-axis representation of the rotation difference between two transformation matrices.
 * 
 * @param Te The current end-effector transformation matrix.
 * @param Tep The desired end-effector transformation matrix.
 * @param e Output vector where the first three elements represent the translation error and the last three represent the rotation error in angle-axis form.
 */
void _angle_axis(MapMatrix4dc Te, Matrix4dc Tep, MapVectorX e);

/**
 * Calculates the Jacobian matrix of the robotic manipulator at a given joint configuration.
 * 
 * @param ets Pointer to the ETS structure containing information about the robotic manipulator.
 * @param q Pointer to an array containing the current joint configuration.
 * @param tool Pointer to an array representing the transformation matrix of the tool with respect to the end-effector. Can be NULL if not used.
 * @param eJ Reference to the output Jacobian matrix as a map to Eigen::MatrixXd.
 */
void _ETS_jacob0(ETS *ets, double *q, double *tool, MapMatrixJc &eJ);

/**
 * Computes the forward kinematics of the robotic manipulator and returns the end-effector transformation matrix.
 * 
 * @param ets Pointer to the ETS structure containing information about the robotic manipulator.
 * @param q Pointer to an array containing the current joint configuration.
 * @param base Pointer to an array representing the base transformation matrix. Can be NULL if the base is assumed to be the identity matrix.
 * @param tool Pointer to an array representing the tool transformation matrix with respect to the end-effector. Can be NULL if not used.
 * @param e_ret Output parameter as a map to Eigen::MatrixXd for the resulting end-effector transformation matrix.
 */
void _ETS_fkine(ETS *ets, double *q, double *base, double *tool, MapMatrix4dc &e_ret);

/**
 * Computes the transformation matrix of a single element (ET) based on its parameters and the joint value `eta`.
 * 
 * @param et Pointer to the ET structure representing the manipulator element.
 * @param ret Pointer to the output array where the resulting transformation matrix will be stored.
 * @param eta The current value of the joint variable associated with this ET.
 */
void _ET_T(ET *et, double *ret, double eta);



#endif