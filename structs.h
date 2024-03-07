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


// dummy function
/**
 * @struct ET
 * @brief Represents an elementary transformation (ET) with kinematic and geometric parameters.
 *
 * This structure encapsulates both static and dynamic properties of an elementary transformation
 * used in robotics kinematics, including joint limits and operations applied to data.
 */
class ET
{
public:
    int isstaticsym; /**< Flag indicating if this ET is static and has a symbolic value. */
    int isjoint;     /**< Flag indicating if this element represents a joint. */
    int isflip;      /**< Flag indicating if the joint axis is flipped. */
    int jindex;      /**< Index of the joint in a kinematic chain. */
    int axis;        /**< Axis of the joint transformation. */
    double offset;   /**< The eta offset of the link.*/
    double *T;       /**< Pointer to the link's static transformation matrix. */
    double *qlim;    /**< Pointer to the joint limits array. */
    
    
    MapMatrix4dc Tm; /**< Eigen::Map wrapping an Eigen::Matrix for the transformation, providing easy matrix operations. */
    
    void op(double *data, double eta); /**< Pointer to a function that operates on data with parameter eta. */
};

/**
 * @struct ETS
 * @brief Represents a kinematic chain of Elementary Transformations (ETs) and caches kinematic parameters for efficiency.
 * 
 * This structure is used to store and manipulate a sequence of ETs that define the kinematics of a robotic mechanism or system.
 * It also caches important kinematic parameters for improved computational efficiency, particularly in inverse kinematics.
 */
struct ETS
{
    ET **ets;        /**< Pointer to an array of pointers to ET objects, representing the kinematic chain. */
    int n;           /**< Number of ETs in the kinematic chain. */
    int m;           /**< Additional parameter for internal use, potentially representing the number of movable joints. */

    double *qlim_l;  /**< Pointer to an array of lower joint limits for the kinematic chain. */
    double *qlim_h;  /**< Pointer to an array of upper joint limits for the kinematic chain. */
    double *q_range2;/**< Cached range values for the joints, used for computational efficiency in algorithms like inverse kinematics. */
};


#endif