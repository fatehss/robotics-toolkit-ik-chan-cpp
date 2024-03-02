/**
 * \file new_methods.h
 * \author Fateh Sandhu
 *
 */
/* new_methods.h */

#ifndef NEW_METHODS_H
#define NEW_METHODS_H

// #ifdef __cplusplus
// #endif /* __cplusplus */
#include <vector>
#include "linalg.h"
#include "structs.h"

class revoluteDH
{
    private:
        double d,a,offset;
        bool flip;

        void Rx();
        void Rz();
        void tx();
        void tz();
        ET et;

    public:
        revoluteDH(double d = 0, 
            double a = 0, 
            double offset = 0, 
            double* qlim = nullptr, 
            bool flip = false);
        

};

#endif;