/* structs.cpp */

#include "structs.h"
// #include <Eigen/Dense>
double arr[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
ET::ET(int _isstaticsym, int _isjoint, int _isflip, int _jindex,
         int _axis, double *_T, double *_glim, 
        void (*_op)(double *data, double eta),
        MapMatrix4dc _Tm ): 
        isstaticsym(_isstaticsym), isjoint(_isjoint), isflip(_isflip), jindex(_jindex), op(op), Tm(_Tm)
        { }

