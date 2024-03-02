#include "new_methods.h"


revoluteDH::revoluteDH(double _d , 
            double _a,
            double _alpha, 
            double _offset, 
            double* _qlim, 
            bool _flip): d(_d), a(_a), alpha(_alpha), offset(_offset), flip(_flip), et(0,0, 0, 0, 0, nullptr, _qlim)
            {
                /* NOTE: The ET et described in the initializer list above may be wrong
                
                in this function, the goal is to modify the ET member variable in a way similar to
                https://petercorke.github.io/robotics-toolbox-python/_modules/roboticstoolbox/robot/DHLink.html#RevoluteDH
                and 
                https://petercorke.github.io/robotics-toolbox-python/_modules/roboticstoolbox/robot/DHLink.html#DHLink

                As such, the parameters in the latter's _to_ets function will be sigma=0, theta =0, mdh = false
                
                Consequently we have isrevolute = false 
                 */

            if (offset != 0)  {ets *= revoluteDH::tz(offset);}

            ets*=revoluteDH::tz(flip=flip);

            if (a != 0) {ets*=revoluteDH::tx(a);}

            if (alpha != 0) { ets *= ET.Rx(alpha);}
            }