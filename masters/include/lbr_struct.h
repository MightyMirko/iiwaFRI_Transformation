//
// Created by mirko on 04.01.24.
//

#ifndef FRICLIENT_LBR_STRUCT_H
#define FRICLIENT_LBR_STRUCT_H
#include <rl/math/Transform.h>


struct TCPParameters {
    rl::math::Vector3 orient; // in euler (kuka definition)
    rl::math::Vector3 posi; // in meter
    rl::math::Real a,b,c,x,y,z; // in degree and meter
    rl::math::Vector3 vecV, vecOmega;
    rl::math::Transform matrix_position; // getOperationalPosition (Fkine)
    rl::math::MotionVector xd;
};

//Gelenkdynamiken
struct LBR_Rob {
    rl::math::Vector q;
    rl::math::Vector qd;
    rl::math::Vector qdd;
    rl::math::Vector tau;
    rl::math::Vector home;
};
#endif //FRICLIENT_LBR_STRUCT_H
