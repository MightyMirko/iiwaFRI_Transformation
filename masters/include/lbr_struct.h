//
// Created by mirko on 04.01.24.
//

#ifndef FRICLIENT_LBR_STRUCT_H
#define FRICLIENT_LBR_STRUCT_H
#include <rl/math/Transform.h>


struct TCPParameters {
    rl::math::Vector3 orient;
    rl::math::Vector3 posi;
    rl::math::Real a;
    rl::math::Real b;
    rl::math::Real c;
    rl::math::Vector3 vecV, vecOmega;
};

//Gelenkdynamiken
struct LBR_Rob {
    rl::math::Vector q;
    rl::math::Vector qd;
    rl::math::Vector qdd;
    rl::math::Vector tau;
};
#endif //FRICLIENT_LBR_STRUCT_H
