//
// Created by mirko on 02.01.24.
//

#ifndef FRICLIENT_ROBOTMODEL_H
#define FRICLIENT_ROBOTMODEL_H

#include <rl/mdl/Model.h>
#include <rl/math/Transform.h>
#include <rl/math/Unit.h>
#include <rl/mdl/Kinematic.h>
#include <rl/mdl/Model.h>
#include <rl/mdl/XmlFactory.h>
#include <rl/math/Transform.h>
#include <rl/math/Unit.h>
#include <rl/mdl/Kinematic.h>
#include <rl/mdl/Model.h>
#include <rl/mdl/XmlFactory.h>
#include <cstdio>
#include <cstring>
#include <cmath>
#include "iostream"
#include <rl/hal/WeissWsg50.h>

class robotModel {
public:
    explicit robotModel(const std::string &xmlFilePath);

    void performForwardKinematics();

    void setQ(double *jointPos);

    void getTransformation(double cart[6]);

    void getTCPvelocity();


private:
    std::shared_ptr<rl::mdl::Model> model;
    rl::mdl::Kinematic *kinematics;
    rl::math::Vector q;


    double _tcpVelocity;
};


#endif //FRICLIENT_ROBOTMODEL_H
