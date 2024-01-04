//
// Created by mirko on 02.01.24.
//

#ifndef FRICLIENT_ROBOTMODEL_H
#define FRICLIENT_ROBOTMODEL_H

#include <rl/mdl/XmlFactory.h>
#include <rl/mdl/Model.h>
#include <rl/math/Unit.h>
#include <rl/math/Transform.h>
#include <rl/mdl/Kinematic.h>
#include <rl/mdl/Dynamic.h>
#include "lbr_struct.h"
#include <stdexcept>
#include <boost/lexical_cast.hpp>
#include <rl/math/Unit.h>
#include <rl/mdl/Body.h>
#include <rl/mdl/Dynamic.h>
#include <rl/mdl/Fixed.h>
#include <rl/mdl/Frame.h>
#include <rl/mdl/Revolute.h>

class robotModel {
public:
    explicit robotModel(const std::string &xmlFilePath);

    void performForwardKinematics();
    void getTransformation();
    void getTCPvelocity();
    void printQ();

    void update_model();
    void setQ(std::vector<double> &q, rl::math::Vector &qd);

private:
    // Model and casts
    std::shared_ptr<rl::mdl::Model> model;
    rl::mdl::Kinematic *kinematics;
    rl::mdl::Dynamic *dynamic;

    TCPParameters tcp;

    rl::math::Transform traPosition;
    rl::math::MotionVector traVelocity;
    LBR_Rob lbr;

    // Methods
    void printVector();
};


#endif //FRICLIENT_ROBOTMODEL_H
