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
#include <iomanip> // Include this for std::setw

class robotModel {
public:
    explicit robotModel(const std::string &xmlFilePath);

    void performForwardKinematics();

    void getTransformation(bool printFlag = false);

    void getTCPvelocity(bool printFlag = false /*= false*/);

    void setQ(const std::vector<double> &jointPositions,
              const rl::math::Vector &jointVelocities);

    void getUnitsFromModel() const;

    void printTransform(rl::math::Transform &t, bool addNewLine) const;

    ::rl::math::Real cartesianRobotDistanceToObject(
            rl::math::Vector3& objectPosition) const;
private:
    void printQ();

    void update_model();

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
