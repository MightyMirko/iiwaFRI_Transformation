//
// Created by mirko on 02.01.24.
//

#include "robotModel.h"


robotModel::robotModel(const std::string &xmlFilePath) {
    rl::mdl::XmlFactory factory;
    std::shared_ptr<rl::mdl::Model> tempModel(factory.create(xmlFilePath));
    model = std::move(tempModel);  // Move ownership to the member variable
    kinematics = dynamic_cast<rl::mdl::Kinematic *>(model.get());
    q.resize(kinematics->getDof());
}

void robotModel::performForwardKinematics() {
//    q *= rl::math::DEG2RAD;

    kinematics->setPosition(q);
    kinematics->forwardPosition();

    rl::math::Transform t = kinematics->getOperationalPosition(0);
    rl::math::Vector3 position = t.translation();
    rl::math::Vector3 orientation = t.rotation().eulerAngles(2, 1, 0).reverse();


    std::cout << "Joint configuration in degrees: " << q.transpose() * rl::math::RAD2DEG << std::endl;
    std::cout << "End-effector position: [m] " << position.transpose() << " orientation [deg] "
              << orientation.transpose() * rl::math::RAD2DEG << std::endl;
}

void robotModel::setQ(double *jointPos) {
    q << jointPos[0],
            jointPos[1],
            jointPos[2],
            jointPos[3],
            jointPos[4],
            jointPos[5],
            jointPos[6];
}

void robotModel::getTransformation(double *cart) {

}

void robotModel::getTCPvelocity() {
    kinematics->forwardVelocity();
    kinematics->calculateJacobian(true);

    rl::math::Transform toolTransform = model->tool(0);
    // Get the TCP position
    rl::math::Vector3 position = toolTransform.translation();
    // Print the TCP position
    std::cout << "Tool Position: " << position.transpose() << std::endl;

    // Get the TCP velocity
    //rl::math::MotionVector velocity = kinematics->getVelocity();


    //rl::math::Vector3 linearVelocity = velocity.linear();
    // Print the TCP velocity
    //std::cout << "Tool Velocity: " << velocity.transpose() << std::endl;

    // Note: velocity contains both angular and linear velocities
    // If you want only linear velocity, you can extract it like this:
    // rl::math::Vector3 linearVelocity = velocity.top(3);
    //std::cout << "Linear Velocity: " << linearVelocity.transpose() << std::endl;
}

