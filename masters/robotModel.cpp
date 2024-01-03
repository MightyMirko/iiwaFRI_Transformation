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
    //std::printf("DoF: %i",sizeof(q));
}

void robotModel::performForwardKinematics() {
//    q *= rl::math::DEG2RAD;

    kinematics->setPosition(q);
    kinematics->forwardPosition();
    rl::math::Transform t = kinematics->getOperationalPosition(0);
    rl::math::Vector3 position = t.translation();
    //std::cout << "Was steckt in T " <<position<< std::endl;
    rl::math::Vector3 orientation = t.rotation().eulerAngles(2, 1, 0).reverse();
    printVector(position, orientation);
}

void robotModel::printVector(rl::math::Vector3 &position, rl::math::Vector3 &orientation) {
    std::cout << "x: " << position.x() << " m\ty: " << position.y() << " m\tz: " << position.z() << " m" << std::endl;
    std::cout << "a: " << orientation.x() * rl::math::RAD2DEG << " deg\tb: " << orientation.y() * rl::math::RAD2DEG
              << " deg\tc: " << orientation.z() * rl::math::RAD2DEG << " deg" << std::endl;
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

    rl::math::Vector qd = model->getVelocity();
    auto *kinematic = dynamic_cast<rl::mdl::Kinematic *>(model.get());
    auto *dynamic = dynamic_cast<rl::mdl::Dynamic *>(model.get());
    rl::math::Transform toolTransform = model->tool(0);
    // Get the TCP position
    rl::math::Vector3 position = toolTransform.translation();
    // Print the TCP position
    std::cout << "Tool Position: " << position.transpose() << std::endl;
    const rl::math::Transform &x = kinematic->getOperationalPosition(0); // output value
    kinematic->setPosition(q); // input value
    kinematic->setVelocity(qd); // input value
    kinematic->forwardVelocity(); // modify model
    const rl::math::MotionVector &v = kinematic->getOperationalVelocity(0); // output value
    kinematic->calculateJacobian(); // modify model
    const rl::math::Matrix& j = kinematic->getJacobian(); // output value
    dynamic->forwardDynamics();
}

