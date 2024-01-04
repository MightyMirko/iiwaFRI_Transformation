//
// Created by mirko on 02.01.24.
//

#include "robotModel.h"
#include <cstdio>
#include <cstring>
#include <cmath>
#include <iostream>
#include <rl/math/Transform.h>
#include <rl/math/Unit.h>
#include <rl/mdl/Kinematic.h>
#include <rl/mdl/Model.h>
#include <rl/mdl/XmlFactory.h>

robotModel::robotModel(const std::string &xmlFilePath) {
    rl::mdl::XmlFactory factory;
    try {
        std::shared_ptr<rl::mdl::Model> tempModel(factory.create(xmlFilePath));
        model = std::move(tempModel);  // Move ownership to the member variable
        std::cout << "Model created successfully!" << std::endl;
    } catch (const std::exception &e) {
        std::cerr << "Exception caught: " << e.what() << std::endl;
        // Handle the exception as needed
    }

    kinematics = dynamic_cast<rl::mdl::Kinematic *>(model.get());
    //dynamic = dynamic_cast<rl::mdl::Dynamic *>(model.get());

    if (kinematics != nullptr) {//|| dynamic != nullptr) {
        // Dynamic cast was successful
        // You can use kinematics here
    } else {
        // Dynamic cast failed
        // Handle the failure or print an error message
        std::cerr << "Dynamic cast to rl::mdl::Kinematic or Dynamic failed." << std::endl;
    }

    lbr.q.resize(model->getDof());
    //std::printf("DoF: %i",sizeof(lbr_q));
    std::cout << "\n\n\n Using File: " << xmlFilePath << std::endl;

    lbr.q = model->getPosition();
    // lbr.qd = q.der
    lbr.qd = model->getVelocity();
    lbr.qdd = model->getAcceleration();
    lbr.tau = model->getTorque();
    //  this->printQ();

}


void robotModel::setQ(std::vector<double> &q, rl::math::Vector &qd) {
    // Setze akuelle FRI Werte in Kinematicberechnung
    assert(q.size() == lbr.q.size());
    assert(qd.size() == lbr.qd.size());
    lbr.q(q.data(), q.size());
    lbr.qd = qd;
}

void robotModel::performForwardKinematics() {
    kinematics->setPosition(this->lbr.q);
    kinematics->setVelocity(this->lbr.qd);

    kinematics->forwardPosition();
    kinematics->forwardVelocity();
}

void robotModel::getTransformation() {
    //auto xd = kinematics->getJacobian() * lbr.qd;

    rl::math::MotionVector xd = kinematics->getOperationalVelocity(0);
    kinematics->forwardPosition();
    tcp.vecV = kinematics->getOperationalPosition(0).linear() * xd.linear();
    tcp.vecOmega = kinematics->getOperationalPosition(0).linear() * xd.angular();
}

void robotModel::getTCPvelocity() {
    // Convert values to degrees
    tcp.vecV = rl::math::RAD2DEG * tcp.vecV;
    tcp.vecOmega = rl::math::RAD2DEG * tcp.vecOmega;
    // Print all values of vecV with labels
    std::cout << "Linear Velocity (vecV): [Vx, Vy, Vz] = [" << tcp.vecV.transpose() << "]" << std::endl;
    // Print all values of vecOmega with labels
    std::cout << "Angular Velocity (vecOmega): [Omega_x, Omega_y, Omega_z] = [" << tcp.vecOmega.transpose() << "]"
              << std::endl;
}


void robotModel::printQ() {
    std::cout << "\nq: " << lbr.q.transpose() << std::endl;
    std::cout << "\nqd: " << lbr.qd.transpose() << std::endl;

}

void robotModel::printVector() {
    std::cout << "x: " << tcp.posi.x() << " m\ty: " << tcp.posi.y() << " m\tz: " << tcp.posi.z() << " m" << std::endl;
    std::cout << "a: " << tcp.orient.x() * rl::math::RAD2DEG << " deg\tb: " << tcp.orient.y() * rl::math::RAD2DEG
              << " deg\tc: " << tcp.orient.z() * rl::math::RAD2DEG << " deg" << std::endl;
    std::cout << "Joint configuration in degrees: " << lbr.q.transpose() * rl::math::RAD2DEG << std::endl;
    std::cout << "End-effector tcp_kuka_posi: [m] " << tcp.posi.transpose() << " tcp_kuka_orient [deg] "
              << tcp.orient.transpose() * rl::math::RAD2DEG << std::endl;
}

void robotModel::update_model() {
    // std::cout << model->getPosition().transpose();
    /*lbr.q = model->getPosition();
    lbr.qd = model->getVelocity();
    lbr.qdd = model->getAcceleration();
    lbr.tau = model->getTorque();
*/
}
