//
// Created by mirko on 02.01.24.
//

#include "robotModel.h"
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
    }
    catch (const std::exception &e) {
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
        std::cerr << "Dynamic cast to rl::mdl::Kinematic or Dynamic failed."
                  << std::endl;
    }

    lbr.q.resize(model->getDof());
    //std::printf("DoF: %i",sizeof(lbr_q));
    std::cout << "\n\n\n Using File: " << xmlFilePath << std::endl;

    lbr.q = model->getPosition();
    // lbr.qd = q.der
    lbr.qd = model->getVelocity();
    lbr.qdd = model->getAcceleration();
    lbr.tau = model->getTorque();
    lbr.home = model->getHomePosition();
    //std::cout<< " Home: " << lbr.home.matrix()<<std::endl;

}


void robotModel::setQ(const std::vector<double> &jointPositions,
                      const rl::math::Vector &jointVelocities) {
    // Ensure that the sizes of input vectors match the sizes of robot model vectors
    if (jointPositions.size() != lbr.q.size() ||
        jointVelocities.size() != lbr.qd.size()) {
        throw std::invalid_argument(
                "Invalid jointPositions or jointVelocities size"
                                   );
    }
    // Set current joint positions in the robot model
    for (size_t i = 0; i < jointPositions.size(); ++i) {
        lbr.q(i) = jointPositions[i];
    }
    lbr.qd = jointVelocities;
}



void robotModel::performForwardKinematics() {
    kinematics->setPosition(this->lbr.q);
    kinematics->setVelocity(this->lbr.qd);

    kinematics->forwardPosition();
    kinematics->forwardVelocity();

    kinematics->calculateJacobian();
}

void robotModel::getTransformation(bool printFlag /*= false*/) {
    // Endeffektor Geschwindgkeitsvekor
//dynamic->setPosition(q); // input value
//dynamic->calculateMassMatrix(); // modify model
//const rl::math::Matrix& m = dynamic->getMassMatrix(); // output value
    tcp.matrix_position = kinematics->getOperationalPosition(0); // output value
    tcp.xd = kinematics->getOperationalVelocity(0);
    const rl::math::Matrix& j = kinematics->getJacobian(); // output value
    //tcp.matrix_position.translate(); Setze vektor auf tcp basis um?

    tcp.posi = tcp.matrix_position.translation();
    tcp.x = tcp.posi.row(0)(0);
    tcp.y = tcp.posi.row(1)(0);
    tcp.z = tcp.posi.row(2)(0);

    //std::cout << tcp.posi.matrix()<< std::endl;
    tcp.orient = tcp.matrix_position.rotation().eulerAngles(2,1,0);
    tcp.a = tcp.orient(0) * rl::math::RAD2DEG;
    tcp.b = tcp.orient(1) * rl::math::RAD2DEG;
    tcp.c = tcp.orient(2) * rl::math::RAD2DEG;

    tcp.vecV = tcp.matrix_position.linear() * tcp.xd.linear();
    tcp.vecOmega = tcp.matrix_position.linear() * tcp.xd.angular();


    //std::cout<< "\t"<< tcp.matrix_position.matrix()<<std::endl;
    //std::cout<< "\t"<< tcp.vecV.matrix()<<std::endl;
    if (printFlag) {
        // Set the width for better formatting
        const int width = 20;

        //std::cout << *tcp.matrix_position.data() << std::endl; // pointer auf erste Position in Matrix
        std::cout << "Matrix()\n" << tcp.matrix_position.matrix() << std::endl; // komplette Matrix
        std::cout << "linear()\n" << tcp.matrix_position.linear() << std::endl; // Matrix linear Anteil (3x3
        std::cout << "translation()\n" << tcp.matrix_position.translation().transpose()  <<std::endl; // komplette Matrix
        std::cout << "Jacobi()\n" << j.matrix() << std::endl; // komplette Matrix

        //std::cout << *tcp.matrix_position.data() << std::endl; // pointer auf erste Position in Matrix
        //std::cout << tcp.xd.matrix() << "\n Matrix()" << std::endl; // komplette Matrix
        //std::cout << tcp.xd.linear().transpose() << "\n linear()" <<std::endl; // Matrix linear Anteil (3x3
        //std::cout << tcp.xd.angular().transpose() << "\n translation()" <<std::endl; // komplette Matrix


        std::cout
                << "Aktueller Vektor zum TCP in WeltKoordinaten xyz \n"
                << std::setw(width) << tcp.x << std::setw(width)
                << tcp.y << std::setw(width) << tcp.z
                << std::endl;
        std::cout
                << "Aktueller Vektor zum TCP radial in WeltKoordinaten abc \n"
                << std::setw(width) << tcp.a << std::setw(width)
                << tcp.b << std::setw(width) << tcp.c
                << std::endl;

        std::cout
                << "Aktuelle Geschwindigkeitsvektor TCP linear in WeltKoordinaten xyz \n"
                << std::setw(width) << tcp.vecV.x() << std::setw(width)
                << tcp.vecV.y() << std::setw(width) << tcp.vecV.z()
                << std::endl;

        std::cout
                << "Aktuelle Geschwindigkeitsvektor TCP radial in WeltKoordinaten xyz \n"
                << std::setw(width) << tcp.vecOmega.x() << std::setw(width)
                << tcp.vecOmega.y() << std::setw(width) << tcp.vecOmega.z()
                << std::endl;
    }
}

void robotModel::getUnitsFromModel() const {
    auto q_units = model->getPositionUnits();
    auto qd_units = model->getVelocityUnits();
    auto qdd_units = model->getAccelerationUnits();
}

void robotModel::getTCPvelocity(bool printFlag /*= false*/) {
    // Convert values to degrees
    tcp.vecV = rl::math::RAD2DEG * tcp.vecV;
    tcp.vecOmega = rl::math::RAD2DEG * tcp.vecOmega;

    if (printFlag) {
        // Set the width for better formatting
        const int width = 25;

        // Print linear velocity
        std::cout << "[Vx, Vy, Vz] = [" << std::setw(width) << tcp.vecV.coeff(0)
                  << std::setw(width) << tcp.vecV.coeff(1) << std::setw(width)
                  << tcp.vecV.coeff(2) << "]" << std::endl;

        // Print angular velocity
        std::cout << "[Omega_x, Omega_y, Omega_z] = [" << std::setw(width)
                  << tcp.vecOmega.coeff(0) << std::setw(width)
                  << tcp.vecOmega.coeff(1) << std::setw(width)
                  << tcp.vecOmega.coeff(2) << "]" << std::endl;
    }
}


void robotModel::printQ() {
    std::cout << "\nq: " << lbr.q.transpose() << std::endl;
    std::cout << "\nqd: " << lbr.qd.transpose() << std::endl;
}

void robotModel::printVector() {
    std::cout << "x: " << tcp.posi.x() << " m\ty: " << tcp.posi.y() << " m\tz: "
              << tcp.posi.z() << " m" << std::endl;
    std::cout << "a: " << tcp.orient.x() * rl::math::RAD2DEG << " deg\tb: "
              << tcp.orient.y() * rl::math::RAD2DEG
              << " deg\tc: " << tcp.orient.z() * rl::math::RAD2DEG << " deg"
              << std::endl;
    std::cout << "Joint configuration in degrees: "
              << lbr.q.transpose() * rl::math::RAD2DEG << std::endl;
    std::cout << "End-effector tcp_kuka_posi: [m] " << tcp.posi.transpose()
              << " tcp_kuka_orient [deg] "
              << tcp.orient.transpose() * rl::math::RAD2DEG << std::endl;
}

void robotModel::update_model() {

}

// src: https://github.com/sjentzsch/mopl/blob/3347103df78cf60103fff1b62f225c9110554b1d/src/DamaModel.cpp
void robotModel::printTransform(rl::math::Transform &t, bool addNewLine) const {
    const rl::math::Transform::TranslationPart position = t.translation();
    const rl::math::Vector3 orientation = t.rotation().eulerAngles(2, 1, 0);

    ::std::cout << position.x() << " | " << position.y() << " | " << position.z()
                << " || " << (orientation.x() * rl::math::RAD2DEG) << " | "
                << (orientation.y() * rl::math::RAD2DEG) << " | "
                << (orientation.z() * rl::math::RAD2DEG);

    if (addNewLine)
        std::cout << std::endl;
}


::rl::math::Real
robotModel::cartesianRobotDistanceToObject(rl::math::Vector3 &objectPosition) const {
   // std::cout << "Current Joints Position: " << model->getPosition().transpose()<< std::endl;

    // Calculate the Euclidean distance between the current and target TCP positions
    rl::math::Vector3 distanceVector =
            objectPosition - tcp.posi;

    rl::math::Real distance = distanceVector.norm();

    // Debug print: Display the current TCP position
   /* std::cout << "Current TCP Position: "
              << currentTcpPosition.translation().transpose() << std::endl;

    // Debug print: Display the distance vector components
    std::cout << "Distance Vector (X, Y, Z): " << distanceVector.transpose()
              << std::endl;
*/
    // Debug print: Display the calculated distance
    if (distance<= 0.8)
    {
    std::cerr << "Calculated Distance: " << distance << std::endl;
    }
    else
    {
        std::cout << "Calculated Distance: " << distance << std::endl;
    }


    return distance;
}
