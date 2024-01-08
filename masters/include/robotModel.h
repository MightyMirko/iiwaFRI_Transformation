/**
 * @file robotModel.h
 *
 * @brief Definition of the robotModel class.
 *
 * This file contains the declaration of the robotModel class, which is responsible for managing the
 * robot model, performing forward kinematics, and providing information about the robot's transformation.
 */

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

/**
 * @class robotModel
 *
 * @brief Class representing the robot model and providing kinematic information.
 *
 * The robotModel class is responsible for managing the robot model, performing forward kinematics,
 * and providing information about the robot's transformation.
 */
class robotModel {
public:
    /**
     * @brief Constructor for the robotModel class.
     *
     * @param xmlFilePath The path to the XML file containing the robot model description.
     */
    explicit robotModel(const std::string &xmlFilePath);

    /**
     * @brief Updates the forward kinematics of the robot.
     *
     * This method sets the joint positions and velocities, computes the forward position and velocity kinematics,
     * and calculates the Jacobian.
     */
    void performForwardKinematics();

    /**
     * @brief Retrieves and prints the transformation of the end-effector.
     *
     * This method calculates and prints the linear and angular velocities of the end-effector based on the kinematic state.
     *
     * @param printFlag If true, the transformation information will be printed.
     */
    void getTransformation(bool printFlag = false);

    /**
     * @brief Converts and prints the linear and angular velocities of the end-effector.
     *
     * This method converts linear and angular velocities to degrees and prints the values.
     *
     * @param printFlag If true, the velocity information will be printed.
     */
    void getTCPvelocity(bool printFlag = false);

    /**
     * @brief Sets the joint positions and velocities for the robot model.
     *
     * @param jointPositions Vector containing joint positions.
     * @param jointVelocities Vector containing joint velocities.
     */
    void setQ(const std::vector<double> &jointPositions,
              const rl::math::Vector &jointVelocities);

    /**
     * @brief Prints the units of joint positions, velocities, and accelerations from the robot model.
     */
    void getUnitsFromModel() const;

    /**
     * @brief Calculates and returns the Euclidean distance between the robot's TCP and a given object position.
     *
     * @param objectPosition The position of the object in 3D space.
     * @return The calculated distance between the robot's TCP and the object position.
     */
    ::rl::math::Real cartesianRobotDistanceToObject(
            rl::math::Vector3& objectPosition) const;

private:
    /**
     * @brief Prints the joint positions.
     */
    void printQ();

    /**
     * @brief Updates the robot model.
     */
    void update_model();

    // Model and casts
    std::shared_ptr<rl::mdl::Model> model; ///< Shared pointer to the robot model.
    rl::mdl::Kinematic *kinematics; ///< Pointer to the kinematics of the robot model.
    rl::mdl::Dynamic *dynamic; ///< Pointer to the dynamic model of the robot (commented-out).

    TCPParameters tcp; ///< Structure to hold TCP parameters.

    rl::math::Transform traPosition; ///< Transformation for position.
    rl::math::MotionVector traVelocity; ///< Motion vector for velocity.
    LBR_Rob lbr; ///< Structure to hold LBR robot parameters.

    // Methods
    /**
     * @brief Prints a vector.
     */
    void printVector();
};

#endif //FRICLIENT_ROBOTMODEL_H
