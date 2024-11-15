/**
* \author Mirko Matošin
* \file mastersclient.cpp
* \version {0.1}
* \brief The code is executed on this Turtle.
*/
#include "include/mastersclient.h"
#include "friLBRState.h"
#include <cmath>
#include <filesystem>
#include <future>
#include <iostream>
#include <chrono>

using namespace KUKA::FRI;
//ctor
mastersclient::mastersclient(bool doPlot, double plotCount)
        : doPlot(doPlot), plotter(plotCount) {
    // init vars
    jointPosition.resize(LBRState::NUMBER_OF_JOINTS, 0.0);
    jointvel.resize(LBRState::NUMBER_OF_JOINTS, 1);
    jointvel.setZero();
    this->dQ_JointP_history =
            std::deque<d_vecJointPosition>(
                    5,
                    d_vecJointPosition(jointPosition.size(), 0.0));

    std::vector<std::string> xmlpath{//
            "descr/rlmdl/kuka-lbr-iiwa-7-r800.xml",
            "/usr//local/share/rl-0.7.0/examples/rlmdl/mitsubishi-rv6sl.xml"
    };
    // Check if the file exists
    for (const auto &i: xmlpath) {
        if (!std::filesystem::exists(i)) {
            std::cerr << "Error: File does not exist - " << i << std::endl;
            throw std::invalid_argument("File does not exist");
        }
    }
    robotmdl = new robotModel(xmlpath[0]);
    robotmdl->getUnitsFromModel();
}

//dtor
mastersclient::~mastersclient() {
    delete s_eSessionstate;
}


void mastersclient::onStateChange(ESessionState oldState,
                                  ESessionState newState) {
    LBRClient::onStateChange(oldState, newState);
    switch (newState) {
        case IDLE: {
            this->s_eSessionstate = "IDLE";
            break;
        }
        case MONITORING_READY: {
            this->s_eSessionstate = "MONITORING_READY";
            break;
        }
        case COMMANDING_WAIT: {
            this->s_eSessionstate = "COMMANDING_WAIT";
            break;
        }
        case COMMANDING_ACTIVE: {
            this->s_eSessionstate = "COMMANDING_ACTIVE";
            break;
        }
        default: {
            break;
        }
    }
}

rl::math::Vector
mastersclient::calculateJointVelocityOneSided(const std::vector<double> &oldJointPos,
                                              const std::vector<double> &currJointPos,
                                              double dt) {
    if (oldJointPos.size() != currJointPos.size()) {
        throw std::runtime_error(
                "oldJointPos and currJointPos have different sizes."
                                );
    }

    std::lock_guard<std::mutex> lock(oneSidedJointVelMutex);
    size_t size = currJointPos.size();
    rl::math::Vector derivativeVector(size);
    derivativeVector.setZero();

    // Central differencing for numerical differentiation
    for (size_t i = 0; i < size - 1; ++i) {
        derivativeVector(i) = (currJointPos[i] - oldJointPos[i]) / (dt);
        //derivativeVector(i) = std::round(derivativeVector(i) * ROUND_AFTER_COMMA) /
        //    ROUND_AFTER_COMMA;
    }
    //std::cout << "One Sided: : " << derivativeVector.transpose() << std::endl;

    return derivativeVector;
}


rl::math::Vector //
mastersclient::calculateJointVelocityMultiSided(
        const std::deque<d_vecJointPosition> &cJointHistory,
        double dt) {

    std::lock_guard<std::mutex> lock(multiSidedJointVelMutex);
    if (!cJointHistory.empty() && cJointHistory.size() >= 5) {
        size_t size = cJointHistory.front().size();
        rl::math::Vector derivativeVector(size);
        derivativeVector.setZero();

        for (size_t j = 0; j < size; ++j) {
            derivativeVector[j] =
                    ((-1 * cJointHistory[0][j]) + (8 * cJointHistory[1][j]) -
                     (8 * cJointHistory[3][j]) + (cJointHistory[4][j])) / (12 * dt);
        }
        return derivativeVector;
    } else {
        throw std::runtime_error(
                "cJointHistory is empty or its size is less than 5. Unable to perform calculations."
                                );
    }
}

std::mutex mastersclient::oneSidedJointVelMutex;

bool
mastersclient::compareVectors(const rl::math::Vector &v1, const rl::math::Vector &v2,
                              double tolerance, bool verbosity) {
    if (v1.size() != v2.size()) {
        std::cerr << "Vectors differ in size." << std::endl;
        return false; // Vectors must have the same size
    }
    // Calculate the error vector
    rl::math::Vector errorVector = v1 - v2;
    for (size_t i = 0; i < errorVector.size(); ++i)
        errorVector(i) =
                std::round(errorVector(i) * ROUND_AFTER_COMMA) / ROUND_AFTER_COMMA;
    for (std::size_t i = 0; i < v1.size(); ++i) {
        if (std::abs(v1(i) - v2(i)) > tolerance && verbosity) {
            std::cerr << "Vectors differ at index " << i << "." << std::endl;
            std::cout << (v1.matrix() - v2.matrix()).transpose() << std::endl;
            std::cout << errorVector.transpose() << std::endl;
            //return false; // Elements differ by more than the tolerance
        }
    }
    if (verbosity) std::cout << "Vectors are approximately equal." << std::endl;
    // Calculate the norm of the error vector
    double errorNorm = errorVector.norm();

    // Print or handle the error
    if (errorNorm > 1.0) {
        std::cerr << "Error Norm: " << errorNorm << std::endl;

    }

    return true; // Vectors are approximately equal
}

void
mastersclient::getCurrentTimestamp() {// Get seconds since 0:00, January 1st, 1970 (UTC)
    auto desiredDeltaTime = std::chrono::duration<double>(5e-3);
    unsigned int currentSampleTimeSec = robotState().getTimestampSec();
    unsigned int currentSampleTimeNanoSec = robotState().getTimestampNanoSec();
    // Convert seconds and nanoseconds to std::chrono::time_point
    currentSampleTime = std::chrono::time_point<std::chrono::seconds, std::chrono::nanoseconds>(
            std::chrono::seconds(currentSampleTimeSec) +
            std::chrono::nanoseconds(currentSampleTimeNanoSec));

    deltaTime = currentSampleTime - prvSampleTime;
    prvSampleTime = currentSampleTime;
    std::cout <<"Tatsächliche Abtastrate [s]: \t "<< deltaTime.count() <<std::endl;
    // Check if the error between deltaTime and desiredDeltaTime is larger than 500 microseconds
    //const auto error = std::chrono::duration_cast<std::chrono::microseconds>(
    //        deltaTime - desiredDeltaTime);
    if (std::abs(deltaTime.count()) > desiredDeltaTime.count()) {
        // If the error is too large, log the error and throw an exception
        std::cerr << "Error: " << deltaTime.count() << " microseconds." << std::endl;
     //   throw std::runtime_error(
       //         "DeltaTime error exceeds 500 microseconds. Handle the exception accordingly."
        //                        );
    }

}

void mastersclient::monitor() {
    //std::cout << "mon: "<< std::endl;
    // Start time measurement
    getCurrentTimestamp();
    auto startTime = std::chrono::high_resolution_clock::now();
    LBRClient::monitor();
    doPositionAndVelocity();
    // todo Interface preparation for network with Sick Laserscanner? Get EFI Data via Profinet and UDP? or even ros2
    rl::math::Vector3 humanPos;  // Assuming 3 elements for XYZ

    // Set the XYZ values relative to the robot's world frame
    humanPos(0) = -0.8;  // X coordinate
    humanPos(1) = 1.5;  // Y coordinate
    humanPos(2) = .8;  // Z coordinate

    robotmdl->cartesianRobotDistanceToObject(humanPos);
    // Plot velocity histories
    if (doPlot)
        this->plotVelocityHistories();
    // End time measurement
    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                    endTime - startTime
                                                                );
    // Print the duration
    /*
    if (duration.count() > 5e5) {
        std::cout << "Time taken by threads: " << duration.count()
                  << " nanoseconds." << std::endl;
    }*/
}


void mastersclient::waitForCommand() {}

//******************************************************************************
void mastersclient::command() {

}


void mastersclient::doProcessJointData(const rl::math::Vector &jointVel) {
    //std::cout << "jointVel: " << jointVel.transpose() << std::endl;
    // Set joint positions and velocities in the robot model
    robotmdl->setQ(this->jointPosition, jointVel);

    // Perform forward kinematics
    robotmdl->performForwardKinematics();

    // Get transformation matrix at joint 0
    robotmdl->getTransformation(1);

}

void mastersclient::plotVelocityHistories() {
    // Check if the size of the velocity history is sufficient for plotting
    if (plotter.getHistSize()) {
        // Plot the velocity histories using plotter object
        if (plotter.plotLiveFirstAxis("r-", "b-"))
            plotter.clearData();
    }
}

std::mutex mastersclient::historyMutex;
std::mutex mastersclient::multiSidedJointVelMutex;

void mastersclient::doPositionAndVelocity() {
    try {
        rl::math::Vector multiSided_jointVel;
        multiSided_jointVel.setZero();
        const double *measuredJointPosPtr = robotState().getMeasuredJointPosition();
        const std::vector<double> oldJointPos = jointPosition;

        std::copy(
                measuredJointPosPtr, measuredJointPosPtr +
                                     LBRState::NUMBER_OF_JOINTS,
                jointPosition.begin());

        {
            std::lock_guard<std::mutex> lock(historyMutex);
            dQ_JointP_history.push_back(jointPosition);
        }
        // One Sided:
        auto oneSidedResult = std::async(
                std::launch::async, [&]() {
                    return calculateJointVelocityOneSided(
                            oldJointPos, jointPosition, deltaTime.count());
                }
                                        );

        auto multiSidedResult = std::async(
                std::launch::async,
                [&]() {
                    auto result =
                            calculateJointVelocityMultiSided(
                                    dQ_JointP_history,
                                    deltaTime.count()
                                                            );

                    // Sende
                    return result;
                }
                                          );

        multiSided_jointVel = multiSidedResult.get() * -1;

        // Process the joint data asynchronously

        auto processJointDataResult = std::async(
                std::launch::async, [&]() {
                    doProcessJointData(multiSided_jointVel);
                }
                                                );

        // Remove the oldest joint position from history
        {
            std::lock_guard<std::mutex> lock(historyMutex);
            dQ_JointP_history.pop_front();
        }
        // Wait for the asynchronous computations to finish
        rl::math::Vector oneSided_jointVel = oneSidedResult.get();

        if (doPlot) {
            plotter.addVelocityData(oneSided_jointVel, multiSided_jointVel);
        }

        processJointDataResult.wait();
    }

    catch (const std::runtime_error &e) {
        std::cerr << "Runtime Error in mastersclient->robotCalc() caught: "
                  << e.what() << std::endl;
    }
    catch (const std::exception &e) {
        std::cerr << "Exception caught in mastersclient->robotCalc(): " << e.what()
                  << std::endl;
    }
}
