/**

\file
\version {1.17}
*/
#include "include/mastersclient.h"
#include "friLBRState.h"
#include <cstdio>
#include <cmath>
#include <filesystem>
#include <future>
#include <iostream>
#include <chrono>

using namespace KUKA::FRI;

//******************************************************************************
mastersclient::mastersclient(unsigned int jointMask, double freqHz,
                             double amplRad, double filterCoeff,
                             plotMaster plotter)
        : _jointMask(jointMask), _freqHz(freqHz), _amplRad(amplRad),
          _filterCoeff(filterCoeff), _offset(0.0), _phi(0.0), _stepWidth(0.0),
          plotter(plotter) {
    printf(
            "mastersclient initialized:\n"
            "\tjoint mask: 0x%x\n"
            "\tfrequency (Hz): %f\n"
            "\tamplitude (rad): %f\n"
            "\tfilterCoeff: %f\n",
            jointMask, freqHz, amplRad, filterCoeff
          );
    //jointPosition = new std::vector<double>;
    //jointPosition.resize(LBRState::NUMBER_OF_JOINTS,0.0);
    //std::cout << "\nSize : " << jointPos.size();
    jointPosition.resize(LBRState::NUMBER_OF_JOINTS, 0.0);
    //oldJointPos.resize(LBRState::NUMBER_OF_JOINTS, 0.0);
    jointvel.resize(LBRState::NUMBER_OF_JOINTS, 1);
    jointvel.setZero();

    //
    std::vector<std::string> xmlpath{//
            "/home/mirko/CLionProjects/thesis2024_orphaned/masters/descr/rlmdl/kuka-lbr-iiwa-7-r800.xml",
            "/usr/share/rl-0.7.0/examples/rlmdl/mitsubishi-rv6sl.xml"
    };
    // Check if the file exists
    for (const auto &i: xmlpath) {
        if (!std::filesystem::exists(i)) {
            std::cerr << "Error: File does not exist - " << i << std::endl;
            // You might want to handle this error in an appropriate way for your application
            // For example, throw an exception or return from the constructor
            // Alternatively, you can set some default values for the model
            throw std::invalid_argument("File does not exist");
        }
    }
    robotmdl = new robotModel(xmlpath[0]);

    robotmdl->getUnitsFromModel();
    this->dQ_JointP_history = std::deque<d_vecJointPosition>(
            5,
            d_vecJointPosition(
                    jointPosition.size(), 0.0
                              ));


}

//******************************************************************************
//mastersclient::~mastersclient() = default;
mastersclient::~mastersclient() {
    delete s_eSessionstate;
}

/*!
 * @brief
 @param oldState
 @param newState
*/

void mastersclient::onStateChange(ESessionState oldState,
                                  ESessionState newState) {
    LBRClient::onStateChange(oldState, newState);
    switch (newState) {
        case IDLE: {
            this->s_eSessionstate = "IDLE";
            break;
        }
        case MONITORING_READY: {
            _offset = 0.0;
            _phi = 0.0;
            _stepWidth = 2 * M_PI * _freqHz * robotState().getSampleTime();
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

/*
 * One Sided differencing for numerical differentiation
 */
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


/*
 * Multi Sided differencing for numerical differentiation
 */
rl::math::Vector mastersclient::calculateJointVelocityMultiSided(
        const std::deque<d_vecJointPosition> &cJointHistory, double dt) {


    std::lock_guard<std::mutex> lock(multiSidedJointVelMutex);


    if (!cJointHistory.empty()) {
        size_t size = cJointHistory.front().size();
        rl::math::Vector derivativeVector(size);
        derivativeVector.setZero();

        for (size_t j = 0; j < size; ++j) {
            derivativeVector[j] =
                    ((-1 * cJointHistory[0][j]) + (8 * cJointHistory[1][j]) -
                     (8 * cJointHistory[3][j]) + (cJointHistory[4][j])) / (12 * dt);

            //  derivativeVector(j) =
            //        std::round(derivativeVector(j) * ROUND_AFTER_COMMA) /
            //      ROUND_AFTER_COMMA;
        }
        /* std::cout << "MultiSided: Derivative Vector: "
                   << derivativeVector.transpose()
                   << std::endl;
 */
        return derivativeVector;
    } else {
        throw std::runtime_error(
                "cJointHistory is empty. Unable to perform calculations."
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

    // Check if the error between deltaTime and desiredDeltaTime is larger than 500 microseconds
    const auto error = std::chrono::duration_cast<std::chrono::microseconds>(
            deltaTime - desiredDeltaTime
                                                                            );
    if (std::abs(error.count()) > 500) {
        // If the error is too large, log the error and throw an exception
        std::cerr << "Error: " << error.count() << " microseconds." << std::endl;
        throw std::runtime_error(
                "DeltaTime error exceeds 500 microseconds. Handle the exception accordingly."
                                );
    }

}

void mastersclient::monitor() {

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
    // Uncomment the line below if you want to set joint positions in robotCommand
    // robotCommand().setJointPosition(jointPos);
    // robotCommand().setJointPosition()
}
/*!
 * @brief
 * @param jointVel
 */
void mastersclient::doProcessJointData(const rl::math::Vector &jointVel) {
    //std::cout << "jointVel: " << jointVel.transpose() << std::endl;
    // Set joint positions and velocities in the robot model
    robotmdl->setQ(this->jointPosition, jointVel);

    // Perform forward kinematics
    robotmdl->performForwardKinematics();

    // Get transformation matrix at joint 0
    robotmdl->getTransformation(0);

}
/*!
 * @brief
 */
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

        // Multi Sided:
        if (dQ_JointP_history.size() < 6) {

        }
            // Start the asynchronous computation only if dQ_JointP_history size is sufficient
        else {
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
