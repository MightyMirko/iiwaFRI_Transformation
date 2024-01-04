/**

\file
\version {1.17}
*/
#include "include/mastersclient.h"
#include "friLBRState.h"
#include <cstdio>
#include <cstring>
// Visual studio needs extra define to use math constants
#include <cmath>
#include <filesystem>
#include <iostream>


using namespace KUKA::FRI;

//******************************************************************************
mastersclient::mastersclient(unsigned int jointMask, double freqHz,
                             double amplRad, double filterCoeff)
        : _jointMask(jointMask), _freqHz(freqHz), _amplRad(amplRad),
          _filterCoeff(filterCoeff), _offset(0.0), _phi(0.0), _stepWidth(0.0) {
    printf("mastersclient initialized:\n"
           "\tjoint mask: 0x%x\n"
           "\tfrequency (Hz): %f\n"
           "\tamplitude (rad): %f\n"
           "\tfilterCoeff: %f\n",
           jointMask, freqHz, amplRad, filterCoeff);
    //jointTest = new std::vector<double>;
    //jointTest.resize(LBRState::NUMBER_OF_JOINTS,0.0);
    //std::cout << "\nSize : " << jointPos.size();
    jointTest.resize(LBRState::NUMBER_OF_JOINTS, 0.0);
    oldJointPos.resize(LBRState::NUMBER_OF_JOINTS, 0.0);
    jointvel.resize(LBRState::NUMBER_OF_JOINTS, 0.0);

    jointPos = new double[LBRState::NUMBER_OF_JOINTS];
    std::cout << "%\n" << jointPos << "%\n" << "\n With * " << *jointPos << std::endl;
    for (int i = 0; i < LBRState::NUMBER_OF_JOINTS; i++) {

        std::cout << i << "before:\t" << jointPos[i] << std::endl;
        jointPos[i] = 0.0;
        std::cout << "\t after:\t" << jointPos[i] << "%\n" << std::endl;
    }

    currentSampleTimeSec = 0;
    currentSampleTimeNanoSec = 0;
    prvSampleTimeSec = 0;
    prvSampleTimeNanoSec = 0;
    //
    std::vector<std::string> xmlpath{//
            "/home/mirko/OneDrive/codespace/fri_cmake/masters/iiwa_dh_model.xml",
            "/home/mirko/OneDrive/codespace/fri_cmake/masters/iiwa_my.xml",
            "/home/mirko/OneDrive/codespace/fri_cmake/masters/rl-examples/rlmdl/kuka-lbr-iiwa-7-r800.xml",
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
    robotmdl = new robotModel(xmlpath[2]);

}

//******************************************************************************
//mastersclient::~mastersclient() = default;
mastersclient::~mastersclient() {
    delete jointPos, s_eSessionstate;
}

//******************************************************************************
void mastersclient::onStateChange(ESessionState oldState,
                                  ESessionState newState) {
    LBRClient::onStateChange(oldState, newState);
    // (re)initialize sine parameters when entering Monitoring
    switch (newState) {
        case IDLE: {
            this->s_eSessionstate = "IDLE";
            break;
        }
        case MONITORING_READY: {
            _offset = 0.0;
            _phi = 0.0;
            _stepWidth = 2 * M_PI * _freqHz * robotState().getSampleTime();
            //this->printJointPos();
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
rl::math::Vector mastersclient::calcJointVel(double dt) {
    size_t size = jointTest.size();
    rl::math::Vector derivativeVector(size);
    derivativeVector.setZero();

    // Central differencing for numerical differentiation
    for (size_t i = 1; i < size - 1; ++i) {
        derivativeVector(i) = (jointTest[i] - oldJointPos[i]) / (dt);
        //derivativeVector(i) = std::round(derivativeVector(i) * 10000.0) / 10000.0;
    }

    return derivativeVector;
}


void mastersclient::calcRobot() {
    try {
        getCurrentTimestamp();
        const double *measuredJointPosPtr = robotState().getMeasuredJointPosition();
        // KOpiere von vorne nach hinten in jointpos
        oldJointPos = jointTest;
        std::copy(measuredJointPosPtr, measuredJointPosPtr + LBRState::NUMBER_OF_JOINTS, jointTest.begin());

        // Calculate the derivative of the joint positions
        rl::math::Vector jointVel = calcJointVel(_stepWidth);  // Assuming dt is 5ms
        //std::cout << "Joint Velocities: " << jointVel << std::endl;
        robotmdl->setQ(jointTest, jointVel);
        robotmdl->performForwardKinematics();
        robotmdl->getTransformation();
        robotmdl->getTCPvelocity();

    } catch (const std::runtime_error &e) {
        printf("Not connected yet;\n");
    } catch (const std::exception &e) {
        std::cerr << "Exception caught: " << e.what() << std::endl;
    }
}

void mastersclient::getCurrentTimestamp() {// Get seconds since 0:00, January 1st, 1970 (UTC)
    this->currentSampleTimeSec = robotState().getTimestampSec();
    // Get nanoseconds elapsed since the last second (in Unix time)
    this->currentSampleTimeNanoSec = robotState().getTimestampNanoSec();

    //std::cout << "Timestamp (sec):\traPosition" << currentSampleTimeSec << std::endl;
    //std::cout << "Timestamp (nanoSec):\traPosition" << currentSampleTimeNanoSec << std::endl;

    // Calculate the time difference
    this->deltaTimeSec = currentSampleTimeSec - prvSampleTimeSec;
    this->deltaTimeNanoSec = currentSampleTimeNanoSec - prvSampleTimeNanoSec;
    this->deltaTime = deltaTimeSec + (deltaTimeNanoSec) / 1e9;

    // Update previous timestamps
    prvSampleTimeSec = currentSampleTimeSec;
    prvSampleTimeNanoSec = currentSampleTimeNanoSec;
    /*std::cout << "Current delta in sec:\traPosition" << deltaTimeSec << std::endl;
    std::cout << "Current delta in nanosec:\traPosition" << deltaTimeNanoSec << std::endl;
    std::cout << "Current delta :\traPosition" << deltaTime << std::endl;
*/
}

void mastersclient::printJointPos() const {
    for (int i = 0; i < LBRState::NUMBER_OF_JOINTS; i++) {
        printf(" J%d: %f\t", i, jointPos[i] * rl::math::RAD2DEG);
    }
    printf("\n");
}

void mastersclient::monitor() {
    LBRClient::monitor();
    calcRobot();
}


void mastersclient::waitForCommand() {}

//******************************************************************************
void mastersclient::command() {
    // Uncomment the line below if you want to set joint positions in robotCommand
    // robotCommand().setJointPosition(jointPos);
    // robotCommand().setJointPosition()
}

//******************************************************************************
// clean up additional defines
#ifdef _USE_MATH_DEFINES
#undef _USE_MATH_DEFINES
#endif
