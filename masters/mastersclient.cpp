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

#include "iostream"


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
    jointPos = new double[LBRState::NUMBER_OF_JOINTS];
    for (int i = 0; i < LBRState::NUMBER_OF_JOINTS; i++) { jointPos[i] = 0.0; }
    currentSampleTimeSec = 0;
    currentSampleTimeNanoSec = 0;
    prvSampleTimeSec = 0;
    prvSampleTimeNanoSec = 0;
    std::string xmlpath = "/home/mirko/OneDrive/codespace/fri_cmake/masters/exa_iiwa_dh_model.xml";
    robotmdl = new robotModel(xmlpath);
}

//******************************************************************************
mastersclient::~mastersclient() {}

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
            this->printJointPos();
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

void mastersclient::monitor() {
    // In waitForCommand(), the joint values have to be mirrored. Which is done,
    // by calling the base method.
    LBRClient::monitor();
//this->printJointPos();
    //double performance = robotState().getTrackingPerformance(); // bringt nichts im monitor modus - immer 0
    try {
        memcpy(jointPos, robotState().getMeasuredJointPosition(),
               LBRState::NUMBER_OF_JOINTS * sizeof(double));
        this->robotmdl->setQ(jointPos);
        this->robotmdl->performForwardKinematics();
        // get TimeStamp and compare
        this->getCurrentTimestamp();
        robotmdl->getTCPvelocity();


    } catch (const std::runtime_error &e) {
        std::printf("Not connected yet;\n");
    } catch (const std::exception &e) {
        std::cerr << "Exception caught: " << e.what() << std::endl;
    }
    /***************************************************************************/
    /*                                                                         */
    /*   Place user Client Code here                                           */
    /*                                                                         */
    /***************************************************************************/

}

void mastersclient::getCurrentTimestamp() {// Get seconds since 0:00, January 1st, 1970 (UTC)
    this->currentSampleTimeSec = robotState().getTimestampSec();
    // Get nanoseconds elapsed since the last second (in Unix time)
    this->currentSampleTimeNanoSec = robotState().getTimestampNanoSec();

    std::cout << "Timestamp (sec):\t" << currentSampleTimeSec << std::endl;
    std::cout << "Timestamp (nanoSec):\t" << currentSampleTimeNanoSec << std::endl;

    // Calculate the time difference
    this->deltaTimeSec = currentSampleTimeSec - prvSampleTimeSec;
    this->deltaTimeNanoSec = currentSampleTimeNanoSec - prvSampleTimeNanoSec;
    this->deltaTime = deltaTimeSec + (deltaTimeNanoSec) / 1e9;

    // Update previous timestamps
    prvSampleTimeSec = currentSampleTimeSec;
    prvSampleTimeNanoSec = currentSampleTimeNanoSec;
    std::cout << "Current delta in sec:\t" << deltaTimeSec << std::endl;
    std::cout << "Current delta in nanosec:\t" << deltaTimeNanoSec << std::endl;
    std::cout << "Current delta :\t" << deltaTime << std::endl;

}


void mastersclient::printJointPos() {
    for (int i = 0; i < LBRState::NUMBER_OF_JOINTS; i++) {
        printf(" J%d: %f\n", i, jointPos[i]);
    }
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
