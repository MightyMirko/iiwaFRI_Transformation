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
  this->printJointPos();

  /***************************************************************************/
  /*                                                                         */
  /*   Place user Client Code here                                           */
  /*                                                                         */
  /***************************************************************************/

}
void mastersclient::printJointPos() {
  double jointPos[LBRState::NUMBER_OF_JOINTS];
  memcpy(jointPos, robotState().getMeasuredJointPosition(),
         LBRState::NUMBER_OF_JOINTS * sizeof(double));

  for (int i = 0; i < LBRState::NUMBER_OF_JOINTS; i++) {
    printf(" J%d: %f\n", i, jointPos[i]);
  }
}
void mastersclient::waitForCommand() {}

//******************************************************************************
void mastersclient::command() {
  double jointPos[LBRState::NUMBER_OF_JOINTS];
  memcpy(jointPos, robotState().getMeasuredJointPosition(),
         LBRState::NUMBER_OF_JOINTS * sizeof(double));

  for (int i = 0; i < LBRState::NUMBER_OF_JOINTS; i++) {
    printf(" J%d: %f\n", i, jointPos[i]);
  }

  // Uncomment the line below if you want to set joint positions in robotCommand
  // robotCommand().setJointPosition(jointPos);
  // robotCommand().setJointPosition()
}

//******************************************************************************
// clean up additional defines
#ifdef _USE_MATH_DEFINES
#undef _USE_MATH_DEFINES
#endif
