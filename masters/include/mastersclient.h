/**
\file
\version {1.17}
*/
#ifndef MASTERSCLIENT_H
#define MASTERSCLIENT_H

#include "friLBRClient.h"
#include "robotModel.h"
#include <string>
#include <memory>
#include <vector>

/**
 * \brief Test client that can overlay interpolator joint positions with sine waves.
 */
class mastersclient : public KUKA::FRI::LBRClient {

public:
    std::vector<double> jointTest, oldJointPos, jointvel;
    double * jointPos;

    //KUKA::FRI::ESessionState eSessionState;
    const char *s_eSessionstate{};

    /**
     * \brief Constructor.
     *
     * @param jointMask Bitmask that encodes the joint indices to be overlayed by sine waves
     * @param freqHz Sine frequency in hertz
     * @param amplRad Sine amplitude in radians
     * @param filterCoeff Filter coefficient between 0 (filter off) and 1 (max filter)
     */
    mastersclient(unsigned int jointMask, double freqHz,
                  double amplRad, double filterCoeff);

    /**
    * \brief Destructor.
    */
    ~mastersclient();

    /**
     * \brief Callback for FRI state changes.
     *
     * @param oldState
     * @param newState
     */
    virtual void waitForCommand();

    virtual void onStateChange(KUKA::FRI::ESessionState oldState, KUKA::FRI::ESessionState newState);

    /**
     * \brief Callback for the FRI session states 'Monitoring Wait' and 'Monitoring Ready'.
     *
     * If you do not want to change the default-behavior, you do not have to implement this method.
     */
    virtual void monitor();

    /**
     * \brief Callback for the FRI state 'Commanding Active'.
     */
    void command() override;

private:

    int _jointMask;         //!< Bitmask encoding of overlay joints
    double _freqHz;         //!< sine frequency (Hertz)
    double _amplRad;        //!< sine amplitude (radians)
    double _filterCoeff;    //!< filter coefficient
    double _offset;         //!< offset for current interpolation step
    double _phi;            //!< phase of sine wave
    double _stepWidth;      //!< stepwidth for sine
    unsigned int currentSampleTimeSec;
    unsigned int currentSampleTimeNanoSec;
    unsigned int prvSampleTimeSec;
    unsigned int prvSampleTimeNanoSec;
    unsigned int deltaTimeSec{};
    unsigned int deltaTimeNanoSec{};
    double deltaTime{};
    robotModel * robotmdl;


    void printJointPos() const;

    void transformation() const;

    void getCurrentTimestamp();
    //std::vector<double> calcJointVel(const std::vector<double> &inputVector, double dt);
    rl::math::Vector calcJointVel(double dt);

    void calcRobot();
};

#endif //MASTERSCLIENT_H
