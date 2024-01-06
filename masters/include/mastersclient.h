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
#include <queue>
#include "gnuplot-iostream.h"
#include "plotMaster.h"
#include <chrono>

#define ROUND_AFTER_COMMA 1e7

/**
 * \brief Test client that can overlay interpolator joint positions with sine waves.
 */
class mastersclient : public KUKA::FRI::LBRClient {

public:
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
                  double amplRad, double filterCoeff,
                  plotMaster plotter);

    /**
    * \brief Destructor.
    */
    ~mastersclient();

    /**
     * \brief Callback for FRI state changes.
     */
    virtual void waitForCommand();

    /// \brief
    /// \param oldState
    /// \param newState
    virtual void onStateChange(KUKA::FRI::ESessionState oldState,
                               KUKA::FRI::ESessionState newState);

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


    std::chrono::time_point<std::chrono::seconds, std::chrono::nanoseconds> currentSampleTime;
    std::chrono::time_point<std::chrono::seconds, std::chrono::nanoseconds> prvSampleTime;
    // Update these declarations accordingly
    std::chrono::duration<double> deltaTime;



    //double deltaTime{};
    robotModel *robotmdl;
    plotMaster plotter;
    static std::mutex calculationMutex;
    //Gnuplot gp;

    using d_vecJointPosition = std::vector<double>;
    using rlvec_Velocity = rl::math::Vector;

    d_vecJointPosition jointPosition;
    rlvec_Velocity jointvel;

    std::deque<d_vecJointPosition> dQ_JointP_history;


    static bool
    compareVectors(const rl::math::Vector &v1, const rl::math::Vector &v2,
                   double tolerance = 1e-6, bool verbosity = false);

    void doPositionAndVelocity();

    void getCurrentTimestamp();

    static rl::math::Vector
    calculateJointVelocityOneSided(const std::vector<double> &oldJointPos,
                                   const std::vector<double> &currJointPos,
                                   double dt);


    static rl::math::Vector
    calculateJointVelocityMultiSided(
            const std::deque<d_vecJointPosition> &cJointHistory,
            double dt);

    void doProcessJointData(const rl::math::Vector &jointVel);

    void plotVelocityHistories();

    static std::mutex historyMutex;
    static std::mutex multiSidedJointVelMutex;
    static std::mutex oneSidedJointVelMutex;

};

#endif //MASTERSCLIENT_H
