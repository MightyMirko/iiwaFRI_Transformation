/**
 * \file
 * \version {1.17}
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
    mastersclient(bool doPlot=false,double plotCount=15e1);
    ~mastersclient();

    /// \brief Callback for FRI state changes.
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
     * \brief Callback for the FRI state 'Commanding Wait'.
     */
    virtual void waitForCommand();

    /**
     * \brief Callback for the FRI state 'Commanding Active'.
     */
    void command() override;

private:


    std::chrono::time_point<std::chrono::seconds,
            std::chrono::nanoseconds> currentSampleTime,prvSampleTime;
    std::chrono::duration<double> deltaTime{};

    bool doPlot = false; ///< Flag to enable or disable plotting.

    static std::mutex historyMutex,calculationMutex,multiSidedJointVelMutex,
            oneSidedJointVelMutex; ///< Mutexes for various operations.

    robotModel *robotmdl; ///< Pointer to the robot model.

    plotMaster plotter; ///< The plotMaster object for plotting.

    using d_vecJointPosition = std::vector<double>;
    using rlvec_Velocity = rl::math::Vector;

    d_vecJointPosition jointPosition; ///< Vector to store joint positions.
    rlvec_Velocity jointvel; ///< Vector to store joint velocities.

    std::deque<d_vecJointPosition> dQ_JointP_history; ///< History of jo


    // Methods private

    /**
     * \brief Compare two vectors for equality within a specified tolerance.
     *
     * \param v1 The first vector.
     * \param v2 The second vector.
     * \param tolerance Tolerance for equality check.
     * \param verbosity If true, print detailed information about differences.
     * \return True if the vectors are approximately equal, false otherwise.
     */
    static bool
    compareVectors(const rl::math::Vector &v1, const rl::math::Vector &v2,
                   double tolerance = 1e-6, bool verbosity = false);

    /**
     * \brief Perform position and velocity calculations.
     */
    void doPositionAndVelocity();

    /**
     * \brief Get the current timestamp and check for timestamp errors.
     */
    void getCurrentTimestamp();

    /**
     * \brief Calculate joint velocity using one-sided differencing.
     *
     * \param oldJointPos The old joint positions.
     * \param currJointPos The current joint positions.
     * \param dt The time step.
     * \return Vector containing joint velocity.
     */
    static rl::math::Vector
    calculateJointVelocityOneSided(const std::vector<double> &oldJointPos,
                                   const std::vector<double> &currJointPos,
                                   double dt);

    /**
     * \brief Calculate joint velocity using multi-sided differencing.
     * \param cJointHistory History of joint positions.  Must have a size greater than or
     * equal to 5 and must not be empty.
     * \param dt Time step (always around 5ms).
     * \return Vector containing joint velocity.
     */
    static rl::math::Vector
    calculateJointVelocityMultiSided(
            const std::deque<d_vecJointPosition> &cJointHistory,
            double dt);

    /**
     * \brief Process joint data, set joint positions and velocities in the robot model.
     *
     * \param jointVel Joint velocity vector.
     */
    void doProcessJointData(const rl::math::Vector &jointVel);

    /**
     * \brief Plot velocity histories if enabled.
     */
    void plotVelocityHistories();
};

#endif //MASTERSCLIENT_H
