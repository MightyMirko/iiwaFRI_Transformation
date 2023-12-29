/**
\file
\version {1.17}
*/
#ifndef MASTERSCLIENT_H
#define MASTERSCLIENT_H

#include "friLBRClient.h"
#include <string>

/**
 * \brief Test client that can overlay interpolator joint positions with sine waves.
 */
class mastersclient : public KUKA::FRI::LBRClient
{
   
public:

    //KUKA::FRI::ESessionState eSessionState;
    const char *  s_eSessionstate ;
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

    void printJointPos();
};

#endif // _KUKA_FRI_LBR_JOINT_SINE_OVERLAY_CLIENT_H
