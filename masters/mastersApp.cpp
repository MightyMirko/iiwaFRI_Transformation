/**
\file
\version {1.17}
*/
#include <cstdlib>
#include <cstdio>
#include <cstring> // strstr
#include "include/mastersclient.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"
#include <stdio.h>

using namespace KUKA::FRI;


#define DEFAULT_PORTID 30201
#define DEFAULT_JOINTMASK 0x8
#define DEFAULT_FREQUENCY 0.5
#define DEFAULT_AMPLITUDE 0.08
#define DEFAULT_FILTER_COEFFICIENT 0.99

/// \brief
/// \param argc
/// \param argv
/// \return
int main(int argc, char **argv) {
    // parse command line arguments
    if (argc > 1) {
        if (strstr(argv[1], "help") != NULL) {
            printf(
                    "\nKUKA LBR joint sine overlay test application\n\n"
                    "\tCommand line arguments:\n"
                    "\t1) remote hostname (optional)\n"
                    "\t2) port ID (optional)\n"
                    "\t3) bitmask encoding of joints to be overlayed (optional)\n"
                    "\t4) sine frequency in Hertz (optional)\n"
                    "\t5) sine amplitude in radians (optional)\n"
                    "\t6) filter coefficient from 0 (off) to 1 (optional)\n"
            );
            return 1;
        }
    }
    char *hostname = (argc >= 2) ? argv[1] : NULL;
    printf("hostname: %s\n", hostname);
    int port = (argc >= 3) ? atoi(argv[2]) : DEFAULT_PORTID;
    unsigned int jointMask = (argc >= 4) ? (unsigned int) atoi(argv[3]) : DEFAULT_JOINTMASK;
    double frequency = (argc >= 5) ? atof(argv[4]) : DEFAULT_FREQUENCY;
    double amplitude = (argc >= 6) ? atof(argv[5]) : DEFAULT_AMPLITUDE;
    double filterCoeff = (argc >= 7) ? atof(argv[6]) : DEFAULT_FILTER_COEFFICIENT;

    if (hostname == NULL) hostname = "172.31.0.147";


    // create new sine overlay client
    mastersclient trafoClient(jointMask, frequency, amplitude, filterCoeff);

    /***************************************************************************/
    /*                                                                         */
    /*   Standard application structure                                        */
    /*   Configuration                                                         */
    /*                                                                         */
    /***************************************************************************/

    // create new udp connection
    UdpConnection connection;


    // pass connection and client to a new FRI client application
    ClientApplication app(connection, trafoClient);

    // connect client application to KUKA Sunrise controller
    app.connect(port, hostname);

    /***************************************************************************/
    /*                                                                         */
    /*   Standard application structure                                        */
    /*   Execution mainloop                                                    */
    /*                                                                         */
    /***************************************************************************/

    // repeatedly call the step routine to receive and process FRI packets
    bool success = true;
    while (success) {
        success = app.step();
        //printf((const char *) trafoClient.robotState().getSessionState());
        // check if we are in IDLE because the FRI session was closed
        //std::printf("Session State:\traPosition%s\n", trafoClient.s_eSessionstate.c_str());
        // std::printf("Session State%s",tmp);
        //std::printf(trafoClient.robotState().getClientCommandMode());
        if (trafoClient.s_eSessionstate){
        //std::printf("Session State:\traPosition%s\n", trafoClient.s_eSessionstate);
        }
        if (trafoClient.robotState().getSessionState() == IDLE) {
            // In this demo application we simply quit.
            // Waiting for a new FRI session would be another possibility.
            break;
        }
    }

    /***************************************************************************/
    /*                                                                         */
    /*   Standard application structure                                        */
    /*   Dispose                                                               */
    /*                                                                         */
    /***************************************************************************/

    // disconnect from controller
    app.disconnect();
    return 1;
}
