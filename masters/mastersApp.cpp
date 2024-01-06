/**
 * \file
 * \version {1.17}
 */

#include <cstdlib>
#include <cstdio>
#include <cstring> // strstr
#include <iostream>
#include <stdexcept>
#include <thread>
#include "include/mastersclient.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"

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

    const int maxRetries = 3;  // You can adjust the maximum number of retries as needed
    const int maxIdleCycles = 20;
    int retryCount = 0;
    int idleCycle = 0;
    int plotCount = 15e2;

    // Set the number of threads to be half of the available hardware threads
    unsigned int numThreads = std::thread::hardware_concurrency() / 2;
    std::cout << "Max threads supported by system: " << std::thread::hardware_concurrency() << std::endl;
    std::cout << "Using " << numThreads << " threads." << std::endl;

    // Create a new sine overlay client
    mastersclient trafoClient(jointMask, frequency, amplitude, filterCoeff, plotMaster(plotCount));

    // Pass connection and client to a new FRI client application
    UdpConnection connection;
    ClientApplication app(connection, trafoClient);

    // Connect client application to KUKA Sunrise controller
    // Repeatedly call the step routine to receive and process FRI packets
    bool success = app.connect(port, hostname.c_str());
    while (success) {
        try {
            success = app.step();

            if (trafoClient.s_eSessionstate) {
                //std::printf("Session State:\traPosition%s\n", trafoClient.s_eSessionstate);
                idleCycle = 0;
            }

            if (trafoClient.robotState().getSessionState() == IDLE) {
                ++idleCycle;

                if (idleCycle >= maxIdleCycles) {
                    throw std::runtime_error("FRI session entered IDLE state too often");
                }
            }
        } catch (const std::runtime_error& e) {
            // Handle the exception (e.g., log the error)
            ++retryCount;

            if (retryCount <= maxRetries) {
                // Retry the connection
                std::cerr << "Caught exception: " << e.what() << ". Retrying connection..."
                          << std::endl;
                app.disconnect();  // Disconnect before retrying
                app.connect(port, hostname);  // Retry connection
            } else {
                // If max retries exceeded, exit the loop
                std::cerr << "Maximum number of retries exceeded. Exiting." << std::endl;
                break;
            }
        }
    }

    // Disconnect from controller
    app.disconnect();
    return 1;
}
