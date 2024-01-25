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

constexpr int DEFAULT_PORT_ID = 30201;
constexpr unsigned int DEFAULT_JOINT_MASK = 0x8;
constexpr double DEFAULT_FREQUENCY = 0.5;
constexpr double DEFAULT_AMPLITUDE = 0.08;
constexpr double DEFAULT_FILTER_COEFFICIENT = 0.99;

void printHelp() {
    std::cout << "\nKUKA LBR joint sine overlay test application\n\n"
              << "\tCommand line arguments:\n"
              << "\t1) remote hostname (optional)\n"
              << "\t2) port ID (optional)\n"
              << "\t3) bitmask encoding of joints to be overlayed (optional)\n"
              << "\t4) sine frequency in Hertz (optional)\n"
              << "\t5) sine amplitude in radians (optional)\n"
              << "\t6) filter coefficient from 0 (off) to 1 (optional)\n";
}

int main(int argc, char** argv) {
    // Parse command line arguments
    if (argc > 1 && strstr(argv[1], "help") != nullptr) {
        printHelp();
        return 1;
    }

    std::string hostname = (argc >= 2) ? argv[1] : "172.31.0.147";
    int port = (argc >= 3) ? std::atoi(argv[2]) : DEFAULT_PORT_ID;
    unsigned int jointMask = (argc >= 4) ? static_cast<unsigned int>(std::atoi(argv[3])) : DEFAULT_JOINT_MASK;
    double frequency = (argc >= 5) ? std::atof(argv[4]) : DEFAULT_FREQUENCY;
    double amplitude = (argc >= 6) ? std::atof(argv[5]) : DEFAULT_AMPLITUDE;
    double filterCoeff = (argc >= 7) ? std::atof(argv[6]) : DEFAULT_FILTER_COEFFICIENT;

    std::cout << "hostname: " << hostname << std::endl;

    const int maxRetries = 3;
    const int maxIdleCycles = 20;
    int retryCount = 0;
    int idleCycle = 0;
    int plotCount = 15e2;

    // Set the number of threads to be half of the available hardware threads
    unsigned int numThreads = std::thread::hardware_concurrency() / 2;
    std::cout << "Max threads supported by system: " << std::thread::hardware_concurrency() << std::endl;
    std::cout << "Using " << numThreads << " threads." << std::endl;

    // Create a new sine overlay client
    mastersclient trafoClient(false,plotCount);
    std::cout << "Create TrafoCient from mastersclient succesfull"<< std::endl;
    // Pass connection and client to a new FRI client application
    UdpConnection connection;
    ClientApplication app(connection, trafoClient);
    std::cout << "Connection establishing...: "<< std::endl;

    // Connect client application to KUKA Sunrise controller
    // Repeatedly call the step routine to receive and process FRI packets
    bool success = app.connect(port, hostname.c_str());
    std::cout << "success: "<<success<< std::endl;

    while (success) {
        try {
            success = app.step();
            std::cout << "step: "<< std::endl;

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
                app.connect(port, hostname.c_str());  // Retry connection
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
