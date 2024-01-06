//
// Created by mirko on 05.01.24.
//

#include "plotMaster.h"

namespace plt = matplotlibcpp;

void plotMaster::printJointPos(const std::vector<double>& jPos) const {
    std::cout << "jointPosition:\t\t";
    for (const auto &value : jPos) {
        std::cout << value << " ";
    }
    std::cout << std::endl;
}

bool plotMaster::addVelocityData(const rl::math::Vector oneside_v1, const rl::math::Vector multiside_v2) {
    // Check if oneside_v1 and multiside_v2 have the same size
    if (oneside_v1.size() != multiside_v2.size()) {
        std::cerr << "Error: Vectors have different sizes." << std::endl;
        return false;
    }
    // Check if oneside_v1 is empty
    if (oneside_v1.size() == 0) {
        std::cerr << "Error: oneside_v1 is empty." << std::endl;
        return false;
    }

    // Check if multiside_v2 is empty
    if (multiside_v2.size() == 0) {
        std::cerr << "Error: multiside_v2 is empty." << std::endl;
        return false;
    }
    // Push back evaluated vectors into the histories

    hist_velocity.push_back(oneside_v1.eval());
    hist_velprecise.push_back(multiside_v2.eval());

    return true;  // Successfully added data
}


bool plotMaster::getHistSize() {
    if (hist_velocity.size() >= histsize &&  hist_velprecise.size() >= histsize) {
        return true;
    }
    return false;
}
bool plotMaster::plotLiveFirstAxis(const std::string& colour1, const std::string& colour2) {
    namespace plt = matplotlibcpp;

    plt::clf();  // Clear the current figure

    // Assuming there are 7 joints
    const size_t numJoints = 7;

    // Create a figure for subplots
    // plt::figure();

    // Plot only the first axis
    size_t jointIndex = 0;

    plt::subplot(1, 1, 1);  // 1-indexed subplot

    std::vector<double> time;
    std::vector<double> jointData1, jointData2;

    // Check if the histories have the same size
    if (hist_velocity.size() != hist_velprecise.size()) {
        std::cerr << "Error: Histories have different sizes." << std::endl;
        return false;
    }



    // Populate data for the current joint
    for (size_t i = 0; i < hist_velocity.size(); ++i) {
        time.push_back(i * 0.005);  // Assuming data is added every 5ms
        jointData1.push_back(hist_velocity[i][jointIndex]);
        jointData2.push_back(hist_velprecise[i][jointIndex]);
        std::cout << "Time: " << time.back() << "\n, OneSide: " <<
        jointData1.back() << ",\n Multi: " << jointData2.back() << std::endl;
    }

    // Plot the specific joint over time for hist_velocity
    plt::named_plot("OneSide",time, jointData1, colour1);

    // Plot the specific joint over time for hist_velprecise
    plt::named_plot("Multi",time, jointData2, colour2);
    plt::legend();
    // Set subplot labels and title
    plt::xlabel("Time (s)");
    plt::ylabel("Joint Value");
    plt::title("Joint " + std::to_string(jointIndex + 1) + " over Time");

    // Show the plot
    plt::pause(0.01);
    plt::show();

    return true;
}

bool plotMaster::clearData() {
    hist_velocity.clear();
    hist_velprecise.clear();
    return false;
}

plotMaster::plotMaster(double histsize) : histsize(histsize) {}



/*
void plotMaster::plotLive7d(const std::string& colour) {
    // Set the size of output image = 1200x780 pixels
    //plt::figure();
    plt::clf();
    //plt::figure_size(1200, 780);
    // Clear the current figure outside the loop
    //plt::clf();

    // Assuming there are 7 joints
    const size_t numJoints = 7;

    // Loop over each joint and plot in a separate subplot
    for (size_t jointIndex = 0; jointIndex < numJoints; ++jointIndex) {
        plt::subplot(4, 2, long(jointIndex + 1));  // 1-indexed subplot
        std::vector<double> time;
        std::vector<double> jointData;

        // Populate data for the current joint
        for (size_t i = 0; i < data.size(); ++i) {
            time.push_back(i * 0.005);  // Assuming data is added every 5ms
            jointData.push_back(data[i][jointIndex]);
        }

        // Plot the specific joint over time
        plt::plot(time, jointData, colour);

        // Set subplot labels and title
        plt::xlabel("Time (s)");
        plt::ylabel("Joint Value");
        plt::title("Joint " + std::to_string(jointIndex + 1) + " over Time");
    }

    // Adjust layout for better visualization
    plt::tight_layout();

    // Show the plot
    //plt::pause(0.01);
    plt::show();
}

void plotMaster::plotLive7d(const std::string& colour1, const std::string& colour2) {

    namespace plt = matplotlibcpp;

    plt::clf();  // Clear the current figure

    // Assuming there are 7 joints
    const size_t numJoints = 7;

    // Create a figure for subplots
    // plt::figure();

    // Loop over each joint and plot in a separate subplot
    for (size_t jointIndex = 0; jointIndex < numJoints; ++jointIndex) {
        plt::subplot(4, 2, long(jointIndex + 1));  // 1-indexed subplot

        std::vector<double> time;
        std::vector<double> jointData1, jointData2;

        // Populate data for the current joint
        for (size_t i = 0; i < data1.size(); ++i) {
            time.push_back(i * 0.005);  // Assuming data is added every 5ms
            jointData1.push_back(data1[i][jointIndex]);
            jointData2.push_back(data2[i][jointIndex]);
        }

        // Plot the specific joint over time for data1
        plt::plot(time, jointData1, colour1);

        // Plot the specific joint over time for data2
        plt::plot(time, jointData2, colour2);

        // Set subplot labels and title
        plt::xlabel("Time (s)");
        plt::ylabel("Joint Value");
        plt::title("Joint " + std::to_string(jointIndex + 1) + " over Time");
    }

    // Adjust layout for better visualization
    plt::tight_layout();

    // Show the plot
    plt::pause(0.01);
    plt::show();
}

void plotMaster::plotLiveFirstAxis(const std::deque<rl::math::Vector> &data1,
                                   const std::deque<rl::math::Vector> &data2,
                                   const std::string& colour1, const std::string& colour2) {

    namespace plt = matplotlibcpp;

    plt::clf();  // Clear the current figure

    // Assuming there are 7 joints
    const size_t numJoints = 7;

    // Create a figure for subplots
    // plt::figure();

    // Plot only the first axis
    size_t jointIndex = 0;

    plt::subplot(1, 1, 1);  // 1-indexed subplot

    std::vector<double> time;
    std::vector<double> jointData1, jointData2;

    // Populate data for the current joint
    for (size_t i = 0; i < data1.size(); ++i) {
        time.push_back(i * 0.005);  // Assuming data is added every 5ms
        jointData1.push_back(data1[i][jointIndex]);
        jointData2.push_back(data2[i][jointIndex]);
    }

    // Plot the specific joint over time for data1
    plt::plot(time, jointData1, colour1);

    // Plot the specific joint over time for data2
    plt::plot(time, jointData2, colour2);

    // Set subplot labels and title
    plt::xlabel("Time (s)");
    plt::ylabel("Joint Value");
    plt::title("Joint " + std::to_string(jointIndex + 1) + " over Time");

    // Show the plot
    plt::pause(0.01);
    plt::show();
}
*/