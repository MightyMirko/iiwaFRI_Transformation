//
// Created by mirko on 05.01.24.
//

#ifndef FRICLIENT_PLOTMASTER_H
#define FRICLIENT_PLOTMASTER_H
#include "matplotlibcpp.h"
#include <string>
#include <memory>
#include <vector>
#include <queue>
#include <rl/math/Unit.h>
#include <rl/math/Vector.h>


class plotMaster {
    std::deque<rl::math::Vector> hist_velocity, hist_velprecise;
    double histsize = 5e2;

public:

    plotMaster(double historyJointsize = 5e2){histsize=historyJointsize;};
    ~plotMaster() {};

    bool plotLiveFirstAxis(const std::string &colour1, const std::string &colour2);
    bool addVelocityData(const rl::math::Vector oneside_v1, const rl::math::Vector multiside_v2);
    bool getHistSize();
    bool clearData();

    void printJointPos(const std::vector<double> &jPos) const;
};


#endif //FRICLIENT_PLOTMASTER_H
