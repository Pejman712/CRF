/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Hannes Gamper CERN BE/CEM/MRO 2021
 *         Laura Rodrigo Pérez CERN BE/CEM/MRO 2021
 *
 *  ================================================================================================================
 */

#include <fstream>
#include <optional>

#include "GeometricMethods/DeBoor/DeBoor.hpp"

using crf::math::geometricmethods::DeBoor;

int main(int argc, char** argv) {
    std::cout << "Load trajectory..." << std::endl;
    int degree(4);
    std::vector<double> knots({0.0, 0.0, 0.0, 0.0, 0.0, 0.0625, 0.125, 0.1875, 0.25, 0.3125, 0.375,
        0.4375, 0.5, 0.5625, 0.625, 0.6875, 0.75, 0.8125, 0.875, 0.9375, 1.0, 1.0, 1.0, 1.0, 1.0});
    std::vector<double> controlPoints({0.0, 0.0, 0.0, 0.0, 0.05650608, 0.236911941, 0.560672572,
        0.972236973, 1.357214323, 1.590212873, 1.590212873, 1.357214323, 0.972236973, 0.560672572,
        0.236911941, 0.05650608, 0.0, 0.0, 0.0, 0.0});

    crf::math::geometricmethods::DeBoor deBoor(degree, knots, controlPoints);

    std::optional<double> duration = deBoor.getRange();
    if (!duration) {
        std::cout << "Duration: Failed to calculate the parameters" << std::endl;
        return -1;
    }
    std::cout << "Spline curve duration = " << duration.value() << std::endl;

    double time = 0.3;

    std::optional<double> position = deBoor.evaluate(time, 0);
    if (!position) {
        std::cout << "Position: Failed to calculate the trajectory" << std::endl;
        return -1;
    }
    std::cout << "The position at " << time << " is " << position.value() << std::endl;

    std::string path = __FILE__;
    path = path.substr(0, path.find("DeBoorSample.cpp"));
    std::ofstream foutPos(path + "pos10.csv");
    foutPos  << "time" << ',' << "positions" << '\n';
    double timeStep = 0.2;
    double timeNumber = (*duration / timeStep);
    std::cout << "The timeNumber is " << timeNumber << std::endl;
    std::cout << " Time | Position" << std::endl;

    std::vector<double> time_, position_;
    double t = 0;
    for (int i=0; i <= timeNumber; i++) {
        time_.push_back(t);
        std::optional<double> pathPos_ = deBoor.evaluate(t, 0);
        position_.push_back(pathPos_.value());
        std::cout << time_[i] << " | " << position_[i] << std::endl;
        foutPos << time_[i] << ',' << position_[i] << '\n';
        t = t + timeStep;
    }
    foutPos.close();
    return 0;
}
