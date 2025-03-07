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

#include "GeometricMethods/Sinusoid/Sinusoid.hpp"

using crf::math::geometricmethods::IGeometricMethods;
using crf::math::geometricmethods::Sinusoid;

int main(int argc, char** argv) {
    double startVel = 1.0;
    double endVel = 4.0;
    double maxAcceleration = 1.0;

    crf::math::geometricmethods::Sinusoid sinusoid(startVel, endVel, maxAcceleration);

    std::optional<double> duration = sinusoid.getRange();
    if (!duration) {
        std::cout << "Duration: Failed to calculate the parameters" << std::endl;
        return -1;
    }
    std::cout << "Sinusoid curve duration = " << duration.value() << std::endl;

    double time = 0.3;

    std::optional<double> velocity = sinusoid.evaluate(time, 0);
    if (!velocity) {
        std::cout << "Velocity: Failed to calculate the parameters" << std::endl;
        return -1;
    }
    std::cout << "The velocity at " << time << " is " << velocity.value() << std::endl;

    std::optional<double> acceleration = sinusoid.evaluate(time, 1);
    if (!acceleration) {
        std::cout << "Acceleration: Failed to calculate the parameters" << std::endl;
        return -1;
    }
    std::cout << "The acceleration at " << time << " is " << acceleration.value() << std::endl;

    std::optional<double> jerk = sinusoid.evaluate(time, 2);
    if (!jerk) {
        std::cout << "Jerk: Failed to calculate the parameters" << std::endl;
        return -1;
    }
    std::cout << "The jerk at " << time << " is " << jerk.value() << std::endl;

    std::string filename = __FILE__;
    filename = filename.substr(0, filename.find("Projects/"));
    filename += "Desktop/Results/sinusoid3.csv";
    std::ofstream fout(filename);
    fout  << "time" << ',' << "velocity" << ',' << "acceleration" << ',' << "jerk" << '\n';
    double timeStep = 0.2;
    double timeNumber = (duration.value() / timeStep);
    std::cout << "The timeNumber is " << timeNumber << std::endl;
    std::cout << " Time | 1st Der | 2nd Der | 3rd Der " << std::endl;

    std::vector<double> timeVector, velocityVector, accelerationVector, jerkVector;
    double t = 0;
    for (int i=0; i <= timeNumber; i++) {
        timeVector.push_back(t);
        velocityVector.push_back(sinusoid.evaluate(t, 0).value());
        accelerationVector.push_back(sinusoid.evaluate(t, 1).value());
        jerkVector.push_back(sinusoid.evaluate(t, 2).value());
        std::cout << timeVector[i] << " | " << velocityVector[i] << " | " << accelerationVector[i]
            << " | " << jerkVector[i] << std::endl;
        fout << timeVector[i] << ',' << velocityVector[i] << ',' << accelerationVector[i] <<
            ',' << jerkVector[i] << '\n';
        t = t + timeStep;
    }
    fout.close();

    double endTime = 10.0;
    crf::math::geometricmethods::Sinusoid sinusoidSetRange(startVel, endVel, endTime,
        crf::math::geometricmethods::ComputationMethod::SetRange);

    std::optional<double> duration2 = sinusoidSetRange.getRange();
    if (!duration2) {
        std::cout << "Duration: Failed to calculate the parameters" << std::endl;
        return -1;
    }
    std::cout << "Sinusoid curve duration = " << duration2.value() << std::endl;

    std::string filename2 = __FILE__;
    filename2 = filename2.substr(0, filename2.find("Projects/"));
    filename2 += "Desktop/Results/sinusoidSetRange.csv";
    std::ofstream fout2(filename2);
    fout2  << "time" << ',' << "velocity" << ',' << "acceleration" << ',' << "jerk" << '\n';
    timeStep = 0.2;
    timeNumber = (endTime / timeStep);
    std::cout << "The timeNumber is " << timeNumber << std::endl;
    std::cout << " Time | 1st Der | 2nd Der | 3rd Der " << std::endl;

    std::vector<double> timeVector2, velocityVector2, accelerationVector2, jerkVector2;
    t = 0;
    for (int i=0; i <= timeNumber; i++) {
        timeVector2.push_back(t);
        velocityVector2.push_back(sinusoidSetRange.evaluate(t, 0).value());
        accelerationVector2.push_back(sinusoidSetRange.evaluate(t, 1).value());
        jerkVector2.push_back(sinusoidSetRange.evaluate(t, 2).value());
        std::cout << timeVector2[i] << " | " << velocityVector2[i] << " | "
            << accelerationVector2[i] << " | " << jerkVector2[i] << std::endl;
        fout2 << timeVector2[i] << ',' << velocityVector2[i] << ',' << accelerationVector2[i] <<
            ',' << jerkVector2[i] << '\n';
        t = t + timeStep;
    }
    fout2.close();

    return 0;
}
