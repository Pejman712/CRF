/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Jean Paul Sulca CERN BM-CEM-MRO 2023
 * 
 *  ==================================================================================================
 */

#include <cmath>
#include <chrono>
#include <thread>
#include <memory>
#include <iostream>

#include "MassSpringDamper/MSDRn/MSDRnDecoupled.hpp"
#include "MassSpringDamper/MSDR1/MSDR1ImpulseInvarianceIIR.hpp"

using crf::math::massspringdamper::MSDR1ImpulseInvarianceIIR;
using crf::math::massspringdamper::MSDRnDecoupled;
using crf::math::massspringdamper::SignalRn;

int main(int argc, char* argv[]) {
    float Ts = 0.1;
    const uint8_t dim = 6;
    Eigen::MatrixXd m = Eigen::DiagonalMatrix<double, dim>(0.5, 0.6, 0.4, 0.5, 0.45, 0.35);
    Eigen::MatrixXd d = Eigen::DiagonalMatrix<double, dim>(2.7, 3, 2.5, 2.8, 2.9, 2.85);
    Eigen::MatrixXd k = Eigen::DiagonalMatrix<double, dim>(2.7, 3, 2.6, 2.5, 2.6, 2.95);

    std::vector<Eigen::MatrixXd> force = {
        Eigen::DiagonalMatrix<double, dim>(0.1, 0.15, 0.1, 0.15, 0.3, 0.3),
        Eigen::DiagonalMatrix<double, dim>(0.2, 0.25, 0.2, 0.2, 0.3, 0.35),
        Eigen::DiagonalMatrix<double, dim>(0.25, 0.25, 0.25, 0.15, 0.4, 0.4),
        Eigen::DiagonalMatrix<double, dim>(0.2, 0.15, 0.2, 0.15, 0.45, 0.4)
    };

    MSDRnDecoupled msdRn = MSDRnDecoupled<MSDR1ImpulseInvarianceIIR>(dim, Ts);

    for (int i = 0; i < force.size(); i++) {
        auto begin = std::chrono::steady_clock::now();
        SignalRn val;
        try {
            val = msdRn.calculate(force[i], m, d, k);
        } catch (std::invalid_argument &ex) {
            std::cout << "A problem has ocurred! Error: " << ex.what() << std::endl;
            break;
        }

        printf("----------------------------------------------------------------\n");
        for (uint8_t j = 0; j < force[i].rows(); j++) {
            printf("Dimension %d - Displacement: %f, Velocity:  %f, Acceleration: %f\n",
                (j+1), val.position[j], val.velocity[j], val.acceleration[j]);
        }
        printf("----------------------------------------------------------------\n");
        auto end = std::chrono::steady_clock::now();

        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
        int Tstep = Ts*1000 - duration;
        std::this_thread::sleep_for(std::chrono::milliseconds(Tstep));
    }

    return 0;
}
