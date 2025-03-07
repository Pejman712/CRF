/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Authors: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#include <vector>
#include <memory>
#include <iostream>

#include "Controller/PositionCtrlVelocityFF/PositionCtrlVelocityFF.hpp"
#include "InverseKinematics/OptCLIK/OptCLIK.hpp"

int main() {
    // Example for five dimensions
    std::vector<double> Kp = {1, 1, 1, 1, 1};
    std::vector<double> Ki = {0.1, 0.1, 0.1, 0.1, 0.1};
    std::vector<double> Kd = {0.01, 0.01, 0.01, 0.01, 0.01};
    double Ts = 2.0;

    crf::control::controller::PositionCtrlVelocityFF feedForward =
        crf::control::controller::PositionCtrlVelocityFF(Kp, Ki, Kd, Ts, nullptr);

    crf::utility::types::JointSignals reference;
    reference.positions = crf::utility::types::JointPositions({1, 1.2, 1.3, 1.4, 1.5});
    reference.velocities = crf::utility::types::JointVelocities({0, 0.2, 0.3, 0.4, 0.5});

    crf::utility::types::JointSignals robotJointData;
    crf::utility::types::TaskSignals robotTaskData;
    robotJointData.positions = crf::utility::types::JointPositions({1, 1, 1, 1, 1});

    crf::utility::types::Signals data = feedForward.calculate(
        reference, robotJointData, robotTaskData);

    for (uint8_t i = 0; i < 5; i++) {
        std::cout << "Resulting Velocity: " << data.joints.velocities.value()[i] << std::endl;
    }
}
