/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>
#include <vector>

#include <nlohmann/json.hpp>

#include "ClosedLoopController/PIDController.hpp"
#include "CANSocket/CANSocket.hpp"
#include "SchunkArm/SchunkArm.hpp"
#include "RobotArmController/RobotArmVelocityController/RobotArmVelocityController.hpp"

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cout << "Too few arguments" << std::endl;
        std::cout << "[1] CAN interface " << std::endl;
        std::cout << "[2] CONFIG_FILENAME " << std::endl;
        return -1;
    }

    std::shared_ptr<crf::communication::cansocket::CANSocket> canSocket =
        std::make_shared<crf::communication::cansocket::CANSocket>(argv[1]);

    std::ifstream robotData(argv[2]);
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    auto arm = std::make_shared<crf::actuators::schunkarm::SchunkArm>(canSocket, robotJSON);

    if (!arm->initialize()) {
        std::puts("Could not initialize the arm");
        return -1;
    }

    auto controller =
        std::make_shared<crf::control::robotarmcontroller::RobotArmVelocityController>(arm);

    if (!controller->initialize()) {
        std::cout << "Could not initialize the controller" << std::endl;
        return -1;
    }

    crf::utility::types::JointPositions pos = controller->getJointPositions();
    crf::utility::types::JointVelocities vel = controller->getJointVelocities();
    std::cout << "Joint Pos: " << pos << " && Joint Vel: " << vel << std::endl;

    crf::utility::types::TaskPose postask = controller->getTaskPose();
    crf::utility::types::TaskVelocity veltask = controller->getTaskVelocity();
    std::cout << "Task Pos: " << postask << " && Task Vel: " << veltask << std::endl;

    controller->deinitialize();
    arm->deinitialize();

    return 0;
}
