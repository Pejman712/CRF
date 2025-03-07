/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO
 *
 *  ==================================================================================================
 */

#include <string>
#include <iostream>
#include <memory>

#include <nlohmann/json.hpp>

#include "KinovaArm/KinovaJaco.hpp"
#include "KinovaArm/KinovaApiInterface.hpp"

#include "RobotArmController/RobotArmVelocityController/RobotArmVelocityController.hpp"

int main(int argc, char* argv[]) {
    std::string armConfig(argv[1]);

    std::ifstream ArmConfig_(armConfig);
    nlohmann::json ArmConfigJSON_ = nlohmann::json::parse(ArmConfig_);

    // Generate arm and controller
    std::shared_ptr<crf::actuators::kinovaarm::KinovaJaco> arm =
        std::make_shared<crf::actuators::kinovaarm::KinovaJaco>(
            std::make_shared<crf::actuators::kinovaarm::KinovaApiInterface>(),
            ArmConfigJSON_);

    if (!arm->initialize()) {
        std::puts("Could not initialize the arm");
        return -1;
    }

    std::shared_ptr<crf::control::robotarmcontroller::RobotArmVelocityController> controller =
        std::make_unique<crf::control::robotarmcontroller::RobotArmVelocityController>(arm);

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
