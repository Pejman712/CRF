/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playan garai CERN BE/CEM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>
#include <string>
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include <initializer_list>

#include "EtherCATDevices/TIMRobotArmWagonMotors/TIMRobotArmWagonMotors.hpp"
#include "TIMArm/TIMArm.hpp"
#include "KinovaArm/KinovaApiInterface.hpp"

#include "RobotArmController/RobotArmVelocityController/RobotArmVelocityController.hpp"

using crf::devices::ethercatdevices::TIMRobotArmWagonMotors;

int main(int argc, char* argv[]) {
    std::string portName(argv[1]);
    std::string armConfigFile(argv[2]);

    std::shared_ptr<TIMRobotArmWagonMotors> motors = std::make_unique<TIMRobotArmWagonMotors>(
        portName);

    if (motors->initialize()) {
        printf("\n\n\n\n[BLM Wagon] Manager and Motors initialized \n");
    } else {
        printf("Cannot initialize BLM Wagon motors \n");
        return -1;
    }

    std::ifstream armData(armConfigFile);
    nlohmann::json armJSON;
    armData >> armJSON;

    auto arm = std::make_shared<crf::actuators::timarm::TIMArm>(armJSON, motors,
        std::make_shared<crf::actuators::kinovaarm::KinovaApiInterface>());

    if (!arm->initialize()) {
        std::cout << "Could not initialize the arm" << std::endl;
        return -1;
    }

    auto controller =
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

    if (motors->deinitialize()) {
        printf("Manager and Motors deinitialized \n");
    } else {
        printf("Manager and Motors cannot be deinitialized \n");
    }
    return 0;
}
