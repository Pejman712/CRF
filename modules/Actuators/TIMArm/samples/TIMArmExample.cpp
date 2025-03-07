/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Francesco Riccardi CERN EN/SMM/MRO
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

using crf::devices::ethercatdevices::TIMRobotArmWagonMotors;
using crf::actuators::timarm::TIMArm;
using crf::actuators::kinovaarm::KinovaApiInterface;

int main(int argc, char* argv[]) {
    std::string portName(argv[1]);
    std::string armConfigFile(argv[2]);

    std::shared_ptr<TIMRobotArmWagonMotors> BLMWagon = std::make_unique<TIMRobotArmWagonMotors>(
        portName);

    printf("\n\nTIM_ARM_EXAMPLE --------------------\n");
    if (BLMWagon->initialize()) {
        printf("\n\n\n\n[BLM Wagon] Manager and Motors initialized \n");
    } else {
        printf("Cannot initialize BLM Wagon motors \n");
        return -1;
    }

    std::ifstream armData(armConfigFile);
    nlohmann::json armJSON;
    armData >> armJSON;

    std::shared_ptr<crf::actuators::timarm::TIMArm> arm =
        std::make_shared<crf::actuators::timarm::TIMArm>(armJSON,
            BLMWagon,
            std::make_shared<crf::actuators::kinovaarm::KinovaApiInterface>());

    if (!arm->initialize()) {
        std::cout << "Could not initialize the arm" << std::endl;
        return -1;
    }

    printf("\n\n--------------------\n");
    sleep(2);
    crf::utility::types::JointVelocities desVel({0, 0, 0, 0, 0, 0, 0, 0, 0});
    if (arm->setJointVelocities(desVel))
        std::cout << "Set Velocity: " << desVel << std::endl;
    else
        printf("Could not set Velocity \n");

    sleep(1);
    auto position = arm->getJointPositions();
    if (position)
        std::cout << "Position: " << position.get() << std::endl;
    else
        printf("Could not retrieve Position \n");

    sleep(1);
    auto velocity = arm->getJointVelocities();
    if (velocity)
        std::cout << "Velocity: " << velocity.get() << std::endl;
    else
        printf("Could not retrieve Velocity \n");

    sleep(1);
    auto torque = arm->getJointForceTorques();
    if (torque)
        std::cout << "Torque: " << torque.get() << std::endl;
    else
        printf("Could not retrieve Torque \n");

    sleep(1);
    auto taskPose = arm->getTaskPose();
    if (taskPose)
        std::cout << "Task Position: " << taskPose.get() << std::endl;
    else
        printf("Could not retrieve Task Position \n");

    sleep(1);
    auto taskVelocity = arm->getTaskVelocity();
    if (taskVelocity)
        std::cout << "Task Velocity: " << taskVelocity.get() << std::endl;
    else
        printf("Could not retrieve Task Velocity \n");

    sleep(2);
    printf("--------------------\n\n");

    if (arm->deinitialize()) {
        printf("Arm deinitialized \n");
    } else {
        printf("Arm cannot be deinitialized \n");
    }

    if (BLMWagon->deinitialize()) {
        printf("Manager and Motors deinitialized \n");
    } else {
        printf("Manager and Motors cannot be deinitialized \n");
    }

    return  -1;
}
