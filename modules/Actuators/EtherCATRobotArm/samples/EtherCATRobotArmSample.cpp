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
#include <chrono>
#include <thread>
#include <initializer_list>

#include <nlohmann/json.hpp>

#include "EtherCATDevices/TIMRobotArmWagonMotors/TIMRobotArmWagonMotors.hpp"
#include "EtherCATRobotArm/EtherCATRobotArm.hpp"
#include "EtherCATDevices/IEtherCATMotor.hpp"

using crf::devices::ethercatdevices::TIMRobotArmWagonMotors;

int main(int argc, char* argv[]) {
    std::unique_ptr<TIMRobotArmWagonMotors> BLMWagon = std::make_unique<TIMRobotArmWagonMotors>(
        argv[1]);

    if (!BLMWagon->initialize()) {
        std::cout << "Could not initialize the BLM Wagon motors" << std::endl;
        return -1;
    }
    std::cout << "Manager and Motors initialized" << std::endl;


    std::optional<std::shared_ptr<crf::devices::ethercatdevices::IEtherCATMotor>> temp;

    temp = BLMWagon->getHarmonicDrive1();
    if (!temp) {
        std::cout << "Cannot retrieve HarmonicDrive1" << std::endl;
        return -1;
    }
    std::shared_ptr<crf::devices::ethercatdevices::IEtherCATMotor> HD1 = temp.value();

    temp = BLMWagon->getHarmonicDrive2();
    if (!temp) {
        std::cout << "Cannot retrieve HarmonicDrive2" << std::endl;
        return -1;
    }
    std::shared_ptr<crf::devices::ethercatdevices::IEtherCATMotor> HD2 = temp.value();

    temp = BLMWagon->getLinearSled();
    if (!temp) {
        std::cout << "Cannot retrieve LinearSled" << std::endl;
        return -1;
    }
    std::shared_ptr<crf::devices::ethercatdevices::IEtherCATMotor> linearSled = temp.value();
    std::cout << "All arm wagon joints retrieved" << std::endl;



    linearSled->setProfileVelocity(20000);
    HD1->setProfileVelocity(34952);
    HD2->setProfileVelocity(69905);
    linearSled->setProfileAcceleration(20000);
    HD1->setProfileAcceleration(34952);
    HD2->setProfileAcceleration(69905);
    linearSled->setProfileDeceleration(20000);
    HD1->setProfileDeceleration(34952);
    HD2->setProfileDeceleration(69905);
    linearSled->setQuickstopDeceleration(20000);
    HD1->setQuickstopDeceleration(69905);
    HD2->setQuickstopDeceleration(69905);



    std::ifstream robotData(argv[2]);
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    crf::actuators::ethercatrobotarm::EtherCATRobotArm arm(robotJSON, HD1, HD2, linearSled);
    if (!arm.initialize()) {
        std::cout << "Could not initialize the EtherCATRobotArm" << std::endl;
        return -1;
    }
    std::this_thread::sleep_for(std::chrono::seconds(2));

    crf::utility::types::JointVelocities velocity1({0, 0, 0});
    arm.setJointVelocities(velocity1);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    crf::utility::types::JointVelocities velocity2({0, 0, 0});
    arm.setJointVelocities(velocity2);
    std::this_thread::sleep_for(std::chrono::seconds(2));



    if (!arm.deinitialize()) {
        std::cout << "Could not deinitialize the EtherCATRobotArm" << std::endl;
        return -1;
    }
    std::cout << "EtherCATRobotArm deinitialized" << std::endl;

    if (!BLMWagon->deinitialize()) {
        std::cout << "Could not deinitialize the BLM Wagon motors" << std::endl;
        return -1;
    }
    std::cout << "Manager and Motors deinitialized" << std::endl;
    return 0;
}
