/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Francesco Riccardi CERN BE/CEM/MRO
 *
 *  ==================================================================================================
 */

#include <string>
#include <iostream>
#include <memory>

#include "EtherCATDevices/TIMRobotArmWagonMotors/TIMRobotArmWagonMotors.hpp"
#include "EtherCATDevices/EtherCATMotor.hpp"
#include "Shielding/RadioactiveSourceShielding/RadioactiveSourceShielding.hpp"

int main(int argc, char* argv[]) {
    std::string portName(argv[1]);
    std::shared_ptr<crf::devices::ethercatdevices::TIMRobotArmWagonMotors> etherCATMotors =
        std::make_shared<crf::devices::ethercatdevices::TIMRobotArmWagonMotors>(portName);

    if (!etherCATMotors->initialize()) {
        std::cout << "Cannot initialize BLM Wagon motors." << std::endl;
        return -1;
    }
    std::cout << "Manager and Motors initialized." << std::endl;

    auto motor = etherCATMotors->getShielding();
    if (!motor) {
        std::cout << "Cannot retrieve Shielding motor." << std::endl;
        return -1;
    }

    std::unique_ptr<crf::actuators::shielding::RadioactiveSourceShielding> shielding =
        std::make_unique<crf::actuators::shielding::RadioactiveSourceShielding>(motor.value());

    if (shielding->initialize()) {
        std::cout << "Shielding initialized" << std::endl;
    } else {
        std::cout << "Shielding NOT initialized" << std::endl;
        return -1;
    }

    if (shielding->open()) {
        std::cout << "Shielding is opening" << std::endl;
    } else {
        std::cout << "Shielding is NOT opening" << std::endl;
    }
    if (shielding->isClosed()) {
        std::cout << "Shielding is closed" << std::endl;
    } else {
        std::cout << "Shielding is NOT closed" << std::endl;
    }
    if (shielding->isOpen()) {
        std::cout << "Shielding is opened" << std::endl;
    } else {
        std::cout << "Shielding is NOT opened" << std::endl;
    }

    if (shielding->close()) {
        std::cout << "Shielding is closing" << std::endl;
    } else {
        std::cout << "Shielding is NOT closing" << std::endl;
    }

    if (shielding->isClosed()) {
        std::cout << "Shielding is closed" << std::endl;
    } else {
        std::cout << "Shielding is NOT closed" << std::endl;
    }
    if (shielding->isOpen()) {
        std::cout << "Shielding is opened" << std::endl;
    } else {
        std::cout << "Shielding is NOT opened" << std::endl;
    }

    if (!shielding->deinitialize()) {
        std::cout << "Cannot deinitialize Shielding." << std::endl;
        return -1;
    }
    std::cout << "Shielding deinitialized." << std::endl;

    if (!etherCATMotors->deinitialize()) {
        std::cout << "Cannot deinitialize BLM Wagon motors." << std::endl;
        return -1;
    }
    std::cout << "Manager and Motors deinitialized." << std::endl;

    return 0;
}
