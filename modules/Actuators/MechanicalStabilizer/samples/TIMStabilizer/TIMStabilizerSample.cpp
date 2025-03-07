/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Francesco Riccardi CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#include <string>
#include <iostream>
#include <memory>

#include "EtherCATDevices/TIMRobotArmWagonMotors/TIMRobotArmWagonMotors.hpp"
#include "EtherCATDevices/IEtherCATMotor.hpp"
#include "MechanicalStabilizer/IMechanicalStabilizer.hpp"
#include "MechanicalStabilizer/TIMStabilizer/TIMStabilizer.hpp"
#include "EtherCATDevices/EtherCATDef.hpp"

using crf::devices::ethercatdevices::TIMRobotArmWagonMotors;
using crf::actuators::mechanicalstabilizer::TIMStabilizer;
using crf::actuators::mechanicalstabilizer::IMechanicalStabilizer;

int main(int argc, char* argv[]) {
    std::string portName(argv[1]);
    std::shared_ptr<TIMRobotArmWagonMotors> motors = std::make_shared<TIMRobotArmWagonMotors>(
        portName);

    if (!motors->initialize()) {
        std::cout << "Cannot initialize BLM Wagon motors." << std::endl;
        return -1;
    }
    std::cout << "Manager and Motors initialized." << std::endl;

    auto motor = motors->getStabilizer();
    if (!motor) {
        std::cout << "Cannot retrieve Stabilizer motor." << std::endl;
        return -1;
    }

    std::shared_ptr<IMechanicalStabilizer> stabilizer = std::make_shared<TIMStabilizer>(
        motor.value());

    if (!stabilizer->initialize()) {
        std::cout << "ERROR: Stabilizer NOT initialized." << std::endl;
        return -1;
    }
    std::cout << "Stabilizer initialized." << std::endl;
    sleep(3);

    auto val = stabilizer->isDeactivated();
    if (val.value()) {
        std::cout << "The Stabilizer is open." << std::endl;
    }
    sleep(3);

    val = stabilizer->isActivated();
    if (val.value()) {
        std::cout << "The Stabilizer is closed." << std::endl;
    }
    sleep(3);

    if (!stabilizer->activate()) {
        std::cout << "ERROR: Stabilizer is NOT closing." << std::endl;
        return -1;
    }
    std::cout << "Stabilizer closing function." << std::endl;
    sleep(3);

    val = stabilizer->isDeactivated();
    if (val.value()) {
        std::cout << "The Stabilizer is open." << std::endl;
    }
    sleep(3);

    val = stabilizer->isActivated();
    if (val.value()) {
        std::cout << "The Stabilizer is closed." << std::endl;
    }
    sleep(3);

    if (!stabilizer->deactivate()) {
        std::cout << "ERROR: Stabilizer is NOT opening." << std::endl;
        return -1;
    }
    std::cout << "Stabilizer opening function." << std::endl;
    sleep(3);

    val = stabilizer->isDeactivated();
    if (val.value()) {
        std::cout << "The Stabilizer is open." << std::endl;
    }
    sleep(3);

    val = stabilizer->isActivated();
    if (val.value()) {
        std::cout << "The Stabilizer is closed." << std::endl;
    }
    sleep(3);

    if (!stabilizer->activate()) {
        std::cout << "ERROR: Stabilizer is NOT closing." << std::endl;
        return -1;
    }
    std::cout << "Stabilizer closing function." << std::endl;
    sleep(3);

    val = stabilizer->isDeactivated();
    if (val.value()) {
        std::cout << "The Stabilizer is open." << std::endl;
    }
    sleep(3);

    val = stabilizer->isActivated();
    if (val.value()) {
        std::cout << "The Stabilizer is closed." << std::endl;
    }
    sleep(3);

    if (!stabilizer->deinitialize()) {
        std::cout << "Cannot deinitialize Stabilizer." << std::endl;
        return -1;
    }
    std::cout << "Stabilizer deinitialized." << std::endl;

    if (!motors->deinitialize()) {
        std::cout << "Cannot deinitialize BLM Wagon motors." << std::endl;
        return -1;
    }
    std::cout << "Manager and Motors deinitialized." << std::endl;

    return 0;
}
