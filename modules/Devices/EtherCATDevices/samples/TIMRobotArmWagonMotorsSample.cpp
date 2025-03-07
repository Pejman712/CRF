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
#include <optional>
#include <chrono>
#include <thread>

#include "EtherCATDevices/TIMRobotArmWagonMotors/TIMRobotArmWagonMotors.hpp"
#include "EtherCATDevices/IEtherCATMotor.hpp"
#include "EtherCATDevices/EtherCATDef.hpp"

using crf::devices::ethercatdevices::TIMRobotArmWagonMotors;
using crf::devices::ethercatdevices::IEtherCATMotor;
using crf::devices::ethercatdevices::modesofoperation::ProfileVelocityMode;

int main(int argc, char* argv[]) {
    std::unique_ptr<TIMRobotArmWagonMotors> BLMWagon =
        std::make_unique<TIMRobotArmWagonMotors>(argv[1]);
    if (!BLMWagon->initialize()) {
        std::cout << "Could not initialize the BLM Wagon motors" << std::endl;
        return -1;
    }
    std::cout << "Manager and Motors initialized" << std::endl;


    std::optional<std::shared_ptr<IEtherCATMotor>> temp;

    temp = BLMWagon->getHarmonicDrive1();
    if (!temp) {
        std::cout << "Cannot retrieve HarmonicDrive1" << std::endl;
        return -1;
    }
    std::shared_ptr<IEtherCATMotor> HD1 = temp.value();

    temp = BLMWagon->getHarmonicDrive2();
    if (!temp) {
        std::cout << "Cannot retrieve HarmonicDrive2" << std::endl;
        return -1;
    }
    std::shared_ptr<IEtherCATMotor> HD2 = temp.value();

    temp = BLMWagon->getLinearSled();
    if (!temp) {
        std::cout << "Cannot retrieve LinearSled" << std::endl;
        return -1;
    }
    std::shared_ptr<IEtherCATMotor> linearSled = temp.value();

    temp = BLMWagon->getStabilizer();
    if (!temp) {
        std::cout << "Cannot retrieve Stabilizer" << std::endl;
        return -1;
    }
    std::shared_ptr<IEtherCATMotor> stabilizer = temp.value();

    temp = BLMWagon->getShielding();
    if (!temp) {
        std::cout << "Cannot retrieve Shielding" << std::endl;
        return -1;
    }
    std::shared_ptr<IEtherCATMotor> shielding = temp.value();
    std::cout << "All arm wagon joints retrieved" << std::endl;


    std::cout << "------------------ HD1 ------------------" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));

    if (!HD1->setMaxCurrent(50000)) {
        std::cout << "Harmonic drive 1 maximum current can't be set" << std::endl;
        return -1;
    }
    std::cout << "Harmonic drive 1 maximum current set" << std::endl;

    if (!HD1->setModeOfOperation(ProfileVelocityMode)) {
        std::cout << "Harmonic drive 1 can't enter in Profile Velocity Mode" << std::endl;
        return -1;
    }
    std::cout << "Harmonic drive 1 is in Profile Velocity Mode" << std::endl;

    std::optional<bool> optionalHD1Value = HD1->inFault();
    if (!optionalHD1Value) {
        std::cout << "Communication problem with the harmonic drive 1" << std::endl;
        return -1;
    }
    if (optionalHD1Value.value()) {
        std::cout << "Harmonic drive 1 is fault. Trying to reset the fault..." << std::endl;
        if (!HD1->faultReset()) {
            std::cout << "Harmonic drive 1 fault reset can't be performed" << std::endl;
            return -1;
        }
        std::cout << "Harmonic drive 1 fault reset performed" << std::endl;
    }

    if (!HD1->shutdown()) {
        std::cout << "Harmonic drive 1 shutdown can't be performed" << std::endl;
        return -1;
    }
    std::cout << "Harmonic drive 1 shutdown performed" << std::endl;

    if (!HD1->enableOperation()) {
        std::cout << "Harmonic drive 1 can't be enabled for operations" << std::endl;
        return -1;
    }
    std::cout << "Harmonic drive 1 enabled for operations" << std::endl;


    std::cout << "------------------ HD2 ------------------" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));

    if (!HD2->setMaxCurrent(50000)) {
        std::cout << "Harmonic drive 2 maximum current can't be set" << std::endl;
        return -1;
    }
    std::cout << "Harmonic drive 2 maximum current set" << std::endl;

    if (!HD2->setModeOfOperation(ProfileVelocityMode)) {
        std::cout << "Harmonic drive 2 can't enter in Profile Velocity Mode" << std::endl;
        return -1;
    }
    std::cout << "Harmonic drive 2 is in Profile Velocity Mode" << std::endl;

    std::optional<bool> optionalHD2Value = HD2->inFault();
    if (!optionalHD2Value) {
        std::cout << "Communication problem with the harmonic drive 2" << std::endl;
        return -1;
    }
    if (optionalHD2Value.value()) {
        std::cout << "Harmonic drive 2 is fault. Trying to reset the fault..." << std::endl;
        if (!HD2->faultReset()) {
            std::cout << "Harmonic drive 2 fault reset can't be performed" << std::endl;
            return -1;
        }
        std::cout << "Harmonic drive 2 fault reset performed" << std::endl;
    }

    if (!HD2->shutdown()) {
        std::cout << "Harmonic drive 2 shutdown can't be performed" << std::endl;
        return -1;
    }
    std::cout << "Harmonic drive 2 shutdown performed" << std::endl;

    if (!HD2->enableOperation()) {
        std::cout << "Harmonic drive 2 can't be enabled for operations" << std::endl;
        return -1;
    }
    std::cout << "Harmonic drive 2 enabled for operations" << std::endl;


    std::cout << "-------------- Linear Sled --------------" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));

    if (!linearSled->setMaxCurrent(50000)) {
        std::cout << "Linear sled maximum current can't be set" << std::endl;
        return -1;
    }
    std::cout << "Linear sled maximum current set" << std::endl;

    if (!linearSled->setModeOfOperation(ProfileVelocityMode)) {
        std::cout << "Linear sled can't enter in Profile Velocity Mode" << std::endl;
        return -1;
    }
    std::cout << "Linear sled is in Profile Velocity Mode" << std::endl;

    std::optional<bool> optionalLinearSledValue = linearSled->inFault();
    if (!optionalLinearSledValue) {
        std::cout << "Communication problem with the linear sled" << std::endl;
        return -1;
    }
    if (optionalLinearSledValue.value()) {
        std::cout << "Linear sled is fault. Trying to reset the fault..." << std::endl;
        if (!linearSled->faultReset()) {
            std::cout << "Linear sled fault reset can't be performed" << std::endl;
            return -1;
        }
        std::cout << "Linear sled fault reset performed" << std::endl;
    }

    if (!linearSled->shutdown()) {
        std::cout << "Linear sled shutdown can't be performed" << std::endl;
        return -1;
    }
    std::cout << "Linear sled shutdown performed" << std::endl;

    if (!linearSled->enableOperation()) {
        std::cout << "Linear sled can't be enabled for operations" << std::endl;
        return -1;
    }
    std::cout << "Linear sled enabled for operations" << std::endl;


    std::cout << "---------------- Stabilizer ----------------" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));

    if (!stabilizer->setMaxCurrent(50000)) {
        std::cout << "Stabilizer maximum current can't be set" << std::endl;
        return -1;
    }
    std::cout << "Stabilizer maximum current set" << std::endl;

    if (!stabilizer->setModeOfOperation(ProfileVelocityMode)) {
        std::cout << "Stabilizer can't enter in Profile Velocity Mode" << std::endl;
        return -1;
    }
    std::cout << "Stabilizer is in Profile Velocity Mode" << std::endl;

    std::optional<bool> optionalStabilizerValue = stabilizer->inFault();
    if (!optionalStabilizerValue) {
        std::cout << "Communication problem with the harmonic drive 2" << std::endl;
        return -1;
    }
    if (optionalStabilizerValue.value()) {
        std::cout << "Stabilizer is fault. Trying to reset the fault..." << std::endl;
        if (!stabilizer->faultReset()) {
            std::cout << "Stabilizer fault reset can't be performed" << std::endl;
            return -1;
        }
        std::cout << "Stabilizer fault reset performed" << std::endl;
    }

    if (!stabilizer->shutdown()) {
        std::cout << "Stabilizer shutdown can't be performed" << std::endl;
        return -1;
    }
    std::cout << "Stabilizer shutdown performed" << std::endl;

    if (!stabilizer->enableOperation()) {
        std::cout << "Stabilizer can't be enabled for operations" << std::endl;
        return -1;
    }
    std::cout << "Stabilizer enabled for operations" << std::endl;


    std::cout << "---------------- Shielding ----------------" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));

    if (!shielding->setMaxCurrent(50000)) {
        std::cout << "Shielding maximum current can't be set" << std::endl;
        return -1;
    }
    std::cout << "Shielding maximum current set" << std::endl;

    if (!shielding->setModeOfOperation(ProfileVelocityMode)) {
        std::cout << "Shielding can't enter in Profile Velocity Mode" << std::endl;
        return -1;
    }
    std::cout << "Shielding is in Profile Velocity Mode" << std::endl;

    std::optional<bool> optionalShieldingValue = shielding->inFault();
    if (!optionalShieldingValue) {
        std::cout << "Communication problem with the harmonic drive 2" << std::endl;
        return -1;
    }
    if (optionalShieldingValue.value()) {
        std::cout << "Shielding is fault. Trying to reset the fault..." << std::endl;
        if (!shielding->faultReset()) {
            std::cout << "Shielding fault reset can't be performed" << std::endl;
            return -1;
        }
        std::cout << "Shielding fault reset performed" << std::endl;
    }

    if (!shielding->shutdown()) {
        std::cout << "Shielding shutdown can't be performed" << std::endl;
        return -1;
    }
    std::cout << "Shielding shutdown performed" << std::endl;

    if (!shielding->enableOperation()) {
        std::cout << "Shielding can't be enabled for operations" << std::endl;
        return -1;
    }
    std::cout << "Shielding enabled for operations" << std::endl;



    int32_t velSled = -1008;
    int32_t velHD1 = -17476;
    int32_t velHD2 = 34952;
    linearSled->setProfileAcceleration(2016);
    HD1->setProfileAcceleration(34952);
    HD2->setProfileAcceleration(69905);
    linearSled->setProfileDeceleration(2016);
    HD1->setProfileDeceleration(34952);
    HD2->setProfileDeceleration(69905);
    linearSled->setQuickstopDeceleration(2016);
    HD1->setQuickstopDeceleration(69905);
    HD2->setQuickstopDeceleration(69905);

    for (int i = 1; i < 100; i++) {
        std::cout << "--------------- Cycle " << i << "---------------" << std::endl;
        std::cout << "Linear Sled statusword: " << linearSled->getStatusWord().value() << std::endl;
        std::cout << "Harmonic drive 1 statusword: " << HD1->getStatusWord().value() << std::endl;
        std::cout << "Harmonic drive 2 statusword: " << HD2->getStatusWord().value() << std::endl;

        // Invert speed for next cycle
        velSled = -velSled;
        velHD1 = -velHD1;
        velHD2 = -velHD2;

        if (linearSled->setVelocity(velSled)) {
            std::cout << "Linear Sled velocity: " << velSled << std::endl;
        }
        if (HD1->setVelocity(velHD1)) {
            std::cout << "Harmonic drive 1 velocity: " << velHD1 << std::endl;
        }
        if (HD2->setVelocity(velHD2)) {
            std::cout << "Harmonic drive 2 velocity: " << velHD2 << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::seconds(3));

        if (linearSled->setVelocity(0)) {
            std::cout << "Linear Sled velocity: 0" << std::endl;
        }
        if (HD1->setVelocity(0)) {
            std::cout << "Harmonic drive 1 velocity: 0" << std::endl;
        }
        if (HD2->setVelocity(0)) {
            std::cout << "Harmonic drive 2 velocity: 0" << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::seconds(3));
    }

    std::this_thread::sleep_for(std::chrono::seconds(4));
    if (!BLMWagon->deinitialize()) {
        std::cout << "Could not deinitialize the BLM Wagon motors" << std::endl;
        return -1;
    }
    std::cout << "Manager and Motors deinitialized" << std::endl;
    return 0;
}
