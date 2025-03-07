/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <chrono>

#include "EtherCATDevices/EtherCATManager.hpp"
#include "EtherCATDevices/IEtherCATMotor.hpp"
#include "EtherCATDevices/EtherCATMotor.hpp"
#include "EtherCATDevices/EtherCATDef.hpp"

#include "LinearStage/EtherCATLinearActuator/EtherCATLinearActuator.hpp"

using crf::devices::ethercatdevices::EtherCATManager;
using crf::devices::ethercatdevices::IEtherCATMotor;
using crf::devices::ethercatdevices::EtherCATMotor;
using crf::devices::ethercatdevices::modesofoperation::ProfilePositionMode;

using crf::actuators::linearactuator::EtherCATLinearActuator;

int main(int argc, char* argv[]) {
    if (argc != 4) {
        std::cout << "Too few arguments" << std::endl;
        std::cout << "[1] ethercat port " << std::endl;
        std::cout << "[2] velocity (500000 or -500000) " << std::endl;
        std::cout << "[3] time (in seconds, 1s, 2s etc) " << std::endl;
        return -1;
    }


    std::string ifname = argv[1];
    int velocity = std::stoi(argv[2]);
    int seconds = std::stoi(argv[3]);

    int IOMapSize = 4096;
    int nSlaves = 1;
    std::shared_ptr<EtherCATManager> manager = std::make_shared<EtherCATManager>(ifname,
    nSlaves, IOMapSize, 1000);
    std::shared_ptr<IEtherCATMotor> motor = std::make_shared<EtherCATMotor>(1, manager);

    std::unique_ptr<EtherCATLinearActuator> linearAct =
        std::make_unique<EtherCATLinearActuator>(motor);

    // ----- MANAGER AND MOTOR INITIALIZATION -------
    if (manager->initialize()) {
        printf("Manager initialized\n");
    } else {
        printf("ERROR: Cannot initiliaze Manager\n");
        return -1;
    }

    if (motor->initialize()) {
        printf("Linear Actuator initialized\n");
    } else {
        printf("ERROR: Cannot initiliaze Linear Actuator\n");
        return -1;
    }

    if (manager->configureIOMap()) {
        printf("IOMap configured\n");
    } else {
        printf("ERROR: Cannot configure IOMap\n");
        return -1;
    }

    if (motor->bindPDOs()) {
        printf("PDO registers linked to the motor\n");
    } else {
        printf("ERROR: Cannot link PDO registers to the motor\n");
        return -1;
    }
    // ----------------------------------------------


    std::this_thread::sleep_for(std::chrono::seconds(1));


    // ---- MANAGER IN OPERATION (NMT - START PDO COMMUNICATION -----
    if (manager->enterOp()) {
        printf("Manager in Operation Mode \n");
    } else {
        printf("Manager can't enter in Operation Mode \n");
        return -1;
    }
    // --------------------------------------------------------------


    std::this_thread::sleep_for(std::chrono::seconds(1));


    // ---- CONFIGURATION (MANDATORY) ----
    if (motor->setMaxCurrent(50000)) {
        printf("Set motor max current\n");  // ALWAYS TO SET
    } else {
        printf("ERROR: Max current can't be set \n");
        return -1;
    }

    if (motor->setMaxTorque(50000)) {
        printf("Set motor max current\n");  // ALWAYS TO SET
    } else {
        printf("ERROR: Max current can't be set \n");
        return -1;
    }
    // -------------------------------------


    std::this_thread::sleep_for(std::chrono::seconds(1));


    // ---- MOTOR IN OPERATION ----
    if (motor->setModeOfOperation(ProfilePositionMode)) {
        printf("Motor in Profile Position Mode \n");
    } else {
        printf("Motor can't enter in Profile Position Mode \n");
        return -1;
    }

    std::optional<bool> retval = motor->inFault();
    if (!retval) {
        printf("ERROR: Communication problem with the motor \n");
        return -1;
    }
    if (retval.value()) {
        printf("Motor in fault. Trying to reset the fault... \n");
        if (motor->faultReset()) {
            printf("Fault reset performed \n");
        } else {
            printf("ERROR: Fault reset can't be performed \n");
            return -1;
        }
    }

    if (motor->shutdown()) {
        printf("Motor shutdown performed \n");
    } else {
        printf("ERROR: Shutdown can't be performed \n");
        return -1;
    }

    if (motor->enableOperation()) {
        printf("Motor enabled for operations \n");
    } else {
        printf("ERROR: Motor can't be enabled for operations \n");
        return -1;
    }
    // ------------------------------

    std::this_thread::sleep_for(std::chrono::seconds(1));

    std::puts("Starting the linear actuator!");

    std::cout << "Position: " << linearAct->getPosition().value() << std::endl;
    std::cout << "Velocity: " << linearAct->getVelocity().value() << std::endl;

    linearAct->setVelocity(velocity);
    std::this_thread::sleep_for(std::chrono::seconds(seconds));

    std::cout << "Position: " << linearAct->getPosition().value() << std::endl;
    std::cout << "Velocity: " << linearAct->getVelocity().value() << std::endl;

    return 0;
}
