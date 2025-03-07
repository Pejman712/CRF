/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Francesco Riccardi CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <chrono>
#include <inttypes.h>

#include "EtherCATDevices/EtherCATManager.hpp"
#include "EtherCATDevices/IEtherCATMotor.hpp"
#include "EtherCATDevices/EtherCATMotor.hpp"
#include "EtherCATDevices/EtherCATDef.hpp"

using crf::devices::ethercatdevices::EtherCATManager;
using crf::devices::ethercatdevices::IEtherCATMotor;
using crf::devices::ethercatdevices::EtherCATMotor;
using crf::devices::ethercatdevices::modesofoperation::ProfileVelocityMode;

int main() {
    std::string ifname = "eno2";
    int IOMapSize = 4096;
    int nSlaves = 4;
    std::shared_ptr<EtherCATManager> manager = std::make_shared<EtherCATManager>(
        ifname, nSlaves, IOMapSize, 1000);
    std::unique_ptr<IEtherCATMotor> motor = std::make_unique<EtherCATMotor>(3, manager);


    // ----- MANAGER AND MOTOR INITIALIZATION -------
    if (manager->initialize()) {
        printf("Manager initialized\n");
    } else {
        printf("ERROR: Cannot initiliaze Manager\n");
        return -1;
    }

    if (motor->initialize()) {
        printf("Motor initialized\n");
    } else {
        printf("ERROR: Cannot initiliaze Motor\n");
        return -1;
    }

    if (manager->configureIOMap()) {
        printf("IOMap configured\n");
    } else {
        printf("ERROR: Cannot configure IOMap\n");
        return -1;
    }

    if (motor->bindPDOs()) {
        printf("PDO registers linked to the motor\n\n\n\n");
    } else {
        printf("ERROR: Cannot link PDO registers to the motor\n");
        return -1;
    }
    // ----------------------------------------------


    std::this_thread::sleep_for(std::chrono::seconds(1));


    // ---- MANAGER IN OPERATION (NMT - START PDO COMMUNICATION -----
    if (manager->enterOp()) {
        printf("Manager in Operation Mode \n\n\n\n");
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
    if (motor->setModeOfOperation(ProfileVelocityMode)) {
        printf("Motor in Profile Velocity Mode \n");
    } else {
        printf("Motor can't enter in Profile Velocity Mode \n");
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
        printf("Motor enabled for operations \n\n\n\n");
    } else {
        printf("ERROR: Motor can't be enabled for operations \n");
        return -1;
    }
    // ------------------------------


    std::this_thread::sleep_for(std::chrono::seconds(1));


    // ---- Velocity Setpoint (Velocity) ------
    printf("Test 1: Velocity Setpoint (Velocity) \n");
    printf("Velocity: 1000 [cnt/s] \nAcceleration: 200 [cnt/s2] \n"
    "Deceleration: 200[cnt/s2]\n\n\n");
    motor->setProfileAcceleration(200);
    motor->setProfileDeceleration(200);
    motor->setVelocity(1000);
    std::this_thread::sleep_for(std::chrono::seconds(15));
    motor->stop();
    // ----------------------------------------


    std::this_thread::sleep_for(std::chrono::seconds(1));


    // ---- Velocity Setpoint (Velocity, Acceleration) ----
    printf("Test 2: Velocity Setpoint (Velocity, Acceleration) \n");
    printf("Velocity: 1000 [cnt/s] \nAcceleration: 2000 [cnt/s2] \n"
    "Deceleration: 200[cnt/s2]\n\n\n");
    motor->setVelocity(1000, 2000);
    std::this_thread::sleep_for(std::chrono::seconds(15));
    motor->stop();
    // ------------------------------------------------


    std::this_thread::sleep_for(std::chrono::seconds(1));


    // ---- Velocity Setpoint (Velocity, Acceleration, Deceleration) ----
    printf("Test 3: Velocity Setpoint (Velocity, Acceleration, Deceleration) \n");
    printf("Velocity: 1000 [cnt/s] \nAcceleration: 2000 [cnt/s2] \n"
    "Deceleration: 2000[cnt/s2]\n\n\n");
    motor->setVelocity(1000, 2000, 2000);
    std::this_thread::sleep_for(std::chrono::seconds(10));
    motor->stop();
    // ------------------------------------------------


    printf("\n");
    if (motor->deinitialize()) {
        printf("Motor deinitialized \n");
    } else {
        printf("ERROR: Motor can't be deinitialized \n");
        return -1;
    }

    if (manager->deinitialize()) {
        printf("Manager deinitialized \n");
    } else {
        printf("ERROR: Manager can't be deinitialized \n");
        return -1;
    }

    return 0;
}
