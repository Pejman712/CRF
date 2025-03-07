/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Francesco Riccardi CERN EN/SMM/MRO 2020
 * Contributor: Alejandro Diaz Rosales BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#include <string>
#include <memory>
#include <vector>
#include <chrono>
#include <thread>
#include <optional>

#include "EtherCATDevices/TIMRobotArmWagonMotors/TIMRobotArmWagonMotors.hpp"
#include "EtherCATDevices/EtherCATMotor.hpp"

namespace crf {
namespace devices {
namespace ethercatdevices {

TIMRobotArmWagonMotors::TIMRobotArmWagonMotors(const std::string& ifname, int ioMapSize,
    std::shared_ptr<crf::devices::ethercatdevices::ISoemApi> soemApi) :
    logger_("TIMRobotArmWagonMotors"),
    portName_(ifname),
    ioMapSize_(ioMapSize),
    soemApi_(soemApi),
    initialized_(false),
    ECManager_(nullptr),
    motorsMap_() {
    logger_->debug("CTor");
}

TIMRobotArmWagonMotors::~TIMRobotArmWagonMotors() {
    logger_->debug("DTor");
    if (initialized_) {
        deinitialize();
    }
}

bool TIMRobotArmWagonMotors::initialize() {
    logger_->debug("initialize");
    if (initialized_) {
        logger_->warn("Already initialized");
        return false;
    }

    auto tempManager = std::make_shared<devices::ethercatdevices::EtherCATManager>(portName_,
        numberOfMotors, ioMapSize_, etherCATCycleTime, soemApi_);
    std::vector<std::shared_ptr<devices::ethercatdevices::IEtherCATMotor>> tempVect;
    tempVect.push_back(std::make_shared<devices::ethercatdevices::EtherCATMotor>(1, tempManager));
    tempVect.push_back(std::make_shared<devices::ethercatdevices::EtherCATMotor>(2, tempManager));
    tempVect.push_back(std::make_shared<devices::ethercatdevices::EtherCATMotor>(3, tempManager));
    tempVect.push_back(std::make_shared<devices::ethercatdevices::EtherCATMotor>(4, tempManager));
    tempVect.push_back(std::make_shared<devices::ethercatdevices::EtherCATMotor>(5, tempManager));

    if (!tempManager->initialize()) {
        logger_->error("Failed to initialize the EtherCAT Manager");
        return false;
    }
    logger_->info("EtherCAT Manager initialized");

    for (int i=0; i < numberOfMotors; i++) {
        if (!tempVect[i]->initialize()) {
            logger_->error("Failed to initialize motor {}", i+1);
            return false;
        }
        logger_->info("Motor {} initialized", i+1);
    }
    if (!tempManager->configureIOMap()) {
        logger_->error("Can't configure EtherCAT IOMap");
        return false;
    }
    for (int i=0; i < numberOfMotors; i++) {
        if (!tempVect[i]->bindPDOs()) {
            logger_->error("Can't bind PDO for Motor {}", i+1);
            return false;
        }
    }
    if (!tempManager->enterOp()) {
        logger_->error("Manager can't enter in operation");
        return false;
    }
    ECManager_ = tempManager;
    motorsMap_[1] = tempVect[0];
    motorsMap_[2] = tempVect[1];
    motorsMap_[3] = tempVect[2];
    motorsMap_[4] = tempVect[3];
    motorsMap_[5] = tempVect[4];

    initialized_ = true;
    return true;
}

bool TIMRobotArmWagonMotors::deinitialize() {
    logger_->debug("deinitialize");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }

    bool managedToDeinitialize = true;

    for (int i=1; i < numberOfMotors+1; i++) {
        if (motorsMap_[i]->deinitialize()) {
            logger_->info("Motor {} deinitialized", i);
        } else {
            logger_->error("Failed to deinitialize motor {}", i);
            managedToDeinitialize = false;
        }
    }

    if (ECManager_->deinitialize()) {
        logger_->info("EtherCAT Manager deinitialized");
    } else {
        logger_->error("Failed to deinitialize the EtherCAT Manager");
        managedToDeinitialize = false;
    }

    if (managedToDeinitialize) {
        initialized_ = false;
    }
    return managedToDeinitialize;
}

std::optional<std::shared_ptr<devices::ethercatdevices::IEtherCATMotor>>
    TIMRobotArmWagonMotors::getHarmonicDrive1() {
    if (initialized_) {
        return motorsMap_[harmonicDrive1Index];
    }
    logger_->error("EtherCATMotors are not initialized. Can't retrieve the motor");
    return std::nullopt;
}

std::optional<std::shared_ptr<devices::ethercatdevices::IEtherCATMotor>>
    TIMRobotArmWagonMotors::getHarmonicDrive2() {
    if (initialized_) {
        return motorsMap_[harmonicDrive2Index];
    }
    logger_->error("EtherCATMotors are not initialized. Can't retrieve the motor");
    return std::nullopt;
}

std::optional<std::shared_ptr<devices::ethercatdevices::IEtherCATMotor>>
    TIMRobotArmWagonMotors::getLinearSled() {
    if (initialized_) {
        return motorsMap_[LinearSledIndex];
    }
    logger_->error("EtherCATMotors are not initialized. Can't retrieve the motor");
    return std::nullopt;
}

std::optional<std::shared_ptr<devices::ethercatdevices::IEtherCATMotor>>
    TIMRobotArmWagonMotors::getStabilizer() {
    if (initialized_) {
        return motorsMap_[stabilizerIndex];
    }
    logger_->error("EtherCATMotors are not initialized. Can't retrieve the motor");
    return std::nullopt;
}

std::optional<std::shared_ptr<devices::ethercatdevices::IEtherCATMotor>>
    TIMRobotArmWagonMotors::getShielding() {
    if (initialized_) {
        return motorsMap_[shieldingIndex];
    }
    logger_->error("EtherCATMotors are not initialized. Can't retrieve the motor");
    return std::nullopt;
}

std::optional<std::shared_ptr<devices::ethercatdevices::EtherCATManager>>
    TIMRobotArmWagonMotors::getManager() {
    if (initialized_) {
        return ECManager_;
    }
    logger_->error("EtherCATMotors are not initialized. Can't retrieve the manager");
    return std::nullopt;
}

}  // namespace ethercatdevices
}  // namespace devices
}  // namespace crf
