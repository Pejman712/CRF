/* © Copyright CERN 2020.  All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Francesco Riccardi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
*/

#include <memory>
#include <string>
#include <vector>

#include "RobotBase/EtherCATRobotBase.hpp"
#include "EtherCATDevices/EtherCATMotor.hpp"

namespace crf {
namespace actuators {
namespace robotbase {
EtherCATRobotBase::EtherCATRobotBase(const std::string& ifname,
int IOMapSize, std::shared_ptr<ISoemApi> soemApi) :
    logger_("EtherCATRobotBase"),
    portName_(ifname),
    initialized_(false),
    soemApi_(soemApi),
    IOMapSize_(IOMapSize) {
        logger_->info("CTor");
}

EtherCATRobotBase::~EtherCATRobotBase() {
    logger_->info("DTor");
    if (initialized_) {
        deinitialize();
    }
}

bool EtherCATRobotBase::initialize() {
    logger_->info("initialize");

    if (initialized_) {
        logger_->warn("Already initialized");
        return false;
    }

    auto tempManager = std::make_shared<devices::ethercatdevices::EtherCATManager>
    (portName_, 4, IOMapSize_, 1000, soemApi_);
    std::vector<std::shared_ptr<devices::ethercatdevices::IEtherCATMotor>> tempVect;
    tempVect.push_back(std::make_shared<devices::ethercatdevices::EtherCATMotor>(1, tempManager));
    tempVect.push_back(std::make_shared<devices::ethercatdevices::EtherCATMotor>(2, tempManager));
    tempVect.push_back(std::make_shared<devices::ethercatdevices::EtherCATMotor>(3, tempManager));
    tempVect.push_back(std::make_shared<devices::ethercatdevices::EtherCATMotor>(4, tempManager));

    sleep(1);

    if (tempManager->initialize()) {
        logger_->info("Manager initialized");
    } else {
        logger_->error("Cannot initiliaze Manager");
        return false;
    }

    for (int i=0; i < 4; i++) {
        sleep(1);
        if (tempVect[i]->initialize()) {
            logger_->info("Motor of wheel {} initialized", i + 1);
        } else {
            logger_->error("Cannot initialize Motor of wheel {}", i + 1);
            return false;
        }
    }

    sleep(1);

    if (!tempManager->configureIOMap()) {
        logger_->error("Cannot configure EtherCAT IOMap");
        return false;
    }

    sleep(1);

    for (int i=0; i < 4; i++) {
        if (!tempVect[i]->bindPDOs()) {
            logger_->error("Cannot bind PDO for Motor of wheel {}", i + 1);
            return false;
        }
    }

    sleep(1);

    if (!tempManager->enterOp()) {
        logger_->error("Manager can't enter in operation");
        return false;
    }

    sleep(1);

    ECManager_ = tempManager;
    motorsMap_[1] = tempVect[0];
    motorsMap_[2] = tempVect[1];
    motorsMap_[3] = tempVect[2];
    motorsMap_[4] = tempVect[3];

    sleep(1);

    initialized_ = true;
    return true;
}

bool EtherCATRobotBase::deinitialize() {
    logger_->info("deinitialize");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }

    bool managedToDeinitialize = true;

    for (int i=1; i <= 4; i++) {
        if (motorsMap_[i]->deinitialize()) {
            logger_->info("Motor of wheel {} deinitialized", i);
        } else {
            logger_->error("Motor of wheel {} can't be deinitialized", i);
            managedToDeinitialize = false;
        }
    }

    if (ECManager_->deinitialize()) {
        logger_->info("Manager deinitialized");
    } else {
        logger_->error("Manager can't be deinitialized");
        managedToDeinitialize = false;
    }
    if (managedToDeinitialize) initialized_ = false;
    return managedToDeinitialize;
}

boost::optional<std::shared_ptr<devices::ethercatdevices::IEtherCATMotor>>
EtherCATRobotBase::getMotorFrontLeft() {
    if (initialized_) {
        return motorsMap_[2];
    }
    logger_->error("EtherCAT Robot Base not initialized");
    return boost::none;
}

boost::optional<std::shared_ptr<devices::ethercatdevices::IEtherCATMotor>>
EtherCATRobotBase::getMotorFrontRight() {
    if (initialized_) {
        return motorsMap_[1];
    }
    logger_->error("EtherCAT Robot Base not initialized");
    return boost::none;
}

boost::optional<std::shared_ptr<devices::ethercatdevices::IEtherCATMotor>>
EtherCATRobotBase::getMotorBackLeft() {
    if (initialized_) {
        return motorsMap_[4];
    }
    logger_->error("EtherCAT Robot Base not initialized");
    return boost::none;
}

boost::optional<std::shared_ptr<devices::ethercatdevices::IEtherCATMotor>>
EtherCATRobotBase::getMotorBackRight() {
    if (initialized_) {
        return motorsMap_[3];
    }
    logger_->error("EtherCAT Robot Base not initialized");
    return boost::none;
}

boost::optional<std::shared_ptr<devices::ethercatdevices::EtherCATManager>>
EtherCATRobotBase::getManager() {
    if (initialized_) {
        return ECManager_;
    }
    logger_->error("EtherCAT Robot Base not initialized");
    return boost::none;
}

}  // namespace robotbase
}  // namespace robots
}  // namespace cern
