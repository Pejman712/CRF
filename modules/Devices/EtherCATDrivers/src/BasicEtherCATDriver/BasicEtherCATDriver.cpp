/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Natalija Topalovic CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#include <memory>
#include <optional>
#include <cstdint>

#include "EtherCATDrivers/BasicEtherCATDriver/BasicEtherCATDriver.hpp"

namespace crf::devices::ethercatdrivers {

BasicEtherCATDriver::BasicEtherCATDriver(
    std::shared_ptr<EtherCATMaster> master, const uint16_t& id):
    initialized_(false),
    ioMapBound_(false),
    master_(master),
    id_(id),
    logger_("BasicEtherCATDriver " + std::to_string(id)) {
    logger_->debug("CTor");
}

BasicEtherCATDriver::~BasicEtherCATDriver() {
    logger_->debug("DTor");
    if (initialized_) deinitialize();
}

bool BasicEtherCATDriver::initialize() {
    logger_->debug("initialize");
    if (initialized_) {
        logger_->warn("Already initialized");
        return false;
    }

    if (!master_->slaveCommunicationCheck(id_)) {
        logger_->error("The master did not initialize the slave correctly");
        return false;
    }

    if (!bindIOMap()) {
        logger_->error("Failed to bind IO map");
        return false;
    }

    ioMapBound_ = true;
    initialized_ = true;
    return true;
}

bool BasicEtherCATDriver::deinitialize() {
    logger_->debug("deinitialize");
    if (!initialized_) {
        logger_->warn("Already deinitialized");
        return false;
    }

    initialized_ = false;
    return true;
}

int BasicEtherCATDriver::getID() const {
    return id_;
}

std::optional<uint16_t> BasicEtherCATDriver::getEtherCatState() const {
    if (!initialized_) {
        logger_->error("Motor is not initialized");
        return std::nullopt;
    }
    return master_->readSlaveState(id_);
}

bool BasicEtherCATDriver::isAlive() const {
    if (!initialized_) {
        logger_->error("Motor is not initialized");
        return false;
    }
    return master_->slaveCommunicationCheck(id_);
}

// Protected

bool BasicEtherCATDriver::bindIOMap() {
    logger_->info("No IO Map bound");
    return true;
}

}  // namespace crf::devices::ethercatdrivers
