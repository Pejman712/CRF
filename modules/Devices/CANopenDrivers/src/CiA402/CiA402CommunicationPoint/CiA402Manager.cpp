/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Sebastien Collomb CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */

#include <nlohmann/json.hpp>

#include "CANopenDrivers/CiA402/CiA402CommunicationPoint/CiA402Manager.hpp"

namespace crf::devices::canopendrivers {

CiA402Manager::CiA402Manager(
    std::shared_ptr<crf::devices::canopendrivers::ICiA402Driver> cia402driver,
    const std::chrono::milliseconds& initializationTimeout,
    const std::chrono::milliseconds& controlAccessTimeout) :
    DeviceManagerWithPriorityAccess(cia402driver, initializationTimeout, controlAccessTimeout),
    cia402driver_(cia402driver) {
    logger_ = crf::utility::logger::EventLogger("CiA402Manager");
    logger_->debug("CTor");
}

CiA402Manager::~CiA402Manager() {
    logger_->debug("DTor");
}

crf::expected<bool> CiA402Manager::setProfilePosition(
    const uint32_t& priority,
    const double& position,
    const double& velocity,
    const double& acceleration,
    const double& deceleration,
    const PositionReference& positionReference) {
    if (!checkCommandPriority(priority)) return crf::Code::Forbidden;
    return cia402driver_->setProfilePosition(
        position, velocity, acceleration, deceleration, positionReference);
}

crf::expected<bool> CiA402Manager::setProfileVelocity(
    const uint32_t& priority,
    const double& velocity,
    const double& acceleration,
    const double& deceleration) {
    if (!checkCommandPriority(priority)) return crf::Code::Forbidden;
    return cia402driver_->setProfileVelocity(velocity, acceleration, deceleration);
}

crf::expected<bool> CiA402Manager::setProfileTorque(
    const uint32_t& priority,
    const double& torque) {
    if (!checkCommandPriority(priority)) return crf::Code::Forbidden;
    return cia402driver_->setProfileTorque(torque);
}

crf::expected<bool> CiA402Manager::setVelocity(
    const uint32_t& priority,
    const double& velocity,
    const double& deltaSpeedAcc,
    const double& deltaTimeAcc,
    const double& deltaSpeedDec,
    const double& deltaTimeDec) {
    if (!checkCommandPriority(priority)) return crf::Code::Forbidden;
    return cia402driver_->setVelocity(
        velocity, deltaSpeedAcc, deltaTimeAcc, deltaSpeedDec, deltaTimeDec);
}

crf::expected<bool> CiA402Manager::setMaximumTorque(
    const uint32_t& priority,
    const double& maxTorque) {
    if (!checkCommandPriority(priority)) return crf::Code::Forbidden;
    return cia402driver_->setMaximumTorque(maxTorque);
}

crf::expected<bool> CiA402Manager::setInterpolatedPosition(
    const uint32_t& priority,
    const double& position,
    const double& velocity,
    const double& acceleration,
    const double& deceleration) {
    if (!checkCommandPriority(priority)) return crf::Code::Forbidden;
    return cia402driver_->setInterpolatedPosition(position, velocity, acceleration, deceleration);
}

crf::expected<bool> CiA402Manager::setModeOfOperation(
    const uint32_t& priority,
    const ModeOfOperation& mode) {
    if (!checkCommandPriority(priority)) return crf::Code::Forbidden;
    return cia402driver_->setModeOfOperation(mode);
}

crf::expected<bool> CiA402Manager::setCyclicPosition(
    const uint32_t& priority,
    const double& position,
    const double& posOffset,
    const double& velOffset,
    const double& torOffset) {
    if (!checkCommandPriority(priority)) return crf::Code::Forbidden;
    return cia402driver_->setCyclicPosition(position, posOffset, velOffset, torOffset);
}

crf::expected<bool> CiA402Manager::setCyclicVelocity(
    const uint32_t& priority,
    const double& velocity,
    const double& velOffset,
    const double& torOffset) {
    if (!checkCommandPriority(priority)) return crf::Code::Forbidden;
    return cia402driver_->setCyclicVelocity(velocity, velOffset, torOffset);
}

crf::expected<bool> CiA402Manager::setCyclicTorque(
    const uint32_t& priority,
    const double& torque,
    const double& torOffset) {
    if (!checkCommandPriority(priority)) return crf::Code::Forbidden;
    return cia402driver_->setCyclicTorque(torque, torOffset);
}

crf::expected<bool> CiA402Manager::quickStop(const uint32_t& priority) {
    if (!checkCommandPriority(priority)) return crf::Code::Forbidden;
    cia402driver_->quickStop();
    return true;
}

crf::expected<bool> CiA402Manager::stop(const uint32_t& priority) {
    if (!checkCommandPriority(priority)) return crf::Code::Forbidden;
    cia402driver_->stop();
    return true;
}

crf::expected<bool> CiA402Manager::resetFault(const uint32_t& priority) {
    if (!checkCommandPriority(priority)) return crf::Code::Forbidden;
    cia402driver_->resetFault();
    return true;
}

crf::expected<bool> CiA402Manager::resetQuickStop(const uint32_t& priority) {
    if (!checkCommandPriority(priority)) return crf::Code::Forbidden;
    cia402driver_->resetQuickStop();
    return true;
}

nlohmann::json CiA402Manager::getStatus() {
    logger_->debug("getStatus");
    nlohmann::json statusJSON;

    if (initializeDevice()) {
        statusJSON["status"] = "initialize";
    } else {
        statusJSON["status"] = "deinitialize";
    }

    statusJSON["position"] = cia402driver_->getPosition();
    statusJSON["velocity"] = cia402driver_->getVelocity();
    statusJSON["torque"] = cia402driver_->getTorque();
    statusJSON["maxTorque"] = cia402driver_->getMaximumTorque();
    statusJSON["inFault"] = cia402driver_->inFault();
    statusJSON["inQuickStop"] = cia402driver_->inQuickStop();

    statusJSON["statusWord"] = statusWordToString.at(cia402driver_->getStatusWord());
    statusJSON["modeOfOperation"] = cia402driver_->getModeOfOperation();

    statusJSON["motorStatus"] = cia402driver_->getMotorStatus();

    return statusJSON;
}

}  // namespace crf::devices::canopendrivers
