/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playan Garai CERN EN/SMM/MRO 2023
 *
 *  ==================================================================================================
 */

#include <memory>

#include "LinearStage/EtherCATLinearActuator/EtherCATLinearActuator.hpp"

namespace crf::actuators::linearactuator {

EtherCATLinearActuator::EtherCATLinearActuator(
    std::shared_ptr<crf::devices::ethercatdevices::IEtherCATMotor> motor):
    motor_(motor),
    initialized_(false),
    logger_("EtherCATLinearActuator") {
    logger_->debug("CTor");
}

EtherCATLinearActuator::~EtherCATLinearActuator() {
    logger_->debug("DTor");
}

bool EtherCATLinearActuator::initialize() {
    logger_->debug("initialize");
    if (initialized_) {
        logger_->error("Already initialized");
        return false;
    }
    initialized_ = true;
    return true;
}

bool EtherCATLinearActuator::deinitialize() {
    logger_->debug("deinitialize");
    if (!initialized_) {
        logger_->error("Already deinitialized");
        return false;
    }
    initialized_ = false;
    return true;
}

crf::expected<bool> EtherCATLinearActuator::setPosition(const double& position) {
    if (!initialized_) {
        logger_->error("Not initialized");
        return crf::Code::NotInitialized;
    }
    return motor_->setPosition(position, velocity_, acceleration_, deceleration_, false);
}

crf::expected<bool> EtherCATLinearActuator::setVelocity(const double& velocity) {
    if (!initialized_) {
        logger_->error("Not initialized");
        return crf::Code::NotInitialized;
    }
    return motor_->setVelocity(velocity, acceleration_, deceleration_);
}

crf::expected<double> EtherCATLinearActuator::getPosition() const {
    if (!initialized_) {
        logger_->error("Not initialized");
        return crf::Code::NotInitialized;
    }
    auto res = motor_->getPosition();
    if (!res) return crf::Code::NotInitialized;
    return res.value();
}

crf::expected<double> EtherCATLinearActuator::getVelocity() const {
    if (!initialized_) {
        logger_->error("Not initialized");
        return crf::Code::NotInitialized;
    }
    auto res =  motor_->getVelocity();
    if (!res) return crf::Code::NotInitialized;
    return res.value();
}

}  // namespace crf::actuators::linearactuator
