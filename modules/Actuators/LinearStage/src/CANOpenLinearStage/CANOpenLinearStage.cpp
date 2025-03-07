/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <limits>
#include <boost/optional.hpp>
#include <memory>

#include "LinearStage/CANOpenLinearStage/CANOpenLinearStage.hpp"

namespace crf::actuators::linearstage {

CANOpenLinearStage::CANOpenLinearStage(
    std::shared_ptr<devices::canopendevices::ICANOpenMotor> motor,
    std::shared_ptr<LinearStageConfiguration> configuration) :
        logger_("CANOpenLinearStage"),
        motor_(motor),
        configuration_(configuration),
        initialized_(false) {
            logger_->debug("CTor");
}

CANOpenLinearStage::~CANOpenLinearStage() {
    logger_->debug("DTor");
    if (initialized_)
        deinitialize();
}

bool CANOpenLinearStage::initialize() {
    logger_->debug("initialize");
    if (initialized_) {
        logger_->warn("Already initialized");
        return false;
    }

    if (!motor_->initialize()) {
        logger_->error("Failed to inizialize the motor");
        return false;
    }

    if (motor_->inFault() && !motor_->faultReset()) {
        logger_->warn("Could not reset the fault state for the motor");
        return false;
    }

    if (!motor_->shutdown()) {
        logger_->error("Could not shuwdown motor");
        return false;
    }

    if (!motor_->enableOperation()) {
        logger_->error("Could not enable motor");
        return false;
    }

    if (!motor_->setProfileAcceleration(
        configuration_->getMaximumAcceleration()*configuration_->getLinearToRotationRatio())) {
            logger_->error("Could not set maximum acceleration");
            return false;
    }

    if (!motor_->setProfileDeceleration(
        configuration_->getMaximumDeceleration()*configuration_->getLinearToRotationRatio())) {
            logger_->error("Could not set maximum deceleration");
            return false;
    }

    if (!motor_->setProfileVelocity(
        configuration_->getMaximumVelocity()*configuration_->getLinearToRotationRatio())) {
            logger_->error("Could not set maximum velocity");
            return false;
    }

    int32_t minimumPosition = std::numeric_limits<int32_t>::min();
    int32_t maximumPosition = std::numeric_limits<int32_t>::max();

    if (configuration_->hasMinimumPosition()) {
        minimumPosition = static_cast<int32_t>(
            configuration_->getMinimumPosition()*configuration_->getLinearToRotationRatio());
    }

    if (configuration_->hasMaximumPosition()) {
        maximumPosition = static_cast<int32_t>(
            configuration_->getMaximumPosition()*configuration_->getLinearToRotationRatio());
    }

    if (!motor_->setPositionLimits({minimumPosition, maximumPosition})) {
        logger_->error("Could not set position limits");
        return false;
    }

    initialized_ = true;
    return true;
}

bool CANOpenLinearStage::deinitialize() {
    logger_->debug("deinitialize");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }

    if (!motor_->deinitialize()) {
        logger_->error("Failed to deinizialize the motor");
        return false;
    }

    initialized_ = false;
    return true;
}

bool CANOpenLinearStage::setTargetPosition(float position) {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }

    int32_t posInt = static_cast<int32_t>(position*configuration_->getLinearToRotationRatio());
    return motor_->setPosition(posInt, false);
}

bool CANOpenLinearStage::setTargetVelocity(float velocity) {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }

    int32_t velInt = static_cast<int32_t>(velocity*configuration_->getLinearToRotationRatio());
    return motor_->setVelocity(velInt);
}

boost::optional<float> CANOpenLinearStage::getActualPosition()  {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return boost::none;
    }

    auto posBoost = motor_->getPosition();
    if (!posBoost) {
        return boost::none;
    }

    return static_cast<float>(posBoost.value())*configuration_->getRotationToLinearRatio();
}

boost::optional<float> CANOpenLinearStage::getActualVelocity()  {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return boost::none;
    }

    auto velBoost = motor_->getVelocity();
    if (!velBoost) {
        return boost::none;
    }

    return static_cast<float>(velBoost.value())*configuration_->getRotationToLinearRatio();
}

std::shared_ptr<LinearStageConfiguration> CANOpenLinearStage::getConfiguration() {
    return configuration_;
}

}  // namespace crf::actuators::linearstage
