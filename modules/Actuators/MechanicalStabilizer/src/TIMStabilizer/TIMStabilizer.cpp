/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Francesco Riccardi CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#include <thread>
#include <chrono>

#include "MechanicalStabilizer/TIMStabilizer/TIMStabilizer.hpp"
#include "EtherCATDevices/EtherCATDef.hpp"

namespace crf::actuators::mechanicalstabilizer {

TIMStabilizer::TIMStabilizer(std::shared_ptr<devices::ethercatdevices::IEtherCATMotor> motor,
    const int cycleTime, const int totalTime) :
    ecMotor_(motor),
    cycleTime_(cycleTime),
    totalTime_(totalTime),
    logger_("TIMStabilizer"),
    isInitialized_(false),
    isActivated_(false) {
    logger_->debug("CTor");
}

TIMStabilizer::~TIMStabilizer() {
    logger_->debug("DTor");
    if (isInitialized_) deinitialize();
}

bool TIMStabilizer::initialize() {
    logger_->info("initialize");
    if (isInitialized_) {
        logger_->warn("Already initialized");
        return false;
    }

    auto isInFault = ecMotor_->inFault();
    if (!isInFault) {
        logger_->error("Could not get if Stabilizer is in fault");
        return false;
    }
    if ((isInFault.value()) && !ecMotor_->faultReset()) {
        logger_->error("Could not reset the fault of the Stabilizer");
        return false;
    }
    if (!ecMotor_->shutdown()) {
        logger_->error("Could not shutdown Stabilizer");
        return false;
    }
    if (!ecMotor_->setMotorRatedCurrent(motorRatedCurrent_)) {
        logger_->error("Can't write Motor Rated Current;");
        return false;
    }
    if (!ecMotor_->setMotorRatedTorque(motorRatedTorque_)) {
        logger_->error("Can't write Motor Rated Torque;");
        return false;
    }
    if (!ecMotor_->setMaxCurrent(motorMaxCurrent_)) {
        logger_->error("Can't write Max Torque;");
        return false;
    }
    if (!ecMotor_->setMaxTorque(motorMaxTorque_)) {
        logger_->error("Can't write Max Torque;");
        return false;
    }
    if (!ecMotor_->setMaxAcceleration(acceleration_)) {
        logger_->error("Can't write Maximum Acceleration;");
        return false;
    }
    if (!ecMotor_->setMaxDeceleration(acceleration_)) {
        logger_->error("Can't write Maximum Deceleration;");
        return false;
    }
    if (!ecMotor_->setQuickstopDeceleration(acceleration_)) {
        logger_->error("Can't write Quickstop Deceleration;");
        return false;
    }
    if (!ecMotor_->setProfileAcceleration(acceleration_)) {
        logger_->error("Can't write Profile Acceleration;");
        return false;
    }
    if (!ecMotor_->setProfileDeceleration(acceleration_)) {
        logger_->error("Can't write Profile Deceleration;");
        return false;
    }
    if (!ecMotor_->setModeOfOperation(
        crf::devices::ethercatdevices::modesofoperation::ProfileVelocityMode)) {
        logger_->error("Could not set the Profile Velocity Mode.");
        return false;
    }
    if (!ecMotor_->enableOperation()) {
        logger_->error("Could not enable operation of Stabilizer");
        return false;
    }
    auto enabled = ecMotor_->isEnabled();
    if (!enabled) {
        logger_->error("Could not check if Stabilizer is enabled. Communication error.");
        return false;
    }
    if (!enabled.value()) {
        logger_->error("Stabilizer did not enable");
        return false;
    }
    isInitialized_ = true;
    isActivated_ = true;

    if (!deactivate()) {
        logger_->error("Stabilizer is not open. Can't initialize the stabilizer.");
        isInitialized_ = false;
        return false;
    }
    return true;
}

bool TIMStabilizer::deinitialize() {
    logger_->debug("deinitialize");
    if (!isInitialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    if (!ecMotor_->disableOperation()) {
        logger_->error("Could not disable operation of Stabilizer");
        return false;
    }
    isInitialized_ = false;
    isActivated_ = false;
    return true;
}

bool TIMStabilizer::activate() {
    logger_->debug("activate");
    if (!isInitialized_) {
        logger_->error("Not initialized");
        return false;
    }

    if (isActivated_) {
        logger_->warn("Stabilizer already active");
        return true;
    }
    if (!ecMotor_->setModeOfOperation(
        crf::devices::ethercatdevices::modesofoperation::ProfileTorqueMode)) {
        logger_->error("Could not set the Profile Torque Mode.");
        return false;
    }
    if (!ecMotor_->setTorque(closingTorque_)) {
        logger_->error("Stabilizer can't be closed");
        return false;
    }

    bool retVal = true;
    if (!functionTimeCheck([this](){return ecMotor_->targetReached();})) {
        logger_->error("Watchdog timeout. Problem with the Stabilizer");
        retVal = false;
    }
    if (!ecMotor_->stop()) {
        logger_->error("Could not stop the motor.");
        return false;
    }
    if (!ecMotor_->setModeOfOperation(
        crf::devices::ethercatdevices::modesofoperation::ProfileVelocityMode)) {
        logger_->error("Could not set the Profile Velocity Mode.");
        return false;
    }

    if (retVal) isActivated_ = true;
    auto state = isDeactivated();
    if (!state) {
        logger_->error("Could not check if the Stabilizer is open. Communication error.");
        return false;
    }
    if (state.value()) {
        logger_->error("Target Torque has been reached, but the Stabilizier"\
        "is still in open position");
        isActivated_ = false;
        return false;
    }
    return retVal;
}

bool TIMStabilizer::deactivate() {
    logger_->debug("deactivate");
    if (!isInitialized_) {
        logger_->error("Not initialized");
        return false;
    }

    if (!isActivated_) {
        logger_->warn("Stabilizer already un-active");
        return true;
    }
    if (!ecMotor_->setModeOfOperation(
        crf::devices::ethercatdevices::modesofoperation::ProfileVelocityMode)) {
        logger_->error("Could not set the Profile Velocity Mode.");
        return false;
    }
    if (!ecMotor_->setVelocity(openingVelocity_)) {
        logger_->error("Stabilizer can't be closed");
        return false;
    }

    isActivated_ = false;
    bool retVal = true;
    if (!functionTimeCheck([this](){return ecMotor_->internalLimitActive();})) {
        logger_->error("Watchdog timeout. Problem with the stabilizer");
        retVal = false;
    }
    return retVal;
}

std::optional<bool> TIMStabilizer::isActivated() {
    logger_->debug("isActivated");
    if (!isInitialized_) {
        logger_->error("Not initialized");
        return std::nullopt;
    }
    auto state = isDeactivated();
    if (!state) {
        logger_->error("Can't check if the Stabilizer is closed. Communication error.");
        return std::nullopt;
    }
    return isActivated_;
}

std::optional<bool> TIMStabilizer::isDeactivated() {
    logger_->debug("isDeactivated");
    if (!isInitialized_) {
        logger_->error("Not initialized");
        return std::nullopt;
    }

    auto state = ecMotor_->internalLimitActive();
    if (!state) {
        logger_->error("Can't check if the Stabilizer is open. Communication error.");
        return std::nullopt;
    }

    if (state.value()) {
        isActivated_ = false;
    }
    return state.value();
}

bool TIMStabilizer::resetFaultState() {
    logger_->debug("resetFaultState");
    std::optional<bool> isInFault = ecMotor_->inFault();
    if (!isInFault) {
        logger_->error("Could not get if Stabilizer is in fault");
        return false;
    }
    if (isInFault.value()) {
        if (!ecMotor_->faultReset()) {
            logger_->error("Could not reset the fault of the Stabilizer");
            return false;
        }
        if (!ecMotor_->shutdown()) {
            logger_->error("Could not shutdown Stabilizer");
            return false;
        }
        if (!ecMotor_->enableOperation()) {
            logger_->error("Could not enable operation of Stabilizer");
            return false;
        }
        auto enabled = ecMotor_->isEnabled();
        if (!enabled) {
            logger_->error("Could not check if Stabilizer is enabled. Communication error.");
            return false;
        }
        if (!enabled.value()) {
            logger_->error("Stabilizer did not enable");
            return false;
        }
    }
    return true;
}

std::optional<bool> TIMStabilizer::isInFault() {
    std::optional<bool> isInFault = ecMotor_->inFault();
    if (!isInFault) {
        logger_->error("Could not get if Stabilizer is in fault");
        return std::nullopt;
    }
    return isInFault.value();
}

bool TIMStabilizer::functionTimeCheck(std::function<std::optional<bool>()> function) {
    std::optional<bool> state;
    auto start = std::chrono::steady_clock::now();
    auto end = std::chrono::steady_clock::now();
    do {
        std::this_thread::sleep_for(std::chrono::microseconds(cycleTime_));
        state = function();
        if (!state) {
            return false;
        }
        if (state.value()) {
            return true;
        }
        end = std::chrono::steady_clock::now();
    } while (
        std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count() <= totalTime_);
    return false;
}

}  // namespace crf::actuators::mechanicalstabilizer
