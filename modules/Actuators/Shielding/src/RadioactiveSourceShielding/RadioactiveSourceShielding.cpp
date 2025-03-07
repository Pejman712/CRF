/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Francesco Riccardi CERN BE/CEM/MRO
 *
 *  ==================================================================================================
 */

#include <thread>
#include <chrono>

#include "Shielding/RadioactiveSourceShielding/RadioactiveSourceShielding.hpp"
#include "EtherCATDevices/EtherCATDef.hpp"

namespace crf::actuators::shielding {

RadioactiveSourceShielding::RadioactiveSourceShielding(
    std::shared_ptr<devices::ethercatdevices::IEtherCATMotor> motor,
    const int cycleTime, const int totalTime) :
    ecMotor_(motor),
    cycleTime_(cycleTime),
    totalTime_(totalTime),
    logger_("RadioactiveSourceShielding"),
    isInitialized_(false) {
    logger_->debug("CTor");
}

RadioactiveSourceShielding::~RadioactiveSourceShielding() {
    logger_->debug("DTor");
    if (isInitialized_) deinitialize();
}

bool RadioactiveSourceShielding::initialize() {
    logger_->info("initialize");
    if (isInitialized_) {
        logger_->warn("Already initialized");
        return false;
    }

    auto isInFault = ecMotor_->inFault();
    if (!isInFault) {
        logger_->error("Could not get if Shielding is in fault");
        return false;
    }
    if ((isInFault.value()) && !ecMotor_->faultReset()) {
        logger_->error("Could not reset the fault of the Shielding");
        return false;
    }
    if (!ecMotor_->shutdown()) {
        logger_->error("Could not shutdown Shielding");
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
    if (!ecMotor_->setProfileVelocity(velocity_)) {
        logger_->error("Can't write Profile Deceleration;");
        return false;
    }
    if (!ecMotor_->setModeOfOperation(
        crf::devices::ethercatdevices::modesofoperation::ProfilePositionMode)) {
        logger_->error("Could not set the Profile Position Mode.");
        return false;
    }
    isInitialized_ = true;

    return true;
}

bool RadioactiveSourceShielding::deinitialize() {
    logger_->debug("deinitialize");
    if (!isInitialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    if (!ecMotor_->disableOperation()) {
        logger_->error("Could not disable operation of Shielding");
        return false;
    }
    isInitialized_ = false;
    return true;
}

bool RadioactiveSourceShielding::open() {
    logger_->debug("open");
    if (!isInitialized_) {
        logger_->error("Not initialized");
        return false;
    }

    if (!ecMotor_->enableOperation()) {
        logger_->error("Could not enable operation of Shielding");
        return false;
    }
    auto enabled = ecMotor_->isEnabled();
    if (!enabled) {
        logger_->error("Could not check if Shielding is enabled. Communication error.");
        return false;
    }
    if (!enabled.value()) {
        logger_->error("Shielding did not enable");
        return false;
    }

    std::optional<bool> opened = isOpen();
    if (!opened) {
        logger_->error("Could not check if Shielding is open. Communication error.");
        return false;
    }
    if (opened.value()) {
        logger_->warn("Shielding already open");
    }

    if (!ecMotor_->setPosition(openPosition_, false)) {
        logger_->error("Shielding can't be opened");
        return false;
    }

    bool retVal = true;
    if (!functionTimeCheck([this](){return ecMotor_->targetReached();})) {
        logger_->error("Watchdog timeout. Problem with the Shielding");
        retVal = false;
    }

    if (!ecMotor_->disableOperation()) {
        logger_->error("Could not disable operation of Shielding");
        return false;
    }
    return retVal;
}

bool RadioactiveSourceShielding::close() {
    logger_->debug("close");
    if (!isInitialized_) {
        logger_->error("Not initialized");
        return false;
    }

    if (!ecMotor_->enableOperation()) {
        logger_->error("Could not enable operation of Shielding");
        return false;
    }
    auto enabled = ecMotor_->isEnabled();
    if (!enabled) {
        logger_->error("Could not check if Shielding is enabled. Communication error.");
        return false;
    }
    if (!enabled.value()) {
        logger_->error("Shielding did not enable");
        return false;
    }

    std::optional<bool> closed = isClosed();
    if (!closed) {
        logger_->error("Could not check if Shielding is close. Communication error.");
        return false;
    }
    if (closed.value()) {
        logger_->warn("Shielding already close");
    }

    if (!ecMotor_->setPosition(closePosition_, false)) {
        logger_->error("Shielding can't be closed");
        return false;
    }

    bool retVal = true;
    if (!functionTimeCheck([this](){return ecMotor_->targetReached();})) {
        logger_->error("Watchdog timeout. Problem with the Shielding");
        retVal = false;
    }

    if (!ecMotor_->disableOperation()) {
        logger_->error("Could not disable operation of Shielding");
        return false;
    }
    return retVal;
}

std::optional<bool> RadioactiveSourceShielding::isOpen() {
    logger_->debug("isOpen");
    if (!isInitialized_) {
        logger_->error("Not initialized");
        return std::nullopt;
    }

    auto state = ecMotor_->getPosition();
    if (!state) {
        logger_->error("Can't check if the Shielding is open. Communication error.");
        return std::nullopt;
    }
    auto position = state.value();

    if (fabs(position-openPosition_) < positionThreshold_) {
        return true;
    }
    return false;
}

std::optional<bool> RadioactiveSourceShielding::isClosed() {
    logger_->debug("isClosed");
    if (!isInitialized_) {
        logger_->error("Not initialized");
        return std::nullopt;
    }

    auto state = ecMotor_->getPosition();
    if (!state) {
        logger_->error("Can't check if the Shielding is close. Communication error.");
        return std::nullopt;
    }
    auto position = state.value();
    if (fabs(position-closePosition_) < positionThreshold_) {
        return true;
    }
    return false;
}

bool RadioactiveSourceShielding::resetFaultState() {
    std::optional<bool> isInFault = ecMotor_->inFault();
    if (!isInFault) {
        logger_->error("Could not get if Shielding is in fault");
        return false;
    }
    if (isInFault.value()) {
        if (!ecMotor_->faultReset()) {
            logger_->error("Could not reset the fault of the Shielding");
            return false;
        }
        if (!ecMotor_->shutdown()) {
            logger_->error("Could not shutdown Shielding");
            return false;
        }
        if (!ecMotor_->enableOperation()) {
            logger_->error("Could not enable operation of Shielding");
            return false;
        }
        auto enabled = ecMotor_->isEnabled();
        if (!enabled) {
            logger_->error("Could not check if Shielding is enabled. Communication error.");
            return false;
        }
        if (!enabled.value()) {
            logger_->error("Shielding did not enable");
            return false;
        }
    }
    return true;
}

std::optional<bool> RadioactiveSourceShielding::isInFault() {
    std::optional<bool> isInFault = ecMotor_->inFault();
    if (!isInFault) {
        logger_->error("Could not get if Stabilizer is in fault");
        return std::nullopt;
    }
    return isInFault.value();
}

bool RadioactiveSourceShielding::functionTimeCheck(std::function<std::optional<bool>()> function) {
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

}  // namespace crf::actuators::shielding
