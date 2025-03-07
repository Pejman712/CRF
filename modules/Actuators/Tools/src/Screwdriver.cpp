/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Francesco Riccardi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <chrono>

#include "Tools/Screwdriver.hpp"

namespace crf {
namespace devices {
namespace tools {

Screwdriver::Screwdriver(std::shared_ptr<ethercatdevices::IEtherCATMotor> motor, int motorIndex) :
    logger_("Screwdriver"),
    ecMotor_(motor),
    motorIndex_(motorIndex),
    initialized_(false),
    startWatchdog_(false),
    resetWatchdog_(false),
    threadWatchdog_() {
    logger_->debug("CTor");
}

Screwdriver::~Screwdriver() {
    logger_->debug("DTor");
    deinitialize();
}

bool Screwdriver::initialize() {
    logger_->debug("initialize");
    if (initialized_) {
        logger_->warn("Already initialized;");
        return false;
    }
    if (!ecMotor_->setModeOfOperation(3)) {
        logger_->error("Can't put Screwdriver in Velocity Mode;");
        return false;
    }
    if (ecMotor_->inFault().get()) {
        if (!ecMotor_->faultReset()) {
            logger_->error("Screwdriver Fault can't be Reset;");
            return false;
        }
    }
    if (!ecMotor_->shutdown()) {
        logger_->error("Screwdriver NOT in Shutdown;");
        return false;
    }
    if (!ecMotor_->enableOperation()) {
        logger_->error("Screwdriver NOT ready for Operation;");
        return false;
    }
    if (!ecMotor_->setMaxCurrent(50000)) {
        logger_->error("Can't write Max Torque;");
        return false;
    }
    if (!ecMotor_->setMaxTorque(50000)) {
        logger_->error("Can't write Max Torque;");
        return false;
    }
    if (!ecMotor_->setQuickstopDeceleration(20000)) {
        logger_->error("Can't write Quickstop Deceleration;");
        return false;
    }
    if (!ecMotor_->setMaxAcceleration(20000)) {
        logger_->error("Can't write Maximum Acceleration;");
        return false;
    }
    if (!ecMotor_->setMaxDeceleration(20000)) {
        logger_->error("Can't write Maximum Deceleration;");
        return false;
    }
    if (!ecMotor_->setProfileAcceleration(10000)) {
        logger_->error("Can't write Profile Acceleration;");
        return false;
    }
    if (!ecMotor_->setProfileDeceleration(10000)) {
        logger_->error("Can't write Profile Deceleration;");
        return false;
    }
    logger_->info("Screwdriver initialized");
    initialized_ = true;
    return true;
}

bool Screwdriver::deinitialize() {
    logger_->debug("deinitialize");
    if (!initialized_) {
        logger_->warn("Not initialized.");
        return false;
    }
    if (startWatchdog_) {
        startWatchdog_ = false;
        threadWatchdog_.join();
    }
    resetWatchdog_ = false;
    initialized_ = false;
    return true;
}

bool Screwdriver::setValue(const std::string& name, bool value) {
    // Bool values are not supported for this tool
    logger_->error("Bool values are not supported for this tool.");
    return false;
}

bool Screwdriver::setValue(const std::string& name, int value) {
    // Int values are not supported for this tool
    logger_->error("Int values are not supported for this tool.");
    return false;
}

bool Screwdriver::setValue(const std::string& name, float value) {
    if (!startWatchdog_) {
        startWatchdog_ = true;
        threadWatchdog_ = std::thread(&Screwdriver::watchdog, this);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    resetWatchdog_ = true;
    int32_t scaledValue = 7000 * value;
    if (name == "velocity") {
        return ecMotor_->setVelocity(scaledValue);
    }
    logger_->error("Command name not valid.");
    return false;
}

boost::optional<boost::any> Screwdriver::getValue(const std::string& name) const {
    logger_->error("No implementation of this function for this tool.");
    return boost::none;
}

boost::optional<ToolValueTypes> Screwdriver::getValueType(const std::string& name) {
    if (name == "velocity") {
        return ToolValueTypes::FLOAT;
    }
    return boost::none;
}

std::vector<std::string> Screwdriver::getValueNames() {
    return std::vector<std::string>({"velocity"});
}

void Screwdriver::watchdog() {
    std::chrono::steady_clock::time_point start;
    std::chrono::steady_clock::time_point now;
    while (startWatchdog_) {
        start = std::chrono::steady_clock::now();
        now = std::chrono::steady_clock::now();
        while (startWatchdog_ && resetWatchdog_ == false &&
        (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() <= 500)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            now = std::chrono::steady_clock::now();
        }
        if (resetWatchdog_ == false) ecMotor_->stop();
        resetWatchdog_ = false;
    }
    ecMotor_->stop();
}

}  // namespace tools
}  // namespace devices
}  // namespace crf
