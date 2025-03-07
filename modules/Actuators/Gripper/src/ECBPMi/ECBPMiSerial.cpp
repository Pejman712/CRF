/* © Copyright CERN 2023.  All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alessandro Vascelli CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#include "Gripper/ECBPMi/ECBPMiSerial.hpp"

#include <urdf_parser/urdf_parser.h>

namespace crf::actuators::gripper {

ECBPMiSerial::ECBPMiSerial(std::shared_ptr<ISerialCommunication> serial,
    const std::string& urdfPath):
    serial_(serial),
    initialized_(false),
    logger_("ECBPMiSerial"),
    controlThread_(),
    currentOption_(Option::Deactivate),
    isRunning_(false),
    model_(urdf::parseURDFFile(urdfPath)) {
    logger_->debug("CTor");
    if (!model_) {
        throw std::invalid_argument("Invalid URDF file");
    }
}

ECBPMiSerial::~ECBPMiSerial() {
    logger_->debug("DTor");
    deinitialize();
}

bool ECBPMiSerial::initialize() {
    logger_->debug("initialize");
    if (initialized_) {
        return true;
    }
    if (!serial_->initialize()) {
        logger_->warn("Failed to initialize");
        return false;
    }
    for (unsigned int i = 0; i < numberOfAttempts_; i++) {
        if (!sendCommand(unlock_)) {
            logger_->warn("Failed to send unlock command");
            return false;
        }
        // Sleep needed for correct initializing process
        std::this_thread::sleep_for(initializeTime_);
    }
    logger_->debug("Vacuum gripper initialized");
    initialized_ = true;
    return true;
}

bool ECBPMiSerial::deinitialize() {
    logger_->debug("deinitialize");
    if (!initialized_) {
        return true;
    }
    if (!deactivate()) {
        logger_->warn("Failed to deactivate");
        return false;
    }
    if (controlThread_.joinable()) {
        controlThread_.join();
    }
    if (!serial_->deinitialize()) {
        return false;
    }
    initialized_ = false;
    logger_->info("The vacuum gripper has been deinitialized");
    return true;
}

crf::expected<bool> ECBPMiSerial::activate() {
    logger_->debug("activate");
    if (!initialized_) {
        return crf::Code::NotInitialized;
    }
    if (currentOption_ != Option::Activate) {
        currentOption_ = Option::Activate;
    }
    if (!isRunning_) {
        if (controlThread_.joinable()) {
            controlThread_.join();
            deinitialize();
        }
        isRunning_ = true;
        controlThread_ = std::thread(&ECBPMiSerial::control, this);
    }
    return true;
}

crf::expected<bool> ECBPMiSerial::deactivate() {
    logger_->debug("deactivate");
    if (!initialized_) {
        return crf::Code::NotInitialized;
    }
    if (currentOption_ == Option::Deactivate) {
        return true;
    }
    currentOption_ = Option::Deactivate;
    // Sleep to set for a bit the air off and then standby
    std::this_thread::sleep_for(blowOffTime_);
    isRunning_ = false;
    if (controlThread_.joinable()) {
        controlThread_.join();
    }
    for (unsigned int i = 0; i < numberOfAttempts_; i++) {
        if (!sendCommand(stayCommand_)) {
            logger_->warn("Blow off failed");
            return crf::Code::ThirdPartyQueryFailed;
        }
    }
    return true;
}

crf::expected<bool> ECBPMiSerial::isActive() {
    logger_->debug("isActive");
    if (currentOption_ == Option::Activate) {
        return true;
    }
    return false;
}

bool ECBPMiSerial::sendCommand(const unsigned char* command) {
    std::string requestString(reinterpret_cast<const char*>(command), sizeof(command));
    if (serial_->write(requestString) == -1) {
        return false;
    }
    // Sleep to guarantee the correct command change while sending
    std::this_thread::sleep_for(sendingTime_);
    checkResponse();
    return true;
}

void ECBPMiSerial::checkResponse() {
    std::string response;
    serial_->read(&response, 256);
    if (!response.empty() && (response[0] == 0x03 || response[0] == 0x43)) {
        logger_->info("Grabbed something");
        return;
    } else if (!response.empty() && (response[0] == 0x040)) {
         logger_->warn("Nothing grabbed");
        return;
    }
    return;
}

void ECBPMiSerial::control() {
    while (isRunning_) {
        switch (currentOption_) {
            case Option::Activate:
                if (!sendCommand(activateCommand_)) {
                    logger_->warn("Suction failed");
                    isRunning_ = false;
                }
                break;
            case Option::Deactivate:
                if (!sendCommand(deactivateCommand_)) {
                    logger_->warn("Blow off failed");
                    isRunning_ =  false;
                }
                break;
            default:
                break;
        }
    }
    currentOption_ = Option::Deactivate;
    return;
}

std::shared_ptr<urdf::ModelInterface> ECBPMiSerial::getURDF() {
    logger_->debug("getURDF");
    return model_;
}

}  // namespace crf::actuators::gripper
