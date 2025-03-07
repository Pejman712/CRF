/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#include <string>
#include <map>
#include <utility>

#include "TIMRPWagon/TIMRPWagonCommands.hpp"
#include "SiemensPLC/SiemensPLCTypeConverter.hpp"

namespace crf::actuators::timrpwagon {

using crf::devices::siemensplc::SiemensPLCTypeConverter;

TIMRPWagonCommands::TIMRPWagonCommands() :
    logger_("TIMRPWagonCommands"),
    isEmpty_(true),
    enableRPArmManualControl_(false),
    retractRPArm_(false),
    deployRPArm_(false),
    moveRPArmUp_(false),
    moveRPArmDown_(false),
    releaseLockRPArm_(false),
    stopRPArm_(false),
    acknowledgeError_(false),
    resetRPArmDriver_(false) {
    logger_->debug("CTor");
}

TIMRPWagonCommands::TIMRPWagonCommands(const TIMRPWagonCommands& input) :
    logger_("TIMRPWagonCommands") {
    logger_->debug("CTor");
    if (input.isEmpty()) {
        isEmpty_ = true;
        return;
    }
    isEmpty_ = false;
    enableRPArmManualControl_ = input.enableRPArmManualControl();
    retractRPArm_ = input.retractRPArm();
    deployRPArm_ = input.deployRPArm();
    moveRPArmUp_ = input.moveRPArmUp();
    moveRPArmDown_ = input.moveRPArmDown();
    releaseLockRPArm_ = input.releaseLockRPArm();
    stopRPArm_ = input.stopRPArm();
    acknowledgeError_ = input.acknowledgeError();
    resetRPArmDriver_ = input.resetRPArmDriver();
}

bool TIMRPWagonCommands::parseSiemensPLCBuffer(const std::string& buffer,
    std::map<std::string, std::pair<unsigned int, unsigned int>> variablesDBLocation) {
    logger_->debug("parseSiemensPLCBuffer");
    try {
        enableRPArmManualControl_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["EnableRPArmManualControl"].second,
            variablesDBLocation["EnableRPArmManualControl"].first);
        retractRPArm_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["RetractRPArm"].second,
            variablesDBLocation["RetractRPArm"].first);
        deployRPArm_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["DeployRPArm"].second,
            variablesDBLocation["DeployRPArm"].first);
        moveRPArmUp_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["MoveRPArmUp"].second,
            variablesDBLocation["MoveRPArmUp"].first);
        moveRPArmDown_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["MoveRPArmDown"].second,
            variablesDBLocation["MoveRPArmDown"].first);
        releaseLockRPArm_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["ReleaseLockRPArm"].second,
            variablesDBLocation["ReleaseLockRPArm"].first);
        stopRPArm_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["StopRPArm"].second,
            variablesDBLocation["StopRPArm"].first);
        acknowledgeError_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["AcknowledgeErrors"].second,
            variablesDBLocation["AcknowledgeErrors"].first);
        resetRPArmDriver_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["ResetRPArmDriver"].second,
            variablesDBLocation["ResetRPArmDriver"].first);
    } catch (std::invalid_argument& e) {
        logger_->error("Failed to parse TIM inputs - {}", e.what());
        return false;
    }
    isEmpty_ = false;
    return true;
}

void TIMRPWagonCommands::clear() {
    isEmpty_ = true;
    enableRPArmManualControl_ = false;
    retractRPArm_ = false;
    deployRPArm_ = false;
    moveRPArmUp_ = false;
    moveRPArmDown_ = false;
    releaseLockRPArm_ = false;
    stopRPArm_ = false;
    acknowledgeError_ = false;
    resetRPArmDriver_ = false;
    return;
}

bool TIMRPWagonCommands::isEmpty() const {
    return isEmpty_;
}

bool TIMRPWagonCommands::enableRPArmManualControl() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM RP Wagon Commands is empty");
    }
    return enableRPArmManualControl_;
}

bool TIMRPWagonCommands::retractRPArm() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM RP Wagon Commands is empty");
    }
    return retractRPArm_;
}

bool TIMRPWagonCommands::deployRPArm() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM RP Wagon Commands is empty");
    }
    return deployRPArm_;
}

bool TIMRPWagonCommands::moveRPArmUp() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM RP Wagon Commands is empty");
    }
    return moveRPArmUp_;
}

bool TIMRPWagonCommands::moveRPArmDown() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM RP Wagon Commands is empty");
    }
    return moveRPArmDown_;
}

bool TIMRPWagonCommands::releaseLockRPArm() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM RP Wagon Commands is empty");
    }
    return releaseLockRPArm_;
}

bool TIMRPWagonCommands::stopRPArm() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM RP Wagon Commands is empty");
    }
    return stopRPArm_;
}

bool TIMRPWagonCommands::acknowledgeError() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM RP Wagon Commands is empty");
    }
    return acknowledgeError_;
}

bool TIMRPWagonCommands::resetRPArmDriver() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM RP Wagon Commands is empty");
    }
    return resetRPArmDriver_;
}

}  // namespace crf::actuators::timrpwagon
