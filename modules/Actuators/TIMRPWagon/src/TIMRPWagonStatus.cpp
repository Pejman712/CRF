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

#include "TIMRPWagon/TIMRPWagonStatus.hpp"
#include "SiemensPLC/SiemensPLCTypeConverter.hpp"

namespace crf::actuators::timrpwagon {

using crf::devices::siemensplc::SiemensPLCTypeConverter;

TIMRPWagonStatus::TIMRPWagonStatus() :
    logger_("TIMRPWagonStatus"),
    isEmpty_(true),
    rpArmRetracted_(false),
    rpArmInTheMiddle_(false),
    rpArmDeployed_(false),
    rpArmInError_(false) {
    logger_->debug("CTor");
}

TIMRPWagonStatus::TIMRPWagonStatus(const TIMRPWagonStatus& input) :
    logger_("TIMRPWagonStatus") {
    logger_->debug("CTor");
    if (input.isEmpty()) {
        isEmpty_ = true;
        return;
    }
    isEmpty_ = false;
    rpArmRetracted_ = input.rpArmRetracted();
    rpArmInTheMiddle_ = input.rpArmInTheMiddle();
    rpArmDeployed_ = input.rpArmDeployed();
    rpArmInError_ = input.rpArmInError();
}

bool TIMRPWagonStatus::parseSiemensPLCBuffer(const std::string& buffer,
    std::map<std::string, std::pair<unsigned int, unsigned int>> variablesDBLocation) {
    logger_->debug("parseSiemensPLCBuffer");
    try {
        rpArmRetracted_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["RPArmRetracted"].second,
            variablesDBLocation["RPArmRetracted"].first);
        rpArmInTheMiddle_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["RPArmInTheMiddle"].second,
            variablesDBLocation["RPArmInTheMiddle"].first);
        rpArmDeployed_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["RPArmDeployed"].second,
            variablesDBLocation["RPArmDeployed"].first);
        rpArmInError_ = SiemensPLCTypeConverter::getBit(buffer,
            variablesDBLocation["RPArmError"].second,
            variablesDBLocation["RPArmError"].first);
    } catch (std::invalid_argument& e) {
        logger_->error("Failed to parse TIM RP Wagon status - {}", e.what());
        return false;
    }
    isEmpty_ = false;
    return true;
}

void TIMRPWagonStatus::clear() {
    isEmpty_ = true;
    rpArmRetracted_ = false;
    rpArmInTheMiddle_ = false;
    rpArmDeployed_ = false;
    rpArmInError_ = false;
    return;
}

bool TIMRPWagonStatus::isEmpty() const {
    return isEmpty_;
}

bool TIMRPWagonStatus::rpArmRetracted() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM RP Wagon Status is empty");
    }
    return rpArmRetracted_;
}

bool TIMRPWagonStatus::rpArmInTheMiddle() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM RP Wagon Status is empty");
    }
    return rpArmInTheMiddle_;
}

bool TIMRPWagonStatus::rpArmDeployed() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM RP Wagon Status is empty");
    }
    return rpArmDeployed_;
}

bool TIMRPWagonStatus::rpArmInError() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM RP Wagon Status is empty");
    }
    return rpArmInError_;
}

}  // namespace crf::actuators::timrpwagon
