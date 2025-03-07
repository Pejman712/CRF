/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#include <string>
#include <map>
#include <utility>

#include "TIM/TIMSettings.hpp"
#include "SiemensPLC/SiemensPLCTypeConverter.hpp"

namespace crf::actuators::tim {

using crf::devices::siemensplc::SiemensPLCTypeConverter;

TIMSettings::TIMSettings() :
    logger_("TIMSettings"),
    isEmpty_(true),
    targetPosition_(0),
    targetVelocity_(0),
    positionSetManually_(0),
    obstacleID_(0),
    obstacleMaximumVelocity_(0) {
    logger_->debug("CTor");
}

TIMSettings::TIMSettings(const TIMSettings& input) :
    logger_("TIMSettings") {
    logger_->debug("CTor");
    if (input.isEmpty()) {
        isEmpty_ = true;
        return;
    }
    isEmpty_ = false;
    targetPosition_ = input.targetPosition();
    targetVelocity_ = input.targetVelocity();
    positionSetManually_ = input.positionSetManually();
    obstacleID_ = input.obstacleID();
    obstacleMaximumVelocity_ = input.obstacleMaximumVelocity();
}

bool TIMSettings::parseSiemensPLCBuffer(const std::string& buffer,
    std::map<std::string, std::array<unsigned int, 2>> variablesDBLocation) {
    logger_->debug("parseSiemensPLCBuffer - {}", buffer.size());
    try {
        targetPosition_ = SiemensPLCTypeConverter::getFloat(buffer,
            variablesDBLocation["TargetPosition"][0]);
        targetVelocity_ = SiemensPLCTypeConverter::getFloat(buffer,
            variablesDBLocation["TargetVelocity"][0]);
        positionSetManually_ = SiemensPLCTypeConverter::getFloat(buffer,
            variablesDBLocation["PositionSetManually"][0]);
    } catch (std::invalid_argument& e) {
        logger_->error("Failed to parse TIM settings - {}", e.what());
        return false;
    }
    try {
        obstacleID_ = SiemensPLCTypeConverter::getSInt(buffer,
            variablesDBLocation["ObstacleID"][0]);
        obstacleMaximumVelocity_ = SiemensPLCTypeConverter::getFloat(buffer,
            variablesDBLocation["ObstacleMaximumVelocity"][0]);
    } catch (std::invalid_argument& e) {
        logger_->debug("No permission to write obstacles - {}", e.what());
    }
    isEmpty_ = false;
    return true;
}

void TIMSettings::clear() {
    isEmpty_ = true;
    targetPosition_ = 0;
    targetVelocity_ = 0;
    positionSetManually_ = 0;
    obstacleID_ = 0;
    obstacleMaximumVelocity_ = 0;
    return;
}

bool TIMSettings::isEmpty() const {
    return isEmpty_;
}

float TIMSettings::targetPosition() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Settings is empty");
    }
    return targetPosition_;
}

void TIMSettings::targetPosition(float input) {
    isEmpty_ = false;
    targetPosition_ = input;
}

float TIMSettings::targetVelocity() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Settings is empty");
    }
    return targetVelocity_;
}

void TIMSettings::targetVelocity(float input) {
    isEmpty_ = false;
    targetVelocity_ = input;
}

float TIMSettings::positionSetManually() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Settings is empty");
    }
    return positionSetManually_;
}

void TIMSettings::positionSetManually(float input) {
    isEmpty_ = false;
    positionSetManually_ = input;
}

int8_t TIMSettings::obstacleID() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Settings is empty");
    }
    return obstacleID_;
}

void TIMSettings::obstacleID(int8_t input) {
    isEmpty_ = false;
    obstacleID_ = input;
}

float TIMSettings::obstacleMaximumVelocity() const {
    if (isEmpty_) {
        throw std::runtime_error("TIM Settings is empty");
    }
    return obstacleMaximumVelocity_;
}

void TIMSettings::obstacleMaximumVelocity(float input) {
    isEmpty_ = false;
    obstacleMaximumVelocity_ = input;
}

}  // namespace crf::actuators::tim
