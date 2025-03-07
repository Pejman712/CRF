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

#include "TIM/LHCObstacle.hpp"

namespace crf::actuators::tim {

LHCObstacle::LHCObstacle() :
    logger_("LHCObstacle"),
    isEmpty_(true),
    identifier_(-1),
    type_(LHCObstacleType::NotDefined),
    startPosition_(-1),
    endPosition_(-1),
    maximumVelocity_(-1),
    mustRetractDevices_(true) {
}

LHCObstacle::LHCObstacle(unsigned int identifier, LHCObstacleType type, float start, float end,
    float vel, bool retract) :
    logger_("LHCObstacle"),
    isEmpty_(false),
    identifier_(identifier),
    type_(type),
    startPosition_(start),
    endPosition_(end),
    maximumVelocity_(vel),
    mustRetractDevices_(retract) {
}

LHCObstacle::LHCObstacle(const LHCObstacle& input) :
    logger_("LHCObstacle") {
    if (input.isEmpty()) {
        isEmpty_ = true;
        return;
    }
    isEmpty_ = false;
    identifier_ = input.identifier();
    type_ = input.type();
    startPosition_ = input.startPosition();
    endPosition_ = input.endPosition();
    maximumVelocity_ = input.maximumVelocity();
    mustRetractDevices_ = input.mustRetractDevices();
}

LHCObstacle& LHCObstacle::operator=(LHCObstacle other) {
    logger_ = crf::utility::logger::EventLogger("LHCObstacle");
    isEmpty_ = other.isEmpty_.load();
    identifier_ = other.identifier_.load();
    type_ = other.type_.load();
    startPosition_ = other.startPosition_.load();
    endPosition_ = other.endPosition_.load();
    maximumVelocity_ = other.maximumVelocity_.load();
    mustRetractDevices_ = other.mustRetractDevices_.load();
    return *this;
}

void LHCObstacle::clear() {
    isEmpty_ = true;
    identifier_ = 0;
    type_ = LHCObstacleType::NotDefined;
    startPosition_ = 0;
    endPosition_ = 0;
    maximumVelocity_ = 0;
    mustRetractDevices_ = true;
}

bool LHCObstacle::isEmpty() const {
    return isEmpty_;
}

int8_t LHCObstacle::identifier() const {
    if (isEmpty_) {
        throw std::runtime_error("LHC Obstacle is empty");
    }
    return identifier_;
}

void LHCObstacle::identifier(int8_t input) {
    isEmpty_ = false;
    identifier_ = input;
}

LHCObstacleType LHCObstacle::type() const {
    if (isEmpty_) {
        throw std::runtime_error("LHC Obstacle is empty");
    }
    return type_;
}

void LHCObstacle::type(LHCObstacleType input) {
    isEmpty_ = false;
    type_ = input;
}

float LHCObstacle::startPosition() const {
    if (isEmpty_) {
        throw std::runtime_error("LHC Obstacle is empty");
    }
    return startPosition_;
}

void LHCObstacle::startPosition(float input) {
    isEmpty_ = false;
    startPosition_ = input;
}

float LHCObstacle::endPosition() const {
    if (isEmpty_) {
        throw std::runtime_error("LHC Obstacle is empty");
    }
    return endPosition_;
}

void LHCObstacle::endPosition(float input) {
    isEmpty_ = false;
    endPosition_ = input;
}

float LHCObstacle::maximumVelocity() const {
    if (isEmpty_) {
        throw std::runtime_error("LHC Obstacle is empty");
    }
    return maximumVelocity_;
}

void LHCObstacle::maximumVelocity(float input) {
    isEmpty_ = false;
    maximumVelocity_ = input;
}

bool LHCObstacle::mustRetractDevices() const {
    if (isEmpty_) {
        throw std::runtime_error("LHC Obstacle is empty");
    }
    return mustRetractDevices_;
}

void LHCObstacle::mustRetractDevices(bool input) {
    isEmpty_ = false;
    mustRetractDevices_ = input;
}

}  // namespace crf::actuators::tim
