/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include "LinearStage/LinearStageConfiguration.hpp"

namespace crf::actuators::linearstage {

LinearStageConfiguration::LinearStageConfiguration() :
    logger_("LinearStageConfiguration"),
    rotationToLinearRatio_(0),
    hasMinimumPosition_(false),
    minimumPosition_(0),
    hasMaximumPosition_(false),
    maximumPosition_(0),
    maximumVelocity_(0),
    maximumAcceleration_(0),
    maximumDeceleration_(0) { }

bool LinearStageConfiguration::parse(const nlohmann::json& config) {
    logger_->debug("parse");
    try {
        rotationToLinearRatio_ = config.at("rotationToLinearRatio").get<float>();
        if (config.count("minimumPosition") > 0) {
            hasMinimumPosition_ = true;
            minimumPosition_ = config.at("minimumPosition").get<float>();
        }
        if (config.count("maximumPosition") > 0) {
            hasMaximumPosition_ = true;
            maximumPosition_ = config.at("maximumPosition").get<float>();
        }

        maximumVelocity_ = config.at("maximumVelocity").get<float>();
        maximumAcceleration_ = config.at("maximumAcceleration").get<float>();
        if (config.find("maximumDeceleration") == config.end()) {
            maximumDeceleration_ = maximumAcceleration_;
        } else {
            maximumDeceleration_ = config.at("maximumDeceleration").get<float>();
        }
    } catch (const std::exception& ex) {
        logger_->error("Failed to parse configuration: {}", ex.what());
        rotationToLinearRatio_ = 0;
        hasMinimumPosition_ = false;
        hasMaximumPosition_ = false;
        minimumPosition_ = 0;
        maximumPosition_ = 0;
        maximumVelocity_ = 0;
        maximumAcceleration_ = 0;
        maximumDeceleration_ = 0;

        return false;
    }

    return true;
}

float LinearStageConfiguration::getLinearToRotationRatio() {
    if (rotationToLinearRatio_ == 0) return 0;
    return 1/rotationToLinearRatio_;
}

float LinearStageConfiguration::getRotationToLinearRatio() {
    return rotationToLinearRatio_;
}

bool LinearStageConfiguration::hasMinimumPosition() {
    return hasMinimumPosition_;
}

float LinearStageConfiguration::getMinimumPosition() {
    return minimumPosition_;
}

bool LinearStageConfiguration::hasMaximumPosition() {
    return hasMaximumPosition_;
}

float LinearStageConfiguration::getMaximumPosition() {
    return maximumPosition_;
}

float LinearStageConfiguration::getMaximumVelocity() {
    return maximumVelocity_;
}

float LinearStageConfiguration::getMaximumAcceleration() {
    return maximumAcceleration_;
}

float LinearStageConfiguration::getMaximumDeceleration() {
    return maximumDeceleration_;
}

}  // namespace crf::actuators::linearstage
