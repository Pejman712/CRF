/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

#include <nlohmann/json.hpp>

#include "EventLogger/EventLogger.hpp"

namespace crf::actuators::linearstage {

class LinearStageConfiguration {
 public:
    LinearStageConfiguration();
    virtual ~LinearStageConfiguration() = default;
    /*
    * We leave this as virtual so one could implement an hard coded configuration
    * for some hardware that never changes
    */ 
    virtual bool parse(const nlohmann::json& config);

    float getRotationToLinearRatio();
    float getLinearToRotationRatio();

    bool hasMinimumPosition();
    float getMinimumPosition();
    bool hasMaximumPosition();
    float getMaximumPosition();
    float getMaximumVelocity();
    float getMaximumAcceleration();
    float getMaximumDeceleration();

 protected:
    utility::logger::EventLogger logger_;

    float rotationToLinearRatio_;

    bool hasMinimumPosition_;
    float minimumPosition_;
    bool hasMaximumPosition_;
    float maximumPosition_;
    float maximumVelocity_;
    float maximumAcceleration_;
    float maximumDeceleration_;
};

}  // namespace crf::actuators::linearstage
