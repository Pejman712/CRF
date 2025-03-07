/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Julia Kabalar CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */
#pragma once

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "Types/Types.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf::actuators::robotbase {

struct RobotWheel {
    float wheelsDistanceX;
    float wheelsDistanceY;
    float wheelsDiameter;
};

struct RobotParameters {
    uint8_t wheelsCount;
    // std::vector<RobotWheel> wheels; we will move to this line only if we will have not symmetric robots NOLINT
    float wheelsDistanceX;
    float wheelsDistanceY;
    float wheelsDiameter;
    float maximumWheelsVelocity;
    float maximumWheelsAcceleration;
    bool hasLiftingStage;
};

struct TaskLimits {
    utility::types::TaskVelocity maximumVelocity;
    utility::types::TaskAcceleration maximumAcceleration;
};

class RobotBaseConfiguration {
 public:
    RobotBaseConfiguration();
    virtual ~RobotBaseConfiguration() = default;

    virtual bool parse(const nlohmann::json&);
    /*
     * Returns:
     *  - Struct containing physical parameters of the robotbase
     *  - Empty struct on parsing failure
     */
    RobotParameters getRobotParameters() const;
    /*
     * Returns:
     *  - Struct containing the maximum applicable velocity and acceleration limits
     *  - Empty struct on parsing failure
     */
    TaskLimits getTaskLimits() const;
    /*
     * Returns:
     *  - Real Time Loop Interval [us] used in publisher thread
     *  - Zero value on parsing failure
     */
    uint64_t getRTLoopTime() const;
    /*
     * Returns:
     *  - Integer value of mounted number of wheels
     *  - Zero value on parsing failure
     */
    uint8_t getNumberOfWheels() const;

 protected:
    utility::logger::EventLogger logger_;
    RobotParameters botConfig_;
    TaskLimits taskLimits_;
    uint64_t rtLoopTimeUs_;
};

}  // namespace crf::actuators::robotbase
