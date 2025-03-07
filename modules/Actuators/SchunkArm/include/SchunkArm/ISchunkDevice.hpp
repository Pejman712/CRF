#pragma once

/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */
#include "CommonInterfaces/IInitializable.hpp"
#include <boost/optional.hpp>
#include <string>

struct can_frame;

namespace crf::actuators::schunkarm {

class ISchunkDevice : public utility::commoninterfaces::IInitializable {
 public:
    ~ISchunkDevice() override = default;
    bool initialize() override = 0;
    bool deinitialize() override = 0;

    virtual bool setPosition(float rad) = 0;
    virtual bool setVelocity(float radPerSec) = 0;
    virtual bool setTargetVelocityToPercentage(float percentage) = 0;
    virtual bool setTargetAccelerationToPercentage(float percentage) = 0;
    virtual bool setTargetCurrentToPercentage(float percentage) = 0;
    virtual bool applyBreak() = 0;
    virtual bool fastStop() = 0;
    virtual bool handleError(std::string error) = 0;
    virtual bool getState() = 0;
    virtual bool getStatePeriodic(int milisecond) = 0;
    virtual bool parseStatePeriodic(can_frame frame) = 0;

    // Returns float in radians , [-pi, +pi]
    virtual boost::optional<float> getMotorPosition() = 0;
    virtual boost::optional<float> getMotorVelocity() = 0;
    virtual boost::optional<float> getMaxPosition() = 0;
    virtual boost::optional<float> getMinPosition() = 0;
    virtual boost::optional<float> getMaxVelocity() = 0;

    virtual bool isBrakeActive() = 0;
    /* Returns whether the motor path was blocked, all of the following is true:
     * motor velocity is 0
     * the motor current is over the limit by 15%
     * the break timed out (the break is not moving)
     */
    virtual bool isMoveBlocked() = 0;
    // returns none if no error, otherwise returns the errorcode
    virtual boost::optional<uint8_t> getErrorCode() = 0;
};


}  // namespace crf::actuators::schunkarm
