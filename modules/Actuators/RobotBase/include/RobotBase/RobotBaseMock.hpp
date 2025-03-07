#pragma once
/* Â© Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Julia Kabalar CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>

#include <memory>
#include <vector>

#include "RobotBase/IRobotBase.hpp"

namespace crf::actuators::robotbase {

class RobotBaseMock : public IRobotBase {
 public:
    MOCK_METHOD0(initialize,
        bool());
    MOCK_METHOD0(deinitialize,
        bool());
    MOCK_METHOD0(getTaskPose,
        boost::optional<utility::types::TaskPose>());
    MOCK_METHOD0(getTaskVelocity,
        boost::optional<utility::types::TaskVelocity>());
    MOCK_METHOD0(getMotorsVelocities,
        boost::optional<std::vector<float> >());
    MOCK_METHOD0(getMotorsCurrent,
        boost::optional<std::vector<float> >());
    MOCK_METHOD1(setTaskVelocity,
        bool(const utility::types::TaskVelocity& velocity));
    MOCK_METHOD1(setWheelsVelocity,
        bool(const std::vector<float> velocity));
    MOCK_METHOD0(stopBase,
        bool());
    MOCK_METHOD0(errorActive,
        bool());
    MOCK_METHOD0(acknowledgeError,
        bool());
    MOCK_METHOD0(getConfiguration,
        std::shared_ptr<RobotBaseConfiguration>());
    MOCK_METHOD1(setMaximumWheelsAcceleration,
        bool(float acceleration));
    MOCK_METHOD1(setMaximumWheelsVelocity,
        bool(float velocity));
    MOCK_METHOD1(setMaximumTaskVelocity,
        bool(const utility::types::TaskVelocity& velocity));
    MOCK_CONST_METHOD0(getMaximumWheelsAcceleration,
        float());
    MOCK_CONST_METHOD0(getMaximumWheelsVelocity,
        float());
    MOCK_CONST_METHOD0(getMaximumTaskVelocity,
        utility::types::TaskVelocity());
};

}  // namespace crf::actuators::robotbase
