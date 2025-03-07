#pragma once

/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */
#include <memory>
#include <string>
#include <nlohmann/json.hpp>

#include "KinovaArm/KinovaJaco.hpp"
#include "Types/Types.hpp"

namespace crf::actuators::kinovaarm {

class KinovaJacoMock : public KinovaJaco {
 public:
    explicit KinovaJacoMock(const nlohmann::json& config):KinovaJaco(nullptr, config) {}
    MOCK_METHOD0(initialize,
    bool());
    MOCK_METHOD0(deinitialize,
    bool());
    MOCK_METHOD0(getJointPositions,
            boost::optional<utility::types::JointPositions>());
    MOCK_METHOD0(getJointVelocities,
            boost::optional<utility::types::JointVelocities>());
    MOCK_METHOD0(getJointForceTorques,
            boost::optional<crf::utility::types::JointForceTorques>());
    MOCK_METHOD0(getTaskPose,
            boost::optional<utility::types::TaskPose>());
    MOCK_METHOD0(getTaskVelocity,
            boost::optional<utility::types::TaskVelocity>());
    MOCK_METHOD0(getTaskForceTorque,
            boost::optional<crf::utility::types::TaskForceTorque>());
    MOCK_METHOD1(setJointPositions,
    bool(const utility::types::JointPositions& jointPositions));
    MOCK_METHOD1(setJointVelocities,
    bool(const utility::types::JointVelocities& jointVelocities));
    MOCK_METHOD1(setJointForceTorques,
    bool(const crf::utility::types::JointForceTorques& jointForceTorques));
    MOCK_METHOD1(setTaskPose,
    bool(const utility::types::TaskPose& position));
    MOCK_METHOD2(setTaskVelocity,
    bool(const utility::types::TaskVelocity& velocity, bool TCP));
    MOCK_METHOD0(stopArm,
    bool());
    MOCK_METHOD0(enableBrakes,
    bool());
    MOCK_METHOD0(disableBrakes,
    bool());
    MOCK_METHOD0(getConfiguration,
    std::shared_ptr<robotarm::RobotArmConfiguration>());
    MOCK_METHOD0(moveHomePosition,
    bool());
    MOCK_METHOD0(zeroJointForceTorques,
    bool());
};

}  // namespace crf::actuators::kinovaarm
