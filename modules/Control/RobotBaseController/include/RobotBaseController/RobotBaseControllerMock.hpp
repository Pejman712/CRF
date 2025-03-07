/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */
#pragma once

#include <memory>
#include <vector>

#include "RobotBaseController/IRobotBaseController.hpp"

namespace crf::control::robotbasecontroller {

class RobotBaseControllerMock : public IRobotBaseController {
 public:
    MOCK_METHOD0(initialize,
        bool());
    MOCK_METHOD0(deinitialize,
        bool());

    MOCK_METHOD1(setPosition,
        std::future<bool>(const crf::utility::types::TaskPose& targetPosition));
    MOCK_METHOD1(setPosition,
        std::future<bool>(const std::vector<crf::utility::types::TaskPose>& targetPositions));

    MOCK_METHOD1(setVelocity,
        bool(const crf::utility::types::TaskVelocity& targetVelocity));
    MOCK_METHOD1(setVelocity,
        bool(const std::vector<crf::utility::types::TaskVelocity>& targetVelocities));

    MOCK_METHOD0(interruptTrajectory,
        bool());

    MOCK_METHOD0(getPosition,
        crf::utility::types::TaskPose());
    MOCK_METHOD0(getVelocity,
        crf::utility::types::TaskVelocity());

    MOCK_METHOD1(setMaximumVelocity,
        bool(const utility::types::TaskVelocity& maxVelocity));
    MOCK_METHOD1(setMaximumAcceleration,
        bool(const utility::types::TaskAcceleration& maxAcceleration));
};

}  // namespace crf::control::robotbasecontroller
