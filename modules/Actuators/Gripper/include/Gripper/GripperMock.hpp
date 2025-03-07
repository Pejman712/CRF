/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#pragma once

#include <gmock/gmock.h>

#include "Gripper/IGripper.hpp"

namespace crf::actuators::gripper {

class GripperMock : public IGripper {
 public:
    MOCK_METHOD(bool, initialize, (), (override));
    MOCK_METHOD(bool, deinitialize, (), (override));

    MOCK_METHOD(boost::optional<float>, getPosition, (), (override));
    MOCK_METHOD(bool, isGrasping, (), (override));
    MOCK_METHOD(bool, setPosition, (float), (override));
    MOCK_METHOD(bool, setPosition, (GripperState), (override));
    MOCK_METHOD(bool, setGraspingForce, (float), (override));
    MOCK_METHOD(bool, stopGripper, (), (override));
    MOCK_METHOD(bool, setVelocity, (float), (override));
};

}  // namespace crf::actuators::gripper
