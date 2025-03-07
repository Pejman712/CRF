/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <vector>

#include "RobotArm/IRobotArm.hpp"

namespace crf::actuators::robotarm {

class RobotArmMock : public IRobotArm {
 public:
  MOCK_METHOD(bool, initialize, (), (override));
  MOCK_METHOD(bool, deinitialize, (), (override));

  MOCK_METHOD(boost::optional<crf::utility::types::JointPositions>, getJointPositions, (), (override));  // NOLINT
  MOCK_METHOD(boost::optional<crf::utility::types::JointVelocities>, getJointVelocities, (), (override));  // NOLINT
  MOCK_METHOD(boost::optional<crf::utility::types::JointForceTorques>, getJointForceTorques, (), (override)); // NOLINT
  MOCK_METHOD(boost::optional<crf::utility::types::TaskPose>, getTaskPose, (), (override));
  MOCK_METHOD(boost::optional<crf::utility::types::TaskVelocity>, getTaskVelocity, (), (override));
  MOCK_METHOD(boost::optional<crf::utility::types::TaskForceTorque>, getTaskForceTorque, (), (override)); // NOLINT

  MOCK_METHOD(bool, setJointPositions, (const crf::utility::types::JointPositions&), (override));
  MOCK_METHOD(bool, setJointVelocities, (const crf::utility::types::JointVelocities&), (override));
  MOCK_METHOD(bool, setJointForceTorques, (const crf::utility::types::JointForceTorques&), (override)); // NOLINT
  MOCK_METHOD(bool, setTaskPose, (const crf::utility::types::TaskPose&), (override));
  MOCK_METHOD(bool, setTaskVelocity, (const crf::utility::types::TaskVelocity&, bool), (override));

  MOCK_METHOD(bool, stopArm, (), (override));
  MOCK_METHOD(bool, enableBrakes, (), (override));
  MOCK_METHOD(bool, disableBrakes, (), (override));
  MOCK_METHOD(std::shared_ptr<RobotArmConfiguration>, getConfiguration, (), (override));
};

}  // namespace crf::actuators::robotarm
