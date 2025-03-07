/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Julia Kabalar CERN EN/SMM/MRO
 * 
 *  ==================================================================================================
 */

#pragma once

#include <gmock/gmock.h>

#include "TrajectoryPointGenerator/ITrajectoryPointGenerator.hpp"

namespace crf::control::trajectorypointgenerator {

class TrajectoryPointGeneratorMock : public ITrajectoryPointGenerator {
 public:
  MOCK_CONST_METHOD0(getTaskTrajectoryPoint,
      boost::optional<utility::types::TaskTrajectoryData>());
  MOCK_METHOD1(updatePositionTarget,
      bool(const utility::types::TaskPose& targetPosition));
  MOCK_METHOD1(updateVelocityTarget,
      bool(const utility::types::TaskVelocity& targetVelocity));
  MOCK_METHOD1(updateCurrentState,
      bool(const utility::types::TaskTrajectoryData& currentState));
  MOCK_METHOD1(updateMotionConstraints,
      bool(const utility::types::TaskTrajectoryData& maximumState));
  MOCK_CONST_METHOD0(getControlMode,
      ControlMode());
};

}  // namespace crf::control::trajectorypointgenerator
