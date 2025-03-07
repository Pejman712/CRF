/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Julia Kabalar CERN EN/SMM/MRO 2019
 * 
 *  ==================================================================================================
 */

#pragma once

#include <vector>

#include <gmock/gmock.h>

#include "TrajectoryGeneratorDeprecated/ITaskTrajectoryGenerator.hpp"

namespace crf::control::trajectorygeneratordeprecated {

class TaskTrajectoryGeneratorMock : public ITaskTrajectoryGenerator {
 public:
  MOCK_METHOD1(computeTrajectory,
      bool(const std::vector<utility::types::TaskPose> &path));
  MOCK_CONST_METHOD0(getDuration,
      boost::optional<float>());
  MOCK_CONST_METHOD1(getTaskPose,
      boost::optional<utility::types::TaskPose>(float time));
  MOCK_CONST_METHOD1(getTaskVelocity,
      boost::optional<utility::types::TaskVelocity>(float time));
  MOCK_CONST_METHOD1(getTaskAcceleration,
      boost::optional<utility::types::TaskAcceleration>(float time));
  MOCK_CONST_METHOD0(getTaskTrajectory,
      boost::optional<TaskTrajectoryData>());
};

}  // namespace crf::control::trajectorygeneratordeprecated
