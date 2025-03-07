/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Julia Kabalar CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

#include <gmock/gmock.h>

#include <memory>
#include <vector>

#include "LinearStage/ILinearStage.hpp"
#include "LinearStage/LinearStageConfiguration.hpp"

namespace crf::actuators::linearstage {

class LinearStageMock : public ILinearStage {
 public:
  MOCK_METHOD0(initialize,
      bool());
  MOCK_METHOD0(deinitialize,
      bool());
  MOCK_METHOD1(setTargetPosition,
      bool(float position));
  MOCK_METHOD1(setTargetVelocity,
      bool(float velocity));
  MOCK_METHOD0(getActualPosition,
      boost::optional<float>());
  MOCK_METHOD0(getActualVelocity,
      boost::optional<float>());
  MOCK_METHOD0(getConfiguration,
      std::shared_ptr<LinearStageConfiguration>());
};

}  // namespace crf::actuators::linearstage
