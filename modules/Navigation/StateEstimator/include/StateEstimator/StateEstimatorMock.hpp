/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Julia Kabalar CERN EN/SMM/MRO
 * 
 *  ==================================================================================================
 */

#pragma once

#include <vector>

#include <gmock/gmock.h>

#include "StateEstimator/IStateEstimator.hpp"

namespace crf {
namespace algorithms {
namespace stateestimator {

class StateEstimatorMock : public IStateEstimator {
 public:
  MOCK_METHOD0_T(getEstimate,
      std::vector<float>());
  MOCK_METHOD1_T(addMeasurement,
      bool(const std::vector<float>& measuredValues));
};

}  // namespace stateestimator
}  // namespace algorithms
}  // namespace crf

