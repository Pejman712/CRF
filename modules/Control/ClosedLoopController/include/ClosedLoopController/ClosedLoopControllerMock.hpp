/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pawel Ptasznik CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#pragma once

#include <gmock/gmock.h>

#include <vector>

#include "ClosedLoopController/IClosedLoopController.hpp"

namespace crf::control::closedloopcontroller {

class ClosedLoopControllerMock : public IClosedLoopController {
 public:
  MOCK_METHOD2(calculate,
      boost::optional<std::vector<double> >(const std::vector<double>& setpoint,
        const std::vector<double>& feedbackValue));
  MOCK_METHOD0(reset,
      void());
};

}  // namespace crf::control::closedloopcontroller
