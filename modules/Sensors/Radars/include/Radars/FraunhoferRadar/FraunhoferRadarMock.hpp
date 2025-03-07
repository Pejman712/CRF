/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Arturs Ivanovs CERN EN-SMM-MRO
 *
 *  ==================================================================================================
 */

#pragma once

#include <vector>
#include <complex>


#include <gmock/gmock.h>

namespace crf {
namespace sensors {
namespace fraunhoferradar {

class RadarMock : public IRadar {
 public:
  MOCK_METHOD0(initialize,
      bool());
  MOCK_METHOD0(deinitialize,
      bool());
  MOCK_METHOD0(getFrame,
      std::vector<std::vector<float>>());
  MOCK_METHOD0(getMaxObservationFrequency,
      float());
};

}  // namespace fraunhoferradar
}  // namespace sensors
}  // namespace crf
