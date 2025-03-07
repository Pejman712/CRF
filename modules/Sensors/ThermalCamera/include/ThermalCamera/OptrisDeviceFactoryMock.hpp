/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Pawel Ptasznik CERN EN/STI/ECE
 * 
 *  ==================================================================================================
 */

#pragma once

#include <gmock/gmock.h>

#include "ThermalCamera/IOptrisDeviceFactory.hpp"

namespace crf {
namespace sensors {
namespace thermalcamera {

class OptrisDeviceFactoryMock : public IOptrisDeviceFactory {
 public:
  MOCK_METHOD0(createDevice,
      std::shared_ptr<evo::IRDevice>());
  MOCK_METHOD3(createImager,
      std::shared_ptr<evo::IRImager>(unsigned int frequency, unsigned int width, unsigned int height));  // NOLINT
};

}  // namespace thermalcamera
}  // namespace sensors
}  // namespace crf
