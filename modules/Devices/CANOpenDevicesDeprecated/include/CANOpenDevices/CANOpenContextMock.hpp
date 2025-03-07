/* Â© Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

#include <gmock/gmock.h>

#include <chrono>
#include <memory>

#include "CANOpenDevices/ICANOpenContext.hpp"
#include "CANOpenDevices/ICANOpenDevice.hpp"

namespace crf {
namespace devices {
namespace canopendevices {

class CANOpenContextMock : public ICANOpenContext {
 public:
  MOCK_METHOD0(initialize,
      bool());
  MOCK_METHOD0(deinitialize,
      bool());
  MOCK_METHOD1(addDevice,
      bool(std::shared_ptr<ICANOpenDevice>));
  MOCK_METHOD0(sendSync,
      bool());
  MOCK_METHOD0(sendGuard,
      bool());
  MOCK_METHOD1(setSyncFrequency,
      bool(const std::chrono::milliseconds& frequency));
  MOCK_METHOD1(setGuardFrequency,
      bool(const std::chrono::milliseconds& frequency));
};

}  // namespace canopendevices
}  // namespace devices
}  // namespace crf
