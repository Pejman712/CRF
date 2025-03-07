/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Leanne Attard CERN EN-SMM-MRO
 * 
 *  ==================================================================================================
 */

#pragma once

#include <gmock/gmock.h>

#include "ThermalCamera/IFLIRFactory.hpp"

namespace crf {
namespace sensors {
namespace thermalcamera {

class FLIRFactoryMock : public IFLIRFactory {
 public:
  MOCK_METHOD1(createDeviceParams,
      DeviceParams* (PvDevice *device));
  MOCK_METHOD1(createTempConverter,
      TempConverter* (DeviceParams* params));
  MOCK_METHOD1(selectThermalDevice,
      PvString* (int index));
};

}  // namespace thermalcamera
}  // namespace sensors
}  // namespace crf
